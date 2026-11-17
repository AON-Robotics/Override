#include "../../../include/aon/drivetrain/differential-drive.hpp"
#include <array>
#include <new>
#include <type_traits>

namespace aon {

// The path worker owns request storage and motor writes across callback deletion.
// Callbacks transfer only bounded plain data and wait without application locks.
class Drivetrain::PersistentPathWorker final : public Drivetrain::PathWorker {
  using Status = PurePursuit::Status;
  static_assert(std::atomic<uint32_t>::is_always_lock_free &&
                std::atomic<pros::task_t>::is_always_lock_free,
                "Callback mailboxes require lock-free epoch/task atomics");
  static_assert(std::is_trivially_copyable<Pose>::value,
                "Deletable submitters may transfer only trivial request data");
  enum Phase : uint32_t { FREE, WRITING, READY, EXECUTING, DONE };
  static constexpr uint32_t phaseMask = 7, callbackBit = 8, epochMask = 0x0fffffff;
  static constexpr size_t maximumPoints = 4096;
  Drivetrain& drive;
  std::atomic<bool> ready{false};
  std::atomic<uint32_t> callbackEpoch{0};
  std::atomic<pros::task_t> callbackTask{nullptr};
  pros::task_t executor = nullptr;
  std::atomic<uint32_t> stopRequested{0}, stopCompleted{0};

  struct Request {
    std::atomic<uint32_t> token{FREE};
    // Only trivially destructible data is written by a deletable submitter.
    // An interrupted copy can be abandoned without repairing a vector/allocator.
    std::array<Pose,maximumPoints> points;
    std::array<double,maximumPoints> caps;
    size_t count = 0, capCount = 0, captureCapacity = 0;
    uint32_t timeoutMs = 0, submittedAt = 0, generation = 0;
    bool reversed = false, hasTrace = false, invalidTrace = false;
    bool alignHeading = false;
    PathResult result;
    PathTrace capture;
    // These containers are accessed only by the persistent executor, or by a
    // live submitter after DONE. They are never modified during request transfer.
    std::vector<PathSample> samples;
  } request;

  static void wait() { pros::c::task_delay(1); }
  uint32_t epoch() const { return callbackEpoch.load() & epochMask; }
  uint32_t stamp() const {
    return (epoch()<<4) | (pros::c::task_get_current()==callbackTask.load() ? callbackBit : 0);
  }
  bool stale(uint32_t token) const { return (token>>4)!=epoch(); }
  void reclaim(std::atomic<uint32_t>& token) {
    uint32_t value = token.load();
    if ((value&callbackBit) && stale(value) && (value&phaseMask)!=EXECUTING)
      token.compare_exchange_strong(value,FREE);
  }
  void executeRequests() {
    while (true) {
      if (!ready.load()) { wait(); continue; }
      const uint32_t stop = stopRequested.load();
      if (stop != stopCompleted.load()) {
        std::lock_guard<pros::Mutex> lock(drive.pathCommandMutex);
        drive.pathCancelGeneration.fetch_add(1);
        if (!drive.stopPathMotors()) drive.pathMotorFailure = true;
        stopCompleted.store(stop);
      }
      reclaim(request.token);
      uint32_t token=request.token.load();
      if ((token&phaseMask)!=READY ||
          !request.token.compare_exchange_strong(token,(token&~phaseMask)|EXECUTING)) {
        wait(); continue;
      }
      PathResult result;
      result.status=result.translationStatus=Status::CANCELLED;
      request.capture=PathTrace{};
      if (!stale(token)) {
        try {
          // Allocation and configuration copying belong to the durable owner.
          std::vector<Pose> path(request.points.begin(),request.points.begin()+request.count);
          std::vector<double> caps(request.caps.begin(),request.caps.begin()+request.capCount);
          if (request.hasTrace && !request.invalidTrace) {
            request.samples.resize(request.captureCapacity);
            request.capture.samples=request.samples.data();
            request.capture.capacity=request.samples.size();
          } else if (request.invalidTrace) request.capture.capacity=1;
          const uint32_t spent=uint32_t(pros::millis()-request.submittedAt);
          // Copy/queue time consumes translation's cooperative budget too.
          const uint32_t budget=spent>=request.timeoutMs ? 0 : request.timeoutMs-spent;
          // Recheck after preparation: a mode change must not lend a new epoch
          // to an old READY request. Mode handoff also waits for this scope.
          if (!stale(token)) {
            result=drive.executePath(path,budget,caps,request.reversed,
                request.hasTrace ? &request.capture : nullptr,request.generation,request.alignHeading);
          }
        } catch (...) {
          drive.stop();
          result.status=result.translationStatus=Status::CONTROL_FAILURE;
          drive.pathStatus.store(Status::CONTROL_FAILURE);
        }
      }
      result.elapsedMs=uint32_t(pros::millis()-request.submittedAt);
      request.capture.status=result.status;
      request.capture.translationStatus=result.translationStatus;
      drive.pathStatus.store(result.status);
      request.result=result;
      request.token.store((token&~phaseMask)|DONE);
    }
  }

 public:
  explicit PersistentPathWorker(Drivetrain& owner):drive(owner) {}
  bool start() {
    callbackTask.store(pros::c::task_get_current());
    executor=pros::c::task_create([](void* self) {
      static_cast<PersistentPathWorker*>(self)->executeRequests();
    },this,TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT,"AON path execution");
    ready.store(executor != nullptr);
    return ready.load();
  }
  bool ownsExecution() const override {
    return ready.load() && pros::c::task_get_current()==executor;
  }
  PathResult follow(const std::vector<Pose>& path,uint32_t timeoutMs,
      const std::vector<double>& caps,bool reversed,
      PathTrace* trace, bool alignHeading) override {
    PathResult rejected;
    rejected.status=Status::CONTROL_FAILURE;
    if (!ready.load()) return rejected;
    const uint32_t identity=stamp();
    const uint32_t generation=drive.pathCancelGeneration.load();
    uint32_t free=FREE;
    if (!request.token.compare_exchange_strong(free,identity|WRITING)) return rejected;
    request.generation=generation;
    request.submittedAt=pros::millis(); request.timeoutMs=timeoutMs;
    // Oversize or mismatched inputs reach the original executor as invalid
    // geometry; bounded transfer never reads beyond the caller's containers.
    request.count=path.size()<=maximumPoints ? path.size() : 0;
    request.capCount=caps.size()<=maximumPoints ? caps.size() : 0;
    if ((!caps.empty() && caps.size()!=path.size()) || caps.size()>maximumPoints)
      request.count=0;
    for (size_t i=0;i<request.count;++i) request.points[i]=path[i];
    for (size_t i=0;i<request.capCount;++i) request.caps[i]=caps[i];
    request.reversed=reversed;
    request.alignHeading=alignHeading;
    request.hasTrace=trace!=nullptr;
    request.captureCapacity=trace ? trace->capacity : 0;
    request.invalidTrace=trace && trace->capacity && !trace->samples;
    request.token.store(identity|READY);
    while (request.token.load()!=(identity|DONE)) wait();
    const PathResult result=request.result;
    if (trace) {
      const auto storage=trace->samples;
      const size_t capacity=trace->capacity;
      *trace=request.capture;
      trace->samples=storage; trace->capacity=capacity;
      for (size_t i=0;i<trace->size;++i) storage[i]=request.samples[i];
    }
    request.token.store(FREE);
    return result;
  }
  void stopAndWait() override {
    if (!ready.load()) return;
    const uint32_t ticket = stopRequested.fetch_add(1) + 1;
    // Outstanding stops must span fewer than 2^31 tickets across rollover.
    while (int32_t(stopCompleted.load() - ticket) < 0) wait();
  }
  void beginMode() override {
    // Only actual framework callback entry may prove its predecessor gone.
    callbackTask.store(pros::c::task_get_current());
    callbackEpoch.fetch_add(1);
    drive.pathEnabled.store(false);
    drive.pathCancelGeneration.fetch_add(1);
    if (!ready.load()) return;
    stopAndWait();
    // Do not enable a new motion owner before old execution releases its locks
    // and storage. DONE is safe: only its caller/result copy remains.
    while (true) {
      reclaim(request.token);
      const auto phase=request.token.load()&phaseMask;
      if (phase==FREE || phase==DONE) break;
      wait();
    }
  }
};

bool Drivetrain::initializePathWorker() {
  if (pathWorkerRequired) return pathWorkerStarted;
  pathWorkerRequired=true;
  auto* worker=new(std::nothrow) PersistentPathWorker(*this);
  if (!worker) { pathEnabled.store(false); pathStatus.store(PurePursuit::Status::CONTROL_FAILURE); return false; }
  pathWorker.reset(worker);
  const bool started=worker->start();
  pathWorkerStarted=started;
  if (!started) { pathEnabled.store(false); pathStatus.store(PurePursuit::Status::CONTROL_FAILURE); }
  return started;
}

void Drivetrain::beginCompetitionMode() { if (pathWorker) pathWorker->beginMode(); }

PurePursuit::Status Drivetrain::publishPath(uint32_t generation, double left, double right) {
  std::lock_guard<pros::Mutex> lock(pathCommandMutex);
  if (!pathEnabled.load() || pathCancelGeneration.load() != generation)
    return PurePursuit::Status::CANCELLED;
  if (pathMotorFailure || !commandPath(left,right)) {
    pathMotorFailure = true;
    return PurePursuit::Status::CONTROL_FAILURE;
  }
  return PurePursuit::Status::RUNNING;
}

void Drivetrain::stop() {
  // Revoke admission before waiting so the publishing task cannot repeatedly
  // reacquire the mutex with further commands while a stop is pending.
  pathCancelGeneration.fetch_add(1);
  if (pathWorkerStarted && !pathWorker->ownsExecution()) {
    pathWorker->stopAndWait();
    return;
  }
  std::lock_guard<pros::Mutex> lock(pathCommandMutex);
  if (!stopPathMotors()) pathMotorFailure = true;
}

void Drivetrain::cancelPath() {
  pathCancelGeneration.fetch_add(1);
  if (pathWorkerStarted && !pathWorker->ownsExecution()) {
    pathWorker->stopAndWait();
    return;
  }
  std::lock_guard<pros::Mutex> lock(pathCommandMutex);
}

void Drivetrain::setPathEnabled(bool enabled) {
  if (!enabled) {
    pathEnabled.store(false);
    pathCancelGeneration.fetch_add(1);
  }
  if (pathWorkerStarted && !pathWorker->ownsExecution()) {
    if (!enabled) pathWorker->stopAndWait();
    pathEnabled.store(enabled);
    return;
  }
  std::lock_guard<pros::Mutex> lock(pathCommandMutex);
  // Revoke again at the serialized boundary: an overlapping enable may have
  // admitted a request carrying the first revocation's generation while this
  // disable waited. That request must stay cancelled after a later enable.
  // The pre-lock revocation still interrupts earlier requests immediately.
  if (!enabled) pathCancelGeneration.fetch_add(1);
  pathEnabled.store(enabled);
  if (!enabled && !stopPathMotors()) pathMotorFailure = true;
}

Drivetrain::PathResult Drivetrain::followPath(const std::vector<Pose>& path, uint32_t timeoutMs,
    const std::vector<double>& caps, bool reversed,
    PathTrace* trace) {
  return runPath(path, timeoutMs, caps, reversed, trace, false);
}

Drivetrain::PathResult Drivetrain::runPath(
    const std::vector<Pose>& path, uint32_t timeoutMs,
    const std::vector<double>& caps, bool reversed, PathTrace* trace,
    bool alignHeading) {
  if (pathWorker && !pathWorker->ownsExecution())
    return pathWorker->follow(path, timeoutMs, caps, reversed, trace, alignHeading);
  return executePath(path, timeoutMs, caps, reversed, trace,
                     pathCancelGeneration.load(), alignHeading);
}

void Drivetrain::follow(const std::vector<Pose>& path) { (void)followPath(path); }

bool DifferentialDrive::commandPath(double left, double right) {
  // Pure Pursuit has already profiled and bounded these commands. A second,
  // independent wheel slew limiter can override speed caps and curvature.
  // Conversion must preserve coupled center/turn bounds, not just each wheel.
  const auto command = PurePursuit::motorCommands(left,right,thetaProfile->getMaxVelocity());
  const auto l = leftMotors.moveVelocityDirect(command.first);
  const auto r = rightMotors.moveVelocityDirect(command.second);
  return l == 1 && r == 1;
}

bool DifferentialDrive::stopPathMotors() {
  // Always attempt both zeros, even when one transport reports failure.
  const auto l = leftMotors.moveVelocityDirect(0);
  const auto r = rightMotors.moveVelocityDirect(0);
  return l == 1 && r == 1;
}


void DifferentialDrive::tank(const double &left, const double &right){
  this->leftMotors.move_velocity(left);
  this->rightMotors.move_velocity(right);
}

void DifferentialDrive::setBrakeMode(pros::MotorBrake brakeMode){
  leftMotors.set_brake_mode(brakeMode);
  rightMotors.set_brake_mode(brakeMode);
}

void DifferentialDrive::setGearset(pros::MotorGears gearset){
  leftMotors.set_gearing(gearset);
  rightMotors.set_gearing(gearset);
}

void DifferentialDrive::setEncoderUnits(pros::MotorEncoderUnits units){
  leftMotors.set_encoder_units(units);
  leftMotors.tare_position();
  rightMotors.set_encoder_units(units);
  rightMotors.tare_position();
  if (odometry) odometry->resetMotorBaselines();
}

void DifferentialDrive::setSlewRate(double slew){
  leftMotors.SetAcceleration(slew);
  rightMotors.SetAcceleration(slew);
}

double DifferentialDrive::getRPM(){
  double left = leftMotors.get_actual_velocity();
  double right = rightMotors.get_actual_velocity();
  return (left + right) / 2;
}

void DifferentialDrive::goToPose(const Pose& pose) {
  // Let followPath report missing odometry and attempt its terminal stops.
  const Pose start = odometry ? getPose() : Pose();
  (void)runPath({start, pose}, 15000, {}, false, nullptr, true);
}

Drivetrain::PathResult Drivetrain::executePath(const std::vector<Pose>& path, uint32_t timeoutMs,
    const std::vector<double>& caps, bool reversed,
    PathTrace* trace, uint32_t generation, bool alignHeading) {
  using Status = PurePursuit::Status;
  PathResult result;
  if (pathWorkerRequired && !pathWorker) {
    result.status = Status::CONTROL_FAILURE;
    return result;
  }
  if (pathActive.exchange(true)) {
    result.status = Status::CONTROL_FAILURE;
    return result;
  }
  // Reentrant requests fail above without waiting for the command mutex.
  struct ReleaseOnExit {
    Drivetrain* drive;
    ~ReleaseOnExit() { drive->pathActive.store(false); }
  } releaseOnExit{this};
  if (trace) trace->reset(); // rejected overlapping requests never touch capture
  bool admitted = false;
  {
    std::lock_guard<pros::Mutex> lock(pathCommandMutex);
    admitted = pathEnabled.load();
    if (admitted && pathCancelGeneration.load() == generation) pathMotorFailure = false;
    pathStatus.store(Status::RUNNING);
  }
  const uint32_t started = pros::millis();
  const auto requestStatus = [&] {
    if (pathCancelGeneration.load() != generation) return Status::CANCELLED;
    if (uint32_t(pros::millis()-started) >= timeoutMs)
      return Status::TIMEOUT;
    return Status::RUNNING;
  };
  auto finish = [&](Status status) {
    std::lock_guard<pros::Mutex> lock(pathCommandMutex);
    if (pathCancelGeneration.load() != generation && status != Status::CONTROL_FAILURE)
      status = Status::CANCELLED;
    // Publish terminal status only after both zero attempts have completed.
    // A failed terminal write must not be reported as settled success.
    const bool stopped = stopPathMotors();
    if (!stopped) pathMotorFailure = true;
    if (pathCancelGeneration.load() != generation && status != Status::CONTROL_FAILURE)
      status = Status::CANCELLED;
    if (pathMotorFailure) status = Status::CONTROL_FAILURE;
    const uint32_t finished = pros::millis();
    // Cooperative I/O may overrun a budget. Once writes return, a known
    // expired deadline must not become SUCCESS; stop errors/cancellation win.
    if (status == Status::SUCCESS &&
        uint32_t(finished-started) >= timeoutMs)
      status = Status::TIMEOUT;
    result.status = status;
    if (result.translationStatus == Status::IDLE) result.translationStatus = status;
    result.elapsedMs = uint32_t(finished-started);
    pathStatus.store(status);
    if (trace) {
      trace->status = status;
      trace->translationStatus = result.translationStatus;
      trace->terminalStopSucceeded = stopped;
    }
    return result;
  };
  if (!admitted) return finish(Status::CANCELLED);
  if (trace && trace->capacity && !trace->samples) return finish(Status::CONTROL_FAILURE);
  if (!supportsPurePursuit() || reversed)
    return finish(Status::UNSUPPORTED_CONFIGURATION);
  if (!yProfile || !thetaProfile) return finish(Status::CONTROL_FAILURE);
  constexpr double arrivalTolerance = 2.5; // inches
  PurePursuit controller(*yProfile, *thetaProfile, 5, arrivalTolerance, 2.5);
  const Status prepared = controller.setPath(path,caps);
  if (prepared != Status::RUNNING) return finish(prepared);
  result.finalHeading = controller.finalHeading();
  const auto record = [&](PathStage stage, const Pose& pose, uint32_t timestamp,
                          uint32_t interval, uint32_t compute, uint32_t publication,
                          double left, double right, Status status, bool published) {
    if (!trace) return;
    PathSample sample;
    sample.timestampUs = timestamp; sample.intervalUs = interval;
    sample.computeUs = compute; sample.publicationUs = publication;
    sample.stage = stage; sample.pose = pose; sample.diagnostics = result.diagnostics;
    sample.requestedLeftRPM = left; sample.requestedRightRPM = right;
    const auto pair = PurePursuit::motorCommands(left,right,thetaProfile->getMaxVelocity());
    sample.leftRPM = pair.first; sample.rightRPM = pair.second;
    sample.status = status; sample.published = published;
    trace->record(sample);
  };
  const auto publishAndRecord = [&](PathStage stage, const Pose& pose, uint32_t now,
                                   uint32_t elapsedUs, uint32_t computeUs,
                                   double left, double right) {
    const uint32_t publicationStart = trace ? uint32_t(pros::micros()) : 0;
    const Status status = publishPath(generation,left,right);
    record(stage,pose,now,elapsedUs,computeUs,
           trace ? uint32_t(uint32_t(pros::micros())-publicationStart) : 0,
           left,right,status,status == Status::RUNNING);
    return status;
  };
  uint32_t lastTime = uint32_t(pros::micros());
  bool first = true;
  const auto sampleInterval = [&](uint32_t now) {
    const uint32_t elapsedUs = first ? 0 : uint32_t(now-lastTime);
    const double dt = first ? 0.01 : elapsedUs/1e6;
    first = false; lastTime = now;
    if (trace) trace->maxIntervalUs = std::max(trace->maxIntervalUs,elapsedUs);
    return std::make_pair(elapsedUs,dt);
  };
  bool turning = false;
  MotionProfile turnProfile = *thetaProfile;
  turnProfile.setVelocity(0);
  turnProfile.setAccel(0);
  turnProfile.setFinalVelocity(0);
  while (true) {
    if (const Status status = requestStatus(); status != Status::RUNNING) return finish(status);
    Pose current;
    uint32_t estimateMs;
    if (!odometry || !odometry->getFreshPose(current, estimateMs))
      return finish(Status::CONTROL_FAILURE);
    const uint32_t now = uint32_t(pros::micros());
    const auto [elapsedUs,dt] = sampleInterval(now);
    if (turning) {
      if (!std::isfinite(current.theta) || dt <= 0 ||
          dt > PurePursuit::maxPathInterval)
        return finish(Status::CONTROL_FAILURE);
      const double error = std::remainder(
          std::remainder(result.finalHeading, 360.0) -
          std::remainder(current.theta, 360.0), 360.0);
      if (std::abs(error) <= 2.5) return finish(Status::SUCCESS);
      const double arc = std::abs(error) * DRIVE_WIDTH * M_PI / 360.0;
      const double rpm = std::copysign(std::min(turnProfile.update(arc, dt),
                                               double(MAX_RPM)), error);
      if (!std::isfinite(rpm)) return finish(Status::CONTROL_FAILURE);
      const Status published = publishPath(generation, rpm, -rpm);
      if (published != Status::RUNNING) return finish(published);
      pros::delay(10);
      continue;
    }
    const uint32_t computeStart = uint32_t(pros::micros());
    const auto output = controller.update(current, dt, estimateMs);
    const uint32_t computeUs = uint32_t(uint32_t(pros::micros())-computeStart);
    result.maxUpdateUs = std::max(result.maxUpdateUs,computeUs);
    result.diagnostics = controller.getDiagnostics();
    // Recheck asynchronous cancellation and deadline before publishing commands
    // or success. Zero commands alone are never evidence of completion.
    if (const Status status = requestStatus(); status != Status::RUNNING) return finish(status);
    if (const Status status = controller.status(); status != Status::RUNNING) {
      record(PathStage::TRANSLATION,current,now,elapsedUs,computeUs,0,
             output.first,output.second,status,false);
      result.translationStatus = status;
      if (!alignHeading || status != Status::SUCCESS) return finish(status);
      const Status stopped = publishPath(generation, 0, 0);
      if (stopped != Status::RUNNING) return finish(stopped);
      turning = true;
      pros::delay(10);
      continue;
    }
    const Status published = publishAndRecord(PathStage::TRANSLATION,current,now,
        elapsedUs,computeUs,output.first,output.second);
    if (published != Status::RUNNING) return finish(published);
    pros::delay(10);
  }
}

}  // namespace aon
