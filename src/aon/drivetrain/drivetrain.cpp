#include "../../../include/aon/drivetrain/drivetrain.hpp"
#include <array>
#include <new>
#include <type_traits>

namespace aon {

/**
 * @brief Durable owners for callback requests and application motor locks.
 * @details PROS deletes competition callback tasks without unwinding C++ scopes.
 * Submitters therefore transfer only trivial data into service-owned mailboxes,
 * then wait without owning a mutex. Only these two permanent tasks enter the
 * original synchronous executor or concrete motor transport. The executor owns
 * its vectors/configuration/capture; it never retains a callback stack pointer.
 *
 * A mailbox token combines phase, callback ownership and a 28-bit callback epoch.
 * A new framework callback proves the preceding callback was deleted/returned;
 * it permits reclaiming abandoned callback slots, but never an executing slot.
 * Other tasks must survive their requests and acknowledge their own slots.
 * Fewer than 2^28 callback transitions may occur during one outstanding request.
 * Startup allocates fixed transfer storage (4096 poses/caps) and two PROS stacks.
 * @todo Measure target heap/stack margin, manual handoff and loaded I/O latency.
 */
class Drivetrain::PersistentMotionService final : public Drivetrain::MotionService {
  using Status = PurePursuit::Status;
  static_assert(std::atomic<uint32_t>::is_always_lock_free &&
                std::atomic<pros::task_t>::is_always_lock_free,
                "Callback mailboxes require lock-free epoch/task atomics");
  static_assert(std::is_trivially_copyable<Pose>::value &&
                std::is_trivially_copyable<HeadingCorrection>::value,
                "Deletable submitters may transfer only trivial request data");
  enum Phase : uint32_t { FREE, WRITING, READY, EXECUTING, DONE };
  static constexpr uint32_t phaseMask = 7, callbackBit = 8, epochMask = 0x0fffffff;
  static constexpr size_t maximumPoints = 4096, commandSlots = 8;
  Drivetrain& drive;
  std::atomic<bool> ready{false};
  std::atomic<uint32_t> callbackEpoch{0};
  std::atomic<pros::task_t> callbackTask{nullptr};
  pros::task_t publisher = nullptr, executor = nullptr;

  struct Command {
    std::atomic<uint32_t> token{FREE};
    MotionOperation operation = MotionOperation::STOP;
    double a = 0, b = 0, c = 0;
    Pose pose;
  };
  std::array<Command,commandSlots> commands;
  struct Request {
    std::atomic<uint32_t> token{FREE};
    // Only trivially destructible data is written by a deletable submitter.
    // An interrupted copy can be abandoned without repairing a vector/allocator.
    std::array<Pose,maximumPoints> points;
    std::array<double,maximumPoints> caps;
    size_t count = 0, capCount = 0, captureCapacity = 0;
    uint32_t timeoutMs = 0, submittedAt = 0, generation = 0;
    bool reversed = false, hasHeading = false, hasTrace = false, invalidTrace = false;
    HeadingCorrection heading{PID(0,0,0,.01,0,0),0,0,0,0};
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
  void publishCommands() {
    uint32_t observedEpoch = epoch();
    const auto observeEpoch = [&] {
      while (observedEpoch!=epoch()) {
        observedEpoch=epoch();
        drive.setPathEnabled(false);
      }
    };
    while (true) {
      if (!ready.load()) { wait(); continue; }
      observeEpoch();
      for (auto& command : commands) {
        reclaim(command.token);
        uint32_t token=command.token.load();
        if ((token&phaseMask)!=READY) continue;
        if (!command.token.compare_exchange_strong(token,(token&~phaseMask)|EXECUTING)) continue;
        // A callback can enter while this scan is already in progress. Complete
        // its revocation barrier before acknowledging any command of that epoch,
        // otherwise a next-pass revocation could undo its acknowledged ENABLE.
        observeEpoch();
        if (!stale(token)) {
          switch(command.operation) {
            case MotionOperation::TANK: drive.tank(command.a,command.b); break;
            case MotionOperation::SIDEWAYS: drive.sideways(command.a); break;
            case MotionOperation::HOLONOMIC: drive.holonomic(command.a,command.b,command.c); break;
            case MotionOperation::BRAKE: drive.setBrakeMode(static_cast<pros::MotorBrake>(int(command.a))); break;
            case MotionOperation::GEAR: drive.setGearset(static_cast<pros::MotorGears>(int(command.a))); break;
            case MotionOperation::ENCODER: drive.setEncoderUnits(static_cast<pros::MotorEncoderUnits>(int(command.a))); break;
            case MotionOperation::SLEW: drive.setSlewRate(command.a); break;
            case MotionOperation::STOP: drive.stop(); break;
            case MotionOperation::CANCEL: drive.cancelPath(); break;
            case MotionOperation::ENABLE: drive.setPathEnabled(command.a!=0); break;
            case MotionOperation::POSE: command.pose=drive.getPose(); break;
            case MotionOperation::RESET_POSE: drive.setPose(Pose(command.a,command.b,command.c)); break;
            case MotionOperation::SET_X: drive.setX(command.a); break;
            case MotionOperation::SET_Y: drive.setY(command.a); break;
            case MotionOperation::SET_HEADING: drive.setTheta(command.a); break;
          }
        }
        command.token.store((token&~phaseMask)|DONE);
      }
      reclaim(request.token);
      wait();
    }
  }
  void executeRequests() {
    while (true) {
      if (!ready.load()) { wait(); continue; }
      uint32_t token=request.token.load();
      if ((token&phaseMask)!=READY ||
          !request.token.compare_exchange_strong(token,(token&~phaseMask)|EXECUTING)) {
        wait(); continue;
      }
      PathResult result;
      request.capture=PathTrace{};
      if (stale(token)) {
        result.status=result.translationStatus=Status::CANCELLED;
      } else {
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
          if (stale(token)) {
            result.status=result.translationStatus=Status::CANCELLED;
          } else {
            result=drive.executePath(path,budget,caps,request.reversed,
                request.hasHeading ? &request.heading : nullptr,
                request.hasTrace ? &request.capture : nullptr,request.generation);
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
  explicit PersistentMotionService(Drivetrain& owner):drive(owner) {}
  bool start() {
    callbackTask.store(pros::c::task_get_current());
    publisher=pros::c::task_create([](void* self) {
      static_cast<PersistentMotionService*>(self)->publishCommands();
    },this,TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT,"AON motor publication");
    executor=pros::c::task_create([](void* self) {
      static_cast<PersistentMotionService*>(self)->executeRequests();
    },this,TASK_PRIORITY_DEFAULT,TASK_STACK_DEPTH_DEFAULT,"AON path execution");
    ready.store(publisher && executor);
    return ready.load();
  }
  bool ownsTransport() const override {
    const auto current=pros::c::task_get_current();
    return ready.load() && (current==publisher || current==executor);
  }
  bool ownsExecution() const override {
    return ready.load() && pros::c::task_get_current()==executor;
  }
  Pose transfer(MotionOperation operation,double a,double b,double c) {
    if (!ready.load()) return Pose(NAN,NAN,NAN); // failed startup never publishes from a callback
    const uint32_t identity=stamp();
    Command* selected=nullptr;
    while (!selected) {
      if ((identity>>4)!=epoch()) return Pose(NAN,NAN,NAN);
      for (auto& command : commands) {
        uint32_t free=FREE;
        if (command.token.compare_exchange_strong(free,identity|WRITING)) {
          selected=&command; break;
        }
      }
      if (!selected) wait();
    }
    selected->operation=operation; selected->a=a; selected->b=b; selected->c=c;
    selected->pose=Pose(NAN,NAN,NAN);
    selected->token.store(identity|READY);
    while (selected->token.load()!=(identity|DONE)) wait();
    const Pose pose=selected->pose;
    selected->token.store(FREE);
    return pose;
  }
  void command(MotionOperation operation,double a,double b,double c) override {
    (void)transfer(operation,a,b,c);
  }
  Pose readPose() override { return transfer(MotionOperation::POSE,0,0,0); }
  PathResult follow(const std::vector<Pose>& path,uint32_t timeoutMs,
      const std::vector<double>& caps,bool reversed,
      const HeadingCorrection* heading,PathTrace* trace) override {
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
    request.reversed=reversed; request.hasHeading=heading!=nullptr;
    if (heading) request.heading=*heading;
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
  void beginMode() override {
    // Only actual framework callback entry may prove its predecessor gone.
    callbackTask.store(pros::c::task_get_current());
    callbackEpoch.fetch_add(1);
    drive.pathEnabled.store(false);
    drive.pathCancelGeneration.fetch_add(1);
    if (!ready.load()) return;
    command(MotionOperation::STOP,0,0,0);
    // Do not enable a new motion owner before old execution releases its locks
    // and storage. DONE is safe: only its caller/result copy remains.
    while (true) {
      const auto phase=request.token.load()&phaseMask;
      if (phase==FREE || phase==DONE) break;
      wait();
    }
  }
};

bool Drivetrain::initializeMotionService() {
  if (motionServiceRequired) return motionServiceStarted;
  motionServiceRequired=true;
  auto* service=new(std::nothrow) PersistentMotionService(*this);
  if (!service) { pathEnabled.store(false); pathStatus.store(PurePursuit::Status::CONTROL_FAILURE); return false; }
  motionService.reset(service);
  const bool started=service->start();
  motionServiceStarted=started;
  if (!started) { pathEnabled.store(false); pathStatus.store(PurePursuit::Status::CONTROL_FAILURE); }
  return started;
}

} // namespace aon
