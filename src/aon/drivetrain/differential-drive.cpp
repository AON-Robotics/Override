#include "../../../include/aon/drivetrain/differential-drive.hpp"

namespace aon {

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
  if (dispatchMotion(MotionOperation::TANK,left,right)) return;
  std::lock_guard<pros::Mutex> lock(pathCommandMutex);
  this->leftMotors.move_velocity(left);
  this->rightMotors.move_velocity(right);
}

void DifferentialDrive::setBrakeMode(pros::MotorBrake brakeMode){
  if (dispatchMotion(MotionOperation::BRAKE,static_cast<int>(brakeMode))) return;
  leftMotors.set_brake_mode(brakeMode);
  rightMotors.set_brake_mode(brakeMode);
}

void DifferentialDrive::setGearset(pros::MotorGears gearset){
  if (dispatchMotion(MotionOperation::GEAR,static_cast<int>(gearset))) return;
  leftMotors.set_gearing(gearset);
  rightMotors.set_gearing(gearset);
}

void DifferentialDrive::setEncoderUnits(pros::MotorEncoderUnits units){
  if (dispatchMotion(MotionOperation::ENCODER,static_cast<int>(units))) return;
  leftMotors.set_encoder_units(units);
  leftMotors.tare_position();
  rightMotors.set_encoder_units(units);
  rightMotors.tare_position();
  if (odometry) odometry->resetMotorBaselines();
}

void DifferentialDrive::setSlewRate(double slew){
  if (dispatchMotion(MotionOperation::SLEW,slew)) return;
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
  if (motionServiceRequired && !motionService) {
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
