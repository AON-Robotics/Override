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
  const auto l = leftMotors.move_velocity(0);
  const auto r = rightMotors.move_velocity(0);
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
  // Compatibility wrapper: use the same validated translation/settlement and
  // cancellation contract as paths. Explicit followPath callers receive errors.
  // Final heading is opt-in at the movement layer, never an unconditional turn
  // after this void API's timeout. No distance-to-uint32 timeout conversion.
  (void)followPath({getPose(), pose});
}

}  // namespace aon
