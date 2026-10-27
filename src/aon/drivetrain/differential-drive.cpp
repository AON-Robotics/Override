#include "../../../include/aon/drivetrain/differential-drive.hpp"

namespace aon {

/// @brief Serialize a manual wheel pair with path publication and terminal stopping.
/// @param left Signed left-side RPM requested by the caller.
/// @param right Signed right-side RPM requested by the caller.
/// @details The lock prevents interleaving a manual pair with a path pair, but
/// does not cancel the path or enforce competition admission. Higher-level code
/// must choose one motion owner; public tank() is not called from locked hooks.
void DifferentialDrive::tank(const double &left, const double &right){
  std::lock_guard<pros::Mutex> lock(pathCommandMutex);
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

/// @brief Delegate a two-point translation to the shared result-producing executor.
/// @param pose Target in AON inches with clockwise-degree final-heading metadata.
/// @pre Configured odometry exists for the initial getPose() read.
/// @note This legacy void signature discards the result. Autonomous code that
/// must stop its sequence on failure should call followPath() and check status.
void DifferentialDrive::goToPose(const Pose& pose) {
  // Compatibility wrapper: use the same validated translation/settlement and
  // cancellation contract as paths. Explicit followPath callers receive errors.
  // Final heading is opt-in at the movement layer, never an unconditional turn
  // after this void API's timeout. No distance-to-uint32 timeout conversion.
  (void)followPath({getPose(), pose});
}

}  // namespace aon
