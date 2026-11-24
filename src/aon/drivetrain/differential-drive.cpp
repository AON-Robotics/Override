#include "../../../include/aon/drivetrain/differential-drive.hpp"

namespace aon {

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

MovementStatus DifferentialDrive::goToPose(const Pose& pose) {
  return this->follow(std::vector<Pose>{pose});
}

}  // namespace aon
