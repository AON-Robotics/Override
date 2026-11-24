#include "../../../include/aon/drivetrain/x-drive.hpp"

namespace aon {

void XDrive::sideways(const double &rpm, const int& delay) {
  this->frontLeftMotors.move_velocity(rpm);
  this->frontRightMotors.move_velocity(-rpm);
  this->backLeftMotors.move_velocity(-rpm);
  this->backRightMotors.move_velocity(rpm);
  if (delay == 0) return;
  pros::delay(delay);
  this->stop();
}

void XDrive::tank(const double &left, const double &right){
  this->frontLeftMotors.move_velocity(left);
  this->backLeftMotors.move_velocity(left);
  this->frontRightMotors.move_velocity(right);
  this->backRightMotors.move_velocity(right);
}

void XDrive::holonomic(const double &forward, const double &sideways, const double &turn){
  Vector direction = Vector().SetPosition(sideways, forward);
  Vector command = translateToMotorCommand(direction);
  double topRightDiag = command.GetX();
  double topLeftDiag = command.GetY();

  double frontLeft = topRightDiag + turn;
  double frontRight = topLeftDiag - turn;
  double backLeft = topLeftDiag + turn;
  double backRight = topRightDiag - turn;

  // Normalize if anything exceeds MAX_RPM
  double maxVal = std::max({std::abs(frontLeft), std::abs(frontRight), std::abs(backLeft), std::abs(backRight)});
  if(maxVal > MAX_RPM) {
    frontLeft = (frontLeft / maxVal) * MAX_RPM;
    frontRight = (frontRight / maxVal) * MAX_RPM;
    backLeft = (backLeft / maxVal) * MAX_RPM;
    backRight = (backRight / maxVal) * MAX_RPM;
  }

  this->frontLeftMotors.move_velocity(frontLeft);
  this->frontRightMotors.move_velocity(frontRight);
  this->backLeftMotors.move_velocity(backLeft);
  this->backRightMotors.move_velocity(backRight);
}

Vector XDrive::translateToMotorCommand(Vector direction){
  Vector result;
  result.SetX(direction.GetX() * 0.70710678118 + direction.GetY() * 0.70710678118);
  result.SetY(direction.GetX() * -0.70710678118 + direction.GetY() * 0.70710678118);
  return result;
}

void XDrive::setBrakeMode(pros::MotorBrake brakeMode){
  frontLeftMotors.set_brake_mode(brakeMode);
  frontRightMotors.set_brake_mode(brakeMode);
  backLeftMotors.set_brake_mode(brakeMode);
  backRightMotors.set_brake_mode(brakeMode);
}

void XDrive::setGearset(pros::MotorGears gearset){
  frontLeftMotors.set_gearing(gearset);
  frontRightMotors.set_gearing(gearset);
  backLeftMotors.set_gearing(gearset);
  backRightMotors.set_gearing(gearset);
}

void XDrive::setEncoderUnits(pros::MotorEncoderUnits units){
  frontLeftMotors.set_encoder_units(units);
  frontLeftMotors.tare_position();
  frontRightMotors.set_encoder_units(units);
  frontRightMotors.tare_position();
  backLeftMotors.set_encoder_units(units);
  backLeftMotors.tare_position();
  backRightMotors.set_encoder_units(units);
  backRightMotors.tare_position();
}

void XDrive::setSlewRate(double slew){
  frontLeftMotors.SetAcceleration(slew);
  frontRightMotors.SetAcceleration(slew);
  backLeftMotors.SetAcceleration(slew);
  backRightMotors.SetAcceleration(slew);
}

double XDrive::getRPM(){
  double frontLeft = abs(frontLeftMotors.get_actual_velocity());
  double frontRight = abs(frontRightMotors.get_actual_velocity());
  double backLeft = abs(backLeftMotors.get_actual_velocity());
  double backRight = abs(backRightMotors.get_actual_velocity());
  return (frontLeft + frontRight + backLeft + backRight) / 4;
}

// Using Motion Profile
MovementStatus XDrive::goToPose(const Pose& target) {
  return this->goToPoseProfiled(target);
}

}  // namespace aon
