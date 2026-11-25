#include "../../../include/aon/drivetrain/x-drive.hpp"

namespace aon {

bool XDrive::stopPathMotors() {
  const auto fl = frontLeftMotors.moveVelocityDirect(0);
  const auto fr = frontRightMotors.moveVelocityDirect(0);
  const auto bl = backLeftMotors.moveVelocityDirect(0);
  const auto br = backRightMotors.moveVelocityDirect(0);
  return fl == 1 && fr == 1 && bl == 1 && br == 1;
}


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
void XDrive::goToPose(const Pose& target){
  if (!hasFreshPose()) { stop(); return; }
  const double delay = 20; // ms
  Timer timer;
  timer.start(std::max<uint32_t>(3000, static_cast<uint32_t>(getPose().distanceTo(target) * 1000)));

  double remainingX = abs(target.x - this->getX());
  double remainingY = abs(target.y - this->getY());
  double remainingTheta = abs(target.theta - this->getTheta());

  const double drive_width = 10.5;
  const double drive_length = 8.25;
  const double ROBOT_RADIUS = hypot(drive_width, drive_length) / 2;
  const double circumference = M_TWOPI * ROBOT_RADIUS;

  while((remainingX > 0.05 || remainingY > 0.05 || remainingTheta > 0.05) &&
        !timer.isCompleted()){
    if (!hasFreshPose()) { stop(); return; }

    pros::lcd::print(0, "(x, y, theta): (%.2f, %.2f, %.2f)", this->getX(), this->getY(), this->getTheta());
    remainingX = target.x - this->getX();
    remainingY = target.y - this->getY();
    remainingTheta = target.theta - this->getTheta();

    double xSign = (remainingX > 0) - (remainingX < 0);
    double ySign = (remainingY > 0) - (remainingY < 0);
    double thetaSign = (remainingTheta > 0) - (remainingTheta < 0);

    remainingX = abs(remainingX);
    remainingY = abs(remainingY);
    remainingTheta = abs(remainingTheta);

    double x = this->xProfile->update(remainingX) * xSign;
    double y = this->yProfile->update(remainingY) * ySign;
    double theta = this->thetaProfile->update(circumference * (remainingTheta / 360.0)) * thetaSign;

    Vector direction = Vector().SetPosition(x, y);
    direction.SetDegrees(direction.GetDegrees() + this->getTheta());

    this->holonomic(direction.GetY(), direction.GetX(), theta);

    pros::delay(delay);
  }

  pros::lcd::clear();
  this->stop();
}

void XDrive::follow(const std::vector<Pose>& path) {
  Drivetrain::follow(path); // safely rejects unvalidated holonomic kinematics
}

}  // namespace aon
