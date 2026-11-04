#include "../../../include/aon/drivetrain/mecanum.hpp"

namespace aon {

// TODO: verify implementation with a physical model

void MecanumDrive::sideways(const double &rpm, const int& delay) {
  this->frontLeftMotors.move_velocity(rpm);
  this->frontRightMotors.move_velocity(-rpm);
  this->backRightMotors.move_velocity(rpm);
  this->backLeftMotors.move_velocity(-rpm);
  if (delay == 0) return;
  pros::delay(delay);
  this->stop();
}

void MecanumDrive::tank(const double &left, const double &right){
  this->frontLeftMotors.move_velocity(left);
  this->backLeftMotors.move_velocity(left);
  this->frontRightMotors.move_velocity(right);
  this->backRightMotors.move_velocity(right);
}

void MecanumDrive::holonomic(const double &forward, const double &sideways, const double &turn){
  double frontLeft = forward + sideways + turn;
  double frontRight = forward - sideways - turn;
  double backLeft = forward - sideways + turn;
  double backRight = forward + sideways - turn;

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

void MecanumDrive::setBrakeMode(pros::MotorBrake brakeMode){
  frontLeftMotors.set_brake_mode(brakeMode);
  frontRightMotors.set_brake_mode(brakeMode);
  backLeftMotors.set_brake_mode(brakeMode);
  backRightMotors.set_brake_mode(brakeMode);
}

void MecanumDrive::setGearset(pros::MotorGears gearset){
  frontLeftMotors.set_gearing(gearset);
  frontRightMotors.set_gearing(gearset);
  backLeftMotors.set_gearing(gearset);
  backRightMotors.set_gearing(gearset);
}

void MecanumDrive::setEncoderUnits(pros::MotorEncoderUnits units){
  frontLeftMotors.set_encoder_units(units);
  frontLeftMotors.tare_position();
  frontRightMotors.set_encoder_units(units);
  frontRightMotors.tare_position();
  backLeftMotors.set_encoder_units(units);
  backLeftMotors.tare_position();
  backRightMotors.set_encoder_units(units);
  backRightMotors.tare_position();
}

void MecanumDrive::setSlewRate(double slew){
  frontLeftMotors.SetAcceleration(slew);
  frontRightMotors.SetAcceleration(slew);
  backLeftMotors.SetAcceleration(slew);
  backRightMotors.SetAcceleration(slew);
}

double MecanumDrive::getRPM(){
  double frontLeft = abs(frontLeftMotors.get_actual_velocity());
  double frontRight = abs(frontRightMotors.get_actual_velocity());
  double backLeft = abs(backLeftMotors.get_actual_velocity());
  double backRight = abs(backRightMotors.get_actual_velocity());
  return (frontLeft + frontRight + backLeft + backRight) / 4;
}

// Using Motion Profile
MovementStatus MecanumDrive::goToPose(const Pose& target) {
  return this->goToPoseProfiled(target);
}

/*
void MecanumDrive::follow(const std::vector<Pose>& path) {
  if (path.empty() || !hasFreshPose()) { stop(); return; }
  PurePursuit controller = PurePursuit(*this->yProfile, *this->thetaProfile, 5, 2.5, 2.5);

  std::pair<double, double> output = {-1, -1};

  double dt = 0.02;
  double now = pros::micros() / 1E6;
  double lastTime = now;

  // Generous timeout
  const uint32_t timeoutMs = (math::length(path)) * 1E3;
  Timer timer;
  timer.start(timeoutMs);
  while (odometry->getPose().distanceTo(path.back()) > 2.0 && !timer.isCompleted()) {
    if (!hasFreshPose()) { stop(); return; }
    now = pros::micros() / 1E6;
    dt = now - lastTime;
    output = controller.follow(path, this->odometry->getPose(), dt);
    lastTime = now;
    this->tank(output.first, output.second);

    pros::lcd::print(0, "Current: Pose(%.2f, %.2f, %.2f)", odometry->getX(), odometry->getY(), odometry->getDegrees());
    pros::lcd::print(1, "Target: Pose(%.2f, %.2f, %.2f)", path.back().x, path.back().y, path.back().theta);
    pros::lcd::print(2, "Distance: %.2f", odometry->getPose().distanceTo(path.back()));
    pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "Distance: %.2f", odometry->getPose().distanceTo(path.back()));

    if (output.first == 0 && output.second == 0) { break; }

    pros::delay(10);
  }

  if (hasFreshPose()) this->turnToHeading(path.back().theta);

  this->stop();
}
*/

}  // namespace aon
