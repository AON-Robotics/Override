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
  PurePursuit controller = PurePursuit(*this->yProfile, *this->thetaProfile, 5, 2.5, 2.5);

  std::pair<double, double> output = {-1, -1};

  double dt = 0.02;
  double now = pros::micros() / 1E6;
  double lastTime = now;

  // Generous timeout
  const uint32_t timeoutMs = (this->odometry->getPose().distanceTo(pose)) * 1E3;
  Timer timer;
  timer.start(timeoutMs);
  while (odometry->getPose().distanceTo(pose) > 2.0 && !timer.isCompleted()){
    now = pros::micros() / 1E6;
    dt = now - lastTime;
    output = controller.go(pose, this->odometry->getPose(), dt);
    lastTime = now;
    this->tank(output.first, output.second);

    pros::lcd::print(0, "Current: Pose(%.2f, %.2f, %.2f)", odometry->getX(), odometry->getY(), odometry->getDegrees());
    pros::lcd::print(1, "Target: Pose(%.2f, %.2f, %.2f)", pose.x, pose.y, pose.theta);
    pros::lcd::print(2, "Distance: %.2f", odometry->getPose().distanceTo(pose));
    pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "Distance: %.2f", odometry->getPose().distanceTo(pose));

    if (output.first == 0 && output.second == 0) { break; }

    pros::delay(10);
  }

  this->turnToHeading(pose.theta);

  this->stop();
}

}  // namespace aon
