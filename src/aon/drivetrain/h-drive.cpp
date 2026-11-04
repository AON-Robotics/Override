#include "../../../include/aon/drivetrain/h-drive.hpp"

namespace aon {

void HDrive::sideways(const double &rpm, const int& delay) {
  this->midMotors.move_velocity(rpm);
  if (delay == 0) return;
  pros::delay(delay);
  this->stop();
}

void HDrive::tank(const double &left, const double &right){
  this->leftMotors.move_velocity(left);
  this->rightMotors.move_velocity(right);
}

void HDrive::setBrakeMode(pros::MotorBrake brakeMode){
  leftMotors.set_brake_mode(brakeMode);
  rightMotors.set_brake_mode(brakeMode);
  midMotors.set_brake_mode(brakeMode);
}

void HDrive::setGearset(pros::MotorGears gearset){
  leftMotors.set_gearing(gearset);
  rightMotors.set_gearing(gearset);
  midMotors.set_gearing(gearset);
}

void HDrive::setEncoderUnits(pros::MotorEncoderUnits units){
  leftMotors.set_encoder_units(units);
  leftMotors.tare_position();
  rightMotors.set_encoder_units(units);
  rightMotors.tare_position();
  midMotors.set_encoder_units(units);
  midMotors.tare_position();
}

void HDrive::setSlewRate(double slew){
  leftMotors.SetAcceleration(slew);
  rightMotors.SetAcceleration(slew);
  midMotors.SetAcceleration(slew);
}

double HDrive::getRPM(){
  double left = leftMotors.get_actual_velocity();
  double right = rightMotors.get_actual_velocity();
  return (left + right) / 2;
}

void HDrive::stop(){
  this->motors(0);
  this->sideways(0);
}

// Using Motion Profile
void HDrive::goToPose(const Pose& target){
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

void HDrive::follow(const std::vector<Pose>& path) {
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

}  // namespace aon
