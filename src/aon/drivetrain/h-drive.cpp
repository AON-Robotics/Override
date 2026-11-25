#include "../../../include/aon/drivetrain/h-drive.hpp"

namespace aon {

bool HDrive::stopPathMotors() {
  const auto l = leftMotors.moveVelocityDirect(0);
  const auto r = rightMotors.moveVelocityDirect(0);
  const auto m = midMotors.moveVelocityDirect(0);
  return l == 1 && r == 1 && m == 1;
}


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
  Drivetrain::stop();
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
  Drivetrain::follow(path); // safely rejects unvalidated holonomic kinematics
}

}  // namespace aon
