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

/// @brief Reuse generation revocation and the all-three-group stop hook.
/// @details The base lifecycle coordinates with in-flight path publication and
/// dispatches stopPathMotors() virtually, so the strafe group is not omitted.
/// Physical rest is not established by merely attempting these zero requests.
void HDrive::stop(){
  Drivetrain::stop();
}

// Using PID
// void HDrive::goToPose(const Pose& target){
//   const double delay = 20; // ms
//   const Pose initialPose = this->getPose();
//   Pose currPose = this->getPose();
//   PID xPID = PID(50, 10, 0, delay / 1000, 2, 50);
//   PID yPID = PID(50, 10, 0, delay / 1000, 2, 50);
//   PID thetaPID = PID(2.8, 1.25, 0, delay / 1000, 5, 50);

//   // while(!(withinError(this->getPose().x, target.x, 10) && withinError(this->getPose().y, target.y, 10) && withinError(this->getPose().theta, target.theta, 10))){
//   while(abs(this->getPose().x - target.x) > 0.5 || abs(this->getPose().y - target.y) > 0.5 || abs(this->getPose().theta - target.theta) > 2.5){
//     pros::lcd::print(1, "X: %.2f / %.2f", this->getPose().x, target.x);
//     pros::lcd::print(2, "Y: %.2f / %.2f", this->getPose().y, target.y);
//     pros::lcd::print(3, "Theta: %.2f / %.2f", this->getPose().theta, target.theta);
//     double x = xPID.Output(target.x, currPose.x);
//     double y = yPID.Output(target.y, currPose.y);
//     double theta = thetaPID.Output(target.theta, currPose.theta);

//     Vector direction = Vector().SetPosition(x, y);
//     direction.SetDegrees(direction.GetDegrees() + currPose.theta);// - initialPose.theta);
//     Vector command = translateToMotorCommand(direction);
//     double topRightDiag = command.GetX();
//     double topLeftDiag = command.GetY();
//     double turn = theta;

//     this->leftMotors.move_velocity(topRightDiag + turn);
//     this->rightMotors.move_velocity(topLeftDiag - turn);
//     this->midMotors.move_velocity(topLeftDiag + turn);
//     this->backRightMotors.move_velocity(topRightDiag - turn);
//     pros::delay(delay);
//     currPose.x += getSpeed(x) * delay / 1000; //# in case of odom failure
//     currPose.y += getSpeed(y) * delay / 1000; //# in case of odom failure
//     currPose.theta += rotationSpeed(theta) * delay / 1000; //# in case of odom failure
//     this->setPose(currPose);
//   }

//   pros::lcd::clear();
//   this->stop();
// }

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

  // TODO: add timeouts for safety
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
    direction.SetDegrees(direction.GetDegrees() + this->getTheta());// - initialPose.theta);

    this->holonomic(direction.GetY(), direction.GetX(), theta);

    pros::delay(delay);

    // this->setX(this->getX() + math::linearSpeed(x) * delay / 1000); //# in case of odom failure
    // this->setY(this->getY() + math::linearSpeed(y) * delay / 1000); //# in case of odom failure
    // this->setTheta(this->getTheta() + math::rotationalSpeed(theta) * delay / 1000); //# in case of odom failure
  }

  pros::lcd::clear();
  this->stop();
}

/// @brief Delegate unsupported H-drive paths to the single shared rejection owner.
/// @param path Ordered AON-inch waypoints; no holonomic path tracking is attempted.
/// @details The base executor applies overlap/admission precedence and attempts
/// all-group stopping for an admitted rejection. It does not perform heading
/// alignment. Keeping one owner avoids divergent controller/cleanup copies.
void HDrive::follow(const std::vector<Pose>& path) {
  Drivetrain::follow(path); // safely rejects unvalidated holonomic kinematics
}

}  // namespace aon
