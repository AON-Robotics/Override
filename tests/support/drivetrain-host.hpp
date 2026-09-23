#pragma once
#include "odometry-host.hpp"
#include "aon/drivetrain/drivetrain.hpp"

// Real Drivetrain API and follow loop, with ideal wheel motion in place of motors.
class SimDrive : public aon::Drivetrain {
public:
  using Drivetrain::odometry;
  SimDrive(aon::Pose start = {}) : Drivetrain(start,
      std::make_unique<aon::Odometry>(1,2,3,4,5), {},
      std::make_unique<aon::MotionProfile>(200,2500,200,2500),
      std::make_unique<aon::MotionProfile>(200,2500,200,2500),
      std::make_unique<aon::MotionProfile>(200,7500,160,7500)) {}
  int opposedCommands = 0, driveCommands = 0, stops = 0;
  double targetLeft = 0, targetRight = 0, actualLeft = 0, actualRight = 0;
  bool stalled = false;
  void stop() override { ++stops; targetLeft = targetRight = 0; }
  void tank(const double& left, const double& right) override {
    targetLeft = left; targetRight = right;
    if (left*right < 0) ++opposedCommands;
    if (left != 0 || right != 0) ++driveCommands;
  }
  void setBrakeMode(pros::MotorBrake) override {}
  void setGearset(pros::MotorGears) override {}
  void setEncoderUnits(pros::MotorEncoderUnits) override {}
  void setSlewRate(double) override {}
  void goToPose(const aon::Pose&) override {}
  double getRPM() override { return (actualLeft+actualRight)/2; }
  std::pair<double,double> wheelRpm() override { return {actualLeft,actualRight}; }
  void advance(unsigned ms) {
    if (stalled) return;
    auto pose = getPose();
    const double dt = ms / 1000.0;
    actualLeft += std::clamp(targetLeft-actualLeft, -MAX_ACCEL*dt, MAX_ACCEL*dt);
    actualRight += std::clamp(targetRight-actualRight, -MAX_ACCEL*dt, MAX_ACCEL*dt);
    const double scale = M_PI*DRIVE_WHEEL_DIAMETER*MOTOR_TO_DRIVE_RATIO/60;
    const double delta = (actualLeft-actualRight)*scale*dt/DRIVE_WIDTH;
    const double distance = (actualLeft+actualRight)*scale*dt/2;
    const double chord = std::abs(delta)<1e-9 ? distance : distance*2*std::sin(delta/2)/delta;
    const double heading = pose.theta*M_PI/180+delta/2;
    pose.x += chord*std::cos(heading);
    pose.y += chord*std::sin(heading);
    pose.theta += delta*180/M_PI;
    odometry->SetPosition(pose.x, pose.y);
    odometry->setDegrees(pose.theta);
  }
};
