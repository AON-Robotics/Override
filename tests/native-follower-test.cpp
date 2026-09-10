#include "support/odometry-host.hpp"
#include <cmath>
#include <cassert>
#include <iostream>
#include <vector>
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4458 4267 4018 4101)
#endif
#include "aon/controls/pure-pursuit.hpp"
#ifdef _MSC_VER
#pragma warning(pop)
#endif
#include "../src/aon/paths/static-path.cpp"

aon::PurePursuit controller() {
  return {aon::MotionProfile(200, MAX_ACCEL, MAX_DECEL, MAX_ACCEL),
          aon::MotionProfile(200, MAX_ACCEL*3, MAX_DECEL*0.8, MAX_ACCEL*3),
          6, 2, 2};
}

void simulate(const std::vector<aon::Pose>& path, aon::Pose pose) {
  auto follower = controller();
  follower.setMaximumRpm(200);
  const double initialHeading = pose.theta;
  aon::Odometry odom(1,2,3,4,5);
  odom.resetCurrent(pose.x, pose.y, pose.theta);
  double left = 0, right = 0;
  constexpr double dt = 0.01;
  bool complete = false;
  double previousProgress = 0;
  for (int step = 0; step < 3000; ++step) {
    auto command = follower.follow(path, odom.getPose(), dt);
    assert(follower.valid());
    assert(follower.progress() >= previousProgress);
    previousProgress = follower.progress();
    if (follower.complete()) {
      command = follower.turn(path.back(), odom.getPose(), dt);
      if (command.first == 0 && command.second == 0) { complete = true; break; }
    }
    assert(std::abs(command.first) <= 200.0001);
    assert(std::abs(command.second) <= 200.0001);
    left += std::clamp(command.first-left, -MAX_ACCEL*dt, MAX_ACCEL*dt);
    right += std::clamp(command.second-right, -MAX_ACCEL*dt, MAX_ACCEL*dt);
    const double scale = M_PI * DRIVE_WHEEL_DIAMETER * MOTOR_TO_DRIVE_RATIO / 60;
    const double distance = (left+right)*0.5*scale*dt;
    const double turn = (left-right)*scale*dt/DRIVE_WIDTH;
    const double chord = std::abs(turn)<1e-9 ? distance : distance*2*std::sin(turn/2)/turn;
    const double heading = pose.theta*M_PI/180 + turn/2;
    pose.x += chord*std::cos(heading);
    pose.y += chord*std::sin(heading);
    pose.theta += turn*180/M_PI;
    const double units = 36000 / (M_PI*TRACKING_WHEEL_DIAMETER);
    odom.encoderLeft.position += (distance + turn*DISTANCE_LEFT_TRACKING_WHEEL_CENTER)*units;
    odom.encoderRight.position += (distance - turn*DISTANCE_RIGHT_TRACKING_WHEEL_CENTER)*units;
    odom.gyroscope.heading = std::fmod(pose.theta-initialHeading+720,360);
    odom.update();
  }
  std::cout << "Native route: done=" << complete << " error="
            << pose.distanceTo(path.back()) << " heading=" << pose.theta << std::endl;
  assert(complete);
  assert(pose.distanceTo(odom.getPose()) < 0.1);
  assert(pose.distanceTo(path.back()) < 2.2);
  assert(std::abs(std::remainder(pose.theta-path.back().theta,360)) <= 2.01);
}

int main() {
  auto follower = controller();
  const std::vector<aon::Pose> closeLanes{{0,0,0},{20,0,0},{20,1,0},{0,1,180}};
  follower.follow(closeLanes, {0,0.9,0});
  assert(follower.progress() < 1); // must not jump to the nearby return lane
  follower.follow(closeLanes, {8,0,0});
  const double advanced = follower.progress();
  follower.follow(closeLanes, {4,0,0});
  assert(follower.progress() >= advanced);
  auto empty = controller();
  empty.follow({}, {});
  assert(!empty.valid());
  auto duplicate = controller();
  duplicate.follow({{0,0,0},{0,0,0}}, {});
  assert(!duplicate.valid());
  simulate({{0,0,0},{12,0,0},{24,0,0}}, {});
  simulate({{0,0,0},{10,0,0},{18,2,0},{23,7,0},{25,15,90}}, {});
  simulate(aon::generated::staticPathAt({}), {});
  simulate(aon::generated::staticPathAt({40,-20,137}), {40,-20,137});
  std::cout << "Native AON follower tests passed\n";
}
