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
#include "aon/generated/static-path.hpp"

aon::PurePursuit controller() {
  return {aon::MotionProfile(200, MAX_ACCEL, MAX_DECEL, MAX_ACCEL),
          aon::MotionProfile(200, MAX_ACCEL*3, MAX_DECEL*0.8, MAX_ACCEL*3),
          6, 2, 2};
}

void simulate(aon::PathView path, aon::Pose pose) {
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

void testFollower() {
  // Catch ignored exported speed caps, wrong steering sign, and fixed lookahead.
  const std::vector<aon::Pose> straight{{0,0,0},{40,0,0},{80,0,0}};
  const std::uint8_t slow[] = {32,32,0};
  auto limited = controller();
  limited.setMaximumRpm(200);
  std::pair<double,double> command;
  for (int i=0; i<200; ++i) command = limited.follow(aon::PathView{straight.data(),3,slow}, {});
  assert(command.first > 0 && command.first <= 200.0*32/127+0.01);
  assert(std::abs(command.first-command.second) < 1e-9);
  // A sliced leg may end at a positive export cap; retain that approach speed.
  const std::uint8_t approach[] = {127,32,8};
  auto approaching = controller();
  approaching.setMaximumRpm(200);
  for (int i=0; i<100; ++i)
    command = approaching.follow(aon::PathView{straight.data(),3,approach},{76,0,0});
  assert(command.first <= 200.0*(8+24*0.1)/127+0.01);
  const std::vector<aon::Pose> speedDrop{{0,0,0},{2,0,0},{100,0,0}};
  const std::uint8_t drop[] = {127,16,0};
  auto preview = controller();
  preview.setMaximumRpm(200);
  for (int i=0; i<100; ++i) command = preview.follow({speedDrop.data(),3,drop},{});
  assert(command.first < 100); // braking starts before reaching the low-speed sample
  auto steering = controller();
  command = steering.follow(straight, {0,-2,0});
  assert(command.first > command.second); // native positive lateral = clockwise
  aon::FollowOptions tuning;
  tuning.lookahead = 3;
  tuning.lookaheadAtSpeed = 10;
  tuning.maximumRpm = 200;
  auto adaptive = controller();
  adaptive.configure(tuning);
  adaptive.follow(straight, {});
  const double initialTarget = adaptive.target().x;
  for (int i=0; i<200; ++i) adaptive.follow(straight, {});
  assert(initialTarget < 3.1 && adaptive.target().x > 9);
  auto invalidSpeed = controller();
  const std::uint8_t stopped[] = {127,0,0};
  invalidSpeed.follow(aon::PathView{straight.data(),3,stopped}, {});
  assert(!invalidSpeed.valid());
  auto configured = controller();
  tuning.positionTolerance = 0.25;
  configured.configure(tuning);
  configured.follow(straight, {79,0,0});
  assert(!configured.complete());
  const std::vector<aon::Pose> bend{{0,0,0},{5,1,0},{8,4,0},{10,10,90}};
  auto gentle = controller();
  tuning.lateralAcceleration = 0.5;
  gentle.configure(tuning);
  auto unrestricted = controller();
  tuning.lateralAcceleration = 0;
  unrestricted.configure(tuning);
  std::pair<double,double> faster;
  for (int i=0; i<100; ++i) {
    command = gentle.follow(bend,{});
    faster = unrestricted.follow(bend,{});
  }
  assert(command.first+command.second < faster.first+faster.second);

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
  duplicate.follow(std::vector<aon::Pose>{{0,0,0},{0,0,0}}, {});
  assert(!duplicate.valid());
  simulate(std::vector<aon::Pose>{{0,0,0},{12,0,0},{24,0,0}}, {});
  simulate(std::vector<aon::Pose>{{0,0,0},{10,0,0},{18,2,0},{23,7,0},{25,15,90}}, {});
  assert(aon::generated::staticRouteAt({}, "missing-route").points.empty());
  simulate(aon::generated::staticRouteAt({}).view(), {});
  simulate(aon::generated::staticRouteAt({40,-20,137}).view(), {40,-20,137});
  std::cout << "Native AON follower tests passed\n";
}
