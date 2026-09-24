#include "support/pros-host.hpp"
#include "odometry-test.cpp"
#include "native-follower-test.cpp"
#include "path-trace-test.cpp"
#include "follow-runtime-test.cpp"
#include "static-path-actions-test.cpp"
#include "gui-selection-test.cpp"
#include "path-tuning-test.cpp"
#include "../src/aon/drivetrain/differential-drive.cpp"

void testPoseSetters() {
  pros::reset();
  SimDrive drive({1,2,30});
  assert(drive.getX() == 1 && drive.getY() == 2 && drive.getTheta() == 30);
  drive.odometry->encoderLeft.position = 123;
  drive.odometry->encoderRight.position = 456;
  drive.odometry->gyroscope.heading = 70;
  drive.setPose({3,4,50});
  assert(drive.getX() == 3 && drive.getY() == 4 && drive.getTheta() == 50);
  drive.setX(9);
  drive.setY(-7);
  drive.setTheta(120);
  auto pose = drive.getPose();
  assert(pose.x == 9 && pose.y == -7 && pose.theta == 120);
  // Setters must not reset sensors or wait for IMU calibration.
  assert(pros::millis() == 0);
  assert(drive.odometry->encoderLeft.position == 123);
  assert(drive.odometry->encoderRight.position == 456);
  assert(drive.odometry->gyroscope.heading == 70);
  drive.odometry->SetPosition(20,30);
  assert(drive.getPose().x == 20 && drive.getPose().y == 30);
  drive.resetPose(4,5,60);
  drive.odometry->update();
  assert(drive.getX() == 4 && drive.getY() == 5 && std::abs(drive.getTheta()-60) < 1e-9);
}

void testPoseWithoutOdometry() {
  pros::reset();
  // Actual derived construction, including its default null sensor/profile arguments.
  aon::DifferentialDrive defaultDrive;
  assert(defaultDrive.getX() == 0 && defaultDrive.getY() == 0 && defaultDrive.getTheta() == 0);
  aon::DifferentialDrive drive({1},{2},{3,4,50});
  assert(drive.getPose().x == 3 && drive.getPose().y == 4 && drive.getPose().theta == 50);
  drive.setPose({6,7,80});
  drive.setX(9);
  assert(drive.getX() == 9 && drive.getY() == 7 && drive.getTheta() == 80);
  drive.setY(-5);
  drive.setTheta(120);
  const auto pose = drive.getPose();
  assert(pose.x == 9 && pose.y == -5 && pose.theta == 120);
  drive.resetPose(4,5,60);
  assert(drive.getX() == 4 && drive.getY() == 5 && drive.getTheta() == 60);
  drive.initialize(); // no sensors to initialize; no task loop or calibration delay
  assert(pros::millis() == 0);
  drive.resetPose();
  assert(drive.getX() == 0 && drive.getY() == 0 && drive.getTheta() == 0);
  const std::vector<aon::Pose> path{{0,0,0},{10,0,0}};
  assert(drive.follow(path,aon::FollowOptions{}) == aon::Drivetrain::FollowResult::InvalidOptions);
  assert(pros::millis() == 300); // the existing braking cleanup still runs
}

void testGeneratedStops() {
  // Hand-derived north/east editor fixture, anchored facing native +Y.
  const auto route = aon::generated::staticRouteAt({5,7,90},"__host_corner");
  assert(route.stops.size() == 2 && route.stops[0].index == 2 && route.stops[1].index == 3);
  assert(std::abs(route.stops[0].heading-90) < 1e-8);
  assert(std::abs(route.stops[1].heading-180) < 1e-8);
  assert(route.points[2].distanceTo({5,11,0}) < 1e-8);
  assert(route.points[3].distanceTo({1,11,0}) < 1e-8);
  assert((route.speeds == std::vector<std::uint8_t>{100,80,60,0}));
  const auto leg = route.view().slice(2,3);
  assert(leg.data() == route.points.data()+2 && leg.speeds == route.speeds.data()+2);
  assert(leg.speeds[0] == 60 && leg.speeds[1] == 0);
  assert(route.points.back().theta == 180);
}

int main() {
  testTuning();
  testGeneratedStops();
  testSelection();
  testPoseSetters();
  testPoseWithoutOdometry();
  testOdometry();
  testFollower();
  testTrace();
  pros::reset();
  testRuntime();
  testActions();
}
