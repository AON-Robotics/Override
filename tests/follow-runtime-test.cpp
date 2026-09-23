#include "support/drivetrain-host.hpp"
#include "../src/aon/drivetrain/follow.cpp"
#include "../src/aon/competition/path-sequence.cpp"
#include "../src/aon/competition/static-path.cpp"
#include "../src/aon/competition/path-diagnostics.cpp"

void testRuntime() {
  using Result = aon::Drivetrain::FollowResult;
  const std::vector<aon::Pose> path{{0,0,0},{20,0,90}};
  SimDrive drive;
  aon::FollowOptions options;
  options.timeoutMs = 1000;
  options.lookahead = 0;
  assert(drive.follow(path, options) == Result::InvalidOptions);
  options.lookahead = 6;
  options.finalHeading = 0;
  int observations = 0;
  aon::FollowHooks hooks;
  hooks.context = &observations;
  hooks.sample = [](void* ctx, const aon::FollowSample& sample) {
    ++*static_cast<int*>(ctx);
    assert(std::isfinite(sample.target.x) && std::isfinite(sample.measuredLeft));
  };
  hooks.update = [](void*, double) { return pros::millis() < 40; };
  pros::timeMs = 0;
  assert(drive.follow(path, options, hooks) == Result::Cancelled);
  assert(observations > 0);
  hooks.update = nullptr;
  pros::timeMs = 0;
  const std::vector<aon::Pose> near{{0,0,0},{1,0,90}};
  assert(drive.follow(near, options, hooks) == Result::Completed); // explicit final heading
  assert(drive.opposedCommands == 0);
  pros::timeMs = 0;
  drive.actualLeft = drive.actualRight = 50;
  assert(drive.follow(near, options) == Result::TimedOut); // position alone isn't settled
  drive.actualLeft = drive.actualRight = 0;
  assert(drive.follow({}, 100, 200) == Result::InvalidPath);
  assert(drive.follow(path, 0, 200) == Result::InvalidOptions);
  pros::timeMs = 0;
  assert(drive.follow(path, 100, 200) == Result::TimedOut);
  assert(drive.driveCommands > 0);
  assert(drive.opposedCommands == 0); // timeout must not start final alignment
  pros::timeMs = 0;
  pros::cancelAt = 30;
  assert(drive.follow(path, 1000, 200) == Result::Cancelled);
  pros::cancelAt = UINT32_MAX;
  pros::timeMs = 0;
  pros::disableAt = 30;
  assert(drive.follow(path, 1000, 200) == Result::Disabled);
  pros::disableAt = UINT32_MAX;
  pros::timeMs = 0;
  assert(drive.follow({{0,0,0},{1,0,0}}, 1000, 200) == Result::Completed);
  drive.odometry->SetPosition(NAN,0);
  assert(drive.follow(path, 1000, 200) == Result::InvalidPath);
  assert(drive.stops > 0);
  SimDrive moving;
  pros::timeMs = 0;
  pros::advance = [&](unsigned ms) { moving.advance(ms); };
  const auto route = aon::generated::staticRouteAt({});
  const auto result = moving.follow(route.view(), aon::FollowOptions{});
  pros::advance = nullptr;
  std::cout << "Execution simulation: result=" << static_cast<int>(result)
            << " time=" << pros::timeMs << " endpoint="
            << moving.getPose().distanceTo(route.points.back()) << std::endl;
  assert(result == Result::Completed);
  assert(moving.getPose().distanceTo(route.points.back()) < 2.2);
  assert(std::abs(std::remainder(moving.getPose().theta-route.points.back().theta,360)) <= 2.01);
  assert(moving.actualLeft == 0 && moving.actualRight == 0);
  for (int index = 0; index < 3; ++index) {
    pros::reset();
    SimDrive diagnostic({20,-40,137});
    pros::advance = [&](unsigned ms) { diagnostic.advance(ms); };
    // Execute the shipped routine; it owns all diagnostic tuning.
    assert(aon::runPathDiagnostic(diagnostic,index) == 1);
    const char* names[] = {"diagnostic-straight","diagnostic-curve","path"};
    const auto route = aon::generated::staticRouteAt({20,-40,137},names[index]);
    assert(diagnostic.getPose().distanceTo(route.points.back()) <= 2.1);
    assert(std::abs(std::remainder(diagnostic.getTheta()-route.points.back().theta,360)) <= 2.01);
    assert(diagnostic.actualLeft == 0 && diagnostic.actualRight == 0);
    pros::advance = nullptr;
  }
  // Per-run tuning changes the real controller, leaving the drive's profiles reusable.
  auto firstCommand = [](double scale) {
    pros::reset();
    SimDrive drive;
    aon::FollowOptions options;
    options.accelerationScale = scale;
    options.timeoutMs = 50;
    double largest = 0;
    aon::FollowHooks hooks;
    hooks.context = &largest;
    hooks.sample = [](void* ctx, const aon::FollowSample& sample) {
      *static_cast<double*>(ctx) = std::max(*static_cast<double*>(ctx),sample.left);
    };
    const std::vector<aon::Pose> straight{{0,0,0},{100,0,0}};
    assert(drive.follow(straight,options,hooks) == Result::TimedOut);
    return largest;
  };
  assert(firstCommand(0.5) < firstCommand(1));
  for (double invalid : {0.0, -1.0, 3.0, double(NAN), double(INFINITY)}) {
    pros::reset();
    SimDrive drive;
    aon::FollowOptions options;
    options.decelerationScale = invalid;
    assert(drive.follow(path,options) == Result::InvalidOptions);
    assert(drive.driveCommands == 0);
  }
  std::cout << "AON execution loop tests passed\n";
}
