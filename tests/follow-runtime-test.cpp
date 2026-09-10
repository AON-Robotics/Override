// Exercise the production execution loop with deterministic hardware/time.
#define _USE_MATH_DEFINES
#define AON_DRIVETRAIN_HPP_
#include <cassert>
#include <cstdint>
#include <memory>
#include <functional>
#include <vector>
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4458)
#endif
#include "aon/controls/pure-pursuit.hpp"
#ifdef _MSC_VER
#pragma warning(pop)
#endif
namespace pros {
inline std::uint32_t timeMs = 0, cancelAt = UINT32_MAX, disableAt = UINT32_MAX;
inline std::function<void(unsigned)> advance;
inline std::uint32_t millis() { return timeMs; }
inline void delay(unsigned ms) { if (advance) advance(ms); timeMs += ms; }
namespace competition { inline bool is_disabled() { return timeMs >= disableAt; } }
enum { E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_B, E_TEXT_MEDIUM_CENTER };
namespace c { inline bool controller_get_digital(int, int) { return timeMs >= cancelAt; } }
namespace screen { template<typename... Args> void print(int, int, const char*, Args...) {} }
}
// Prevent the hardware-only headers; all numerical/controller code remains real.
#define _PROS_MISC_H_
#define _PROS_RTOS_HPP_
#define _PROS_SCREEN_HPP_
namespace aon {
class Drivetrain {
public:
  enum class FollowResult { Completed, InvalidPath, InvalidOptions, TimedOut, Disabled, Cancelled };
  struct Odometry { Pose pose; Pose getPose() { return pose; } };
  std::unique_ptr<Odometry> odometry = std::make_unique<Odometry>();
  std::unique_ptr<MotionProfile> yProfile = std::make_unique<MotionProfile>(200,2500,200,2500);
  std::unique_ptr<MotionProfile> thetaProfile = std::make_unique<MotionProfile>(200,7500,160,7500);
  int opposedCommands = 0, driveCommands = 0, stops = 0;
  double targetLeft = 0, targetRight = 0, actualLeft = 0, actualRight = 0;
  void stop() { ++stops; targetLeft = targetRight = 0; }
  void tank(double left, double right) {
    targetLeft = left; targetRight = right;
    if (left*right < 0) ++opposedCommands;
    if (left != 0 || right != 0) ++driveCommands;
  }
  void advance(unsigned ms) {
    const double dt = ms / 1000.0;
    actualLeft += std::clamp(targetLeft-actualLeft, -MAX_ACCEL*dt, MAX_ACCEL*dt);
    actualRight += std::clamp(targetRight-actualRight, -MAX_ACCEL*dt, MAX_ACCEL*dt);
    const double scale = M_PI*DRIVE_WHEEL_DIAMETER*MOTOR_TO_DRIVE_RATIO/60;
    const double delta = (actualLeft-actualRight)*scale*dt/DRIVE_WIDTH;
    const double distance = (actualLeft+actualRight)*scale*dt/2;
    const double chord = std::abs(delta)<1e-9 ? distance : distance*2*std::sin(delta/2)/delta;
    const double heading = odometry->pose.theta*M_PI/180+delta/2;
    odometry->pose.x += chord*std::cos(heading);
    odometry->pose.y += chord*std::sin(heading);
    odometry->pose.theta += delta*180/M_PI;
  }
  FollowResult follow(const std::vector<Pose>&, std::uint32_t, double);
};
}
#include "../src/aon/drivetrain/follow.cpp"
#include "../src/aon/paths/static-path.cpp"

int main() {
  using Result = aon::Drivetrain::FollowResult;
  const std::vector<aon::Pose> path{{0,0,0},{20,0,90}};
  aon::Drivetrain drive;
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
  drive.odometry->pose.x = NAN;
  assert(drive.follow(path, 1000, 200) == Result::InvalidPath);
  assert(drive.stops > 0);
  aon::Drivetrain moving;
  pros::timeMs = 0;
  pros::advance = [&](unsigned ms) { moving.advance(ms); };
  const auto route = aon::paths::staticPathAt({});
  const auto result = moving.follow(route, 30000, 200);
  pros::advance = nullptr;
  std::cout << "Execution simulation: result=" << static_cast<int>(result)
            << " time=" << pros::timeMs << " endpoint="
            << moving.odometry->pose.distanceTo(route.back()) << std::endl;
  assert(result == Result::Completed);
  assert(moving.odometry->pose.distanceTo(route.back()) < 2.2);
  assert(std::abs(std::remainder(moving.odometry->pose.theta-route.back().theta,360)) <= 2.01);
  assert(moving.actualLeft == 0 && moving.actualRight == 0);
  std::cout << "AON execution loop tests passed\n";
}
