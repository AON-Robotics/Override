#define AON_DRIVETRAIN_HPP_
#define _PROS_MISC_H_
#define _PROS_RTOS_HPP_
#define _PROS_SCREEN_HPP_
#include <cassert>
#include <cstdint>
#include <functional>
#include <vector>
#include <iostream>
#include "aon/constants.hpp"
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4458)
#endif
#include "aon/math/pose.hpp"
#ifdef _MSC_VER
#pragma warning(pop)
#endif
namespace pros {
inline std::uint32_t timeMs = 0, cancelAt = UINT32_MAX, disableAt = UINT32_MAX;
inline std::uint32_t millis() { return timeMs; }
inline void delay(unsigned ms) { timeMs += ms; }
namespace competition { inline bool is_disabled() { return timeMs >= disableAt; } }
enum { E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_B, E_TEXT_LARGE_CENTER, E_TEXT_MEDIUM_CENTER };
enum class Color { black, white };
namespace c {
inline bool controller_get_digital(int, int) { return timeMs >= cancelAt; }
template<typename... T> void controller_print(int,int,int,const char*,T...) {}
}
namespace screen {
inline void set_eraser(Color) {} inline void set_pen(Color) {} inline void erase() {}
template<typename... T> void print(int,int,const char*,T...) {}
}
}
namespace aon {
class Drivetrain {
public:
  enum class FollowResult { Completed, InvalidPath, InvalidOptions, TimedOut, Disabled, Cancelled };
  Pose pose{40,-20,137};
  std::vector<std::vector<Pose>> legs;
  int failLeg = -1;
  Pose getPose() { return pose; }
  void stop() {}
  FollowResult follow(const std::vector<Pose>& path, std::uint32_t timeout, double rpm) {
    assert(timeout > 0 && timeout <= 30000 && rpm == 200);
    legs.push_back(path); pros::timeMs += 500;
    if (static_cast<int>(legs.size()) == failLeg) return FollowResult::TimedOut;
    pose = path.back(); return FollowResult::Completed;
  }
};
}
#include "../src/aon/competition/static-path.cpp"

int main() {
  for (int scenario = 0; scenario < 4; ++scenario) {
    pros::timeMs = 0; pros::cancelAt = UINT32_MAX; pros::disableAt = UINT32_MAX;
    aon::Drivetrain drive;
    if (scenario == 1) drive.failLeg = 2;
    if (scenario == 2) pros::cancelAt = 1000;
    if (scenario == 3) pros::disableAt = 1000;
    std::vector<std::pair<int,unsigned>> commands;
    int piston = 0;
    const int result = aon::runStaticPath(drive,
        [&](int rpm) { commands.push_back({rpm,pros::millis()}); }, [&]{ ++piston; });
    assert(commands.back().first == 0);
    if (scenario == 0) {
      assert(result == 1 && piston == 1 && drive.legs.size() == 3);
      assert(pros::timeMs == 5500);
      const aon::Pose editor[] = {{-53.304526,2.487347,0}, {-55.223097,12.301576,270}, {-67.370706,0.340853,270}};
      const double lengths[] = {12,10,20};
      for (std::size_t i = 0; i < 3; ++i) {
        double traveled = 0;
        for (std::size_t j = 1; j < drive.legs[i].size(); ++j) {
          const double step = drive.legs[i][j-1].distanceTo(drive.legs[i][j]);
          assert(step > 1e-6);
          traveled += step;
        }
        assert(std::abs(traveled-lengths[i]) < 0.001);
        auto expected = aon::generated::staticWaypointAt({40,-20,137},editor[i],"testing");
        assert(drive.legs[i].back().distanceTo(expected) < 1e-6);
        assert(std::abs(drive.legs[i].back().theta-expected.theta) < 1e-6);
        if (i) assert(drive.legs[i].front().distanceTo(drive.legs[i-1].back()) < 1e-6);
      }
      assert(commands[1].first == INTAKE_VELOCITY && commands[2].second-commands[1].second == 2000);
      assert(commands[3].first == -INTAKE_VELOCITY && commands[4].second-commands[3].second == 2000);
    } else {
      assert(result == 0 && piston == 0);
      assert(drive.legs.size() == (scenario == 1 ? 2u : 1u));
      for (auto command : commands) assert(command.first >= 0);
    }
  }
  std::cout << "Testing route stops, actions, cancellation and timeout passed\n";
}
