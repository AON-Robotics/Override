#include "../../../include/aon/jerryio/routines.hpp"

#include "../../../include/aon/constants.hpp"
#include "../../../include/aon/drivetrain/drivetrain.hpp"
#include "../../../include/aon/jerryio/asset.hpp"
#include "../../../include/aon/jerryio/path-jerryio.hpp"
#include "../../../include/aon/tools/logging.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"

#include <string>
#include <vector>

AON_JERRYIO_ASSET(path_jerryio_txt);

namespace aon::jerryio {
namespace {

void clearStatusScreen() {
  pros::screen::set_eraser(pros::Color::black);
  pros::screen::erase();
}

void reportStarted(std::uint32_t timeoutMs) {
  clearStatusScreen();
  pros::screen::set_pen(pros::Color::yellow);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "JERRYIO RUNNING");
  pros::screen::set_pen(pros::Color::white);
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3, "Timeout: %lu ms",
                      static_cast<unsigned long>(timeoutMs));
  pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0,
                            "JIO running");
}

void reportDecodeFailure(std::size_t line) {
  clearStatusScreen();
  pros::screen::set_pen(pros::Color::red);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "JERRYIO FAILED");
  pros::screen::set_pen(pros::Color::white);
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3, "Decode error line %u",
                      static_cast<unsigned>(line));
  pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0,
                            "JIO decode err");
  pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
}

void reportMotionResult(MotionResult result, std::uint32_t elapsedMs,
                        Drivetrain& drivetrain) {
  clearStatusScreen();
  pros::screen::set_pen(result ? pros::Color::green : pros::Color::red);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1,
                      result ? "JERRYIO COMPLETE" : "JERRYIO FAILED");
  pros::screen::set_pen(pros::Color::white);
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3, "%s",
                      motionStatusName(result.status));
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "Time: %lu ms",
                      static_cast<unsigned long>(elapsedMs));
  pros::screen::print(pros::E_TEXT_SMALL, 80, 155,
                      "Pose %.1f, %.1f, %.1f", drivetrain.getX(),
                      drivetrain.getY(), drivetrain.getTheta());
  pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "JIO: %s",
                            motionStatusName(result.status));
  pros::c::controller_rumble(pros::E_CONTROLLER_MASTER,
                             result ? ". ." : "---");
}

}  // namespace

int RunPathJerryIOAuton(Drivetrain& drivetrain) {
  const Pose start{0.0, 0.0, 0.0};
  const auto decoded = PathJerryIO::decode(
      reinterpret_cast<const char*>(path_jerryio_txt.data),
      path_jerryio_txt.size);
  if (!decoded) {
    drivetrain.stop();
    reportDecodeFailure(decoded.line);
    logging::Error("PATH.JERRYIO autonomous failed to decode at line " +
                   std::to_string(decoded.line));
    return 0;
  }

  drivetrain.resetPose(start.x, start.y, start.theta);
  FollowPathOptions options;
  options.lookaheadDistance = 10.0;
  options.timeoutMs = 30000;
  options.maximumRpm = 350.0;
  options.maximumLateralAcceleration = 40.0;
  options.finalHeading = 0.0;

  reportStarted(options.timeoutMs);
  const std::uint32_t startedAt = pros::millis();
  // Bind PathAction callbacks here after adding intentional internal
  // zero-speed markers to the exported path.
  const std::vector<PathAction> actions;
  const MotionResult result =
      drivetrain.followPathWithActions(decoded.path, actions, options);
  const std::uint32_t elapsedMs = pros::millis() - startedAt;
  reportMotionResult(result, elapsedMs, drivetrain);
  if (!result) {
    logging::Warn(std::string("PATH.JERRYIO autonomous ") +
                  motionStatusName(result.status));
    return 0;
  }
  logging::Debug("PATH.JERRYIO autonomous completed");
  return 1;
}

}  // namespace aon::jerryio
