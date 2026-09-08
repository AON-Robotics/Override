#include "aon/competition/basic-uturn.hpp"
#include "aon/drivetrain/drivetrain.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"

namespace aon {
namespace {

void report(const char* stage) {
  pros::screen::set_eraser(pros::Color::black);
  pros::screen::erase();
  pros::screen::set_pen(pros::Color::white);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "BASIC U-TURN");
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3, "%s", stage);
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 5, "Timed drive - B stops");
  pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "BAS: %-13s", stage);
}

bool shouldStop() {
  return pros::competition::is_disabled() ||
         pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER,
                                          pros::E_CONTROLLER_DIGITAL_B);
}

}  // namespace

int runBasicUTurn(Drivetrain& drivetrain) {
  bool aborted = false;
  for (const auto& leg : basicUTurnPlan()) {
    report(leg.name);
    const auto started = pros::millis();
    while (pros::millis() - started < leg.durationMs) {
      if (shouldStop()) {
        aborted = true;
        break;
      }
      drivetrain.tank(leg.leftRpm, leg.rightRpm);
      pros::delay(10);
    }
    if (aborted) break;
  }
  // SmartMotorGroup applies slew to zero as well; keep requesting zero so
  // the last nonzero command cannot remain active after this routine returns.
  const auto stopping = pros::millis();
  do {
    drivetrain.stop();
    pros::delay(10);
  } while (pros::millis() - stopping < 300);
  report(aborted ? "Stopped" : "Timing finished");
  // Pose is observation only: it never determines the motor commands above.
  pros::screen::print(pros::E_TEXT_SMALL, 30, 190, "Odom: %.1f, %.1f, %.1f",
                      drivetrain.getX(), drivetrain.getY(), drivetrain.getTheta());
  return aborted ? 0 : 1;
}

}  // namespace aon
