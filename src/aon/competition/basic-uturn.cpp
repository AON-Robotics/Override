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
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 5, "AON move / arc / move");
  pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "BAS: %-13s", stage);
}

bool shouldStop() {
  return pros::competition::is_disabled() ||
         pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER,
                                          pros::E_CONTROLLER_DIGITAL_B);
}

bool settle(Drivetrain& drivetrain) {
  bool stopped = false;
  // SmartMotorGroup applies slew to zero as well; keep requesting zero so
  // the last nonzero command cannot remain active after this routine returns.
  const auto stopping = pros::millis();
  do {
    drivetrain.stop();
    stopped = shouldStop() || stopped;
    pros::delay(10);
  } while (pros::millis() - stopping < 300);
  return !stopped;
}

bool runSequence(Drivetrain& drivetrain) {
  if (shouldStop()) return false;
  report("Forward");
  drivetrain.move(33);                  // Drive out along the first lane.
  if (!settle(drivetrain)) return false;

  report("Right U-turn");
  drivetrain.driveAngleOfArc(8.5, 180);  // Semicircle: 17-inch lane spacing.
  if (!settle(drivetrain)) return false;

  report("Return");
  drivetrain.move(33);                  // Drive forward along the return lane.
  return settle(drivetrain);
}

}  // namespace

int runBasicUTurn(Drivetrain& drivetrain) {
  const bool finished = runSequence(drivetrain);
  drivetrain.stop();
  // Legacy move/arc methods return void, including when their timeout expires.
  report(finished ? "Sequence ended" : "Stopped");
  pros::screen::print(pros::E_TEXT_SMALL, 30, 190, "Odom: %.1f, %.1f, %.1f",
                      drivetrain.getX(), drivetrain.getY(), drivetrain.getTheta());
  return finished ? 1 : 0;
}

}  // namespace aon
