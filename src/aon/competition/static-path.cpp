#include "aon/drivetrain/drivetrain.hpp"
#include "aon/generated/static-path.hpp"
#include "pros/misc.h"
#include "pros/screen.hpp"

namespace aon {

int runStaticPath(Drivetrain& drivetrain) {
  // Anchor to the current pose without resetting/taring the running odometry.
  const auto path = generated::staticPathAt(drivetrain.getPose());
  pros::screen::set_eraser(pros::Color::black);
  pros::screen::erase();
  pros::screen::set_pen(pros::Color::white);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "AON STATIC PATH");
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3, "Running - 200 RPM - B stops");
  pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "AON path running");
  const auto result = drivetrain.follow(path, 30000, 200);
  const char* status = "Invalid path";
  using Result = Drivetrain::FollowResult;
  switch (result) {
    case Result::Completed: status = "Completed"; break;
    case Result::InvalidOptions: status = "Invalid options"; break;
    case Result::TimedOut: status = "Timed out"; break;
    case Result::Disabled: status = "Disabled"; break;
    case Result::Cancelled: status = "Cancelled"; break;
    case Result::InvalidPath: break;
  }
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3, "%-32s", status);
  pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "AON: %-15s", status);
  return result == Result::Completed ? 1 : 0;
}

}  // namespace aon
