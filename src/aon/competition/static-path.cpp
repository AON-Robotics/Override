#include "aon/drivetrain/drivetrain.hpp"
#include "aon/generated/static-path.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <functional>

namespace aon {
namespace {
using Result = Drivetrain::FollowResult;

std::vector<std::vector<Pose>> testingLegs(const Pose& start) {
  const auto path = generated::staticPathAt(start, "testing");
  if (path.size() < 2) return {};
  const Pose editorStops[] = {{-53.304526,2.487347,0}, {-55.223097,12.301576,270}, {-67.370706,0.340853,270}};
  std::vector<std::vector<Pose>> legs;
  std::size_t cursor = 0;
  for (std::size_t stage = 0; stage < 3; ++stage) {
    const auto stop = generated::staticWaypointAt(start, editorStops[stage], "testing");
    std::size_t end = cursor;
    // First approach matters: the last curve passes near the second stop again.
    if (stage == 2) end = path.size()-1;
    else {
      // Prefer an explicit endpoint sample, avoiding a duplicate at the next leg.
      while (end < path.size() && path[end].distanceTo(stop) > 0.005) ++end;
      if (end == path.size()) {
        end = cursor;
        while (end < path.size() && path[end].distanceTo(stop) > 0.8) ++end;
      }
    }
    if (end >= path.size() || !std::isfinite(stop.x) ||
        path[end].distanceTo(stop) > 0.8) return {};
    std::vector<Pose> leg;
    if (!legs.empty()) leg.push_back(legs.back().back());
    leg.insert(leg.end(), path.begin()+cursor, path.begin()+end+1);
    if (leg.empty() || leg.back().distanceTo(stop) > 0.001) leg.push_back(stop);
    else leg.back() = stop;
    if (leg.size() < 2) return {};
    legs.push_back(std::move(leg));
    cursor = end+1;
  }
  return legs;
}

Result interruption(std::uint32_t started) {
  if (pros::competition::is_disabled()) return Result::Disabled;
  if (pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER,
                                     pros::E_CONTROLLER_DIGITAL_B)) return Result::Cancelled;
  if (pros::millis()-started >= 30000) return Result::TimedOut;
  return Result::Completed;
}
}  // namespace

int runStaticPath(Drivetrain& drivetrain, const std::function<void(int)>& intake,
                  const std::function<void()>& piston) {
  // Anchor every leg once; stopping for an action must not shift the next leg.
  const auto legs = testingLegs(drivetrain.getPose());
  pros::screen::set_eraser(pros::Color::black);
  pros::screen::erase();
  pros::screen::set_pen(pros::Color::white);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "JERRYIO TESTING");
  const auto started = pros::millis();
  Result result = legs.size() == 3 ? Result::Completed : Result::InvalidPath;
  intake(0);
  for (std::size_t stage = 0; result == Result::Completed && stage < legs.size(); ++stage) {
    result = interruption(started);
    if (result != Result::Completed) break;
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3, "Leg %u / 3 - 200 RPM",
                        static_cast<unsigned>(stage+1));
    const auto elapsed = pros::millis()-started;
    if (elapsed >= 30000) { result = Result::TimedOut; break; }
    result = drivetrain.follow(legs[stage], 30000-elapsed, 200);
    if (result != Result::Completed) break;
    result = interruption(started);
    if (result != Result::Completed) break;
    const char* action = stage == 0 ? "Intake 2 seconds" :
                         stage == 1 ? "Reverse 2 seconds" : "Arrow activated";
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3, "%-32s", action);
    pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "%s", action);
    if (stage == 2) { piston(); break; }
    intake(stage == 0 ? INTAKE_VELOCITY : -INTAKE_VELOCITY);
    const auto actionStart = pros::millis();
    while (pros::millis()-actionStart < 2000) {
      result = interruption(started);
      if (result != Result::Completed) break;
      drivetrain.stop();
      pros::delay(10);
    }
    intake(0);
  }
  intake(0);
  drivetrain.stop();
  const char* status = "Invalid path";
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
