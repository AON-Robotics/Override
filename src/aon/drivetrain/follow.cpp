#include "aon/drivetrain/drivetrain.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"

namespace aon {

Drivetrain::FollowResult Drivetrain::follow(const std::vector<Pose>& path,
                                            std::uint32_t timeoutMs,
                                            double maximumRpm) {
  const auto finish = [this](FollowResult result) {
    // AON motors slew even toward zero; issue zero until the ramp has settled.
    const auto stopping = pros::millis();
    do { this->stop(); pros::delay(10); } while (pros::millis() - stopping < 300);
    return result;
  };
  if (path.size() < 2) return finish(FollowResult::InvalidPath);
  if (!odometry || !yProfile || !thetaProfile || timeoutMs == 0 ||
      !std::isfinite(maximumRpm) || maximumRpm <= 0 || maximumRpm > MAX_RPM) {
    return finish(FollowResult::InvalidOptions);
  }
  PurePursuit controller(*yProfile, *thetaProfile, 6, 2, 2);
  controller.setMaximumRpm(maximumRpm);
  const auto started = pros::millis();
  auto previous = started;
  auto alignedSince = started;
  bool aligning = false, headingSettled = false;
  while (true) {
    const auto now = pros::millis();
    if (pros::competition::is_disabled()) return finish(FollowResult::Disabled);
    if (pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER,
                                       pros::E_CONTROLLER_DIGITAL_B)) {
      return finish(FollowResult::Cancelled);
    }
    if (now - started >= timeoutMs) return finish(FollowResult::TimedOut);
    const Pose current = odometry->getPose();
    if (!std::isfinite(current.x) || !std::isfinite(current.y) || !std::isfinite(current.theta)) {
      return finish(FollowResult::InvalidPath);
    }
    const double dt = std::clamp((now - previous) / 1000.0, 0.001, 0.05);
    previous = now;
    // Reacquire position if braking/alignment carries us outside tolerance.
    if (aligning && current.distanceTo(path.back()) > 2.0) {
      aligning = false;
      headingSettled = false;
    }
    std::pair<double, double> output;
    if (!aligning) {
      output = controller.follow(path, current, dt);
      if (!controller.valid()) return finish(FollowResult::InvalidPath);
      aligning = controller.complete();
    }
    if (aligning) {
      output = controller.turn(path.back(), current, dt);
      if (std::abs(std::remainder(path.back().theta-current.theta, 360.0)) <= 2.0) {
        if (!headingSettled) alignedSince = now;
        headingSettled = true;
        if (now - alignedSince >= 150) return finish(FollowResult::Completed);
      } else {
        headingSettled = false;
      }
    }
    tank(output.first, output.second);
    if ((now-started) % 100 < 10) {
      pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "%.1fin  Pose %.1f %.1f %.1f",
                          controller.progress(), current.x, current.y, current.theta);
      pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 5, "L %.0f  R %.0f  %s",
                          output.first, output.second, aligning ? "Heading" : "Path");
    }
    pros::delay(10);
  }
}

}  // namespace aon
