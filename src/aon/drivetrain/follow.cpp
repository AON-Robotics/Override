#include "aon/drivetrain/drivetrain.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"

namespace aon {

Drivetrain::FollowResult Drivetrain::follow(const std::vector<Pose>& path,
                                          std::uint32_t timeoutMs, double maximumRpm) {
  FollowOptions options;
  options.timeoutMs = timeoutMs;
  options.maximumRpm = maximumRpm;
  return follow(PathView(path), options);
}

Drivetrain::FollowResult Drivetrain::follow(PathView path, const FollowOptions& options,
                                          FollowHooks hooks) {
  const auto started = pros::millis();
  FollowSample sample;
  bool observed = false;
  const auto finish = [&](FollowResult result) {
    // AON motors slew toward zero; keep requesting it during braking.
    const auto stopping = pros::millis();
    do { stop(); pros::delay(10); } while (pros::millis()-stopping < 300);
    if (observed && hooks.sample) {
      sample.elapsedMs = pros::millis()-started;
      sample.pose = odometry->getPose();
      sample.endpointError = sample.pose.distanceTo(path.back());
      sample.left = sample.right = 0;
      const auto measured = wheelRpm();
      sample.measuredLeft = measured.first;
      sample.measuredRight = measured.second;
      hooks.sample(hooks.context, sample);
    }
    return result;
  };
  if (!path.data() || path.size() < 2) return finish(FollowResult::InvalidPath);
  if (!odometry || !yProfile || !thetaProfile || !validFollowOptions(options))
    return finish(FollowResult::InvalidOptions);
  PurePursuit controller(*yProfile, *thetaProfile, options.lookahead,
                         options.positionTolerance, options.headingTolerance);
  controller.configure(options);
  Pose endpoint = path.back();
  if (!std::isnan(options.finalHeading)) endpoint.theta = options.finalHeading;
  auto previous = started, alignedSince = started, lastSample = started;
  bool aligning = false, settled = false;
  while (true) {
    const auto now = pros::millis();
    if (pros::competition::is_disabled()) return finish(FollowResult::Disabled);
    if (pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_B))
      return finish(FollowResult::Cancelled);
    if (now-started >= options.timeoutMs) return finish(FollowResult::TimedOut);
    const Pose current = odometry->getPose();
    if (!std::isfinite(current.x) || !std::isfinite(current.y) || !std::isfinite(current.theta))
      return finish(FollowResult::InvalidPath);
    const double dt = std::clamp((now-previous)/1000.0, 0.001, 0.05);
    previous = now;
    const double endpointError = current.distanceTo(endpoint);
    if (aligning && endpointError > options.positionTolerance) { aligning = false; settled = false; }
    std::pair<double,double> output;
    if (!aligning) {
      output = controller.follow(path, current, dt);
      if (!controller.valid()) return finish(FollowResult::InvalidPath);
      aligning = controller.complete();
    }
    if (hooks.update && !hooks.update(hooks.context, controller.progress()))
      return finish(FollowResult::Cancelled);
    const auto measured = wheelRpm();
    bool complete = false;
    if (aligning) {
      output = controller.turn(endpoint, current, dt);
      const bool withinTolerance = std::abs(std::remainder(endpoint.theta-current.theta,360.0)) <= options.headingTolerance &&
          std::isfinite(measured.first) && std::isfinite(measured.second) &&
          std::abs(measured.first) <= options.settledRpm && std::abs(measured.second) <= options.settledRpm;
      if (withinTolerance) {
        if (!settled) alignedSince = now;
        settled = true;
        complete = now-alignedSince >= options.settleMs;
      } else settled = false;
    }
    tank(output.first, output.second);
    sample = {now-started, current, aligning ? endpoint : controller.target(),
              controller.progress(), controller.crossTrackError(), endpointError,
              output.first, output.second, measured.first, measured.second, aligning};
    if (!observed || now-lastSample >= 100) {
      if (hooks.sample) hooks.sample(hooks.context, sample);
      pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "%.1fin Pose %.1f %.1f %.1f",
                          sample.progress, current.x, current.y, current.theta);
      pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 5, "L %.0f/%.0f R %.0f/%.0f",
                          output.first, measured.first, output.second, measured.second);
      lastSample = now;
    }
    observed = true;
    if (complete) return finish(FollowResult::Completed);
    pros::delay(10);
  }
}
} // namespace aon
