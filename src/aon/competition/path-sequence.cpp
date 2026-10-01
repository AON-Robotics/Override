#include "aon/competition/path-sequence.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

namespace aon {
namespace {
using Result = Drivetrain::FollowResult;
Result interrupted(std::uint32_t started, std::uint32_t timeout) {
  if (pros::competition::is_disabled()) return Result::Disabled;
  if (pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_DIGITAL_B))
    return Result::Cancelled;
  return pros::millis()-started >= timeout ? Result::TimedOut : Result::Completed;
}
struct Events {
  const PathStep& step;
  std::size_t next = 0;
  static bool update(void* context, double progress) {
    auto& self = *static_cast<Events*>(context);
    if (self.step.hooks.update && !self.step.hooks.update(self.step.hooks.context,progress)) return false;
    while (self.next < self.step.eventCount && self.step.events[self.next].distance <= progress) {
      const auto& event = self.step.events[self.next++];
      event.fire(event.context);
    }
    return true;
  }
  static void sample(void* context, const FollowSample& value) {
    const auto& step = static_cast<Events*>(context)->step;
    if (step.hooks.sample) step.hooks.sample(step.hooks.context,value);
  }
};
}

const char* followResultName(Drivetrain::FollowResult result) {
  switch (result) {
    case Result::Completed: return "Completed";
    case Result::InvalidPath: return "Invalid path";
    case Result::InvalidOptions: return "Invalid options";
    case Result::TimedOut: return "Timed out";
    case Result::Disabled: return "Disabled";
    case Result::Cancelled: return "Cancelled";
  }
  return "Unknown";
}

Drivetrain::FollowResult runPathSequence(Drivetrain& drive, const PathStep* steps,
                                        std::size_t count, std::uint32_t timeoutMs) {
  const auto started = pros::millis();
  const auto finish = [&](Result result) { drive.stop(); return result; };
  if (!steps || !count || !timeoutMs) return finish(Result::InvalidOptions);
  // Validate every leg and event before moving or firing a mechanism.
  for (std::size_t i=0; i<count; ++i) {
    const auto& step = steps[i];
    if (!validFollowOptions(step.options)) return finish(Result::InvalidOptions);
    if (!step.path.data() || step.path.size() < 2) return finish(Result::InvalidPath);
    double length = 0;
    for (std::size_t j=0; j<step.path.size(); ++j) {
      const auto& point = step.path[j];
      if (step.path.speeds && (step.path.speeds[j] > 127 ||
          (step.path.speeds[j] == 0 && j+1 < step.path.size()))) return finish(Result::InvalidPath);
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.theta))
        return finish(Result::InvalidPath);
      if (j) {
        const double distance = step.path[j-1].distanceTo(point);
        if (!std::isfinite(distance) || distance <= 1e-9) return finish(Result::InvalidPath);
        length += distance;
      }
    }
    if ((step.eventCount && !step.events) || (step.ready && !step.waitMs))
      return finish(Result::InvalidOptions);
    for (std::size_t j=0; j<step.eventCount; ++j) {
      const auto& event = step.events[j];
      if (!event.fire || !std::isfinite(event.distance) || event.distance < 0 || event.distance > length ||
          (j && event.distance < step.events[j-1].distance)) return finish(Result::InvalidOptions);
    }
  }
  for (std::size_t i=0; i<count; ++i) {
    const auto& step = steps[i];
    auto result = interrupted(started,timeoutMs);
    if (result != Result::Completed) return finish(result);
    auto options = step.options;
    const auto elapsed = pros::millis()-started;
    if (elapsed >= timeoutMs) return finish(Result::TimedOut);
    options.timeoutMs = std::min(options.timeoutMs,timeoutMs-elapsed);
    Events events{step};
    result = drive.follow(step.path,options,{&events,Events::sample,Events::update});
    if (result != Result::Completed) return finish(result);
    result = interrupted(started,timeoutMs);
    if (result != Result::Completed) return finish(result);
    // Completion tolerances can stop progress just short of endpoint markers.
    while (events.next < step.eventCount) {
      const auto& event = step.events[events.next++];
      event.fire(event.context);
    }
    if (step.arrived) step.arrived(step.context);
    const auto waiting = pros::millis();
    const auto endWait = [&](Result value) {
      if (step.afterWait) step.afterWait(step.context);
      return finish(value);
    };
    while (true) {
      result = interrupted(started,timeoutMs);
      if (result != Result::Completed) return endWait(result);
      if (step.ready && step.ready(step.context)) break;
      if (pros::millis()-waiting >= step.waitMs) {
        if (step.ready) return endWait(Result::TimedOut);
        break;
      }
      drive.stop();
      pros::delay(10);
    }
    if (step.afterWait) step.afterWait(step.context);
  }
  return finish(Result::Completed);
}
} // namespace aon
