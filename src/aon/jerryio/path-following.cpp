#include "../../../include/aon/jerryio/path-following.hpp"

#include <cmath>

namespace aon {
namespace {

bool finitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

bool FollowPathOptions::isValid() const {
  const bool validHeading =
      !finalHeading.has_value() ||
      (std::isfinite(*finalHeading) && *finalHeading >= 0.0 &&
       *finalHeading <= 360.0);
  return finitePositive(lookaheadDistance) && finitePositive(trackWidth) &&
         finitePositive(maximumRpm) && finitePositive(maximumAcceleration) &&
         finitePositive(maximumDeceleration) &&
         finitePositive(positionTolerance) && timeoutMs > 0 &&
         loopPeriodMs > 0 && validHeading;
}

MotionStatus evaluateMotionStatus(const MotionLoopSnapshot& snapshot) {
  if (snapshot.disabled) return MotionStatus::Disabled;
  if (snapshot.cancelled) return MotionStatus::Cancelled;
  if (!snapshot.pathValid) return MotionStatus::InvalidPath;
  if (!snapshot.optionsValid) return MotionStatus::InvalidOptions;
  if (snapshot.complete) return MotionStatus::Completed;
  if (snapshot.elapsedMs >= snapshot.timeoutMs) return MotionStatus::TimedOut;
  return MotionStatus::Running;
}

bool shouldAlignFinalHeading(MotionStatus status,
                             const FollowPathOptions& options) {
  return status == MotionStatus::Completed && options.finalHeading.has_value();
}

}  // namespace aon
