#include "../../../include/aon/jerryio/path-following.hpp"

#include <cmath>
#include <algorithm>

namespace aon {
namespace {

bool finitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

double normalizeDegrees(double angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
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
         std::isfinite(terminalRecoveryRpm) && terminalRecoveryRpm >= 0.0 &&
         terminalRecoveryRpm <= maximumRpm &&
         finitePositive(positionTolerance) && projectionWindowSegments > 0 &&
         finitePositive(headingKp) && finitePositive(headingTolerance) &&
         std::isfinite(minimumTurnRpm) && minimumTurnRpm >= 0.0 &&
         minimumTurnRpm <= maximumRpm && timeoutMs > 0 && loopPeriodMs > 0 &&
         validHeading;
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

const char* motionStatusName(MotionStatus status) {
  switch (status) {
    case MotionStatus::Running:
      return "running";
    case MotionStatus::Completed:
      return "completed";
    case MotionStatus::InvalidPath:
      return "invalid path";
    case MotionStatus::InvalidOptions:
      return "invalid options";
    case MotionStatus::TimedOut:
      return "timed out";
    case MotionStatus::Disabled:
      return "disabled";
    case MotionStatus::Cancelled:
      return "cancelled";
  }
  return "unknown";
}

bool shouldAlignFinalHeading(MotionStatus status,
                             const FollowPathOptions& options) {
  return status == MotionStatus::Completed && options.finalHeading.has_value();
}

HeadingAlignmentOutput calculateHeadingAlignment(
    double currentHeading, double targetHeading, double proportionalGain,
    double maximumRpm, double minimumRpm, double tolerance) {
  HeadingAlignmentOutput output;
  if (!std::isfinite(currentHeading) || !std::isfinite(targetHeading) ||
      !finitePositive(proportionalGain) || !finitePositive(maximumRpm) ||
      !std::isfinite(minimumRpm) || minimumRpm < 0.0 ||
      minimumRpm > maximumRpm || !finitePositive(tolerance)) {
    return output;
  }

  const double error = normalizeDegrees(targetHeading - currentHeading);
  output.valid = true;
  if (std::abs(error) <= tolerance) {
    output.complete = true;
    return output;
  }

  double turnRpm = std::clamp(error * proportionalGain, -maximumRpm,
                              maximumRpm);
  if (std::abs(turnRpm) < minimumRpm) {
    turnRpm = std::copysign(minimumRpm, error);
  }
  output.leftRpm = turnRpm;
  output.rightRpm = -turnRpm;
  return output;
}

}  // namespace aon
