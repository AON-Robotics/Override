#include "../../../include/aon/drivetrain/drivetrain.hpp"

#include "../../../include/aon/jerryio/path-follower.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include <algorithm>

namespace aon {

MotionResult Drivetrain::followPath(const Path& path,
                                    const FollowPathOptions& options) {
  return followPathFromStart(path, options, pros::millis());
}

MotionResult Drivetrain::followPathFromStart(
    const Path& path, const FollowPathOptions& options,
    std::uint32_t startedAt) {
  PathFollowerConfig followerConfig;
  followerConfig.lookaheadDistance = options.lookaheadDistance;
  followerConfig.trackWidth = options.trackWidth;
  followerConfig.maximumRpm = options.maximumRpm;
  followerConfig.maximumAcceleration = options.maximumAcceleration;
  followerConfig.maximumDeceleration = options.maximumDeceleration;
  followerConfig.terminalRecoveryRpm = options.terminalRecoveryRpm;
  followerConfig.positionTolerance = options.positionTolerance;
  followerConfig.projectionWindowSegments = options.projectionWindowSegments;
  followerConfig.forwards = options.forwards;
  PathFollower follower(path, followerConfig);

  const auto finish = [this](MotionStatus status) {
    this->stop();
    return MotionResult{status};
  };
  std::uint64_t lastUpdate = pros::micros();

  while (true) {
    const std::uint32_t elapsed = pros::millis() - startedAt;
    MotionLoopSnapshot snapshot;
    snapshot.pathValid = follower.isValid();
    snapshot.optionsValid = options.isValid();
    snapshot.disabled = pros::competition::is_disabled();
    snapshot.cancelled =
        options.cancelRequested && options.cancelRequested();
    snapshot.elapsedMs = elapsed;
    snapshot.timeoutMs = options.timeoutMs;

    MotionStatus status = evaluateMotionStatus(snapshot);
    if (status != MotionStatus::Running) return finish(status);

    const std::uint64_t now = pros::micros();
    const double elapsedSeconds = std::max(
        static_cast<double>(now - lastUpdate) / 1000000.0,
        static_cast<double>(options.loopPeriodMs) / 1000.0);
    lastUpdate = now;
    const PathFollowerOutput output =
        follower.step(this->odometry->getPose(), elapsedSeconds);

    snapshot.pathValid = output.valid;
    snapshot.complete = output.complete;
    snapshot.disabled = pros::competition::is_disabled();
    snapshot.cancelled =
        options.cancelRequested && options.cancelRequested();
    snapshot.elapsedMs = pros::millis() - startedAt;
    status = evaluateMotionStatus(snapshot);

    if (status == MotionStatus::Running) {
      this->tank(output.leftRpm, output.rightRpm);
      pros::delay(options.loopPeriodMs);
      continue;
    }

    this->stop();
    if (!shouldAlignFinalHeading(status, options)) return {status};
    return alignToHeading(*options.finalHeading, options, startedAt);
  }
}

MotionResult Drivetrain::followPathWithActions(
    const Path& path, const std::vector<PathAction>& actions,
    const FollowPathOptions& options) {
  const PathActionPlan plan = buildPathActionPlan(path);
  const auto finish = [this](MotionStatus status) {
    this->stop();
    return MotionResult{status};
  };
  if (!options.isValid() || !validatePathActions(plan, actions)) {
    return finish(MotionStatus::InvalidOptions);
  }

  const std::uint32_t startedAt = pros::millis();
  for (std::size_t legIndex = 0; legIndex < plan.legs.size(); ++legIndex) {
    FollowPathOptions legOptions = options;
    if (legIndex + 1 < plan.legs.size()) legOptions.finalHeading.reset();
    const MotionResult driveResult =
        followPathFromStart(plan.legs[legIndex].path, legOptions, startedAt);
    if (!driveResult) return driveResult;

    const auto marker = plan.legs[legIndex].markerOrdinalAfter;
    if (!marker.has_value()) continue;
    for (const PathAction& action : actions) {
      if (action.markerOrdinal != *marker) continue;

      MotionLoopSnapshot beforeAction;
      beforeAction.pathValid = true;
      beforeAction.optionsValid = true;
      beforeAction.disabled = pros::competition::is_disabled();
      beforeAction.cancelled =
          options.cancelRequested && options.cancelRequested();
      beforeAction.elapsedMs = pros::millis() - startedAt;
      beforeAction.timeoutMs = options.timeoutMs;
      const MotionStatus beforeStatus = evaluateMotionStatus(beforeAction);
      if (beforeStatus != MotionStatus::Running) return finish(beforeStatus);

      this->stop();
      if (action.start) action.start();
      const std::uint32_t actionStartedAt = pros::millis();
      while (true) {
        MotionLoopSnapshot snapshot;
        snapshot.pathValid = true;
        snapshot.optionsValid = true;
        snapshot.complete =
            pros::millis() - actionStartedAt >= action.durationMs;
        snapshot.disabled = pros::competition::is_disabled();
        snapshot.cancelled =
            options.cancelRequested && options.cancelRequested();
        snapshot.elapsedMs = pros::millis() - startedAt;
        snapshot.timeoutMs = options.timeoutMs;
        const MotionStatus status = evaluateMotionStatus(snapshot);
        if (status == MotionStatus::Running) {
          pros::delay(options.loopPeriodMs);
          continue;
        }

        if (action.cleanup) action.cleanup();
        if (status != MotionStatus::Completed) return finish(status);
        break;
      }
    }
  }
  return finish(MotionStatus::Completed);
}

MotionResult Drivetrain::moveToPose(const Pose& target,
                                    FollowPathOptions options) {
  const Pose start = this->odometry->getPose();
  const std::uint32_t startedAt = pros::millis();
  options.finalHeading = target.theta;
  if (start.distanceTo(target) <= options.positionTolerance) {
    return alignToHeading(target.theta, options, startedAt);
  }
  Path path{{start, 127.0}, {target, 0.0}};
  return followPath(path, options);
}

MotionResult Drivetrain::turnToHeadingMonitored(double heading,
                                                FollowPathOptions options) {
  options.finalHeading = heading;
  return alignToHeading(heading, options, pros::millis());
}

MotionResult Drivetrain::alignToHeading(double heading,
                                        const FollowPathOptions& options,
                                        std::uint32_t startedAt) {
  const auto finish = [this](MotionStatus status) {
    this->stop();
    return MotionResult{status};
  };

  while (true) {
    const HeadingAlignmentOutput output = calculateHeadingAlignment(
        this->odometry->getDegrees(), heading, options.headingKp,
        options.maximumRpm, options.minimumTurnRpm, options.headingTolerance);
    MotionLoopSnapshot snapshot;
    snapshot.pathValid = output.valid;
    snapshot.optionsValid = options.isValid();
    snapshot.complete = output.complete;
    snapshot.disabled = pros::competition::is_disabled();
    snapshot.cancelled =
        options.cancelRequested && options.cancelRequested();
    snapshot.elapsedMs = pros::millis() - startedAt;
    snapshot.timeoutMs = options.timeoutMs;

    const MotionStatus status = evaluateMotionStatus(snapshot);
    if (status != MotionStatus::Running) return finish(status);

    this->tank(output.leftRpm, output.rightRpm);
    pros::delay(options.loopPeriodMs);
  }
}

}  // namespace aon
