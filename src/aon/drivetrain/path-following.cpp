#include "../../../include/aon/drivetrain/drivetrain.hpp"

#include "../../../include/aon/jerryio/path-follower.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>

namespace aon {
namespace {

double normalizeDegrees(double angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

PathMetrics finishMetrics(const PathMetricsAccumulator& accumulator,
                          const Path& path,
                          const FollowPathOptions& options,
                          const Pose& pose, std::uint32_t elapsedMs) {
  const double endpointError =
      path.empty() ? 0.0 : pose.distanceTo(path.back().pose);
  const double headingError =
      options.finalHeading.has_value()
          ? normalizeDegrees(*options.finalHeading - pose.theta)
          : 0.0;
  return accumulator.finish(endpointError, headingError, elapsedMs);
}

}  // namespace

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
  followerConfig.driveWheelDiameter = options.driveWheelDiameter;
  followerConfig.motorToWheelRatio = options.motorToWheelRatio;
  followerConfig.maximumLateralAcceleration =
      options.maximumLateralAcceleration;
  followerConfig.terminalRecoveryRpm = options.terminalRecoveryRpm;
  followerConfig.positionTolerance = options.positionTolerance;
  followerConfig.projectionWindowSegments = options.projectionWindowSegments;
  followerConfig.forwards = options.forwards;
  followerConfig.adaptiveLookahead = options.adaptiveLookahead;
  PathFollower follower(path, followerConfig);

  PathMetricsAccumulator metricsAccumulator;
  std::size_t loopCount = 0;
  const auto finish = [this, &metricsAccumulator, &path, &options, startedAt](
                          MotionStatus status, const Pose& pose) {
    this->stop();
    return MotionResult{
        status,
        finishMetrics(metricsAccumulator, path, options, pose,
                      pros::millis() - startedAt)};
  };
  std::uint64_t lastUpdate = pros::micros();
  std::uint32_t nextWake = pros::millis();

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
    if (status != MotionStatus::Running) {
      return finish(status, this->odometry->getPose());
    }

    const std::uint64_t now = pros::micros();
    const std::uint64_t updateIntervalMicros = now - lastUpdate;
    const double elapsedSeconds = std::max(
        static_cast<double>(updateIntervalMicros) / 1000000.0,
        static_cast<double>(options.loopPeriodMs) / 1000.0);
    lastUpdate = now;
    const Pose currentPose = this->odometry->getPose();
    const PathFollowerOutput output =
        follower.step(currentPose, elapsedSeconds);

    PathTelemetrySample telemetry;
    telemetry.elapsedMs = pros::millis() - startedAt;
    telemetry.currentPose = currentPose;
    telemetry.targetPose = output.target;
    telemetry.progressInches = output.progress;
    telemetry.remainingDistanceInches = output.remainingDistance;
    telemetry.crossTrackErrorInches = output.crossTrackErrorInches;
    telemetry.effectiveLookaheadDistanceInches =
        output.effectiveLookaheadDistance;
    telemetry.pathCurvature = output.pathCurvature;
    telemetry.steeringCurvature = output.steeringCurvature;
    telemetry.plannedSpeedRpm = output.plannedSpeedRpm;
    telemetry.profiledSpeedRpm = output.profiledSpeedRpm;
    telemetry.commandedLeftRpm = output.leftRpm;
    telemetry.commandedRightRpm = output.rightRpm;
    telemetry.measuredDriveRpm = this->getRPM();
    telemetry.saturated = output.saturated;
    telemetry.loopOverrun =
        updateIntervalMicros >
        static_cast<std::uint64_t>(options.loopPeriodMs) * 1000;
    if (output.valid) {
      metricsAccumulator.add(telemetry);
      if (options.telemetry &&
          loopCount % options.telemetryEveryNLoops == 0) {
        options.telemetry(telemetry);
      }
      ++loopCount;
    }

    snapshot.pathValid = output.valid;
    snapshot.complete = output.complete;
    snapshot.disabled = pros::competition::is_disabled();
    snapshot.cancelled =
        options.cancelRequested && options.cancelRequested();
    snapshot.elapsedMs = pros::millis() - startedAt;
    status = evaluateMotionStatus(snapshot);

    if (status == MotionStatus::Running) {
      this->tank(output.leftRpm, output.rightRpm);
      pros::Task::delay_until(&nextWake, options.loopPeriodMs);
      continue;
    }

    this->stop();
    MotionResult pathResult = finish(status, currentPose);
    if (!shouldAlignFinalHeading(status, options)) return pathResult;

    MotionResult alignmentResult =
        alignToHeading(*options.finalHeading, options, startedAt);
    const Pose alignedPose = this->odometry->getPose();
    pathResult.status = alignmentResult.status;
    pathResult.metrics.endpointErrorInches =
        alignedPose.distanceTo(path.back().pose);
    pathResult.metrics.finalHeadingErrorDegrees =
        normalizeDegrees(*options.finalHeading - alignedPose.theta);
    pathResult.metrics.elapsedMs = pros::millis() - startedAt;
    return pathResult;
  }
}

MotionResult Drivetrain::followPathWithActions(
    const Path& path, const std::vector<PathAction>& actions,
    const FollowPathOptions& options) {
  const PathActionPlan plan = buildPathActionPlan(path);
  PathMetrics combinedMetrics;
  const auto finish = [this, &combinedMetrics](MotionStatus status) {
    this->stop();
    return MotionResult{status, combinedMetrics};
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
    combinedMetrics = mergePathMetrics(combinedMetrics, driveResult.metrics);
    if (!driveResult) {
      return MotionResult{driveResult.status, combinedMetrics};
    }

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
