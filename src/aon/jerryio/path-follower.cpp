#include "../../../include/aon/jerryio/path-follower.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace aon {
namespace {

constexpr double kJerryIoMaximumSpeed = 127.0;
constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double minimum, double maximum) {
  return std::max(minimum, std::min(maximum, value));
}

double normalizeDegrees(double angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

bool finitePose(const Pose& pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(pose.theta);
}

bool finitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

double pointCurvature(const Pose& previous, const Pose& current,
                      const Pose& next) {
  const double previousToCurrent = previous.distanceTo(current);
  const double currentToNext = current.distanceTo(next);
  const double previousToNext = previous.distanceTo(next);
  const double cross =
      std::abs((current.x - previous.x) * (next.y - previous.y) -
               (current.y - previous.y) * (next.x - previous.x));
  const double denominator =
      previousToCurrent * currentToNext * previousToNext;
  return denominator > 1e-9 ? 2.0 * cross / denominator : 0.0;
}

}  // namespace

PathFollower::PathFollower(Path path, PathFollowerConfig config)
    : path(std::move(path)), config(config) {
  valid = this->path.size() >= 2 &&
          finitePositive(this->config.lookaheadDistance) &&
          finitePositive(this->config.trackWidth) &&
          finitePositive(this->config.maximumRpm) &&
          finitePositive(this->config.maximumAcceleration) &&
          finitePositive(this->config.maximumDeceleration) &&
          finitePositive(this->config.driveWheelDiameter) &&
          finitePositive(this->config.motorToWheelRatio) &&
          std::isfinite(this->config.maximumLateralAcceleration) &&
          this->config.maximumLateralAcceleration >= 0.0 &&
          std::isfinite(this->config.terminalRecoveryRpm) &&
          this->config.terminalRecoveryRpm >= 0.0 &&
          this->config.terminalRecoveryRpm <= this->config.maximumRpm &&
          finitePositive(this->config.positionTolerance) &&
          this->config.projectionWindowSegments > 0;

  const AdaptiveLookaheadConfig& adaptive =
      this->config.adaptiveLookahead;
  if (adaptive.enabled &&
      (!finitePositive(adaptive.minimumDistance) ||
       !finitePositive(adaptive.maximumDistance) ||
       adaptive.minimumDistance > adaptive.maximumDistance ||
       !std::isfinite(adaptive.speedWeight) || adaptive.speedWeight < 0.0 ||
       !std::isfinite(adaptive.curvatureWeight) ||
       adaptive.curvatureWeight < 0.0)) {
    valid = false;
  }

  cumulativeDistance.reserve(this->path.size());
  cumulativeDistance.push_back(0.0);
  for (std::size_t index = 0; index < this->path.size(); ++index) {
    const PathPoint& point = this->path[index];
    if (!finitePose(point.pose) || !std::isfinite(point.speed) ||
        point.speed < 0.0 || point.speed > kJerryIoMaximumSpeed) {
      valid = false;
    }
    if (index == 0) continue;
    const double segmentLength =
        this->path[index - 1].pose.distanceTo(point.pose);
    if (!finitePositive(segmentLength)) valid = false;
    cumulativeDistance.push_back(cumulativeDistance.back() + segmentLength);
  }

  if (!valid) return;

  curvatureProfile.assign(this->path.size(), 0.0);
  for (std::size_t index = 1; index + 1 < this->path.size(); ++index) {
    curvatureProfile[index] = pointCurvature(this->path[index - 1].pose,
                                             this->path[index].pose,
                                             this->path[index + 1].pose);
  }

  velocityProfileRpm.reserve(this->path.size());
  for (const PathPoint& point : this->path) {
    velocityProfileRpm.push_back(
        point.speed / kJerryIoMaximumSpeed * this->config.maximumRpm);
  }

  const double inchesPerSecondPerRpm =
      kPi * this->config.driveWheelDiameter *
      this->config.motorToWheelRatio / 60.0;
  if (this->config.maximumLateralAcceleration > 0.0) {
    for (std::size_t index = 1; index + 1 < this->path.size(); ++index) {
      const double curvature = curvatureProfile[index];
      if (curvature <= 1e-9) continue;
      const double maximumLinearSpeed = std::sqrt(
          this->config.maximumLateralAcceleration / curvature);
      velocityProfileRpm[index] = std::min(
          velocityProfileRpm[index],
          maximumLinearSpeed / inchesPerSecondPerRpm);
    }
  }

  const double accelerationInchesPerSecondSquared =
      this->config.maximumAcceleration * inchesPerSecondPerRpm;
  for (std::size_t index = 1; index < this->path.size(); ++index) {
    const double previousLinearSpeed =
        velocityProfileRpm[index - 1] * inchesPerSecondPerRpm;
    const double segmentLength =
        cumulativeDistance[index] - cumulativeDistance[index - 1];
    const double maximumLinearSpeed = std::sqrt(
        previousLinearSpeed * previousLinearSpeed +
        2.0 * accelerationInchesPerSecondSquared * segmentLength);
    velocityProfileRpm[index] = std::min(
        velocityProfileRpm[index],
        maximumLinearSpeed / inchesPerSecondPerRpm);
  }

  const double decelerationInchesPerSecondSquared =
      this->config.maximumDeceleration * inchesPerSecondPerRpm;
  for (std::size_t index = this->path.size() - 1; index-- > 0;) {
    const double nextLinearSpeed =
        velocityProfileRpm[index + 1] * inchesPerSecondPerRpm;
    const double segmentLength =
        cumulativeDistance[index + 1] - cumulativeDistance[index];
    const double maximumLinearSpeed = std::sqrt(
        nextLinearSpeed * nextLinearSpeed +
        2.0 * decelerationInchesPerSecondSquared * segmentLength);
    velocityProfileRpm[index] = std::min(
        velocityProfileRpm[index],
        maximumLinearSpeed / inchesPerSecondPerRpm);
  }
}

double PathFollower::length() const {
  return cumulativeDistance.empty() ? 0.0 : cumulativeDistance.back();
}

PathPoint PathFollower::sample(double distance) const {
  if (distance <= 0.0) return path.front();
  if (distance >= length()) return path.back();

  const auto upper = std::upper_bound(cumulativeDistance.begin(),
                                      cumulativeDistance.end(), distance);
  const std::size_t endIndex =
      static_cast<std::size_t>(upper - cumulativeDistance.begin());
  const std::size_t startIndex = endIndex - 1;
  const double segmentLength =
      cumulativeDistance[endIndex] - cumulativeDistance[startIndex];
  const double ratio =
      (distance - cumulativeDistance[startIndex]) / segmentLength;
  const PathPoint& start = path[startIndex];
  const PathPoint& end = path[endIndex];

  return {{start.pose.x + (end.pose.x - start.pose.x) * ratio,
           start.pose.y + (end.pose.y - start.pose.y) * ratio,
           start.pose.theta + (end.pose.theta - start.pose.theta) * ratio},
          start.speed + (end.speed - start.speed) * ratio};
}

double PathFollower::plannedSpeedRpm(double distance) const {
  if (!valid || velocityProfileRpm.empty()) return 0.0;
  return sampleProfile(velocityProfileRpm, distance);
}

double PathFollower::sampleProfile(const std::vector<double>& profile,
                                   double distance) const {
  if (profile.empty()) return 0.0;
  if (distance <= 0.0) return profile.front();
  if (distance >= length()) return profile.back();

  const auto upper = std::upper_bound(cumulativeDistance.begin(),
                                      cumulativeDistance.end(), distance);
  const std::size_t endIndex =
      static_cast<std::size_t>(upper - cumulativeDistance.begin());
  const std::size_t startIndex = endIndex - 1;
  const double ratio =
      (distance - cumulativeDistance[startIndex]) /
      (cumulativeDistance[endIndex] - cumulativeDistance[startIndex]);
  return profile[startIndex] +
         (profile[endIndex] - profile[startIndex]) * ratio;
}

double PathFollower::effectiveLookahead(double distance) const {
  const AdaptiveLookaheadConfig& adaptive = config.adaptiveLookahead;
  if (!adaptive.enabled) return config.lookaheadDistance;

  const double speedFraction =
      clamp(plannedSpeedRpm(distance) / config.maximumRpm, 0.0, 1.0);
  const double normalizedCurvature =
      std::abs(sampleProfile(curvatureProfile, distance)) * config.trackWidth;
  const double distanceFromSpeed =
      config.lookaheadDistance * (1.0 + adaptive.speedWeight * speedFraction);
  return clamp(distanceFromSpeed /
                   (1.0 + adaptive.curvatureWeight * normalizedCurvature),
               adaptive.minimumDistance, adaptive.maximumDistance);
}

PathFollower::Projection PathFollower::projectProgress(
    const Pose& current) const {
  double bestDistanceSquared = std::numeric_limits<double>::infinity();
  double bestProgress = progress;

  const auto upper = std::upper_bound(cumulativeDistance.begin(),
                                      cumulativeDistance.end(), progress);
  const std::size_t activeSegment =
      upper == cumulativeDistance.begin()
          ? 0
          : std::min(static_cast<std::size_t>(upper - cumulativeDistance.begin() - 1),
                     path.size() - 2);
  const std::size_t endSegment =
      std::min(path.size() - 1,
               activeSegment + config.projectionWindowSegments);

  for (std::size_t index = activeSegment; index < endSegment; ++index) {
    const Pose& start = path[index].pose;
    const Pose& end = path[index + 1].pose;
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double segmentLength =
        cumulativeDistance[index + 1] - cumulativeDistance[index];
    const double lengthSquared = dx * dx + dy * dy;
    double ratio = ((current.x - start.x) * dx +
                    (current.y - start.y) * dy) /
                   lengthSquared;
    ratio = clamp(ratio, 0.0, 1.0);

    const double minimumRatio =
        clamp((progress - cumulativeDistance[index]) / segmentLength, 0.0,
              1.0);
    ratio = std::max(ratio, minimumRatio);
    const double candidateProgress =
        cumulativeDistance[index] + ratio * segmentLength;
    if (candidateProgress + 1e-9 < progress) continue;

    const double projectedX = start.x + ratio * dx;
    const double projectedY = start.y + ratio * dy;
    const double errorX = current.x - projectedX;
    const double errorY = current.y - projectedY;
    const double distanceSquared = errorX * errorX + errorY * errorY;
    if (distanceSquared < bestDistanceSquared) {
      bestDistanceSquared = distanceSquared;
      bestProgress = candidateProgress;
    }
  }

  Projection projection;
  projection.progress = std::max(progress, bestProgress);
  projection.error = std::sqrt(bestDistanceSquared);
  return projection;
}

double PathFollower::updateProfiledSpeed(double desiredRpm,
                                         double elapsedSeconds) {
  const double rate = desiredRpm >= profiledSpeedRpm
                          ? config.maximumAcceleration
                          : config.maximumDeceleration;
  const double maximumChange = rate * elapsedSeconds;
  profiledSpeedRpm +=
      clamp(desiredRpm - profiledSpeedRpm, -maximumChange, maximumChange);
  return profiledSpeedRpm;
}

PathFollowerOutput PathFollower::step(const Pose& current,
                                      double elapsedSeconds) {
  PathFollowerOutput output;
  output.progress = progress;
  output.remainingDistance = std::max(0.0, length() - progress);
  output.target = path.empty() ? Pose{} : path.front().pose;
  if (!valid || !finitePose(current) || !finitePositive(elapsedSeconds)) {
    return output;
  }

  const Projection projection = projectProgress(current);
  progress = projection.progress;
  output.progress = progress;
  output.remainingDistance = std::max(0.0, length() - progress);
  output.crossTrackErrorInches = projection.error;
  output.pathCurvature = sampleProfile(curvatureProfile, progress);
  output.effectiveLookaheadDistance = effectiveLookahead(progress);
  output.plannedSpeedRpm = plannedSpeedRpm(progress);
  const double terminalDistance = current.distanceTo(path.back().pose);
  if (terminalDistance <= config.positionTolerance &&
      output.remainingDistance <= output.effectiveLookaheadDistance) {
    profiledSpeedRpm = 0.0;
    output.target = path.back().pose;
    output.complete = true;
    output.valid = true;
    return output;
  }

  const PathPoint lookahead =
      sample(std::min(length(),
                      progress + output.effectiveLookaheadDistance));
  output.target = lookahead.pose;
  double desiredRpm = output.plannedSpeedRpm;
  if (output.remainingDistance <= output.effectiveLookaheadDistance &&
      terminalDistance > config.positionTolerance) {
    desiredRpm = std::max(desiredRpm, config.terminalRecoveryRpm);
  }
  const double speed = updateProfiledSpeed(desiredRpm, elapsedSeconds);
  output.profiledSpeedRpm = speed;

  const double dx = lookahead.pose.x - current.x;
  const double dy = lookahead.pose.y - current.y;
  double desiredHeading = std::atan2(dy, dx) * 180.0 / kPi;
  if (!config.forwards) desiredHeading += 180.0;
  const double headingError =
      normalizeDegrees(desiredHeading - current.theta);
  const double targetDistance = std::max(std::hypot(dx, dy), 1e-6);
  const double curvature =
      2.0 * std::sin(headingError * kPi / 180.0) / targetDistance;
  output.steeringCurvature = curvature;
  const double signedSpeed = config.forwards ? speed : -speed;
  double left = signedSpeed * (1.0 + curvature * config.trackWidth / 2.0);
  double right = signedSpeed * (1.0 - curvature * config.trackWidth / 2.0);

  const double largestMagnitude = std::max(std::abs(left), std::abs(right));
  if (largestMagnitude > speed && largestMagnitude > 0.0) {
    output.saturated = true;
    const double scale = speed / largestMagnitude;
    left *= scale;
    right *= scale;
  }

  output.leftRpm = left;
  output.rightRpm = right;
  output.valid = true;
  return output;
}

}  // namespace aon
