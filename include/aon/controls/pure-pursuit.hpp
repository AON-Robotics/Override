#pragma once

#include "path.hpp"
#include <utility>
#include <math.h>
#include <float.h>
#include <vector>
#include <limits>
#include "./s-curve-profile.hpp"

namespace aon {

// TODO: add option for holonomic drives, if necessary
class PurePursuit {
 private:
  // Motion Profiles
  MotionProfile linearProfile;
  MotionProfile angularProfile;

  double lookaheadDistance; // Inches, independent of sample density.
  double maximumRpm = MAX_RPM;
  struct Sample { double distance, rpm; };
  std::vector<Sample> samples;
  std::size_t segment = 1;
  const std::uint8_t* boundSpeeds = nullptr;
  double fastLookahead = 6, lateralAcceleration = 0, lastSpeed = 0;
  Pose pursuitTarget;
  double crossTrack = 0;
  const Pose* boundPath = nullptr;
  std::size_t boundSize = 0;
  double traveled = 0;
  bool pathValid = false;
  bool pathComplete = false;

  std::pair<double, double> bounded(double left, double right) const {
    const double peak = std::max(std::abs(left), std::abs(right));
    const double scale = peak > maximumRpm ? maximumRpm / peak : 1.0;
    return {left * scale, right * scale};
  }

  double linearDeadband;  // Tune
  double angularDeadband;  // Tune

 public:
  PurePursuit(MotionProfile linearProfile, MotionProfile angularProfile,
              double lookaheadDistance, double linearDeadband, double angularDeadband)
      : linearProfile(linearProfile), angularProfile(angularProfile) {
    this->linearProfile.setFinalVelocity(0);
    this->angularProfile.setFinalVelocity(0);
    this->lookaheadDistance = lookaheadDistance;
    fastLookahead = lookaheadDistance;
    this->linearProfile.setVelocity(0);
    this->linearProfile.setAccel(0);
    this->angularProfile.setVelocity(0);
    this->angularProfile.setAccel(0);
    this->linearDeadband = linearDeadband;
    this->angularDeadband = angularDeadband;
  }

  /// @brief Calculates the action from the `current` `Pose` to the `target` `Pose`
  /// @param target The `Pose` we want the robot to get to
  /// @param current  The `Pose` the robot is currently at
  /// @return A pair of \b RPM commands for the left and right sides of the drivetrain
  std::pair<double, double> go(Pose target, Pose current, double dt = 0.02) {
    // Basic Pure Pursuit-style controller (simplified for single target)

    // Extract positions
    double dx = target.x - current.x;
    double dy = target.y - current.y;

    // linearError to target
    double linearError = std::hypot(dx, dy);

    // Desired heading
    double targetAngle = std::atan2(dy, dx) * 180 / M_PI;

    // Current heading (convert to radians)
    double currentHeading = current.theta;

    // Heading angularError (normalize to [-180, 180])
    double angularError = targetAngle - currentHeading;
    while (angularError > 180) angularError -= 360;
    while (angularError < -180) angularError += 360;

    double linearSign = (linearError == 0) ? 0 : (linearError / std::abs(linearError));
    double angularSign = (angularError == 0) ? 0 : (angularError / std::abs(angularError));
    
    // Linear and angular velocities

    // TODO: check if using these three lines instead of the next one maintains accuracy while reducing angular oscillations
    // double angleFactor = std::cos(angularError * M_PI / 180.0);
    // angleFactor = std::clamp(angleFactor, 0.0, 1.0);
    // double linearVel = linearProfile.update(abs(linearError), dt) * * linearSign * angleFactor;
    double linearVel = linearProfile.update(std::abs(linearError), dt) * linearSign;
    
    const double circumference = DRIVE_WIDTH * M_PI;
    double angularArc = circumference * (std::abs(angularError) / 360.0);
    
    double angularVel = angularProfile.update(angularArc, dt) * angularSign;

    // TODO: try using these to see if there is any improvement but I (Kevin G) dont expect it
    // double curvature = (2 * sin(angularErrorRad)) / lookaheadDistance;
    // double angularVel = curvature * linearVel;

    // Convert to tank drive velocities
    // left = v + w, right = v - w
    double left = linearVel + angularVel;
    double right = linearVel - angularVel;

    // Deadband to avoid infinite loop
    if (std::abs(linearError) <= linearDeadband)
      return {0, 0};

    return bounded(left, right);
  }

  /// @brief Calculates the action from the `current` `Pose` to the `target` `Pose` to align heading
  /// @param target The `Pose` we want the robot to get to
  /// @param current  The `Pose` the robot is currently at
  /// @return A pair of \b RPM commands for the left and right sides of the drivetrain
  std::pair<double, double> turn(Pose target, Pose current, double dt = 0.02) {
    // TODO: determine if this is necessary
    // Current heading
    double currentHeading = current.theta;

    // Desired final heading
    double targetHeading = target.theta;

    // Compute angular error (normalize to [-pi, pi])
    double angularError = targetHeading - currentHeading;
    while (angularError > 180) angularError -= 360;
    while (angularError < -180) angularError += 360;

    double sign = (angularError == 0) ? 0 : (angularError / abs(angularError));

    // Pure turning: no linear velocity
    const double arc = std::abs(angularError) * M_PI / 180.0 * DRIVE_WIDTH / 2.0;
    double angularVel = angularProfile.update(arc, dt) * sign;

    // Deadband (symmetric for turning)
    if (std::abs(angularError) <= angularDeadband) return {0, 0};

    return bounded(angularVel, -angularVel);
  }

  void setMaximumRpm(double rpm) {
    maximumRpm = std::clamp(rpm, 1.0, static_cast<double>(MAX_RPM));
    linearProfile.setMaxVelocity(maximumRpm);
    angularProfile.setMaxVelocity(maximumRpm);
  }
  void configure(const FollowOptions& options) {
    setMaximumRpm(options.maximumRpm);
    lookaheadDistance = options.lookahead;
    fastLookahead = options.lookaheadAtSpeed;
    linearDeadband = options.positionTolerance;
    angularDeadband = options.headingTolerance;
    lateralAcceleration = options.lateralAcceleration;
    samples.clear();
  }
  const Pose& target() const { return pursuitTarget; }
  double crossTrackError() const { return crossTrack; }
  bool valid() const { return pathValid; }
  bool complete() const { return pathComplete; }
  double progress() const { return traveled; }

  /// Follows one immutable route per controller instance, using AON's profile.
  /// Progress is monotonic and projection is bounded to nearby forward segments.
  std::pair<double, double> follow(PathView path, Pose current, double dt = 0.02) {
    const double rpmToSpeed = math::linearSpeed(1);
    if (path.data() != boundPath || path.size() != boundSize ||
        path.speeds != boundSpeeds || samples.empty()) {
      boundPath = path.data();
      boundSize = path.size();
      boundSpeeds = path.speeds;
      traveled = lastSpeed = 0;
      segment = 1;
      pathComplete = false;
      pathValid = path.data() && path.size() >= 2 && std::isfinite(lookaheadDistance) &&
                  lookaheadDistance > 0 && std::isfinite(fastLookahead) && fastLookahead > 0;
      samples.assign(path.size(), {});
      for (std::size_t i = 0; i < path.size(); ++i) {
        if (!path.data()) break;
        const auto& point = path[i];
        pathValid = pathValid && std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.theta);
        const double speed = path.speeds ? path.speeds[i] : 127;
        pathValid = pathValid && speed <= 127 && (speed > 0 || i+1 == path.size());
        samples[i].rpm = maximumRpm*speed/127;
        if (i == 0) continue;
        const double length = path[i-1].distanceTo(point);
        pathValid = pathValid && std::isfinite(length) && length > 1e-9;
        samples[i].distance = samples[i-1].distance + length;
        pathValid = pathValid && std::isfinite(samples[i].distance);
      }
      if (pathValid) {
        // Curvature limits use the circumcircle through neighboring samples.
        for (std::size_t i = 1; lateralAcceleration > 0 && i+1 < path.size(); ++i) {
          const double ax = path[i].x-path[i-1].x, ay = path[i].y-path[i-1].y;
          const double bx = path[i+1].x-path[i].x, by = path[i+1].y-path[i].y;
          const double divisor = std::hypot(ax,ay)*std::hypot(bx,by)*path[i-1].distanceTo(path[i+1]);
          const double curvature = divisor > 1e-9 ? 2*std::abs(ax*by-ay*bx)/divisor : 0;
          if (curvature > 1e-9)
            samples[i].rpm = std::min(samples[i].rpm, std::sqrt(lateralAcceleration/curvature)/rpmToSpeed);
        }
        // The profile handles terminal stopping; avoid interpolating toward a
        // zero cap forever. Preview earlier speed reductions with a braking pass.
        if (samples.back().rpm == 0) samples.back().rpm = samples[path.size()-2].rpm;
        for (std::size_t i = path.size()-1; i > 0; --i) {
          const double distance = samples[i].distance-samples[i-1].distance;
          samples[i-1].rpm = std::min(samples[i-1].rpm,
              std::sqrt(samples[i].rpm*samples[i].rpm+linearProfile.maxDeceleration()*distance/rpmToSpeed));
        }
      }
    }
    if (!pathValid || !std::isfinite(current.x) || !std::isfinite(current.y) ||
        !std::isfinite(current.theta) || !std::isfinite(dt) || dt <= 0) {
      pathValid = false;
      return {0, 0};
    }
    const double lookahead = lookaheadDistance+(fastLookahead-lookaheadDistance)*lastSpeed/maximumRpm;
    const double limit = traveled+2*lookahead;
    double bestError = std::numeric_limits<double>::infinity();
    double bestProgress = traveled;
    // Skip completed segments permanently; never scan back through the route.
    while (segment+1 < path.size() && samples[segment].distance < traveled) ++segment;
    for (std::size_t i = segment; i < path.size() && samples[i-1].distance <= limit; ++i) {
      const double length = samples[i].distance-samples[i-1].distance;
      const double dx = path[i].x-path[i-1].x, dy = path[i].y-path[i-1].y;
      const double minimum = std::clamp((traveled-samples[i-1].distance)/length, 0.0, 1.0);
      const double maximum = std::clamp((limit-samples[i-1].distance)/length, minimum, 1.0);
      const double ratio = std::clamp(((current.x-path[i-1].x)*dx+
                                      (current.y-path[i-1].y)*dy)/(length*length), minimum, maximum);
      const double ex = current.x-path[i-1].x-ratio*dx, ey = current.y-path[i-1].y-ratio*dy;
      const double error = ex*ex+ey*ey; // one square root after choosing a projection
      if (error < bestError) {
        bestError = error;
        bestProgress = samples[i-1].distance+ratio*length;
      }
    }
    traveled = std::max(traveled, bestProgress);
    crossTrack = std::sqrt(bestError);
    const double remaining = samples.back().distance-traveled;
    const double endpointError = current.distanceTo(path.back());
    pathComplete = remaining <= lookahead && endpointError <= linearDeadband;
    pursuitTarget = path.back();
    if (pathComplete) { lastSpeed = 0; return {0, 0}; }

    const double targetDistance = std::min(samples.back().distance, traveled+lookahead);
    auto targetSample = std::lower_bound(samples.begin()+segment, samples.end(), targetDistance,
        [](const Sample& sample, double distance) { return sample.distance < distance; });
    const auto index = static_cast<std::size_t>(targetSample-samples.begin());
    const double ratio = (targetDistance-samples[index-1].distance)/
                         (samples[index].distance-samples[index-1].distance);
    pursuitTarget = path[index-1]+(path[index]-path[index-1])*ratio;
    while (segment+1 < path.size() && samples[segment].distance < traveled) ++segment;
    const double fraction = (traveled-samples[segment-1].distance)/
                            (samples[segment].distance-samples[segment-1].distance);
    double cap = samples[segment-1].rpm+(samples[segment].rpm-samples[segment-1].rpm)*fraction;
    const double dx = pursuitTarget.x-current.x, dy = pursuitTarget.y-current.y;
    const double radians = current.theta*M_PI/180.0;
    const double forward = dx*std::cos(radians)+dy*std::sin(radians);
    const double lateral = -dx*std::sin(radians)+dy*std::cos(radians);
    if (forward < 0) {
      lastSpeed = 0;
      linearProfile.setVelocity(0);
      linearProfile.setAccel(0);
      return turn({current.x, current.y, std::atan2(dy,dx)*180/M_PI}, current, dt);
    }
    const double curvature = 2*lateral/std::max(dx*dx+dy*dy, 1e-9);
    if (lateralAcceleration > 0 && std::abs(curvature) > 1e-9)
      cap = std::min(cap, std::sqrt(lateralAcceleration/std::abs(curvature))/rpmToSpeed);
    // Bound the center speed before the profile to preserve the wheel ratio.
    cap = std::min(cap, maximumRpm/(1+std::abs(curvature)*DRIVE_WIDTH/2));
    linearProfile.setMaxVelocity(cap);
    lastSpeed = linearProfile.update(std::max(remaining,endpointError), dt);
    return bounded(lastSpeed*(1+curvature*DRIVE_WIDTH/2), lastSpeed*(1-curvature*DRIVE_WIDTH/2));
  }
};

} // namespace aon
