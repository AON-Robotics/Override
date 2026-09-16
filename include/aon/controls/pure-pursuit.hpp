#pragma once

#include "../math/pose.hpp"
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
  std::vector<double> distances;
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
  bool valid() const { return pathValid; }
  bool complete() const { return pathComplete; }
  double progress() const { return traveled; }

  /// Follows one immutable route per controller instance, using AON's profile.
  /// Progress is monotonic and projection is bounded to nearby forward segments.
  std::pair<double, double> follow(const std::vector<Pose>& path,
                                 Pose current, double dt = 0.02) {
    if (path.data() != boundPath || path.size() != boundSize || distances.empty()) {
      boundPath = path.data();
      boundSize = path.size();
      traveled = 0;
      pathComplete = false;
      pathValid = path.size() >= 2 && std::isfinite(lookaheadDistance) && lookaheadDistance > 0;
      distances.assign(path.size(), 0);
      for (std::size_t i = 0; i < path.size(); ++i) {
        const auto& point = path[i];
        pathValid = pathValid && std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.theta);
        if (i == 0) continue;
        const double length = path[i-1].distanceTo(point);
        pathValid = pathValid && std::isfinite(length) && length > 1e-9;
        distances[i] = distances[i-1] + length;
      }
    }
    if (!pathValid || !std::isfinite(current.x) || !std::isfinite(current.y) ||
        !std::isfinite(current.theta) || !std::isfinite(dt) || dt <= 0) {
      pathValid = false;
      return {0, 0};
    }
    const double limit = traveled + 2 * lookaheadDistance;
    double bestError = std::numeric_limits<double>::infinity();
    double bestProgress = traveled;
    for (std::size_t i = 1; i < path.size(); ++i) {
      if (distances[i] < traveled) continue;
      if (distances[i-1] > limit) break;
      const double length = distances[i] - distances[i-1];
      const double dx = path[i].x - path[i-1].x, dy = path[i].y - path[i-1].y;
      const double minimum = std::clamp((traveled - distances[i-1])/length, 0.0, 1.0);
      const double maximum = std::clamp((limit - distances[i-1])/length, minimum, 1.0);
      const double ratio = std::clamp(((current.x-path[i-1].x)*dx +
                                       (current.y-path[i-1].y)*dy)/(length*length), minimum, maximum);
      const double error = std::hypot(current.x-path[i-1].x-ratio*dx,
                                      current.y-path[i-1].y-ratio*dy);
      if (error < bestError) {
        bestError = error;
        bestProgress = distances[i-1] + ratio*length;
      }
    }
    traveled = std::max(traveled, bestProgress);
    const double remaining = distances.back() - traveled;
    const double endpointError = current.distanceTo(path.back());
    pathComplete = remaining <= lookaheadDistance && endpointError <= linearDeadband;
    if (pathComplete) return {0, 0};

    const double targetDistance = std::min(distances.back(), traveled + lookaheadDistance);
    Pose target = path.back();
    for (std::size_t i = 1; i < path.size(); ++i) {
      if (distances[i] < targetDistance) continue;
      const double ratio = (targetDistance-distances[i-1])/(distances[i]-distances[i-1]);
      target = path[i-1] + (path[i]-path[i-1])*ratio;
      break;
    }
    const double dx = target.x-current.x, dy = target.y-current.y;
    const double radians = current.theta*M_PI/180.0;
    const double forward = dx*std::cos(radians) + dy*std::sin(radians);
    const double lateral = -dx*std::sin(radians) + dy*std::cos(radians);
    if (forward < 0) {
      linearProfile.setVelocity(0);
      linearProfile.setAccel(0);
      return turn({current.x, current.y, std::atan2(dy, dx)*180/M_PI}, current, dt);
    }
    const double curvature = 2*lateral/std::max(dx*dx+dy*dy, 1e-9);
    const double speed = linearProfile.update(std::max(remaining, endpointError), dt);
    return bounded(speed*(1+curvature*DRIVE_WIDTH/2), speed*(1-curvature*DRIVE_WIDTH/2));
  }

};

}  // namespace aon
