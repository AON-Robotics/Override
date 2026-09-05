#pragma once

#include "./path.hpp"

#include <cstddef>
#include <vector>

namespace aon {

struct AdaptiveLookaheadConfig {
  bool enabled = false;
  double minimumDistance = 5.0;
  double maximumDistance = 5.0;
  double speedWeight = 0.0;
  double curvatureWeight = 0.0;
};

struct PathFollowerConfig {
  double lookaheadDistance = 5.0;
  double trackWidth = 12.0;
  double maximumRpm = 600.0;
  double maximumAcceleration = 1200.0;
  double maximumDeceleration = 1800.0;
  double driveWheelDiameter = 2.75;
  double motorToWheelRatio = 0.75;
  // Inches/s^2. Zero disables the curvature cap until it is characterized.
  double maximumLateralAcceleration = 0.0;
  double terminalRecoveryRpm = 30.0;
  double positionTolerance = 2.0;
  std::size_t projectionWindowSegments = 8;
  bool forwards = true;
  AdaptiveLookaheadConfig adaptiveLookahead;
};

struct PathFollowerOutput {
  double leftRpm = 0.0;
  double rightRpm = 0.0;
  double profiledSpeedRpm = 0.0;
  double progress = 0.0;
  double remainingDistance = 0.0;
  double effectiveLookaheadDistance = 0.0;
  double pathCurvature = 0.0;
  double crossTrackErrorInches = 0.0;
  double plannedSpeedRpm = 0.0;
  double steeringCurvature = 0.0;
  Pose target;
  bool saturated = false;
  bool complete = false;
  bool valid = false;
};

/// Stateful, speed-aware pure-pursuit controller for a sampled AON path.
class PathFollower {
 public:
  PathFollower(Path path, PathFollowerConfig config = {});

  bool isValid() const { return valid; }
  double length() const;
  double plannedSpeedRpm(double distance) const;
  PathFollowerOutput step(const Pose& current, double elapsedSeconds);

 private:
  struct Projection {
    double progress = 0.0;
    double error = 0.0;
  };

  Path path;
  PathFollowerConfig config;
  std::vector<double> cumulativeDistance;
  std::vector<double> velocityProfileRpm;
  std::vector<double> curvatureProfile;
  double progress = 0.0;
  double profiledSpeedRpm = 0.0;
  bool valid = false;

  PathPoint sample(double distance) const;
  double sampleProfile(const std::vector<double>& profile,
                       double distance) const;
  double effectiveLookahead(double distance) const;
  Projection projectProgress(const Pose& current) const;
  double updateProfiledSpeed(double desiredRpm, double elapsedSeconds);
};

}  // namespace aon
