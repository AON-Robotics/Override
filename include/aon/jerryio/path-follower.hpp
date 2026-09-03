#pragma once

#include "./path.hpp"

#include <cstddef>
#include <vector>

namespace aon {

struct PathFollowerConfig {
  double lookaheadDistance = 5.0;
  double trackWidth = 12.0;
  double maximumRpm = 600.0;
  double maximumAcceleration = 1200.0;
  double maximumDeceleration = 1800.0;
  double terminalRecoveryRpm = 30.0;
  double positionTolerance = 2.0;
  std::size_t projectionWindowSegments = 8;
  bool forwards = true;
};

struct PathFollowerOutput {
  double leftRpm = 0.0;
  double rightRpm = 0.0;
  double profiledSpeedRpm = 0.0;
  double progress = 0.0;
  double remainingDistance = 0.0;
  Pose target;
  bool complete = false;
  bool valid = false;
};

/// Stateful, speed-aware pure-pursuit controller for a sampled AON path.
class PathFollower {
 public:
  PathFollower(Path path, PathFollowerConfig config = {});

  bool isValid() const { return valid; }
  double length() const;
  PathFollowerOutput step(const Pose& current, double elapsedSeconds);

 private:
  Path path;
  PathFollowerConfig config;
  std::vector<double> cumulativeDistance;
  double progress = 0.0;
  double profiledSpeedRpm = 0.0;
  bool valid = false;

  PathPoint sample(double distance) const;
  double projectProgress(const Pose& current) const;
  double updateProfiledSpeed(double desiredRpm, double elapsedSeconds);
};

}  // namespace aon
