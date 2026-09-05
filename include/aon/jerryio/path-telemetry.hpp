#pragma once

#include "./path.hpp"

#include <cstdint>

namespace aon {

struct PathTelemetrySample {
  std::uint32_t elapsedMs = 0;
  Pose currentPose;
  Pose targetPose;
  double progressInches = 0.0;
  double remainingDistanceInches = 0.0;
  double crossTrackErrorInches = 0.0;
  double effectiveLookaheadDistanceInches = 0.0;
  double pathCurvature = 0.0;
  double steeringCurvature = 0.0;
  double plannedSpeedRpm = 0.0;
  double profiledSpeedRpm = 0.0;
  double commandedLeftRpm = 0.0;
  double commandedRightRpm = 0.0;
  double measuredDriveRpm = 0.0;
  bool saturated = false;
  bool loopOverrun = false;
};

struct PathMetrics {
  std::uint32_t sampleCount = 0;
  double rmsCrossTrackErrorInches = 0.0;
  double maximumCrossTrackErrorInches = 0.0;
  double endpointErrorInches = 0.0;
  double finalHeadingErrorDegrees = 0.0;
  std::uint32_t saturationCount = 0;
  std::uint32_t loopOverrunCount = 0;
  std::uint32_t elapsedMs = 0;
};

class PathMetricsAccumulator {
 public:
  void add(const PathTelemetrySample& sample);
  PathMetrics finish(double endpointErrorInches,
                     double finalHeadingErrorDegrees,
                     std::uint32_t elapsedMs) const;

 private:
  std::uint32_t sampleCount = 0;
  double squaredCrossTrackErrorSum = 0.0;
  double maximumCrossTrackError = 0.0;
  std::uint32_t saturationCount = 0;
  std::uint32_t loopOverrunCount = 0;
};

PathMetrics mergePathMetrics(const PathMetrics& first,
                             const PathMetrics& second);

}  // namespace aon
