#include "../../../include/aon/jerryio/path-telemetry.hpp"

#include <algorithm>
#include <cmath>

namespace aon {

void PathMetricsAccumulator::add(const PathTelemetrySample& sample) {
  const double absoluteError = std::abs(sample.crossTrackErrorInches);
  ++sampleCount;
  squaredCrossTrackErrorSum += absoluteError * absoluteError;
  maximumCrossTrackError = std::max(maximumCrossTrackError, absoluteError);
  if (sample.saturated) ++saturationCount;
  if (sample.loopOverrun) ++loopOverrunCount;
}

PathMetrics PathMetricsAccumulator::finish(
    double endpointErrorInches, double finalHeadingErrorDegrees,
    std::uint32_t elapsedMs) const {
  PathMetrics metrics;
  metrics.sampleCount = sampleCount;
  metrics.rmsCrossTrackErrorInches =
      sampleCount == 0
          ? 0.0
          : std::sqrt(squaredCrossTrackErrorSum / sampleCount);
  metrics.maximumCrossTrackErrorInches = maximumCrossTrackError;
  metrics.endpointErrorInches = endpointErrorInches;
  metrics.finalHeadingErrorDegrees = finalHeadingErrorDegrees;
  metrics.saturationCount = saturationCount;
  metrics.loopOverrunCount = loopOverrunCount;
  metrics.elapsedMs = elapsedMs;
  return metrics;
}

PathMetrics mergePathMetrics(const PathMetrics& first,
                             const PathMetrics& second) {
  PathMetrics merged;
  merged.sampleCount = first.sampleCount + second.sampleCount;
  if (merged.sampleCount > 0) {
    const double squaredErrorSum =
        first.rmsCrossTrackErrorInches * first.rmsCrossTrackErrorInches *
            first.sampleCount +
        second.rmsCrossTrackErrorInches * second.rmsCrossTrackErrorInches *
            second.sampleCount;
    merged.rmsCrossTrackErrorInches =
        std::sqrt(squaredErrorSum / merged.sampleCount);
  }
  merged.maximumCrossTrackErrorInches =
      std::max(first.maximumCrossTrackErrorInches,
               second.maximumCrossTrackErrorInches);
  merged.endpointErrorInches = second.endpointErrorInches;
  merged.finalHeadingErrorDegrees = second.finalHeadingErrorDegrees;
  merged.saturationCount =
      first.saturationCount + second.saturationCount;
  merged.loopOverrunCount =
      first.loopOverrunCount + second.loopOverrunCount;
  merged.elapsedMs = std::max(first.elapsedMs, second.elapsedMs);
  return merged;
}

}  // namespace aon
