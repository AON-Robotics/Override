#include "aon/jerryio/path-telemetry.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

#define CHECK(condition)                                                     \
  do {                                                                       \
    if (!(condition)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << ": " << #condition       \
                << '\n';                                                     \
      std::exit(1);                                                          \
    }                                                                        \
  } while (false)

namespace {

bool near(double actual, double expected, double tolerance = 1e-9) {
  return std::abs(actual - expected) <= tolerance;
}

void accumulatesConstantSpacePathMetrics() {
  aon::PathMetricsAccumulator accumulator;
  aon::PathTelemetrySample first;
  first.crossTrackErrorInches = 3.0;
  first.saturated = true;
  accumulator.add(first);

  aon::PathTelemetrySample second;
  second.crossTrackErrorInches = -4.0;
  second.loopOverrun = true;
  accumulator.add(second);

  const aon::PathMetrics metrics = accumulator.finish(2.0, -5.0, 50);
  CHECK(metrics.sampleCount == 2);
  CHECK(near(metrics.rmsCrossTrackErrorInches, std::sqrt(12.5)));
  CHECK(near(metrics.maximumCrossTrackErrorInches, 4.0));
  CHECK(near(metrics.endpointErrorInches, 2.0));
  CHECK(near(metrics.finalHeadingErrorDegrees, -5.0));
  CHECK(metrics.saturationCount == 1);
  CHECK(metrics.loopOverrunCount == 1);
  CHECK(metrics.elapsedMs == 50);
}

void mergesMetricsAcrossPathLegs() {
  aon::PathMetrics first;
  first.sampleCount = 2;
  first.rmsCrossTrackErrorInches = 5.0;
  first.maximumCrossTrackErrorInches = 6.0;
  first.saturationCount = 1;
  first.elapsedMs = 40;

  aon::PathMetrics second;
  second.sampleCount = 1;
  second.rmsCrossTrackErrorInches = 2.0;
  second.maximumCrossTrackErrorInches = 3.0;
  second.endpointErrorInches = 1.5;
  second.finalHeadingErrorDegrees = -2.0;
  second.loopOverrunCount = 1;
  second.elapsedMs = 70;

  const aon::PathMetrics merged = aon::mergePathMetrics(first, second);
  CHECK(merged.sampleCount == 3);
  CHECK(near(merged.rmsCrossTrackErrorInches, std::sqrt(18.0)));
  CHECK(near(merged.maximumCrossTrackErrorInches, 6.0));
  CHECK(near(merged.endpointErrorInches, 1.5));
  CHECK(near(merged.finalHeadingErrorDegrees, -2.0));
  CHECK(merged.saturationCount == 1);
  CHECK(merged.loopOverrunCount == 1);
  CHECK(merged.elapsedMs == 70);
}

}  // namespace

int main() {
  accumulatesConstantSpacePathMetrics();
  mergesMetricsAcrossPathLegs();
  std::cout << "AON path telemetry tests passed\n";
}
