#include "aon/jerryio/path-follower.hpp"

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

constexpr double kTolerance = 0.0001;

bool near(double actual, double expected) {
  return std::abs(actual - expected) <= kTolerance;
}

aon::PathFollowerConfig immediateConfig() {
  aon::PathFollowerConfig config;
  config.lookaheadDistance = 5.0;
  config.trackWidth = 12.0;
  config.maximumRpm = 600.0;
  config.maximumAcceleration = 100000.0;
  config.maximumDeceleration = 100000.0;
  config.driveWheelDiameter = 4.0;
  config.motorToWheelRatio = 1.0;
  config.positionTolerance = 0.5;
  return config;
}

void followsStraightPathsAtExportedSpeed() {
  const aon::Path path{{{0, 0, 0}, 63.5}, {{0, 10, 0}, 63.5}};
  aon::PathFollower follower(path, immediateConfig());

  const auto output = follower.step({0, 0, 0}, 0.02);

  CHECK(output.valid);
  CHECK(!output.complete);
  CHECK(near(output.target.x, 0.0));
  CHECK(near(output.target.y, 5.0));
  CHECK(near(output.leftRpm, 300.0));
  CHECK(near(output.rightRpm, 300.0));
  CHECK(near(output.remainingDistance, 10.0));
}

void turnsTowardGeometricLookahead() {
  const aon::Path path{{{0, 0, 0}, 127}, {{5, 5, 0}, 127},
                       {{10, 5, 0}, 127}};
  aon::PathFollowerConfig config = immediateConfig();
  config.lookaheadDistance = std::sqrt(50.0);
  aon::PathFollower follower(path, config);

  const auto output = follower.step({0, 0, 0}, 0.02);

  CHECK(output.leftRpm > output.rightRpm);
  CHECK(output.leftRpm <= config.maximumRpm);
  CHECK(output.rightRpm >= -config.maximumRpm);
}

void neverMovesProgressBackward() {
  const aon::Path path{{{0, 0, 0}, 127}, {{0, 10, 0}, 127},
                       {{0, 20, 0}, 127}};
  aon::PathFollower follower(path, immediateConfig());

  const auto advanced = follower.step({0, 16, 0}, 0.02);
  const auto noisy = follower.step({0, 4, 0}, 0.02);

  CHECK(advanced.progress >= 16.0 - kTolerance);
  CHECK(noisy.progress >= advanced.progress);
}

void limitsAccelerationAndDeceleration() {
  const aon::Path path{{{0, 0, 0}, 127}, {{0, 10, 0}, 0},
                       {{0, 20, 0}, 0}};
  aon::PathFollowerConfig config = immediateConfig();
  config.maximumAcceleration = 100.0;
  config.maximumDeceleration = 200.0;
  aon::PathFollower follower(path, config);

  const auto accelerating = follower.step({0, 0, 0}, 0.1);
  const auto decelerating = follower.step({0, 10, 0}, 0.1);

  CHECK(near(accelerating.profiledSpeedRpm, 10.0));
  CHECK(near(decelerating.profiledSpeedRpm, 0.0));
}

void supportsReverseFollowing() {
  const aon::Path path{{{0, 0, 0}, 127}, {{0, 10, 0}, 127}};
  aon::PathFollowerConfig config = immediateConfig();
  config.forwards = false;
  aon::PathFollower follower(path, config);

  const auto output = follower.step({0, 0, 180}, 0.02);

  CHECK(near(output.leftRpm, -600.0));
  CHECK(near(output.rightRpm, -600.0));
}

void normalizesWheelCommandsToConfiguredMaximum() {
  const aon::Path path{{{0, 0, 0}, 127}, {{5, 2, 0}, 127}};
  aon::PathFollower follower(path, immediateConfig());

  const auto output = follower.step({0, 0, 0}, 0.02);

  CHECK(std::abs(output.leftRpm) <= 600.0 + kTolerance);
  CHECK(std::abs(output.rightRpm) <= 600.0 + kTolerance);
  CHECK(near(std::max(std::abs(output.leftRpm), std::abs(output.rightRpm)),
             600.0));
}

void completesInsideTerminalTolerance() {
  const aon::Path path{{{0, 0, 0}, 127}, {{0, 10, 0}, 0}};
  aon::PathFollower follower(path, immediateConfig());

  const auto output = follower.step({0.2, 9.8, 45}, 0.02);

  CHECK(output.valid);
  CHECK(output.complete);
  CHECK(output.leftRpm == 0.0);
  CHECK(output.rightRpm == 0.0);
  CHECK(output.profiledSpeedRpm == 0.0);
}

void recoversWhenProgressReachesAZeroSpeedEndpoint() {
  const aon::Path path{{{0, 0, 0}, 127}, {{0, 10, 0}, 0}};
  aon::PathFollowerConfig config = immediateConfig();
  config.terminalRecoveryRpm = 40.0;
  aon::PathFollower follower(path, config);

  const auto output = follower.step({2, 10, 0}, 0.02);

  CHECK(output.valid);
  CHECK(!output.complete);
  CHECK(near(output.profiledSpeedRpm, 40.0));
  CHECK(std::abs(output.leftRpm) + std::abs(output.rightRpm) > 0.0);
}

void doesNotJumpToASeparateLegAtAPathCrossing() {
  aon::Path path;
  for (int index = 0; index <= 10; ++index) {
    path.push_back({{-10.0 + index, -10.0 + index, 0}, 127});
  }
  for (int index = 1; index <= 10; ++index) {
    path.push_back({{static_cast<double>(index),
                     -static_cast<double>(index), 0},
                    127});
  }
  for (int index = 1; index <= 10; ++index) {
    path.push_back({{10.0 - index, -10.0 + index, 0}, 127});
  }

  aon::PathFollowerConfig config = immediateConfig();
  config.projectionWindowSegments = 4;
  aon::PathFollower follower(path, config);
  const auto output = follower.step({-0.1, 0.1, 0}, 0.02);

  CHECK(output.progress < 8.0);
}

void rejectsUnsafePathsAndConfiguration() {
  aon::PathFollowerConfig config = immediateConfig();
  config.lookaheadDistance = 0.0;
  aon::PathFollower invalidConfig({{{0, 0, 0}, 127}, {{0, 10, 0}, 0}},
                                  config);
  aon::PathFollower invalidPath({{{0, 0, 0}, 128}, {{0, 10, 0}, 0}},
                                immediateConfig());

  CHECK(!invalidConfig.isValid());
  CHECK(!invalidConfig.step({0, 0, 0}, 0.02).valid);
  CHECK(!invalidPath.isValid());
}

void plansBrakingBeforeAStopPoint() {
  const aon::Path path{{{0, 0, 0}, 127}, {{0, 10, 0}, 127},
                       {{0, 20, 0}, 0}};
  aon::PathFollowerConfig config = immediateConfig();
  config.maximumDeceleration = 60.0;
  aon::PathFollower follower(path, config);

  CHECK(follower.isValid());
  CHECK(near(follower.plannedSpeedRpm(20.0), 0.0));
  CHECK(follower.plannedSpeedRpm(10.0) < config.maximumRpm);
  CHECK(follower.plannedSpeedRpm(0.0) < config.maximumRpm);
  CHECK(follower.plannedSpeedRpm(0.0) > follower.plannedSpeedRpm(10.0));
}

void limitsAccelerationAfterAStopPoint() {
  const aon::Path path{{{0, 0, 0}, 127}, {{0, 10, 0}, 0},
                       {{0, 20, 0}, 127}};
  aon::PathFollowerConfig config = immediateConfig();
  config.maximumAcceleration = 60.0;
  aon::PathFollower follower(path, config);

  CHECK(follower.isValid());
  CHECK(near(follower.plannedSpeedRpm(10.0), 0.0));
  // With a 4-inch wheel and 1:1 gearing, 60 RPM/s over 10 inches from
  // rest reaches 75.693975 RPM by v^2 = u^2 + 2as.
  CHECK(near(follower.plannedSpeedRpm(20.0), 75.693975));
}

void limitsSpeedFromPathCurvature() {
  const aon::Path path{{{0, 0, 0}, 127}, {{0, 10, 0}, 127},
                       {{10, 10, 0}, 127}};
  aon::PathFollowerConfig unlimitedConfig = immediateConfig();
  aon::PathFollower unlimited(path, unlimitedConfig);

  aon::PathFollowerConfig limitedConfig = immediateConfig();
  limitedConfig.maximumLateralAcceleration = 10.0;
  aon::PathFollower limited(path, limitedConfig);

  CHECK(near(unlimited.plannedSpeedRpm(10.0), 600.0));
  CHECK(limited.plannedSpeedRpm(10.0) <
        unlimited.plannedSpeedRpm(10.0));
  CHECK(limited.plannedSpeedRpm(10.0) > 0.0);
}

void rejectsInvalidPhysicalProfileConfiguration() {
  const aon::Path path{{{0, 0, 0}, 127}, {{0, 10, 0}, 0}};
  aon::PathFollowerConfig config = immediateConfig();
  config.driveWheelDiameter = 0.0;
  CHECK(!aon::PathFollower(path, config).isValid());

  config = immediateConfig();
  config.motorToWheelRatio = 0.0;
  CHECK(!aon::PathFollower(path, config).isValid());

  config = immediateConfig();
  config.maximumLateralAcceleration = -1.0;
  CHECK(!aon::PathFollower(path, config).isValid());
}

void preservesFixedLookaheadWhenAdaptiveModeIsDisabled() {
  const aon::Path path{{{0, 0, 0}, 127}, {{0, 10, 0}, 127},
                       {{0, 20, 0}, 127}};
  aon::PathFollowerConfig config = immediateConfig();
  config.adaptiveLookahead.enabled = false;
  config.adaptiveLookahead.minimumDistance = 2.0;
  config.adaptiveLookahead.maximumDistance = 20.0;
  config.adaptiveLookahead.speedWeight = 3.0;
  aon::PathFollower follower(path, config);

  const auto output = follower.step({0, 0, 0}, 0.02);
  CHECK(near(output.effectiveLookaheadDistance, 5.0));
}

void adaptsLookaheadFromSpeedAndPathCurvature() {
  aon::PathFollowerConfig config = immediateConfig();
  config.adaptiveLookahead.enabled = true;
  config.adaptiveLookahead.minimumDistance = 2.0;
  config.adaptiveLookahead.maximumDistance = 10.0;
  config.adaptiveLookahead.speedWeight = 1.0;
  config.adaptiveLookahead.curvatureWeight = 1.0;

  const aon::Path straight{{{0, 0, 0}, 127}, {{0, 10, 0}, 127},
                           {{0, 20, 0}, 127}};
  aon::PathFollower straightFollower(straight, config);
  const auto straightOutput = straightFollower.step({0, 0, 0}, 0.02);
  CHECK(near(straightOutput.effectiveLookaheadDistance, 10.0));
  CHECK(near(straightOutput.pathCurvature, 0.0));

  const aon::Path corner{{{0, 0, 0}, 127}, {{0, 10, 0}, 127},
                         {{10, 10, 0}, 127}};
  aon::PathFollower cornerFollower(corner, config);
  const auto cornerOutput = cornerFollower.step({0, 10, 0}, 0.02);
  CHECK(cornerOutput.pathCurvature > 0.14);
  CHECK(cornerOutput.effectiveLookaheadDistance <
        straightOutput.effectiveLookaheadDistance);
  CHECK(cornerOutput.effectiveLookaheadDistance >= 2.0);
}

void rejectsInvalidAdaptiveLookaheadConfiguration() {
  const aon::Path path{{{0, 0, 0}, 127}, {{0, 10, 0}, 0}};
  aon::PathFollowerConfig config = immediateConfig();
  config.adaptiveLookahead.enabled = true;
  config.adaptiveLookahead.minimumDistance = 0.0;
  config.adaptiveLookahead.maximumDistance = 10.0;
  CHECK(!aon::PathFollower(path, config).isValid());

  config.adaptiveLookahead.minimumDistance = 8.0;
  config.adaptiveLookahead.maximumDistance = 4.0;
  CHECK(!aon::PathFollower(path, config).isValid());

  config.adaptiveLookahead.minimumDistance = 2.0;
  config.adaptiveLookahead.maximumDistance = 10.0;
  config.adaptiveLookahead.curvatureWeight = -1.0;
  CHECK(!aon::PathFollower(path, config).isValid());
}

void reportsControlTelemetryFromEachStep() {
  const aon::Path path{{{0, 0, 0}, 127}, {{0, 10, 0}, 127},
                       {{0, 20, 0}, 127}};
  aon::PathFollower follower(path, immediateConfig());

  const auto output = follower.step({3, 0, 0}, 0.02);
  CHECK(output.valid);
  CHECK(near(output.crossTrackErrorInches, 3.0));
  CHECK(near(output.plannedSpeedRpm, 600.0));
  CHECK(output.steeringCurvature < 0.0);
  CHECK(output.saturated);
}

}  // namespace

int main() {
  followsStraightPathsAtExportedSpeed();
  turnsTowardGeometricLookahead();
  neverMovesProgressBackward();
  limitsAccelerationAndDeceleration();
  supportsReverseFollowing();
  normalizesWheelCommandsToConfiguredMaximum();
  completesInsideTerminalTolerance();
  recoversWhenProgressReachesAZeroSpeedEndpoint();
  doesNotJumpToASeparateLegAtAPathCrossing();
  rejectsUnsafePathsAndConfiguration();
  plansBrakingBeforeAStopPoint();
  limitsAccelerationAfterAStopPoint();
  limitsSpeedFromPathCurvature();
  rejectsInvalidPhysicalProfileConfiguration();
  preservesFixedLookaheadWhenAdaptiveModeIsDisabled();
  adaptsLookaheadFromSpeedAndPathCurvature();
  rejectsInvalidAdaptiveLookaheadConfiguration();
  reportsControlTelemetryFromEachStep();
  std::cout << "AON path follower tests passed\n";
}
