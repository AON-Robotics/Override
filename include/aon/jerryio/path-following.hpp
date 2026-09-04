#pragma once

#include "../constants.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>

namespace aon {

enum class MotionStatus {
  Running,
  Completed,
  InvalidPath,
  InvalidOptions,
  TimedOut,
  Disabled,
  Cancelled,
};

struct MotionResult {
  MotionStatus status = MotionStatus::InvalidPath;

  explicit operator bool() const { return status == MotionStatus::Completed; }
};

struct FollowPathOptions {
  double lookaheadDistance = 5.0;
  double trackWidth = DRIVE_WIDTH;
  double maximumRpm = MAX_RPM;
  double maximumAcceleration = MAX_ACCEL;
  double maximumDeceleration = MAX_DECEL;
  double terminalRecoveryRpm = 30.0;
  double positionTolerance = 2.0;
  std::size_t projectionWindowSegments = 8;
  double headingKp = 2.0;
  double headingTolerance = 1.0;
  double minimumTurnRpm = 20.0;
  std::uint32_t timeoutMs = 10000;
  std::uint32_t loopPeriodMs = 10;
  bool forwards = true;
  std::optional<double> finalHeading;
  std::function<bool()> cancelRequested;

  bool isValid() const;
};

struct HeadingAlignmentOutput {
  double leftRpm = 0.0;
  double rightRpm = 0.0;
  bool complete = false;
  bool valid = false;
};

struct MotionLoopSnapshot {
  bool pathValid = false;
  bool optionsValid = false;
  bool complete = false;
  bool disabled = false;
  bool cancelled = false;
  std::uint32_t elapsedMs = 0;
  std::uint32_t timeoutMs = 10000;
};

MotionStatus evaluateMotionStatus(const MotionLoopSnapshot& snapshot);
const char* motionStatusName(MotionStatus status);
bool shouldAlignFinalHeading(MotionStatus status,
                             const FollowPathOptions& options);
HeadingAlignmentOutput calculateHeadingAlignment(
    double currentHeading, double targetHeading, double proportionalGain,
    double maximumRpm, double minimumRpm, double tolerance);

}  // namespace aon
