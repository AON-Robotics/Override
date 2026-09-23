#pragma once
#include "../controls/path.hpp"
#include <cerrno>
#include <cctype>
#include <cstdio>
#include <cstring>

namespace aon {
enum class TuningLoad { Missing, Loaded, Invalid };

// Versioned SD interchange with tools/path-tune.py. Read before motion only.
// Missing is distinct from malformed: malformed settings must never drive.
inline TuningLoad loadPathTuning(FollowOptions& options, std::uint32_t& id,
                                const char* filename) {
  errno = 0;
  auto* file = std::fopen(filename,"r");
  if (!file) return errno == ENOENT ? TuningLoad::Missing : TuningLoad::Invalid;
  char header[64];
  bool valid = std::fgets(header,sizeof(header),file) &&
      (std::strcmp(header,"AON_PATH_TUNING_V1\n") == 0 ||
       std::strcmp(header,"AON_PATH_TUNING_V1\r\n") == 0);
  double values[20]{};
  for (int i=0; valid && i<20; ++i) {
    valid = std::fscanf(file,"%lf",&values[i]) == 1 && std::isfinite(values[i]);
    if (valid && i<19) valid = std::fgetc(file) == ',';
  }
  int tail;
  while (valid && (tail = std::fgetc(file)) != EOF)
    valid = std::isspace(static_cast<unsigned char>(tail)) != 0;
  valid = valid && !std::ferror(file);
  std::fclose(file);
  if (!valid) return TuningLoad::Invalid;
  const double hardware[] = {DRIVE_WHEEL_DIAMETER, MOTOR_TO_DRIVE_RATIO, DRIVE_WIDTH,
                            TRACKING_WHEEL_DIAMETER, MAX_ACCEL, MAX_DECEL};
  for (int i=0; i<6; ++i)
    if (std::abs(values[i+1]-hardware[i]) > 1e-9) return TuningLoad::Invalid;
  for (int i : {0,13,15})
    if (values[i] != std::floor(values[i])) return TuningLoad::Invalid;
  if (values[0] < 1 || values[0] > UINT32_MAX || values[13] < 50 || values[13] > 2000 ||
      values[15] < 100 || values[15] > 30000) return TuningLoad::Invalid;
  FollowOptions candidate;
  candidate.maximumRpm = values[7];
  candidate.lookahead = values[8];
  candidate.lookaheadAtSpeed = values[9];
  candidate.positionTolerance = values[10];
  candidate.headingTolerance = values[11];
  candidate.lateralAcceleration = values[12];
  candidate.settleMs = static_cast<std::uint32_t>(values[13]);
  candidate.settledRpm = values[14];
  candidate.timeoutMs = static_cast<std::uint32_t>(values[15]);
  candidate.accelerationScale = values[16];
  candidate.decelerationScale = values[17];
  candidate.turnAccelerationScale = values[18];
  candidate.turnDecelerationScale = values[19];
  if (!validFollowOptions(candidate) || candidate.maximumRpm > 200 ||
      candidate.lookahead < 2 || candidate.lookahead > 18 ||
      candidate.lookaheadAtSpeed < 2 || candidate.lookaheadAtSpeed > 18 ||
      candidate.positionTolerance > 2 || candidate.headingTolerance > 5 ||
      candidate.lateralAcceleration <= 0 || candidate.lateralAcceleration > 60 ||
      candidate.settledRpm > 5) return TuningLoad::Invalid;
  options = candidate;
  id = static_cast<std::uint32_t>(values[0]);
  return TuningLoad::Loaded;
}
} // namespace aon
