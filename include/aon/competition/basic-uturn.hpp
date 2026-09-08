#pragma once

#include "aon/constants.hpp"
#include <array>
#include <cstdint>
#include <cmath>

namespace aon {

struct TimedDriveLeg {
  const char* name;
  double leftRpm;
  double rightRpm;
  std::uint32_t durationMs;
};

// Sensor-independent approximation of the checked-in route, not a path decoder.
// Ideal geometry: 33 in out, 8.5 in radius right semicircle, 33 in home.
// Actual travel needs physical tuning because acceleration and slip are ignored.
inline std::array<TimedDriveLeg, 3> basicUTurnPlan() {
  constexpr double pi = 3.14159265358979323846;
  constexpr double rpm = 100.0;
  constexpr double radius = 8.5;
  const double speed = rpm * MOTOR_TO_DRIVE_RATIO * DRIVE_WHEEL_DIAMETER * pi / 60.0;
  const auto straightMs = static_cast<std::uint32_t>(std::lround(33000.0 / speed));
  const auto arcMs = static_cast<std::uint32_t>(std::lround(pi * radius * 1000.0 / speed));
  const double outer = rpm * (radius + DRIVE_WIDTH / 2.0) / radius;
  const double inner = rpm * (radius - DRIVE_WIDTH / 2.0) / radius;
  return {{{"Forward", rpm, rpm, straightMs},
           {"Right U-turn", outer, inner, arcMs},
           {"Return", rpm, rpm, straightMs}}};
}

class Drivetrain;
int runBasicUTurn(Drivetrain& drivetrain);

}  // namespace aon
