#pragma once

#include <cstdint>

#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace aon::operator_control {

/// Schedules the driver's endgame rumble notifications.
class DriverFeedback {
 public:
  void update(pros::Controller& controller) {
    const std::uint32_t now = pros::millis();
    if (!started) {
      matchStart = now;
      started = true;
    }

    const std::uint32_t elapsed = now - matchStart;
    if (elapsed >= DRIVER_PERIOD_LENGTH_MS) return;

    while (finalSecond > 0 &&
           elapsed >= FINAL_WARNING_START_MS +
                         (FINAL_COUNTDOWN_START - finalSecond) * 1'000) {
      controller.rumble(".-.-.-");
      --finalSecond;
    }
  }

 private:
  static constexpr unsigned int DRIVER_PERIOD_LENGTH_MS = 90'000;
  static constexpr unsigned int FINAL_WARNING_START_MS = 80'000;
  static constexpr unsigned int FINAL_COUNTDOWN_START = 10;

  std::uint32_t matchStart = 0;
  unsigned int finalSecond = FINAL_COUNTDOWN_START;
  bool started = false;
};

inline DriverFeedback& feedback() {
  static DriverFeedback driverFeedback;
  return driverFeedback;
}

}  // namespace aon::operator_control
