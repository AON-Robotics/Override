#pragma once

#include <cstdint>

#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace aon::operator_control {

/// Schedules the driver's endgame rumble notifications.
class DriverFeedback {
 public:
  void begin(const std::uint32_t startTime) {
    matchStart = startTime;
    finalSecond = FINAL_COUNTDOWN_START;
    thirtySecondSent = false;
    fifteenSecondSent = false;
  }

  void update(pros::Controller& controller) {
    const std::uint32_t elapsed = pros::millis() - matchStart;
    if (elapsed >= DRIVER_PERIOD_LENGTH_MS) return;

    if (elapsed >= THIRTY_SECOND_WARNING_MS && !thirtySecondSent) {
      controller.rumble(".");
      thirtySecondSent = true;
    }

    if (elapsed >= FIFTEEN_SECOND_WARNING_MS && !fifteenSecondSent) {
      controller.rumble("..");
      fifteenSecondSent = true;
    }

    while (finalSecond > 0 &&
           elapsed >= FINAL_WARNING_START_MS +
                         (FINAL_COUNTDOWN_START - finalSecond) * 1'000) {
      controller.rumble(". ..");
      --finalSecond;
    }
  }

 private:
  static constexpr unsigned int DRIVER_PERIOD_LENGTH_MS = 90'000;
  static constexpr unsigned int THIRTY_SECOND_WARNING_MS = 60'000;
  static constexpr unsigned int FIFTEEN_SECOND_WARNING_MS = 75'000;
  static constexpr unsigned int FINAL_WARNING_START_MS = 80'000;
  static constexpr unsigned int FINAL_COUNTDOWN_START = 10;

  std::uint32_t matchStart = 0;
  unsigned int finalSecond = FINAL_COUNTDOWN_START;
  bool thirtySecondSent = false;
  bool fifteenSecondSent = false;
};

inline DriverFeedback& feedback() {
  static DriverFeedback driverFeedback;
  return driverFeedback;
}

inline void Begin(const std::uint32_t startTime = pros::millis()) {
  feedback().begin(startTime);
}

}  // namespace aon::operator_control
