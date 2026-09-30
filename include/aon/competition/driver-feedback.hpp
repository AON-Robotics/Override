#pragma once

#include <atomic>
#include <cstdint>

#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace aon::driver_feedback {

/// Non-blocking controller rumble scheduler.
class Rumble {
 public:
  explicit Rumble(pros::Controller& controller) : controller(controller) {}

  void start() {
    if (task != nullptr) return;
    task = new pros::Task([this] { run(); }, "Driver rumble");
  }

  void beginMatch() {
    matchStart = pros::millis();
    lastFinalWarning = 0;
    thirtySecondSent = false;
    fifteenSecondSent = false;
  }

  void update() {
    const auto elapsed = pros::millis() - matchStart;
    if (elapsed >= MATCH_LENGTH_MS) return;

    if (elapsed >= ENDGAME_START_MS && elapsed < FINAL_WARNING_MS) {
      if (!thirtySecondSent) {
        queue(1);
        thirtySecondSent = true;
      }
      if (elapsed >= FIFTEEN_SECOND_WARNING_MS && !fifteenSecondSent) {
        queue(2);
        fifteenSecondSent = true;
      }
    } else if (elapsed >= FINAL_WARNING_MS && elapsed - lastFinalWarning >= WARNING_INTERVAL_MS) {
      queue(3);
      lastFinalWarning = elapsed;
    }
  }

 private:
  static constexpr std::uint32_t MATCH_LENGTH_MS = 120'000;
  static constexpr std::uint32_t ENDGAME_START_MS = 90'000;
  static constexpr std::uint32_t FIFTEEN_SECOND_WARNING_MS = 105'000;
  static constexpr std::uint32_t FINAL_WARNING_MS = 110'000;
  static constexpr std::uint32_t WARNING_INTERVAL_MS = 1'000;

  void run() {
    while (true) {
      const auto event = pending.exchange(0, std::memory_order_acquire);
      if (event != 0) controller.rumble(event == 1 ? "." : event == 2 ? ".." : ". ..");
      pros::delay(20);
    }
  }

  void queue(const std::uint8_t pattern) {
    pending.store(pattern, std::memory_order_release);
  }

  pros::Controller& controller;
  pros::Task* task = nullptr;
  std::atomic<std::uint8_t> pending{0};
  std::uint32_t matchStart = 0;
  std::uint32_t lastFinalWarning = 0;
  bool thirtySecondSent = false;
  bool fifteenSecondSent = false;
};

}  // namespace aon::driver_feedback
