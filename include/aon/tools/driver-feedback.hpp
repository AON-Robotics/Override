#pragma once

#include <cstdint>
#include <optional>

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
    cancelPending();
    matchStart = pros::millis();
    lastFinalWarning = 0;
    thirtySecondSent = false;
    fifteenSecondSent = false;
  }

  void cancel() {
    cancelPending();
  }

  void update() {
    const auto elapsed = pros::millis() - matchStart;
    if (elapsed >= DRIVER_PERIOD_LENGTH_MS) return;

    if (elapsed >= ENDGAME_START_MS && elapsed < FIFTEEN_SECOND_WARNING_MS) {
      if (!thirtySecondSent) {
        queue(1, elapsed);
        thirtySecondSent = true;
      }
    } else if (elapsed >= FIFTEEN_SECOND_WARNING_MS && elapsed < FINAL_WARNING_MS) {
      if (!fifteenSecondSent) {
        queue(2, elapsed);
        fifteenSecondSent = true;
      }
    } else if (elapsed >= FINAL_WARNING_MS &&
               elapsed - lastFinalWarning >= WARNING_INTERVAL_MS) {
      queue(3, elapsed);
      lastFinalWarning = elapsed;
    }
  }

 private:
  struct PendingEvent {
    std::uint8_t pattern;
    std::uint32_t expiresAt;
    std::uint32_t id;
  };

  static constexpr std::uint32_t DRIVER_PERIOD_LENGTH_MS = 90'000;
  static constexpr std::uint32_t ENDGAME_START_MS = 60'000;
  static constexpr std::uint32_t FIFTEEN_SECOND_WARNING_MS = 75'000;
  static constexpr std::uint32_t FINAL_WARNING_MS = 80'000;
  static constexpr std::uint32_t WARNING_INTERVAL_MS = 1'000;
  static constexpr std::uint32_t RETRY_EXPIRY_MS = 500;

  void run() {
    while (true) {
      const auto now = pros::millis();
      PendingEvent event{};
      bool shouldDeliver = false;

      pendingMutex.take();
      if (pending.has_value()) {
        if (now >= pending->expiresAt) {
          pending.reset();
        } else {
          event = *pending;
          shouldDeliver = true;
        }
      }
      pendingMutex.give();

      if (shouldDeliver) {
        const bool delivered = controller.rumble(
            event.pattern == 1 ? "." : event.pattern == 2 ? ".." : ". ..") == 1;

        pendingMutex.take();
        if (pending.has_value() && pending->id == event.id &&
            (delivered || pros::millis() >= pending->expiresAt)) {
          pending.reset();
        }
        pendingMutex.give();
      }

      pros::delay(20);
    }
  }

  void queue(const std::uint8_t pattern, const std::uint32_t elapsed) {
    pendingMutex.take();
    if (!pending.has_value()) {
      pending = PendingEvent{
          pattern, matchStart + elapsed + RETRY_EXPIRY_MS, nextEventId++};
    }
    pendingMutex.give();
  }

  void cancelPending() {
    pendingMutex.take();
    pending.reset();
    pendingMutex.give();
  }

  pros::Controller& controller;
  pros::Task* task = nullptr;
  pros::Mutex pendingMutex;
  std::optional<PendingEvent> pending;
  std::uint32_t nextEventId = 1;
  std::uint32_t matchStart = 0;
  std::uint32_t lastFinalWarning = 0;
  bool thirtySecondSent = false;
  bool fifteenSecondSent = false;
};

}  // namespace aon::driver_feedback
