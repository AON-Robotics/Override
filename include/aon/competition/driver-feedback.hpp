#pragma once

#include <atomic>
#include <cstdint>

#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace aon::driver_feedback {

/// Driver feedback events and their default controller rumble patterns.
enum class Event : std::uint8_t {
  GAME_OBJECT,        // "."
  MECHANISM_COMPLETE, // "-"
  SCORING_READY,      // ". ."
  ENDGAME,            // "- -"
  WARNING,            // "... "
  COUNT,
};

enum class Profile : std::uint8_t {
  FULL,
  ESSENTIAL,
  QUIET,
  DISABLED,
};

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
    lastObject = false;
    lastMechanism = false;
    lastScoringReady = false;
    lastWarning = false;
    initialized = false;
    endgameSent = false;
  }

  void setProfile(const Profile profile) {
    constexpr std::uint8_t object = 1u << static_cast<std::uint8_t>(Event::GAME_OBJECT);
    constexpr std::uint8_t mechanism = 1u << static_cast<std::uint8_t>(Event::MECHANISM_COMPLETE);
    constexpr std::uint8_t scoring = 1u << static_cast<std::uint8_t>(Event::SCORING_READY);
    constexpr std::uint8_t endgame = 1u << static_cast<std::uint8_t>(Event::ENDGAME);
    constexpr std::uint8_t warning = 1u << static_cast<std::uint8_t>(Event::WARNING);
    constexpr std::uint8_t masks[] = {
        ALL_EVENTS,
        object | mechanism | scoring | endgame | warning,
        scoring | endgame,
        0,
    };
    enabled.store(masks[static_cast<std::uint8_t>(profile)], std::memory_order_relaxed);
  }

  void update(const bool objectDetected, const bool mechanismComplete,
              const bool scoringReady, const bool warning) {
    if (!initialized) {
      lastObject = objectDetected;
      lastMechanism = mechanismComplete;
      lastScoringReady = scoringReady;
      lastWarning = warning;
      initialized = true;
      return;
    }

    if (objectDetected && !lastObject) notify(Event::GAME_OBJECT);
    if (mechanismComplete && !lastMechanism) notify(Event::MECHANISM_COMPLETE);
    if (scoringReady && !lastScoringReady) notify(Event::SCORING_READY);
    if (warning && !lastWarning) notify(Event::WARNING);

    lastObject = objectDetected;
    lastMechanism = mechanismComplete;
    lastScoringReady = scoringReady;
    lastWarning = warning;

    if (!endgameSent && pros::millis() - matchStart >= ENDGAME_MS) {
      notify(Event::ENDGAME);
      endgameSent = true;
    }
  }

  void notify(const Event event) {
    const auto index = static_cast<std::uint8_t>(event);
    if (index >= static_cast<std::uint8_t>(Event::COUNT) ||
        !(enabled.load(std::memory_order_relaxed) & (1u << index))) {
      return;
    }
    pending.store(static_cast<std::uint8_t>(index + 1), std::memory_order_release);
  }

  void setEnabled(const Event event, const bool value) {
    const auto bit = static_cast<std::uint8_t>(1u << static_cast<std::uint8_t>(event));
    if (value) {
      enabled.fetch_or(bit, std::memory_order_relaxed);
    } else {
      enabled.fetch_and(static_cast<std::uint8_t>(~bit), std::memory_order_relaxed);
    }
  }

 private:
  static constexpr std::uint32_t ENDGAME_MS = 75'000;
  static constexpr std::uint8_t ALL_EVENTS =
      (1u << static_cast<std::uint8_t>(Event::COUNT)) - 1;
  static constexpr const char* PATTERNS[] = {".", "-", ". .", "- -", "... "};

  void run() {
    while (true) {
      const auto event = pending.exchange(0, std::memory_order_acquire);
      if (event != 0) controller.rumble(PATTERNS[event - 1]);
      pros::delay(20);
    }
  }

  pros::Controller& controller;
  pros::Task* task = nullptr;
  std::atomic<std::uint8_t> enabled{ALL_EVENTS};
  std::atomic<std::uint8_t> pending{0};
  std::uint32_t matchStart = 0;
  bool lastObject = false;
  bool lastMechanism = false;
  bool lastScoringReady = false;
  bool lastWarning = false;
  bool initialized = false;
  bool endgameSent = false;
};

}  // namespace aon::driver_feedback
