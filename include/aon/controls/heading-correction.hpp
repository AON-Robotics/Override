#pragma once

// Post-movement heading correction ("Pose Lock").
// Hardware-free core: everything that touches the robot goes through an `Io`
// object, so the logic can be unit-tested on a laptop.

#include <algorithm>   // pid.hpp uses std::clamp but does not include this itself
#include <cmath>
#include <cstdint>
#include "./pid/pid.hpp"

namespace aon {

/// Outcome of the translational movement that precedes a correction.
enum class MovementStatus {
  SUCCESS,    ///< Arrived within position tolerance
  TIMEOUT,    ///< Timer expired before arriving
  CANCELLED,  ///< Cancelled / autonomous disabled
  STALLED     ///< Controller stopped (e.g. pure pursuit returned {0,0}) outside tolerance
};

enum class HeadingResult {
  SUCCESS,         ///< Within tolerance for the full settling period
  SKIPPED,         ///< Previous movement did not succeed; no correction attempted
  TIMEOUT,         ///< Did not settle in time
  CANCELLED,       ///< Cancelled / autonomous disabled
  INVALID_SENSOR,  ///< Heading unavailable (NaN/inf) at start or for too many samples
  INVALID_PARAMS   ///< Bad target heading or bad configuration
};

inline const char* toString(HeadingResult r) {
  switch (r) {
    case HeadingResult::SUCCESS: return "SUCCESS";
    case HeadingResult::SKIPPED: return "SKIPPED";
    case HeadingResult::TIMEOUT: return "TIMEOUT";
    case HeadingResult::CANCELLED: return "CANCELLED";
    case HeadingResult::INVALID_SENSOR: return "INVALID_SENSOR";
    case HeadingResult::INVALID_PARAMS: return "INVALID_PARAMS";
  }
  return "?";
}

/// There are deliberately NO default gains: the zero-initialised fields fail
/// validation, so every user must choose tuned values explicitly.
struct HeadingCorrectionConfig {
  double kP = 0;             ///< Output fraction per degree of error (output 1.0 == maxTurnRpm)
  double kI = 0;
  double kD = 0;
  double maxTurnRpm = 0;     ///< Output scale AND cap, in RPM
  double toleranceDeg = 0;   ///< |error| at or below this counts as "on target"
  std::uint32_t settleMs = 0;   ///< Must stay on target this long to succeed
  std::uint32_t timeoutMs = 0;  ///< Hard limit for the whole correction
  std::uint32_t loopMs = 20;    ///< Loop period; also the PID's time step (kT). Odometry publishes heading every 20 ms, so a faster loop re-reads stale values and makes the D term jumpy
  std::uint32_t invalidSampleLimit = 3;  ///< Consecutive bad heading samples tolerated
  double integralZoneDeg = INFINITY;     ///< PID start_integral
  double integralLimit = INFINITY;       ///< PID anti_windup
};

inline bool isValid(const HeadingCorrectionConfig& c) {
  return std::isfinite(c.kP) && c.kP > 0 && std::isfinite(c.kI) && c.kI >= 0 &&
         std::isfinite(c.kD) && c.kD >= 0 && std::isfinite(c.maxTurnRpm) &&
         c.maxTurnRpm > 0 && std::isfinite(c.toleranceDeg) && c.toleranceDeg > 0 &&
         c.toleranceDeg < 180 && c.timeoutMs > 0 && c.loopMs >= 1 && c.loopMs <= 100 &&
         c.invalidSampleLimit >= 1 && !std::isnan(c.integralZoneDeg) &&
         c.integralZoneDeg >= 0 && !std::isnan(c.integralLimit) && c.integralLimit >= 0;
}

/// Shortest signed angle from `current` to `target`, in (-180, 180].
/// Positive means clockwise. Works for any input range.
inline double shortestAngleError(double target, double current) {
  double e = std::fmod(target - current, 360.0);
  if (e > 180.0) e -= 360.0;
  else if (e <= -180.0) e += 360.0;
  return e;
}

/// Tracks "continuously within tolerance for settleMs".
class SettleTracker {
  double tol;
  std::uint32_t settleMs;
  bool inTol = false;
  std::uint32_t since = 0;
 public:
  SettleTracker(double toleranceDeg, std::uint32_t settleMs)
      : tol(toleranceDeg), settleMs(settleMs) {}
  void reset() { inTol = false; }
  /// @return true once the error has stayed within tolerance for settleMs
  bool update(double errorDeg, std::uint32_t nowMs) {
    if (std::fabs(errorDeg) > tol) { inTol = false; return false; }
    if (!inTol) { inTol = true; since = nowMs; }
    return static_cast<std::uint32_t>(nowMs - since) >= settleMs;
  }
};

namespace detail {
template <class Io>
HeadingResult runCorrection(double target, const HeadingCorrectionConfig& cfg, Io& io) {
  PID pid(cfg.kP, cfg.kI, cfg.kD, cfg.loopMs / 1000.0, cfg.integralZoneDeg, cfg.integralLimit);
  SettleTracker settle(cfg.toleranceDeg, cfg.settleMs);
  const std::uint32_t start = io.nowMs();
  std::uint32_t bad = 0;
  bool primed = false;

  while (true) {
    if (io.cancelled()) return HeadingResult::CANCELLED;
    const std::uint32_t now = io.nowMs();
    const double heading = io.headingDeg();

    if (!std::isfinite(heading)) {
      if (!primed || ++bad >= cfg.invalidSampleLimit) return HeadingResult::INVALID_SENSOR;
      io.rotate(0.0);  // never drive on a bad reading
      if (static_cast<std::uint32_t>(now - start) >= cfg.timeoutMs) return HeadingResult::TIMEOUT;
      io.delayMs(cfg.loopMs);
      continue;
    }
    bad = 0;

    const double error = shortestAngleError(target, heading);
    if (settle.update(error, now)) return HeadingResult::SUCCESS;
    if (static_cast<std::uint32_t>(now - start) >= cfg.timeoutMs) return HeadingResult::TIMEOUT;

    // PID::Output's first call computes derivative against last_error = 0 (a large
    // "kick"). Burn one call so the first real command has a sane derivative.
    if (!primed) { pid.Output(error, 0.0); primed = true; }
    const double out = pid.Output(error, 0.0);  // setpoint = wrapped error, pv = 0

    // Inside tolerance: command nothing (no needless correction), keep settling.
    const double rpm = std::fabs(error) <= cfg.toleranceDeg
                           ? 0.0
                           : std::clamp(out, -1.0, 1.0) * cfg.maxTurnRpm;
    io.rotate(rpm);
    io.delayMs(cfg.loopMs);
  }
}
}  // namespace detail

/// Io must provide:
///   uint32_t nowMs();  double headingDeg();  void rotate(double rpm);
///   void stop();  bool cancelled();  void delayMs(uint32_t ms);
/// Every exit path calls io.stop() exactly once.
template <class Io>
HeadingResult correctHeading(double targetDeg, const HeadingCorrectionConfig& cfg, Io& io,
                             MovementStatus prior = MovementStatus::SUCCESS) {
  HeadingResult result;
  if (prior != MovementStatus::SUCCESS) result = HeadingResult::SKIPPED;
  else if (!std::isfinite(targetDeg) || !isValid(cfg)) result = HeadingResult::INVALID_PARAMS;
  else result = detail::runCorrection(targetDeg, cfg, io);
  io.stop();
  return result;
}

}  // namespace aon
