/**
 * @file s-curve-profile-inches.hpp
 * @author Kevin Javier Gomez Guzman @kevgom018
 * @brief S-Curve motion profiling class for smooth and precise robot movements
 * based on odometry, computed in \b inches instead of \b RPM.
 * @version 2.0.0-inches
 * @date 2026-10-22
 *
 * Fixed version of `s-curve-profile.hpp` for issue #19. Every internal value is
 * a distance unit of the robot (in, in/s, in/s^2, in/s^3) instead of a motor unit
 * (RPM, RPM/s, RPM/s^2). The public interface still takes and returns \b RPM so
 * it is a drop-in replacement: units are converted only when values come in
 * (constructor and setters) and once when the velocity goes out (`update()`).
 *
 * Conversion used everywhere (from `math::linearSpeed`):
 *   in/s = RPM * MOTOR_TO_DRIVE_RATIO * (PI * DRIVE_WHEEL_DIAMETER) / 60
 * It is linear, so the same factor converts RPM/s -> in/s^2 and RPM/s^2 -> in/s^3.
 *
 * How it works, every call:
 *   1. Allowed velocity: the fastest velocity from which the robot can still
 *      stop (jerk-limited) in the remaining distance, minus the distance it
 *      travels before a new command takes effect (loop period + motor delay).
 *   2. Target velocity: the smaller of `MAX_VELOCITY` and the allowed velocity.
 *   3. Acceleration moves toward the target by at most `JERK * dt` per call, so
 *      acceleration never jumps (no infinite jerk) and it reaches 0 exactly when
 *      the velocity reaches the target.
 *
 * What changed from the original (see the issue #19 report):
 *   - The 0.5 deceleration correction is gone; `RESPONSE_DELAY` replaces it.
 *   - Braking, reaching cruise speed and stopping are all jerk limited.
 *   - The stopping distance includes the jerk ramps.
 *   - The profile never stalls before the target (`MIN_VELOCITY`).
 *   - Deceleration is bounded with the right sign.
 */

#pragma once

#include <cmath>
#include <algorithm>
#include "../constants.hpp"
#include "../math/misc/misc.hpp"

namespace aon {

class MotionProfileInches {
 public:
  enum Stage {
    SETTLED,
    ACCELERATING,
    CRUISING,
    DECELERATING,
  };

 private:
  /// @brief Inches per second the robot moves for each motor \b RPM
  static double inchesPerRPM() { return math::linearSpeed(1.0); }

  /// @brief Converts a motor value (RPM, RPM/s, RPM/s^2) to its robot value (in/s, in/s^2, in/s^3)
  static double toInches(const double& rpm) { return rpm * inchesPerRPM(); }

  /// @brief Converts a robot velocity in \b in/s back to motor \b RPM
  static double toRPM(const double& inchesPerSecond) { return inchesPerSecond / inchesPerRPM(); }

  double MAX_VELOCITY, MAX_ACCELERATION, MAX_DECELERATION, JERK;
  double currVelocity = 0;         // in/s
  double currAccel = 0;            // in/s^2
  double targetFinalVelocity = 0;  // in/s
  /// @brief Time between commanding a velocity and the robot reaching it, in \b seconds.
  /// @details Replaces the old 0.5 correction. 0.05 s is an estimate from the simulation;
  /// measure it on the robot (time for the measured velocity to reach 63% of a step command).
  double RESPONSE_DELAY = 0.05;
  /// @brief Slowest velocity used before the target is reached, in \b in/s, so the robot never stalls short.
  double MIN_VELOCITY = 1.0;
  /// @brief Fraction of `MAX_DECELERATION` used to plan the braking curve (0 to 1).
  /// @details The plan stays a bit below the limit so the profile has room to brake harder when
  /// the robot lags behind the command. The profile itself may still use 100% of `MAX_DECELERATION`.
  double BRAKING_MARGIN = 0.8;
  double prevTargetVelocity = -1;  // in/s, target of the previous call (-1 = first call)
  Stage stage = SETTLED;

  /// @brief Distance needed to go from velocity `v` to `finalVelocity` with jerk-limited braking, in \b inches.
  /// @details Deceleration ramps 0 -> -A -> 0 at jerk J. The velocity curve is symmetric, so the
  /// average velocity is (v + v_f) / 2 and the distance is that average times the braking time T:
  ///   dv >= A^2 / J (reaches A):  T = dv / A + A / J
  ///   dv <  A^2 / J (never A):    T = 2 * sqrt(dv / J)
  double stoppingDistance(const double& v, const double& finalVelocity) const {
    const double dv = v - finalVelocity;
    if (dv <= 0) return 0;
    const double A = this->MAX_DECELERATION * this->BRAKING_MARGIN, J = this->JERK;
    const double T = (dv >= A * A / J) ? dv / A + A / J : 2 * std::sqrt(dv / J);
    return (v + finalVelocity) / 2 * T;
  }

  /// @brief Fastest velocity from which the robot can still reach `finalVelocity` within `distance`, in \b in/s.
  /// @details Inverse of `stoppingDistance()`, found by bisection (it grows with v, 30 steps is plenty).
  double allowedVelocity(const double& distance, const double& finalVelocity) const {
    if (distance <= 0) return finalVelocity;
    if (stoppingDistance(this->MAX_VELOCITY, finalVelocity) <= distance) return this->MAX_VELOCITY;
    double low = finalVelocity, high = this->MAX_VELOCITY;
    for (int i = 0; i < 30; i++) {
      const double mid = (low + high) / 2;
      if (stoppingDistance(mid, finalVelocity) <= distance) low = mid;
      else high = mid;
    }
    return low;
  }

 public:
  /// @param MAX_VELOCITY Max velocity in \b RPM
  /// @param MAX_ACCELERATION Max acceleration in \b RPM/s
  /// @param MAX_DECELERATION Max deceleration in \b RPM/s
  /// @param JERK Max jerk in \b RPM/s^2
  MotionProfileInches(double MAX_VELOCITY, double MAX_ACCELERATION,
                      double MAX_DECELERATION, double JERK) {
    this->MAX_VELOCITY = toInches(std::abs(MAX_VELOCITY));          // in/s
    this->MAX_ACCELERATION = toInches(std::abs(MAX_ACCELERATION));  // in/s^2
    this->MAX_DECELERATION = toInches(std::abs(MAX_DECELERATION));  // in/s^2
    this->JERK = toInches(std::abs(JERK));                          // in/s^3
  }

  MotionProfileInches(){}

  /// @brief Returns the current stage of the motion profile
  /// @return The stage of the motion profile
  Stage getStage() const { return this->stage; }

  /// @brief Calculates the target velocity to send to the motors for smooth and precise movements using an S-curve profile.
  /// @param remainingDist The remaining distance to the target in \b inches.
  /// @param dt The time elapsed since the last function call in \b seconds.
  /// @return The updated velocity in \b RPM.
  double update(const double& remainingDist, const double& dt = 0.02) {
    // Stop the profile when the target is reached (uses the real distance, not the delayed one).
    if (remainingDist <= 0) {
      this->currAccel = 0;
      this->currVelocity = this->targetFinalVelocity;
      this->stage = (this->targetFinalVelocity == 0) ? SETTLED : CRUISING;
      return toRPM(this->currVelocity);
    }

    // 1. Distance left once the robot reacts: it keeps moving at v for one loop plus the motor delay.
    const double reactionDist = this->currVelocity * (dt + this->RESPONSE_DELAY);
    // If still speeding up, acceleration needs t = a / J to ramp back to 0 before braking can start.
    // During that ramp: x = v*t + a*t^2/2 - J*t^3/6 = v*t + a*t^2/3 (using J*t = a).
    const double accel = std::max(this->currAccel, 0.0);
    const double rampTime = accel / this->JERK;
    const double rampDist = this->currVelocity * rampTime + accel * rampTime * rampTime / 3;
    const double brakingDist = remainingDist - reactionDist - rampDist;

    // 2. Target velocity: as fast as allowed, but never so slow that the robot stalls before the target.
    double targetVelocity = std::min(this->MAX_VELOCITY, allowedVelocity(brakingDist, this->targetFinalVelocity));
    targetVelocity = std::max(targetVelocity, std::min(this->MIN_VELOCITY, this->MAX_VELOCITY));

    // 3. Desired acceleration = how fast the target itself is changing (feed-forward, so the velocity
    //    keeps up with a falling target while braking) + a correction for the remaining error.
    //    Reducing a to 0 at jerk J changes the velocity by a^2 / (2J), so the correction that lands
    //    exactly on the target is a = sqrt(2 * J * |error|).
    const double targetRate = (this->prevTargetVelocity < 0) ? 0 : (targetVelocity - this->prevTargetVelocity) / dt;
    this->prevTargetVelocity = targetVelocity;
    const double error = targetVelocity - this->currVelocity;
    double desiredAccel = targetRate + std::copysign(std::sqrt(2 * this->JERK * std::abs(error)), error);
    desiredAccel = std::clamp(desiredAccel, -this->MAX_DECELERATION, this->MAX_ACCELERATION);
    // Never ask for more than the acceleration that reaches the target within this call (avoids overshooting it).
    desiredAccel = std::clamp(desiredAccel, -std::abs(error) / dt, std::abs(error) / dt);

    // Jerk limit: a(t + dt) = a(t) +/- J * dt, at most.
    const double maxStep = this->JERK * dt;
    this->currAccel += std::clamp(desiredAccel - this->currAccel, -maxStep, maxStep);

    // Euler integration: v(t + dt) = v(t) + a * dt
    this->currVelocity += this->currAccel * dt;
    if (this->currVelocity <= 0) {  // Can't go backwards: stop accelerating too, so the next call starts from rest.
      this->currVelocity = 0;
      this->currAccel = std::max(this->currAccel, 0.0);
    }
    this->currVelocity = std::min(this->currVelocity, this->MAX_VELOCITY);  // Redundancy for safety

    if (this->currAccel > 1e-6) { this->stage = ACCELERATING; }
    else if (this->currAccel < -1e-6) { this->stage = DECELERATING; }
    else { this->stage = CRUISING; }
    return toRPM(this->currVelocity);  // The motors take RPM
  }

  /// @brief Resets the velocity and acceleraton for reusability
  void reset() {
    this->currVelocity = 0;
    this->MAX_VELOCITY = toInches(MAX_RPM);
    this->currAccel = 0;
    this->targetFinalVelocity = 0;
    this->prevTargetVelocity = -1;
    this->stage = SETTLED;
  }

  /// @brief Sets the velocity in case profile is not started from rest
  /// @param velocity The current velocity of the profile in \b RPM
  void setVelocity(const double& velocity = 0) {
    this->currVelocity = toInches(velocity);
    this->prevTargetVelocity = -1;  // A new move starts: no previous target
    if (velocity == 0) { this->stage = SETTLED; }
    else { this->stage = CRUISING; }
  }

  /// @brief Sets the max velocity for the profile
  /// @param max_velocity The new max velocity for the profile in \b RPM
  void setMaxVelocity(const double& max_velocity = MAX_RPM) {
    this->MAX_VELOCITY = toInches(std::abs(max_velocity));
  }

  /// @brief Sets the acceleration in case profile is not started from rest
  /// @param accel The current acceleration of the profile in \b RPM/s
  void setAccel(const double& accel = 0) { this->currAccel = toInches(accel); }

  /// @brief Sets the final velocity for the profile
  /// @param velocity The new final velocity in \b RPM
  void setFinalVelocity(const double& velocity){
    this->targetFinalVelocity = toInches(velocity);
  }

  /// @brief Sets the time the robot takes to respond to a new velocity
  /// @param seconds Measured response delay in \b seconds (0.05 by default)
  void setResponseDelay(const double& seconds = 0.05) { this->RESPONSE_DELAY = std::max(seconds, 0.0); }

  /// @brief Sets the fraction of the deceleration used to plan braking
  /// @param margin Between 0 (exclusive) and 1; 0.8 by default
  void setBrakingMargin(const double& margin = 0.8) { this->BRAKING_MARGIN = std::clamp(margin, 0.05, 1.0); }

  /// @brief Sets the slowest velocity used before reaching the target
  /// @param velocity Minimum velocity in \b RPM
  void setMinVelocity(const double& velocity) { this->MIN_VELOCITY = toInches(std::abs(velocity)); }
};

}
