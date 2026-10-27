/**
 * @file s-curve-profile.hpp
 * @author Kevin Javier Gomez Guzman @kevgom018
 * @brief S-Curve motion profiling class for smooth and precise robot movements
 * based on odometry.
 * @version 1.2.1
 * @date 2026-04-20
 *
 * This class calculates target velocities using an S-curve motion profile to
 * ensure smooth acceleration and deceleration. It improves movement accuracy
 * and reduces overshoot by carefully controlling jerk and acceleration phases.
 */

#pragma once

#include <cmath>
#include <algorithm>
#include "../constants.hpp"
#include "../math/misc/misc.hpp"

namespace aon {

class MotionProfile {
 public:
  enum Stage {
    SETTLED,
    ACCELERATING,
    CRUISING,
    DECELERATING,
  };

 private:
  /// @brief Profile ceilings: RPM, RPM/s acceleration/deceleration, and RPM/s^2 jerk.
  /// Zero initialization makes a default profile deterministically invalid for
  /// path preparation instead of exposing indeterminate limits to arithmetic.
  double MAX_VELOCITY = 0, MAX_ACCELERATION = 0, MAX_DECELERATION = 0, JERK = 0;
  double currVelocity = 0;
  double currAccel = 0;
  double targetFinalVelocity = 0;
  Stage stage = SETTLED;

 public:
  MotionProfile(double MAX_VELOCITY, double MAX_ACCELERATION,
                double MAX_DECELERATION, double JERK) {
    this->MAX_VELOCITY = std::abs(MAX_VELOCITY);          // RPM
    this->MAX_ACCELERATION = std::abs(MAX_ACCELERATION);  // RPM/s
    this->MAX_DECELERATION = std::abs(MAX_DECELERATION);
    this->JERK = std::abs(JERK);  // RPM/(s^2)
  }

  /// @brief Construct an unconfigured profile with zero limits and zero motion.
  /// @note isValid() remains false until all required limits are configured;
  /// PurePursuit rejects it before attempting path motion. This is not a request
  /// for an unlimited profile or an implicit robot-specific tuning preset.
  MotionProfile(){}

  /// @brief Returns the current stage of the motion profile
  /// @return The stage of the motion profile
  Stage getStage() const { return this->stage; }

  /// @brief Read the configured motor-equivalent speed ceiling without changing state.
  /// @return Stored profile velocity limit in RPM; zero if unconfigured. The
  /// setter stores values directly, so callers must check isValid() before
  /// interpreting it as a finite positive ceiling.
  /// @note This is a configured value, not measured wheel velocity. The path
  /// controller may reduce it for curvature, heading, caps or motor resolution.
  double getMaxVelocity() const { return MAX_VELOCITY; }
  /// @brief Read the configured braking magnitude used by path speed previews.
  /// @return Deceleration magnitude in RPM/s, before the experimental 0.5 factor.
  /// @details Path braking converts this motor-equivalent rate into inches/s^2
  /// before applying the distance relation v^2 = u^2 + 2*a*s.
  double getMaxDeceleration() const { return MAX_DECELERATION; }

  /// @brief Check that all four profile limits can support path calculations.
  /// @return True only when velocity, acceleration, deceleration and jerk are
  /// each finite and strictly positive; false for zero/default, NaN or infinity.
  /// @details Checks configuration only: it neither resets motion history nor
  /// establishes physical feasibility, actuator calibration or measured traction.
  /// Constructors normalize signed limits by magnitude; this check sees those
  /// stored magnitudes. The path controller validates its geometry separately.
  bool isValid() const {
    return std::isfinite(MAX_VELOCITY) && MAX_VELOCITY > 0 &&
           std::isfinite(MAX_ACCELERATION) && MAX_ACCELERATION > 0 &&
           std::isfinite(MAX_DECELERATION) && MAX_DECELERATION > 0 &&
           std::isfinite(JERK) && JERK > 0;
  }

  // TODO(Issue 18, physical validation): measure braking/overshoot with the
  // current robot before accepting the inherited half-deceleration heuristic.
  // Plotting the profile can explain its shape but cannot validate traction or
  // establish global wheel jerk bounds after path safety limits are applied.

  /// @brief Calculates the target velocity to send to the motors for smooth and precise movements using an S-curve profile.
  /// @param remainingDist The remaining distance to the target in \b inches.
  /// @param dt The time elapsed since the last function call in \b seconds.
  /// @return The updated velocity in \b RPM.
  double update(const double& remainingDist, const double& dt = 0.02) {
    const double velocity = math::linearSpeed(this->currVelocity);
    const double finalVelocity = math::linearSpeed(this->targetFinalVelocity);
    const double deltaVSquared = (velocity * velocity) - (finalVelocity * finalVelocity);
    // Decelerate early using half the deceleration for better accuracy.
    //* Note: Without this, the system consistently overshoots by about half an inch. Needs further investigation.
    const double decel = math::linearSpeed(this->MAX_DECELERATION * 0.5);
    // Stop the profile when the target is reached.
    if (remainingDist <= 0) {
      this->currAccel = 0;
      this->currVelocity = this->targetFinalVelocity;
      if (this->targetFinalVelocity == 0){
        this->stage = SETTLED;
      } else {
        this->stage = CRUISING;
      }
    }
    // Deceleration
    else if (remainingDist <=  deltaVSquared / (2 * decel)) {
      this->currAccel = -this->MAX_DECELERATION;
      this->stage = DECELERATING;
    }
    // Decelerate if the current velocity exceeds the updated `MAX_VELOCITY`.
    else if (this->currVelocity > this->MAX_VELOCITY) {
      this->currAccel = std::min(this->currAccel - (this->JERK * dt), this->MAX_DECELERATION);
      this->stage = DECELERATING;
    }
    // Maintain constant velocity (no acceleration needed).
    else if (this->currVelocity == this->MAX_VELOCITY) {
      this->currAccel = 0;
      this->stage = CRUISING;
    }
    // Increase acceleration up to the maximum allowed value.
    else {
      this->currAccel = std::min(this->currAccel + (this->JERK * dt), this->MAX_ACCELERATION);
      this->stage = ACCELERATING;
    }

    this->currVelocity += this->currAccel * dt;
    this->currVelocity = std::clamp(this->currVelocity, 0.0, this->MAX_VELOCITY);  // Redundancy for safety
    return this->currVelocity;
  }

  /// @brief Resets the velocity and acceleraton for reusability
  void reset() {
    this->currVelocity = 0;
    this->MAX_VELOCITY = MAX_RPM;
    this->currAccel = 0;
    this->stage = SETTLED;
  }

  /// @brief Sets the velocity in case profile is not started from rest
  /// @param velocity The current velocity of the profile
  void setVelocity(const double& velocity = 0) {
    this->currVelocity = velocity;
    if (velocity == 0) { this->stage = SETTLED; }
    else { this->stage = CRUISING; }
  }

  /// @brief Sets the max velocity for the profile
  /// @param max_velocity The new max velocity for the profile
  void setMaxVelocity(const double& max_velocity = MAX_RPM) {
    this->MAX_VELOCITY = max_velocity;
  }

  /// @brief Sets the acceleration in case profile is not started from rest
  /// @param accel The current acceleration of the profile
  void setAccel(const double& accel = 0) { this->currAccel = accel; }

  /// @brief Sets the final velocity for the profile
  /// @param velocity The new final velocity
  void setFinalVelocity(const double& velocity){
    this->targetFinalVelocity = velocity;
  }
};

}
