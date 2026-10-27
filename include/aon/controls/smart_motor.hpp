#ifndef AON_CONTROLS_SMART_MOTOR_HPP__
#define AON_CONTROLS_SMART_MOTOR_HPP__

#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <mutex>

/**
 * SMART_MOTOR
 *
 * Inherits from okapi Motor class and implements moveVelocity
 * and moveVoltage methods to apply a slew rate and in turn
 * protect our motors from overheating or being damaged over
 * time from drastic changes in current.
 *
 */

namespace aon {

class SmartMotor : public pros::Motor {
 private:
  double current_voltage;
  // Using double because int conversions create loss of data.
  double current_velocity;
  int d_voltage;
  int acceleration;
  uint64_t now = 0;
  uint64_t previous_time = 0;
  int step = 0;

 public:
  explicit SmartMotor(std::int8_t iport, int voltage_delta = 0,
                      int velocity_delta = 0)
      : pros::Motor(iport) {
    current_voltage = 0;
    current_velocity = 0;

    SetDVoltage(voltage_delta);
    // Acceleration has units rpm^2/min^2
    // By default there will be no effect on the Slew Rate Controller
    SetAcceleration(velocity_delta);

    now = previous_time = pros::micros();
  }

  /// Getters
  double GetCurrentVoltage() { return current_voltage; }
  double GetCurrentVelocity() { return current_velocity; }
  int GetDVoltage() { return d_voltage; }
  int GetAcceleration() { return acceleration; }
  uint64_t GetNow() { return now; }

  /// Setters
  void SetCurrentVoltage(double current_vol) { current_voltage = current_vol; }
  void SetCurrentVelocity(double current_vel) {
    current_velocity = current_vel;
  }
  void SetDVoltage(int delta_voltage) { d_voltage = std::abs(delta_voltage); }
  void SetAcceleration(int delta_velocity) {
    acceleration = std::abs(delta_velocity);
  }

  /// @brief Move motor velocity according to slew rate calculations. Slew is in \b rev/min^2
  /// @param ivelocity – The new motor velocity in \b rpm from -+-100, +-200, or +-600 depending on the motor's gearset
  /// @returns 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
  std::int32_t move_velocity(std::int16_t ivelocity) {
    if (GetAcceleration() != 0) {
      now = pros::micros();

      // Kinematics equation v_f = v_o + a * Δt
      const double deltaTime = (GetNow() - previous_time) / 1E6;
      SetCurrentVelocity(std::clamp((double)ivelocity, GetCurrentVelocity() - GetAcceleration() * deltaTime,  GetCurrentVelocity() + GetAcceleration() * deltaTime));
      ivelocity = std::ceil(GetCurrentVelocity());

      previous_time = pros::micros();
    }
    return pros::Motor::move_velocity(ivelocity);
  }

  /// @brief Move motor voltage according to slew rate calculations. Slew is in \b mV/s
  /// @param ivoltage – ivoltage – The new voltage value from -12000 to 12000.
  /// @returns 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
  std::int32_t move_voltage(std::int16_t ivoltage) {
    if (GetDVoltage() != 0) {
      now = pros::micros();

      // Kinematics equation v_f = v_o + a * Δt
      const double deltaTime = (GetNow() - previous_time) / 1E6;
      SetCurrentVoltage(std::clamp((double)ivoltage, GetCurrentVoltage() - GetDVoltage() * deltaTime, GetCurrentVoltage() + GetDVoltage() * deltaTime));
      ivoltage = std::ceil(GetCurrentVoltage());

      previous_time = pros::micros();
    }
    return pros::Motor::move_voltage(ivoltage);
  }
};

/**
 * @brief Serialize a PROS motor group's software slew state and transport writes.
 * @details Manual velocity/voltage commands can limit changes using elapsed
 * seconds. Path commands instead use moveVelocityDirect(): PurePursuit already
 * limits the coupled wheel pair, so a second independent slew calculation would
 * distort curvature or raise a previously reduced speed. Velocity zero bypasses
 * manual slew so terminal path stopping cannot leave a ramped nonzero request.
 * @note Stored values are software command baselines, not encoder measurements.
 * Each operation locks only this group; a drivetrain-level command mutex is
 * still needed to serialize a left/right pair. Lock order is drivetrain first,
 * then group. Calls through inherited PROS methods do not automatically acquire
 * this wrapper's mutex and remain the caller's responsibility.
 * @warning Slew limiting is not a guarantee against overheating, slip or damage.
 * A successful API return describes transport acceptance, not physical motion.
 * @todo Issue 18 physical validation: measure real group-write latency and stop
 * response under loaded V5 scheduling; host mutex tests cannot establish these.
 */

class SmartMotorGroup : public pros::MotorGroup {
 private:
  /// @brief Protect software state and the associated PROS write as one operation.
  /// Helpers that already hold this nonrecursive mutex access fields directly;
  /// calling a locking getter/setter from those helpers would lock it twice.
  pros::Mutex stateMutex;
  double current_voltage;
  double current_velocity;
  int d_voltage;
  double acceleration;
  uint64_t now = 0;
  uint64_t previous_time = 0;
  int step = 0;

 public:
  /// @brief Configure a motor group and initialize software slew baselines to zero.
  /// @param ports Signed PROS smart ports (absolute values 1-21); negative reverses
  /// that motor. Port selection must match the reviewed physical wiring.
  /// @param voltage_delta Voltage slew magnitude in mV/s; zero disables limiting.
  /// @param velocity_delta Manual velocity slew magnitude in RPM/s; zero disables it.
  /// @details Initializes both timing observations from the microsecond clock.
  /// Construction supplies no path ownership and performs no physical calibration.
  SmartMotorGroup(const std::initializer_list<std::int8_t> &ports,
                  int voltage_delta = 0, int velocity_delta = 0)
      : pros::MotorGroup(ports) {
    current_voltage = 0;
    current_velocity = 0;

    SetDVoltage(voltage_delta);
    // deltaTime below is in seconds, so this slew magnitude is RPM/s.
    // A zero setting disables manual slew; path commands bypass it explicitly.
    SetAcceleration(velocity_delta);

    now = previous_time = pros::micros();
  }

  /// @brief Read the software voltage baseline under the group lock.
  /// @return Last maintained voltage baseline in mV, not measured terminal voltage.
  double GetCurrentVoltage() { std::lock_guard<pros::Mutex> lock(stateMutex); return current_voltage; }
  /// @brief Read the software velocity baseline under the group lock.
  /// @return Maintained signed RPM baseline, not actual motor velocity. Some
  /// unslewed manual writes bypass baseline updates; use PROS feedback for motion.
  double GetCurrentVelocity() { std::lock_guard<pros::Mutex> lock(stateMutex); return current_velocity; }
  /// @brief Read the configured manual voltage slew magnitude.
  /// @return Nonnegative mV/s; zero disables voltage slew.
  int GetDVoltage() { std::lock_guard<pros::Mutex> lock(stateMutex); return d_voltage; }
  /// @brief Read the configured manual velocity slew magnitude.
  /// @return RPM/s magnitude; zero disables manual velocity slew.
  double GetAcceleration() { std::lock_guard<pros::Mutex> lock(stateMutex); return acceleration; }
  /// @brief Read the most recently maintained software timing observation.
  /// @return Microsecond timestamp used by this wrapper, not a sensor timestamp.
  uint64_t GetNow() { std::lock_guard<pros::Mutex> lock(stateMutex); return now; }

  /// @brief Replace the voltage baseline without publishing a hardware command.
  /// @param current_vol Voltage baseline in mV, stored directly without clamping.
  /// @pre Caller supplies a finite value appropriate for its voltage command range.
  void SetCurrentVoltage(double current_vol) { std::lock_guard<pros::Mutex> lock(stateMutex); current_voltage = current_vol; }
  /// @brief Replace the velocity baseline without publishing a hardware command.
  /// @param current_vel Signed motor RPM baseline, stored without validation.
  /// @pre Caller supplies a finite value within the intended motor configuration.
  void SetCurrentVelocity(double current_vel) {
    std::lock_guard<pros::Mutex> lock(stateMutex);
    current_velocity = current_vel;
  }
  /// @brief Set manual voltage slew by magnitude under the group lock.
  /// @param delta_voltage Voltage change limit in mV/s; zero disables the limiter.
  /// @pre The signed input must have a representable absolute value.
  void SetDVoltage(int delta_voltage) { std::lock_guard<pros::Mutex> lock(stateMutex); d_voltage = std::abs(delta_voltage); }
  /// @brief Set manual velocity slew by magnitude without resetting its history.
  /// @param delta_velocity Velocity change limit in RPM/s; zero disables slew.
  /// @pre Supply a finite value. std::fabs normalizes its sign, not NaN/infinity.
  void SetAcceleration(double delta_velocity) {
    std::lock_guard<pros::Mutex> lock(stateMutex);
    acceleration = std::fabs(delta_velocity);
  }

  /// @brief Publish an already bounded integer path request without manual slew.
  /// @param velocity Signed RPM selected by the controller's paired conversion.
  /// @return PROS transport result: 1 on success or PROS_ERR on failure.
  /// @pre The caller has validated the complete wheel pair, motor limits and
  /// movement generation before entering this group; this method does not do so.
  /// @details Updates the velocity/time baseline and performs the PROS write
  /// under one lock. Later slew-enabled manual commands therefore start from the
  /// path request rather than a stale pre-path baseline. A failed write still
  /// leaves that software baseline updated; it is never physical feedback.
  /// @note The drivetrain attempts both sides even if one write fails, then
  /// reports CONTROL_FAILURE and attempts zeros. This method alone is one group.
  std::int32_t moveVelocityDirect(std::int16_t velocity) {
    std::lock_guard<pros::Mutex> lock(stateMutex);
    current_velocity = velocity;
    now = previous_time = pros::micros();
    return pros::MotorGroup::move_velocity(velocity);
  }

  /// @brief Publish a manual RPM request, or an immediate unslewed velocity stop.
  /// @param ivelocity Signed motor RPM within the selected cartridge's limits.
  /// @return 1 if PROS accepts the request, or PROS_ERR with errno on failure.
  /// @details With velocity slew enabled, nonzero requests use the configured
  /// RPM/s magnitude and elapsed seconds to clamp the software velocity change,
  /// then ceil to integer RPM. When slew is disabled, nonzero requests pass
  /// through without updating that software velocity/time baseline.
  /// Exactly zero always resets the velocity/time baseline and writes zero
  /// immediately, even when manual slew is enabled. This is essential for path
  /// timeout, cancellation and unsupported-configuration cleanup.
  /// @note Neither a zero request nor transport success proves physical rest.
  /// The group lock covers state and I/O; the drivetrain lock covers wheel pairs.
  std::int32_t move_velocity(std::int16_t ivelocity) {
    std::lock_guard<pros::Mutex> lock(stateMutex);
    // A single stop request must not leave a ramped nonzero setpoint active
    // after a path timeout, cancellation, or unsupported-configuration return.
    if (ivelocity == 0) {
      current_velocity = 0;
      now = previous_time = pros::micros();
      return pros::MotorGroup::move_velocity(0);
    }
    if (acceleration != 0) {
      now = pros::micros();

      // Kinematics equation v_f = v_o + a * Δt
      const double deltaTime = (now - previous_time) / 1E6;
      const double upperLimit = current_velocity + acceleration * deltaTime;
      const double lowerLimit = current_velocity - acceleration * deltaTime;
      current_velocity = std::clamp((double)ivelocity, lowerLimit, upperLimit);
      ivelocity = std::ceil(current_velocity);

      previous_time = pros::micros();
    }
    return pros::MotorGroup::move_velocity(ivelocity);
  }

  /// @brief Publish a voltage request while serializing shared slew state.
  /// @param ivoltage Requested voltage in mV, normally between -12000 and 12000.
  /// @return 1 if PROS accepts the request, or PROS_ERR with errno on failure.
  /// @details With voltage slew enabled, clamp the software baseline by mV/s
  /// times elapsed seconds before integer conversion. This legacy voltage path
  /// does not implement move_velocity()'s special zero-stop bypass and is not
  /// used by PurePursuit's terminal cleanup. Timing state is shared with velocity
  /// operations, so callers must deliberately manage transitions between modes.
  std::int32_t move_voltage(std::int16_t ivoltage) {
    std::lock_guard<pros::Mutex> lock(stateMutex);
    if (d_voltage != 0) {
      now = pros::micros();

      // Kinematics equation v_f = v_o + a * Δt
      const double deltaTime = (now - previous_time) / 1E6;
      current_voltage = std::clamp((double)ivoltage, current_voltage - d_voltage * deltaTime, current_voltage + d_voltage * deltaTime);
      ivoltage = std::ceil(current_voltage);

      previous_time = pros::micros();
    }
    return pros::MotorGroup::move_voltage(ivoltage);
  }
};

}  // namespace aon

#endif  // AON_CONTROLS_SMART_MOTOR_HPP__
