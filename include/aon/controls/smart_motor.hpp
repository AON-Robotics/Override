#ifndef AON_CONTROLS_SMART_MOTOR_HPP__
#define AON_CONTROLS_SMART_MOTOR_HPP__

#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <atomic>
#include <cerrno>

/**
 * SMART_MOTOR
 *
 * Inherits from PROS Motor class and implements moveVelocity
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
    // Acceleration has units RPM/s
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

  /// @brief Move motor velocity according to slew rate calculations. Slew is in \b RPM/s
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
 * SMART_MOTOR_GROUP
 *
 * Inherits from PROS MotorGroup class and implements moveVelocity
 * and moveVoltage methods to apply a slew rate and in turn
 * protect our motors from overheating or being damaged over
 * time from drastic changes in current.
 *
 */

class SmartMotorGroup : public pros::MotorGroup {
 private:
  // A competition callback can be deleted during a write. No callback-owned
  // lock may block the worker's stop. Motion writers are serialized by the caller.
  static_assert(std::atomic<float>::is_always_lock_free &&
                std::atomic<int>::is_always_lock_free &&
                std::atomic<uint32_t>::is_always_lock_free,
                "Slew state must not retain locks after callback deletion");
  // Float precision is below the integer RPM/mV transport resolution.
  std::atomic<float> current_voltage{0}, current_velocity{0}, acceleration{0};
  std::atomic<int> d_voltage{0};
  std::atomic<uint32_t> now{0}, previous_time{0};
  int step = 0;

 public:
  SmartMotorGroup(const std::initializer_list<std::int8_t> &ports,
                  int voltage_delta = 0, int velocity_delta = 0)
      : pros::MotorGroup(ports) {
    current_voltage = 0;
    current_velocity = 0;

    SetDVoltage(voltage_delta);
    // Acceleration has units RPM/s
    // By default there will be no effect on the Slew Rate Controller
    SetAcceleration(velocity_delta);

    now = previous_time = pros::micros();
  }

  /// Getters
  double GetCurrentVoltage() { return current_voltage; }
  double GetCurrentVelocity() { return current_velocity; }
  int GetDVoltage() { return d_voltage; }
  double GetAcceleration() { return acceleration; }
  uint64_t GetNow() { return now; }

  /// Setters
  void SetCurrentVoltage(double current_vol) { current_voltage = current_vol; }
  void SetCurrentVelocity(double current_vel) {
    current_velocity = current_vel;
  }
  void SetDVoltage(int delta_voltage) { d_voltage = std::abs(delta_voltage); }
  void SetAcceleration(double delta_velocity) {
    acceleration = std::fabs(delta_velocity);
  }

  /// @brief Publish an already profiled RPM and reset the manual slew baseline.
  std::int32_t moveVelocityDirect(std::int16_t velocity) {
    const auto count = size();
    if (count <= 0) { errno = EDOM; return PROS_ERR; }
    int error = 0;
    // MotorGroup reports only its first motor's status. Attempt every port,
    // preserving signed ports so reversed motors retain their configuration.
    for (uint8_t i = 0; i < count; ++i) {
      if (pros::c::motor_move_velocity(get_port(i), velocity) != 1 && error == 0)
        error = errno ? errno : EIO;
    }
    if (error) { errno = error; return PROS_ERR; }
    current_velocity = velocity;
    now = previous_time = pros::micros();
    return 1;
  }

  /// @brief Move motor velocity according to slew rate calculations. Slew is in \b RPM/s
  /// @param ivelocity – The new motor velocity in \b rpm from -+-100, +-200, or +-600 depending on the motor's gearset
  /// @returns 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
  std::int32_t move_velocity(std::int16_t ivelocity) {
    if (acceleration != 0) {
      now = pros::micros();

      // Kinematics equation v_f = v_o + a * Δt
      const double deltaTime = uint32_t(now.load() - previous_time.load()) / 1E6;
      const double velocity = current_velocity.load(), slew = acceleration.load();
      const double upperLimit = velocity + slew * deltaTime;
      const double lowerLimit = velocity - slew * deltaTime;
      current_velocity = std::clamp((double)ivelocity, lowerLimit, upperLimit);
      ivelocity = std::ceil(current_velocity);

      previous_time = pros::micros();
    }
    return pros::MotorGroup::move_velocity(ivelocity);
  }

  /// @brief Move motor voltage according to slew rate calculations. Slew is in \b mV/s
  /// @param ivoltage – ivoltage – The new voltage value from -12000 to 12000.
  /// @returns 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
  std::int32_t move_voltage(std::int16_t ivoltage) {
    if (d_voltage != 0) {
      now = pros::micros();

      // Kinematics equation v_f = v_o + a * Δt
      const double deltaTime = uint32_t(now.load() - previous_time.load()) / 1E6;
      const double voltage = current_voltage.load(), slew = d_voltage.load();
      current_voltage = std::clamp((double)ivoltage, voltage - slew * deltaTime, voltage + slew * deltaTime);
      ivoltage = std::ceil(current_voltage);

      previous_time = pros::micros();
    }
    return pros::MotorGroup::move_voltage(ivoltage);
  }
};

}  // namespace aon

#endif  // AON_CONTROLS_SMART_MOTOR_HPP__
