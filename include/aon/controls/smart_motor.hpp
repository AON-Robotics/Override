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
 * SMART_MOTOR_GROUP
 *
 * Inherits from okapi MotorGroup class and implements moveVelocity
 * and moveVoltage methods to apply a slew rate and in turn
 * protect our motors from overheating or being damaged over
 * time from drastic changes in current.
 *
 */

class SmartMotorGroup : public pros::MotorGroup {
 private:
  pros::Mutex stateMutex;
  double current_voltage;
  double current_velocity;
  int d_voltage;
  double acceleration;
  uint64_t now = 0;
  uint64_t previous_time = 0;
  int step = 0;

 public:
  SmartMotorGroup(const std::initializer_list<std::int8_t> &ports,
                  int voltage_delta = 0, int velocity_delta = 0)
      : pros::MotorGroup(ports) {
    current_voltage = 0;
    current_velocity = 0;

    SetDVoltage(voltage_delta);
    // Acceleration has units rpm/min^2
    // By default there will be no effect on the Slew Rate Controller
    SetAcceleration(velocity_delta);

    now = previous_time = pros::micros();
  }

  /// Getters
  double GetCurrentVoltage() { std::lock_guard<pros::Mutex> lock(stateMutex); return current_voltage; }
  double GetCurrentVelocity() { std::lock_guard<pros::Mutex> lock(stateMutex); return current_velocity; }
  int GetDVoltage() { std::lock_guard<pros::Mutex> lock(stateMutex); return d_voltage; }
  double GetAcceleration() { std::lock_guard<pros::Mutex> lock(stateMutex); return acceleration; }
  uint64_t GetNow() { std::lock_guard<pros::Mutex> lock(stateMutex); return now; }

  /// Setters
  void SetCurrentVoltage(double current_vol) { std::lock_guard<pros::Mutex> lock(stateMutex); current_voltage = current_vol; }
  void SetCurrentVelocity(double current_vel) {
    std::lock_guard<pros::Mutex> lock(stateMutex);
    current_velocity = current_vel;
  }
  void SetDVoltage(int delta_voltage) { std::lock_guard<pros::Mutex> lock(stateMutex); d_voltage = std::abs(delta_voltage); }
  void SetAcceleration(double delta_velocity) {
    std::lock_guard<pros::Mutex> lock(stateMutex);
    acceleration = std::fabs(delta_velocity);
  }

  /// @brief Publish an already profiled RPM and reset the manual slew baseline.
  /// All state and the transport write share one boundary with manual commands.
  std::int32_t moveVelocityDirect(std::int16_t velocity) {
    std::lock_guard<pros::Mutex> lock(stateMutex);
    current_velocity = velocity;
    now = previous_time = pros::micros();
    return pros::MotorGroup::move_velocity(velocity);
  }

  /// @brief Move motor velocity according to slew rate calculations. Slew is in \b rev/min^2
  /// @param ivelocity – The new motor velocity in \b rpm from -+-100, +-200, or +-600 depending on the motor's gearset
  /// @returns 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
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

  /// @brief Move motor voltage according to slew rate calculations. Slew is in \b mV/s
  /// @param ivoltage – ivoltage – The new voltage value from -12000 to 12000.
  /// @returns 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
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
