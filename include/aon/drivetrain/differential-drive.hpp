#pragma once

#include "./drivetrain.hpp"

namespace aon {

class DifferentialDrive : public Drivetrain {
 protected:
  bool supportsPurePursuit() const override { return true; }

  bool commandPath(double left, double right) override;

  bool stopPathMotors() override;

 private:
  SmartMotorGroup leftMotors;
  SmartMotorGroup rightMotors;

 public:
  DifferentialDrive(const std::initializer_list<std::int8_t> &leftPorts = {0},
            const std::initializer_list<std::int8_t> &rightPorts = {0},
            Pose pose = Pose(),
            std::unique_ptr<Odometry> odometry = nullptr,
            SpeedFactors speedFactors = SpeedFactors(),
            std::unique_ptr<MotionProfile> yProfile = nullptr,
            std::unique_ptr<MotionProfile> thetaProfile = nullptr
          )
      : leftMotors(leftPorts, 0, MAX_ACCEL),
        rightMotors(rightPorts, 0, MAX_ACCEL),
        Drivetrain(pose, std::move(odometry), speedFactors, nullptr, std::move(yProfile), std::move(thetaProfile)) {
    if (this->odometry) this->odometry->setDriveMotorGroups(&leftMotors, &rightMotors);
  }

  /// @brief Drives the robot using tank control, mapping left and right inputs directly to each side of the drivetrain
  /// @param left The \b RPM to send to the left-side motors (positive is forward)
  /// @param right The \b RPM to send to the right-side motors (positive is forward)
  void tank(const double &left, const double &right) override;

  /// @brief Sets the brake mode for all motors of the drivetrain
  /// @param brakeMode The new brake mode for the drivetrain
  void setBrakeMode(pros::MotorBrake brakeMode) override;

  /// @brief Sets the gearset for all motors of the drivetrain
  /// @param gearset The new gearset for the drivetrain
  void setGearset(pros::MotorGears gearset) override;

  /// @brief Sets the units for all encoders of the motors of the drivetrain
  /// @param units The new units for the drivetrain
  void setEncoderUnits(pros::MotorEncoderUnits units) override;

  /// @brief Sets the slew rate for all motors of the drivetrain
  /// @param slew The new slew rate for the drivetrain
  void setSlewRate(double slew) override;

  /// @brief Calculates average RPM forward
  /// @return The RPM of the motors with respect to the front of the robot
  double getRPM() override;

  /// @brief Translation-only two-point followPath wrapper; discards the result.
  /// @param pose Target in inches and clockwise degrees; heading alignment is opt-in
  /// through followPath, not this wrapper. Configured odometry is required.
  void goToPose(const Pose &pose) override; // TODO: add optional `settle` boolean

};

}  // namespace aon
