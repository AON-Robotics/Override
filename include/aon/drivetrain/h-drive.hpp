#pragma once

#include "./drivetrain.hpp"

namespace aon {

class HDrive : public Drivetrain {
 protected:
  /// @brief Attempt zero velocity on both drive sides and the strafe group.
  /// @return True only when all three PROS group writes return 1.
  /// @pre The shared drivetrain stop/path lifecycle holds pathCommandMutex.
  /// @details H-drive is not opted into PurePursuit. Rejected path requests must
  /// still stop the independent mid-wheel group, including when either drive
  /// side reports a transport error. Each write occurs before combining results.
  /// This hook avoids public motion methods to prevent recursive locking.
  bool stopPathMotors() override {
    const auto l = leftMotors.move_velocity(0);
    const auto r = rightMotors.move_velocity(0);
    const auto m = midMotors.move_velocity(0);
    return l == 1 && r == 1 && m == 1;
  }

 private:
  SmartMotorGroup leftMotors;
  SmartMotorGroup rightMotors;
  SmartMotorGroup midMotors;

 public:
  HDrive(const std::initializer_list<std::int8_t> &leftPorts = {0},
         const std::initializer_list<std::int8_t> &rightPorts = {0},
         const std::initializer_list<std::int8_t> &midPorts = {0},
         Pose pose = Pose(),
         std::unique_ptr<Odometry> odometry = nullptr,
         SpeedFactors speedFactors = SpeedFactors(),
         std::unique_ptr<MotionProfile> xProfile = nullptr,
         std::unique_ptr<MotionProfile> yProfile = nullptr,
         std::unique_ptr<MotionProfile> thetaProfile = nullptr
      )
      : leftMotors(leftPorts),
        rightMotors(rightPorts),
        midMotors(midPorts),
        Drivetrain(pose, std::move(odometry), speedFactors, std::move(xProfile), std::move(yProfile), std::move(thetaProfile)) {}

  /// @brief Configures the general settings for the motors
  /// @param brakeMode The braking paradigm we will use, usually `holding` for
  /// auton and `brake` for drivers
  /// @param gearset The gearbox the physical motors contain, they MUST be all
  /// the same
  void configure(pros::MotorBrake brakeMode, pros::MotorGears gearset, double slew) {
    Drivetrain::configure(brakeMode, gearset, slew);
    midMotors.set_brake_mode(brakeMode);
    midMotors.set_gearing(pros::MotorGears::red);
    midMotors.set_encoder_units(pros::MotorEncoderUnits::degrees);
    midMotors.tare_position();
  }

  void stop() override;

  /// @brief Moves all motors the same `rpm` to move sideways
  /// @param rpm The speed in which to move all motors in \b rpm (positive is right and negative is left)
  /// @param delay The amount of milliseconds between activation and deactivation, a delay of 0 will never deactivate the motors
  void sideways(const double &rpm = MAX_RPM, const int& delay = 0) override;

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

  /// @brief Goes to the target point
  /// @param target The intended destination using the gps coordinate system (x, y) both need to be in the range (-1.8, 1.8)
  /// @note Pose X/Y are inches; heading is clockwise degrees.
  void goToPose(const Pose& target) override;

  /// @brief Route a path request through the shared unsupported-configuration gate.
  /// @param path Ordered AON-inch poses; retained only for API compatibility here.
  /// @details H-drive holonomic path kinematics have not been validated. The base
  /// movement owner rejects the request, attempts all three group stops and
  /// records UNSUPPORTED_CONFIGURATION, or CONTROL_FAILURE if a stop fails.
  /// No path is tracked and no final-heading alignment is performed. Inspect
  /// getPathStatus(), or use followPath() to obtain the explicit PathResult.
  /// @note A concurrent request rejected before ownership does not stop its owner.
  void follow(const std::vector<Pose>& path) override;
};

}  // namespace aon
