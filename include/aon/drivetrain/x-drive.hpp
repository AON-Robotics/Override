#pragma once

#include "./drivetrain.hpp"

namespace aon {

class XDrive : public Drivetrain {
 protected:
  /// @brief Attempt zero velocity on all four X-drive wheel groups.
  /// @return True only when all four PROS writes return 1, after every attempt.
  /// @pre The drivetrain stop/path lifecycle holds pathCommandMutex.
  /// @details Path following is not supported for this wheel model. Rejection
  /// still needs an all-wheel stop rather than a differential two-group stop;
  /// API errors on one group must not prevent attempts on the others.
  bool stopPathMotors() override {
    const auto fl = frontLeftMotors.move_velocity(0);
    const auto fr = frontRightMotors.move_velocity(0);
    const auto bl = backLeftMotors.move_velocity(0);
    const auto br = backRightMotors.move_velocity(0);
    return fl == 1 && fr == 1 && bl == 1 && br == 1;
  }

 private:
  SmartMotorGroup frontLeftMotors;
  SmartMotorGroup frontRightMotors;
  SmartMotorGroup backLeftMotors;
  SmartMotorGroup backRightMotors;

 public:
  XDrive(const std::initializer_list<std::int8_t> &FLPorts = {0},
         const std::initializer_list<std::int8_t> &FRPorts = {0},
         const std::initializer_list<std::int8_t> &BLPorts = {0},
         const std::initializer_list<std::int8_t> &BRPorts = {0},
         Pose pose = Pose(),
         std::unique_ptr<Odometry> odometry = nullptr,
         SpeedFactors speedFactors = SpeedFactors(),
         std::unique_ptr<MotionProfile> xProfile = nullptr,
         std::unique_ptr<MotionProfile> yProfile = nullptr,
         std::unique_ptr<MotionProfile> thetaProfile = nullptr
        )
      : frontLeftMotors(FLPorts),
        frontRightMotors(FRPorts),
        backLeftMotors(BLPorts),
        backRightMotors(BRPorts),
        Drivetrain(pose, std::move(odometry), speedFactors, std::move(xProfile), std::move(yProfile), std::move(thetaProfile)) {}

  /// @brief Moves all motors the same `rpm` to move sideways
  /// @param rpm The speed in which to move all motors in \b rpm (positive is right and negative is left)
  /// @param delay The amount of milliseconds between activation and deactivation, a delay of 0 will never deactivate the motors
  void sideways(const double &rpm = MAX_RPM, const int& delay = 0) override;

  /// @brief Drives the robot using tank control, mapping left and right inputs directly to each side of the drivetrain
  /// @param left The \b RPM to send to the left-side motors (positive is forward)
  /// @param right The \b RPM to send to the right-side motors (positive is forward)
  void tank(const double &left, const double &right) override;

  /// @brief Drives a holonomic (e.g. mecanum or X-drive) robot with independent forward, sideways, and rotational control
  /// @param forward The \b RPM to send to all motors for linear forward/backward movement (positive is forward)
  /// @param sideways The \b RPM to send to all motors for lateral strafe movement (positive is rightward)
  /// @param turn The \b RPM to send to all motors for rotational movement (positive is clockwise)
  void holonomic(const double &forward, const double &sideways, const double &turn) override;

  /// @brief Takes a `direction` vector and converts it into a command for the motors.
  /// @param direction The direction with respect to the robot to move in
  /// @return A `Vector` whose `x` component is the command for the diagonal that goes from bottom left to top right and whose `y` is the command for the other diagonal
  /// @note See https://understandinglinearalgebra.org/sec-bases.html to understand the conversion between bases
  /// @details The basis B is formed by the crossed wheels (in 45º and 135º angles with respect to the horizontal)
  static Vector translateToMotorCommand(Vector direction);

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

  /// @brief Reject X-drive paths through the common movement owner and stop hooks.
  /// @param path Ordered AON-inch poses, retained for source compatibility only.
  /// @details X-drive does not opt into the validated forward differential
  /// kinematics. An admitted request stops all four groups and records
  /// UNSUPPORTED_CONFIGURATION, or CONTROL_FAILURE on stop failure. It performs
  /// neither path translation nor a final-heading turn. followPath() returns
  /// the explicit result; getPathStatus() exposes this void wrapper's outcome.
  /// @note Overlapping rejected requests never stop the already active owner.
  void follow(const std::vector<Pose>& path) override;
};

}  // namespace aon
