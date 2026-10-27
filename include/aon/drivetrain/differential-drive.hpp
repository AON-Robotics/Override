#pragma once

#include "./drivetrain.hpp"

namespace aon {

class DifferentialDrive : public Drivetrain {
 protected:
  /// @brief Opt this drivetrain into the validated forward differential wheel model.
  /// @return True because paired left/right wheel RPM matches PurePursuit's
  /// kinematics. Profiles, path data and odometry are still validated separately.
  bool supportsPurePursuit() const override { return true; }

  /// @brief Convert and publish one profiled wheel pair under the path owner lock.
  /// @param left Finite left wheel request in motor RPM; positive drives forward.
  /// @param right Finite right wheel request in motor RPM; positive drives forward.
  /// @return True only when both group APIs return 1; false if either fails.
  /// @pre followPath/publishPath holds pathCommandMutex, has a valid thetaProfile,
  /// and supplies the bounded forward pair produced by the active controller.
  /// @details motorCommands() preserves center, turn and lateral bounds across
  /// integer conversion; independent truncation is insufficient. Both writes are
  /// attempted even on failure. The APIs are sequential, so failure may leave a
  /// partially applied pair; the movement owner must terminate and attempt zeros.
  /// Direct group publication resets manual slew baselines without reshaping the
  /// path command. Transport acceptance does not establish actual wheel motion.
  bool commandPath(double left, double right) override {
    // Pure Pursuit has already profiled and bounded these commands. A second,
    // independent wheel slew limiter can override speed caps and curvature.
    // Conversion must preserve coupled center/turn bounds, not just each wheel.
    const auto command = PurePursuit::motorCommands(left,right,thetaProfile->getMaxVelocity());
    const auto l = leftMotors.moveVelocityDirect(command.first);
    const auto r = rightMotors.moveVelocityDirect(command.second);
    return l == 1 && r == 1;
  }

  /// @brief Attempt unslewed zero velocity on both sides of the drivetrain.
  /// @return True only if both zero requests are accepted by PROS.
  /// @pre The path command mutex is held by the stop/finish/lifecycle owner.
  /// @note Never short-circuit the second write on a first-side failure. This
  /// hook must not call tank(), stop() or cancelPath(), which would reacquire the
  /// same nonrecursive drivetrain mutex. Physical stopping remains unmeasured.
  bool stopPathMotors() override {
    // Always attempt both zeros, even when one transport reports failure.
    const auto l = leftMotors.move_velocity(0);
    const auto r = rightMotors.move_velocity(0);
    return l == 1 && r == 1;
  }

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

  /// @brief Publish caller-owned tank commands under the same wheel-pair mutex as paths.
  /// @param left The \b RPM to send to the left-side motors (positive is forward)
  /// @param right The \b RPM to send to the right-side motors (positive is forward)
  /// @details Serializes the pair with path/stop transport and uses the group's
  /// manual slew behavior. It does not check pathEnabled or cancel an active
  /// path. The caller must serialize driver/legacy motion with followPath;
  /// mutex protection alone does not decide which movement owns the robot.
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

  /// @brief Run the native two-point translation through the path movement owner.
  /// @param pose Target X/Y in AON inches and requested clockwise heading in degrees.
  /// @pre Configured odometry is required because this compatibility wrapper
  /// reads getPose() before followPath can perform freshness validation.
  /// @details Uses current pose as the first waypoint and the default 15000 ms
  /// translation budget. It waits for the same position/progress/motion settlement
  /// as a path, stops on exit and never automatically corrects final heading.
  /// The void API discards PathResult; use followPath for failure-dependent
  /// autonomous actions or Issue 33's explicitly configured heading stage.
  /// @todo Issue 33 integration: migrate remaining legacy movement callers that
  /// need explicit results/heading alignment; do not add an unconditional turn
  /// after this wrapper or use zero wheel commands as proof of arrival.
  void goToPose(const Pose &pose) override; // TODO: add optional `settle` boolean

};

}  // namespace aon
