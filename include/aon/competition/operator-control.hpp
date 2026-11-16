#pragma once

#include <cmath>
#include "../constants.hpp"
#include "../globals.hpp"
#include "./driver-feedback.hpp"

/// @brief Encapsulates functions and state for operator control.
/// @details Practically uses Singleton design pattern, but classes would have
///          made it more complicated for beginners to understand. Also makes extensive
///          use of USING_BIG_ROBOT global constant and preprocessor directives to
///          make switching between robots not require separate branches, which could make
///          fixes and updates to one branch not apply to the other.
namespace aon::operator_control {


// ============================================================================
//    ___      _
//   |   \ _ _(_)_ _____ _ _ ___
//   | |) | '_| \ V / -_) '_(_-<
//   |___/|_| |_|\_/\___|_| /__/
//
// ============================================================================


/// Default Operator Control configuration
inline void DriveDefault() { }

#if !USING_BIG_ROBOT
/// R1 runs the intake forward; R2 reverses it; release both to stop.
inline void RunIntake() {
  if (mainController.get_digital(DIGITAL_R1)) {
    intake.corridor(INTAKE_VELOCITY);
  } else if (mainController.get_digital(DIGITAL_R2)) {
    intake.corridor(-INTAKE_VELOCITY);
  } else {
    intake.corridor(0);
  }
}

/// Each mechanism runs while its button is held and brakes when released.
inline void RunLiftAndClaw() {
  if (mainController.get_digital(DIGITAL_L1)) {
    intake.lift(127);
  } else if (mainController.get_digital(DIGITAL_L2)) {
    intake.lift(-127);
  } else {
    intake.lift(0);
  }

  if (mainController.get_digital(DIGITAL_UP)) {
    intake.claw(127);
  } else if (mainController.get_digital(DIGITAL_DOWN)) {
    intake.claw(-127);
  } else {
    intake.claw(0);
  }

  if (mainController.get_digital(DIGITAL_X)) {
    intake.clawArm(-127);
  } else if (mainController.get_digital(DIGITAL_B)) {
    intake.clawArm(127);
  } else {
    intake.clawArm(0);
  }
}
#endif

/// Driver1's Operator Control configuration
inline void DriveDRIVER1() { 
  #if !USING_BIG_ROBOT
  //# From now on, all drivetrains used will need to use this format for driving
  double leftX = scaler -> scale(mainController.get_analog(ANALOG_LEFT_X));
  double leftY = scaler -> scale(mainController.get_analog(ANALOG_LEFT_Y));
  double rightX = scaler -> scale(mainController.get_analog(ANALOG_RIGHT_X));
  double rightY = scaler -> scale(mainController.get_analog(ANALOG_RIGHT_Y));
  drivetrain.drive(leftX, leftY, rightX, rightY, Drivetrain::HOLONOMIC);

  RunIntake();
  RunLiftAndClaw();

  if(mainController.get_digital_new_press(DIGITAL_RIGHT)) {
    drivetrain.toggleTurbo();
  }

  #endif
}

/// Driver2's Operator Control configuration
inline void DriveDRIVER2() {
  #if USING_BIG_ROBOT
  // Left stick: forward/backward and strafe. Right stick X: turn.
  double leftX = scaler -> scale(mainController.get_analog(ANALOG_LEFT_X));
  double leftY = scaler -> scale(mainController.get_analog(ANALOG_LEFT_Y));
  double rightX = scaler -> scale(mainController.get_analog(ANALOG_RIGHT_X));
  drivetrain.drive(leftX, leftY, rightX, 0, Drivetrain::HOLONOMIC);

  if (mainController.get_digital(DIGITAL_L1)) {
    intake.lift(127);
  } else if (mainController.get_digital(DIGITAL_L2)) {
    intake.lift(-127);
  } else {
    intake.lift(0);
  }

  // Both intake-to-claw motors run together, independently of the lift.
  if (mainController.get_digital(DIGITAL_R1)) {
    intake.move(INTAKE_VELOCITY);
  } else if (mainController.get_digital(DIGITAL_R2)) {
    intake.move(-INTAKE_VELOCITY);
  } else {
    intake.move(0);
  }

  if (mainController.get_digital(DIGITAL_X)) {
    intake.roller(INTAKE_VELOCITY);
  } else if (mainController.get_digital(DIGITAL_B)) {
    intake.roller(-INTAKE_VELOCITY);
  } else {
    intake.roller(0);
  }

  if (mainController.get_digital_new_press(DIGITAL_RIGHT)) {
    drivetrain.toggleTurbo();
  }
  #endif
}

// ============================================================================
//    __  __      _        ___             _   _
//   |  \/  |__ _(_)_ _   | __|  _ _ _  __| |_(_)___ _ _
//   | |\/| / _` | | ' \  | _| || | ' \/ _|  _| / _ \ ' \
//   |_|  |_\__,_|_|_||_| |_| \_,_|_||_\__|\__|_\___/_||_|
//
// ============================================================================

/// @brief Main function for operator control.
/// @details Control configurations for the different drivers are manipulated here.
/// @param driver the name of the person driving the robot
/// @see aon::operator_control::Driver
inline void Run(const Driver driver) {
  // Layout depends on the robot; `driver` only selects the scaling profile
  // (applied once via LoadDriverProfile in opcontrol).
  #if USING_BIG_ROBOT
  DriveDRIVER2();
  #else
  DriveDRIVER1();
  #endif
}

}  // namespace aon::operator_control
