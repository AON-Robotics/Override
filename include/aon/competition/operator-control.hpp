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

#if USING_BIG_ROBOT
bool sortActive = false;
bool sortEnabled = true;
#endif


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

  // Change Height
  if(mainController.get_digital_new_press(DIGITAL_B)) {
    intake.toggleScorerHeight();
  }
  // Match loaders mechanism
  else if(mainController.get_digital_new_press(DIGITAL_A)) {
    intake.toggleCart();
  }
  else if(mainController.get_digital_new_press(DIGITAL_RIGHT)) {
    drivetrain.toggleTurbo();
  }
  else if(mainController.get_digital_new_press(DIGITAL_Y)) {
    intake.toggleTrapdoor();
  }
  else if(mainController.get_digital_new_press(DIGITAL_UP)) {
    brooks.toggle();
  }

  if(mainController.get_digital(DIGITAL_DOWN)) {
    arrow.deactivate();
  } else {
    arrow.activate();
  }

  #endif
}

/// Driver2's Operator Control configuration
inline void DriveDRIVER2() {
  #if USING_BIG_ROBOT
  //# From now on, all drivetrains used will need to use this format for driving
  double leftX = scaler -> scale(-mainController.get_analog(ANALOG_LEFT_X));
  double leftY = scaler -> scale(-mainController.get_analog(ANALOG_LEFT_Y));
  double rightX = scaler -> scale(-mainController.get_analog(ANALOG_RIGHT_X));
  double rightY = scaler -> scale(-mainController.get_analog(ANALOG_RIGHT_Y));
  drivetrain.drive(leftX, leftY, rightX, rightY, Drivetrain::HOLONOMIC);

  if(mainController.get_digital(DIGITAL_L1)){
    intake.store();
  }
  else if(mainController.get_digital(DIGITAL_L2)){
    intake.score(Intake::BOTTOM);
  }
  else if(!sortActive){
    intake.stop();
  }

  // Evaluate new_press unconditionally so internal state resets on release
  bool r1NewPress = mainController.get_digital_new_press(DIGITAL_R1);
  bool r2NewPress = mainController.get_digital_new_press(DIGITAL_R2);

  if (sortEnabled) {
    // R1 held — sort normally (correct→TOP, wrong→MIDDLE)
    if(mainController.get_digital(DIGITAL_R1)) {
      if(r1NewPress) {
        intake.setSortHeights(Intake::TOP);
        intake.startReleasing();
        sortActive = true;
      }
    }
    // R2 held — sort inverted (correct→MIDDLE, wrong→TOP)
    else if(mainController.get_digital(DIGITAL_R2)) {
      if(r2NewPress) {
        intake.setSortHeights(Intake::MIDDLE);
        intake.startReleasing();
        sortActive = true;
      }
    }
    // neither held — stop sorting only if it was previously active
    else if(sortActive) {
      intake.stopReleasing();
      sortActive = false;
    }
  } else {
    // Sort off — reuse scoring behavior
    if(mainController.get_digital(DIGITAL_R1)) {
      intake.score(Intake::TOP);
    } else if(mainController.get_digital(DIGITAL_R2)) {
      intake.score(Intake::MIDDLE);
    }
  }

  // Change Brooks Height
  if(mainController.get_digital_new_press(DIGITAL_B)) {
    brooks.toggle();
  }

  else if(mainController.get_digital_new_press(DIGITAL_LEFT)) {
    sem.toggle();
  }
  // Match loaders mechanism
  else if(mainController.get_digital_new_press(DIGITAL_UP)) {
    intake.toggleCart();
  }

  else if(mainController.get_digital_new_press(DIGITAL_X)) {
    drivetrain.toggleTurbo();
  }
  else if(mainController.get_digital_new_press(DIGITAL_Y)) {
    sortEnabled = !sortEnabled;
    if (!sortEnabled && sortActive) {
      intake.stopReleasing();
      sortActive = false;
    }
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
