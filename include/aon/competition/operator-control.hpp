#pragma once

#include <cmath>
#include "../constants.hpp"
#include "../globals.hpp"
#include "../shadow.hpp"

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
#else
size_t lastR1PressTime = 0;
size_t lastR2PressTime = 0;
const int DOUBLE_TAP_TIME = 250;
bool mergeCorridorAndElevator = true;
#endif


/// Default Operator Control configuration
inline void DriveDefault() { }

/// Kevin's Operator Control configuration
inline void DriveKevin() { 
  #if !USING_BIG_ROBOT
  //# From now on, all drivetrains used will need to use this format for driving
  double leftX = scaler.transform(mainController.get_analog(ANALOG_LEFT_X));
  double leftY = scaler.transform(mainController.get_analog(ANALOG_LEFT_Y));
  double rightX = scaler.transform(mainController.get_analog(ANALOG_RIGHT_X));
  double rightY = scaler.transform(mainController.get_analog(ANALOG_RIGHT_Y));
  drivetrain.drive(leftX, leftY, rightX, rightY, Drivetrain::SPLIT_ARCADE);
  shadow::poll(leftY, 0, rightX);

  if(mainController.get_digital_new_press(DIGITAL_R2)) {
    size_t currentTime = pros::millis();

    if(currentTime - lastR2PressTime < DOUBLE_TAP_TIME){
      toggle(mergeCorridorAndElevator);
    }

    lastR2PressTime = currentTime;
  }

  // Storing
  if(mainController.get_digital(DIGITAL_R2)) {
    if (mergeCorridorAndElevator){
      intake.store();
      shadow::event(shadow::Kind::Intake, static_cast<std::int8_t>(shadow::IntakeMode::Store));
    } else {
      intake.corridor();
      shadow::event(shadow::Kind::Intake, static_cast<std::int8_t>(shadow::IntakeMode::Corridor));
    }
  }
  // Reject
  else if(mainController.get_digital(DIGITAL_L2)) {
    intake.reject();
    shadow::event(shadow::Kind::Intake, static_cast<std::int8_t>(shadow::IntakeMode::Reject));
  }
  // Score Low
  else if(mainController.get_digital(DIGITAL_L1)) {
    intake.score(Intake::BOTTOM);
    shadow::event(shadow::Kind::Intake, static_cast<std::int8_t>(shadow::IntakeMode::Bottom));
  }

  // Lever
  if(mainController.get_digital_new_press(DIGITAL_R1)) {
    size_t currentTime = pros::millis();

    if(currentTime - lastR1PressTime < DOUBLE_TAP_TIME){
      intake.resetLever();
      shadow::event(shadow::Kind::Lever, 0);
    } else {
      intake.extendLever();
      shadow::event(shadow::Kind::Lever, 1);
    }

    lastR1PressTime = currentTime;
  } else if (intake.leverFinished()) {
    intake.resetLever();
    shadow::event(shadow::Kind::Lever, 0);
  }

  // Optional single tap
  // Lever
  // const bool pressedR1 = mainController.get_digital_new_press(DIGITAL_R1);
  // if(pressedR1 && intake.leverController->getTarget() == 0 && intake.leverController->getError() < 10){
  //   intake.leverController->setTarget(140);
  // } else if ((pressedR1 && intake.leverController->getTarget() == 140 && !intake.leverController->isSettled())
  //             || (intake.leverController->getTarget() == 140 && intake.leverController->getError() < 10)){
  //   intake.leverController->setTarget(0);
  // } 

  if(!(mainController.get_digital(DIGITAL_R2) || mainController.get_digital(DIGITAL_L2) || mainController.get_digital(DIGITAL_L1))){
    intake.corridor(0);
    intake.elevator(0);
    intake.judge(0);
    shadow::event(shadow::Kind::Intake, static_cast<std::int8_t>(shadow::IntakeMode::Idle));
  }
  
  // Change Height
  if(mainController.get_digital_new_press(DIGITAL_B)) {
    intake.toggleScorerHeight();
    shadow::toggleEvent(shadow::Kind::ScorerHeight);
  }
  // Match loaders mechanism
  else if(mainController.get_digital_new_press(DIGITAL_A)) {
    intake.toggleCart();
    shadow::toggleEvent(shadow::Kind::Cart);
  }
  else if(mainController.get_digital_new_press(DIGITAL_RIGHT)) {
    drivetrain.toggleTurbo();
  }
  else if(mainController.get_digital_new_press(DIGITAL_Y)) {
    intake.toggleTrapdoor();
    shadow::toggleEvent(shadow::Kind::Trapdoor);
  }
  else if(mainController.get_digital_new_press(DIGITAL_UP)) {
    brooks.toggle();
    shadow::toggleEvent(shadow::Kind::Brooks);
  }

  if(mainController.get_digital(DIGITAL_DOWN)) {
    arrow.deactivate();
    shadow::event(shadow::Kind::Arrow, 0);
  } else {
    arrow.activate();
    shadow::event(shadow::Kind::Arrow, 1);
  }

  #endif
}

/// Fabian's Operator Control configuration
inline void DriveFabian() {
  #if USING_BIG_ROBOT
  //# From now on, all drivetrains used will need to use this format for driving
  double leftX = scaler.transform(-mainController.get_analog(ANALOG_LEFT_X));
  double leftY = scaler.transform(-mainController.get_analog(ANALOG_LEFT_Y));
  double rightX = scaler.transform(-mainController.get_analog(ANALOG_RIGHT_X));
  double rightY = scaler.transform(-mainController.get_analog(ANALOG_RIGHT_Y));
  drivetrain.drive(leftX, leftY, rightX, rightY, Drivetrain::HOLONOMIC);
  shadow::poll(leftY, leftX, rightX);

  // Record the final intake intent, excluding commands superseded below.
  const bool unsortedScore = !sortEnabled &&
      (mainController.get_digital(DIGITAL_R1) || mainController.get_digital(DIGITAL_R2));
  if(mainController.get_digital(DIGITAL_L1)){
    intake.store();
    if (!unsortedScore)
      shadow::event(shadow::Kind::Intake, static_cast<std::int8_t>(shadow::IntakeMode::Store));
  }
  else if(mainController.get_digital(DIGITAL_L2)){
    intake.score(Intake::BOTTOM);
    if (!unsortedScore)
      shadow::event(shadow::Kind::Intake, static_cast<std::int8_t>(shadow::IntakeMode::Bottom));
  }
  else if(!sortActive){
    intake.stop();
    // Unsorted R1/R2 scoring below supersedes this transient stop in this loop.
    if (!unsortedScore)
      shadow::event(shadow::Kind::Intake, static_cast<std::int8_t>(shadow::IntakeMode::Idle));
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
        shadow::event(shadow::Kind::Sort, 1);
        sortActive = true;
      }
    }
    // R2 held — sort inverted (correct→MIDDLE, wrong→TOP)
    else if(mainController.get_digital(DIGITAL_R2)) {
      if(r2NewPress) {
        intake.setSortHeights(Intake::MIDDLE);
        intake.startReleasing();
        shadow::event(shadow::Kind::Sort, 2);
        sortActive = true;
      }
    }
    // neither held — stop sorting only if it was previously active
    else if(sortActive) {
      intake.stopReleasing();
      shadow::event(shadow::Kind::Sort, 0);
      sortActive = false;
    }
  } else {
    // Sort off — reuse scoring behavior
    if(mainController.get_digital(DIGITAL_R1)) {
      intake.score(Intake::TOP);
      shadow::event(shadow::Kind::Intake, static_cast<std::int8_t>(shadow::IntakeMode::Top));
    } else if(mainController.get_digital(DIGITAL_R2)) {
      intake.score(Intake::MIDDLE);
      shadow::event(shadow::Kind::Intake, static_cast<std::int8_t>(shadow::IntakeMode::Middle));
    }
  }

  // Change Brooks Height
  if(mainController.get_digital_new_press(DIGITAL_B)) {
    brooks.toggle();
    shadow::toggleEvent(shadow::Kind::Brooks);
  }

  else if(mainController.get_digital_new_press(DIGITAL_LEFT)) {
    sem.toggle();
    shadow::toggleEvent(shadow::Kind::Sem);
  }
  // Match loaders mechanism
  else if(mainController.get_digital_new_press(DIGITAL_UP)) {
    intake.toggleCart();
    shadow::toggleEvent(shadow::Kind::Cart);
  }

  else if(mainController.get_digital_new_press(DIGITAL_X)) {
    drivetrain.toggleTurbo();
  }
  else if(mainController.get_digital_new_press(DIGITAL_Y)) {
    sortEnabled = !sortEnabled;
    if (!sortEnabled && sortActive) {
      intake.stopReleasing();
      shadow::event(shadow::Kind::Sort, 0);
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
  switch (driver) {
    case KEVIN:
      DriveKevin();
      break;

    case FABIAN:
      DriveFabian();
      break;

    default:
      DriveDefault();
      break;
  }
}

}  // namespace aon::operator_control
