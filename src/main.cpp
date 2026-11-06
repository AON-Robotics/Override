#include "../include/main.hpp"

void initialize() {
  if (!drivetrain.initializeMotionService()) return;
  drivetrain.beginCompetitionMode();
  pros::Task guiLoopTask([]{aon::gui->initialize();});
  aon::logging::Initialize();
  aon::Configure(false);
  pros::Task odomTask([]{drivetrain.initialize();});
  pros::delay(3000);
  pros::Task safetyTask(aon::autonSafety);
  // pros::Task turretFollowTask([]{orbit.follow();});
  // pros::Task turretScanTask([]{orbit.scan();}); // TODO: combine this with the follow task
  pros::Task intakeScanning([]{intake.scan();});
  pros::Task intakeSorting([]{intake.sort();});
}

/// @brief Revoke the previous callback epoch and stop through durable owners.
/// Waiting holds no application locks; blocked motor I/O can delay stopping.
void disabled() {
  drivetrain.beginCompetitionMode();
  drivetrain.setPathEnabled(false);
}

void competition_initialize() {
  drivetrain.beginCompetitionMode();
  drivetrain.setPathEnabled(false);
}

/// @brief Await old execution cleanup before admitting fresh autonomous requests.
void autonomous() {
  drivetrain.beginCompetitionMode();
  drivetrain.setPathEnabled(true);
  aon::Configure(false); // Set drivetrain to hold for auton
  // TODO: add presetFunction
  aon::autonomousReader->ExecuteFunction("autonomous");
  pros::delay(10);
}

// During development
// Program slot 1 with Pizza Icon is for opcontrol
// Program slot 2 with Planet Icon is for autonomous routine
// Program slot 3 with Alien Icon is for tests or miscellaneous components
/// @brief Await old execution cleanup before driver publication.
/// Callers must serialize driver and path ownership.
void opcontrol() {
  drivetrain.beginCompetitionMode();
  drivetrain.setPathEnabled(true);
  aon::Configure();
  aon::operator_control::LoadDriverProfile(driver);
  while (true) {
    #if TESTING_AUTONOMOUS
    aon::Configure(false); // Set drivetrain to hold for auton testing

    // TODO: add presetFunction
    // aon::autonomousReader->ExecuteFunction("autonomous");

    pros::delay(5000);
    #else
    aon::operator_control::Run(driver);
    #endif
    pros::delay(10);
  }
}
