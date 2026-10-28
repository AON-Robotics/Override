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

/**
 * @brief Revoke path admission and attempt a drivetrain stop on competition disable.
 * @details beginCompetitionMode invalidates the preceding callback epoch and
 * waits without holding application locks. Persistent owners finish stopping
 * and path cleanup even if PROS deletes this callback on another transition.
 * A later enable permits a fresh request; it cannot revive the old generation.
 * Blocking motor I/O still prevents a hard stop-latency guarantee.
 * @note This path gate does not prohibit raw driver/legacy motor commands.
 * @todo Issue 18 physical validation: verify actual PROS callback scheduling,
 * disable/re-enable interleavings and stopping latency with the configured robot.
 */
void disabled() {
  drivetrain.beginCompetitionMode();
  drivetrain.setPathEnabled(false);
}

void competition_initialize() {
  drivetrain.beginCompetitionMode();
  drivetrain.setPathEnabled(false);
}

/**
 * @brief Admit fresh path requests before executing the selected autonomous entry.
 * @details First acknowledges old execution/publication cleanup, then reopens
 * the path lifecycle after disabled(); cancellation
 * generations still prevent an old request from publishing new commands. The
 * selected routine must check each movement result before advancing to actions.
 * This callback does not enable optional telemetry or tune heading correction.
 * @todo Hardware setup: confirm actual globals.hpp port assignments and pose
 * sources before robot trials. The current zero/'z' mechanism placeholders do
 * not configure working devices, and disabled tracking ports change localization.
 */
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
/**
 * @brief Enter operator control with fresh-request path admission available.
 * @details The mode barrier completes old path cleanup before driver publication.
 * Admission allows deliberate development path requests during this
 * mode; it does not grant concurrent ownership to paths and joystick commands.
 * The caller must serialize them. A previous cancelled request stays cancelled
 * even though new requests are now permitted. Normal driver setup follows.
 */
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
