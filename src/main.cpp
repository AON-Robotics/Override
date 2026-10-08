#include "../include/main.hpp"
#include "../include/aon/shadow.hpp"

void initialize() {
  pros::Task guiLoopTask([]{aon::gui->initialize();});
  aon::logging::Initialize();
  aon::Configure(false);
  pros::Task odomTask([]{drivetrain.initialize();});
  pros::delay(3000);
  // pros::Task turretFollowTask([]{orbit.follow();});
  // pros::Task turretScanTask([]{orbit.scan();}); // TODO: combine this with the follow task
  static pros::Task intakeScanning([]{intake.scan();});
  static pros::Task intakeSorting([]{intake.sort();});
  #if USING_BIG_ROBOT
  aon::shadow::bind(drivetrain, intake, mainController, brooks, sem, intakeScanning, intakeSorting);
  #else
  aon::shadow::bind(drivetrain, intake, mainController, brooks, arrow, intakeScanning, intakeSorting);
  #endif
  aon::autonomousReader->AddFunction("shadow", aon::shadow::runSelected);
  // Reuse the existing safety task as the sole Shadow playback command owner;
  // X keeps its existing STOP behavior outside Shadow playback.
  pros::Task safetyTask([]{
    while (true) {
      aon::shadow::safetyPoll();
      if (mainController.get_digital(DIGITAL_X)) aon::STOP();
      pros::delay(10);
    }
  });
}

void disabled() {
  aon::shadow::cancel();
  while (pros::competition::is_disabled()) {
    aon::shadow::controls();
    pros::delay(20);
  }
}

void competition_initialize() {}

void autonomous() {
  aon::Configure(false); // Set drivetrain to hold for auton
  // TODO: add presetFunction
  const bool shadowSelected = aon::shadow::selected();
  if (!shadowSelected) aon::shadow::prepareNativeControl();
  aon::autonomousReader->ExecuteFunction(shadowSelected ? "shadow" : "autonomous");
  pros::delay(10);
}

// During development
// Program slot 1 with Pizza Icon is for opcontrol
// Program slot 2 with Planet Icon is for autonomous routine
// Program slot 3 with Alien Icon is for tests or miscellaneous components
void opcontrol() {
  aon::Configure();
  aon::shadow::enterDriverControl();
  while (true) {
    #if TESTING_AUTONOMOUS
    aon::Configure(false); // Set drivetrain to hold for auton testing

    // TODO: add presetFunction
    // aon::autonomousReader->ExecuteFunction("autonomous");

    pros::delay(5000);
    #else
    if (!aon::shadow::controls()) aon::operator_control::Run(driver);
    #endif
    pros::delay(10);
  }
}
