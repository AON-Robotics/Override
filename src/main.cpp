#include "../include/main.hpp"

void initialize() {
  aon::gui->setTestRegister([] {
    aon::gui->registerTestFunction(std::function<int()>([] { return aon::routines::PathDiagnostic(0); }), "Path: straight");
    aon::gui->registerTestFunction(std::function<int()>([] { return aon::routines::PathDiagnostic(1); }), "Path: gentle curve");
    aon::gui->registerTestFunction(std::function<int()>([] { return aon::routines::PathDiagnostic(2); }), "Path: U-turn");
  });
  aon::gui->setDataRegister([]{
  aon::gui->registerDataEntry("X",       []{ return drivetrain.getX(); });
  aon::gui->registerDataEntry("Y",       []{ return drivetrain.getY(); });
  aon::gui->registerDataEntry("Heading", []{ return drivetrain.getTheta(); });
});



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

void disabled() {}

void competition_initialize() {}

void autonomous() {
  aon::Configure(false); // Set drivetrain to hold for auton
  // TODO: add presetFunction
  aon::autonomousReader->ExecuteFunction("autonomous");
  pros::delay(10);
}

// During development
// Program slot 1 with Pizza Icon is for opcontrol
// Program slot 2 with Planet Icon is for autonomous routine
// Program slot 3 with Alien Icon is for tests or miscellaneous components
void opcontrol() {
  aon::Configure();
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
