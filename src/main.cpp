#include "../include/main.hpp"

void initialize() {
  aon::gui->setPoseReadyProvider([] { return drivetrain.hasFreshPose(); });
  aon::gui->setMapDataProvider([] { return drivetrain.getPose(); });
  aon::gui->setGraphDataProviders([] { return drivetrain.getX(); },
                                  [] { return drivetrain.getY(); });
  aon::gui->registerDataEntry("OTOS live", [] { return drivetrain.hasFreshPose() ? 1.0 : 0.0; });
  aon::gui->registerDataEntry("Pose X (in)", [] { return drivetrain.getX(); });
  aon::gui->registerDataEntry("Pose Y (in)", [] { return drivetrain.getY(); });
  aon::gui->registerDataEntry("Heading (deg)", [] { return drivetrain.getTheta(); });
  aon::gui->registerDataEntry("OTOS H (deg)", [] { return drivetrain.getOtosTheta(); });
  aon::gui->registerDataEntry("IMU heading", [] { return drivetrain.isImuFusing() ? 1.0 : 0.0; });
  aon::gui->registerResetHandler("OTOS pose", [] {
    drivetrain.resetPose(INITIAL_ODOMETRY_X, INITIAL_ODOMETRY_Y,
                         INITIAL_ODOMETRY_THETA);
  });
  aon::gui->registerTestFunction(aon::tests::otosSquareBuilder, "OTOS Square Builder");
  aon::gui->variableChanger(aon::tests::squareMove1, "Move 1 (in)");
  aon::gui->variableChanger(aon::tests::squareTurn1, "Turn 1 (deg)");
  aon::gui->variableChanger(aon::tests::squareMove2, "Move 2 (in)");
  aon::gui->variableChanger(aon::tests::squareTurn2, "Turn 2 (deg)");
  aon::gui->variableChanger(aon::tests::squareMove3, "Move 3 (in)");
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
  if (!drivetrain.hasFreshPose()) {
    drivetrain.stop();
    pros::lcd::print(0, "Auton blocked: OTOS packets missing");
    return;
  }
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
