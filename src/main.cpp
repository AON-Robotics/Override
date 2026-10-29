#include "../include/main.hpp"
#include <cstdio>

namespace {
// TEMPORARY hardware tests. Set false to restore the normal autonomous reader.
constexpr bool pursuitHardwareTesting = true;
// Select ONE run, then rebuild: 0=24in, 1=48in, 2=right, 3=left, 4=S-curve.
constexpr unsigned pursuitHardwareTest = 0;
constexpr double pursuitTestRPM = 120; // equivalent motor RPM, not inches/second
static_assert(pursuitHardwareTest < 5, "Select pursuit test 0 through 4");

const char* pursuitStatusName(aon::PurePursuit::Status status) {
  using Status = aon::PurePursuit::Status;
  switch (status) {
    case Status::IDLE: return "IDLE";
    case Status::RUNNING: return "RUNNING";
    case Status::SUCCESS: return "SUCCESS";
    case Status::TIMEOUT: return "TIMEOUT";
    case Status::CANCELLED: return "CANCELLED";
    case Status::INVALID_PATH: return "INVALID_PATH";
    case Status::UNSUPPORTED_CONFIGURATION: return "UNSUPPORTED_CONFIGURATION";
    case Status::CONTROL_FAILURE: return "CONTROL_FAILURE";
  }
  return "UNKNOWN";
}

#if !USING_BIG_ROBOT
// Application-lifetime inputs survive competition callback deletion. Coordinates
// are inches: +X forward from the marked origin, +Y right, theta clockwise.
static const std::vector<aon::Pose> paths[] = {
  {{0, 0, 0}, {24, 0, 0}},
  {{0, 0, 0}, {48, 0, 0}},
  {{0, 0, 0}, {8, 0, 0}, {16, 2, 0}, {23, 7, 0}, {28, 14, 0}, {30, 22, 0}},
  {{0, 0, 0}, {8, 0, 0}, {16, -2, 0}, {23, -7, 0}, {28, -14, 0}, {30, -22, 0}},
  {{0, 0, 0}, {8, 0, 0}, {16, 3, 0}, {24, 8, 0}, {32, 13, 0}, {40, 16, 0}, {48, 16, 0}}
};
static const char* const names[] = {"24in straight", "48in straight", "Right curve", "Left curve", "S-curve"};
const auto& path = paths[pursuitHardwareTest];
static const std::vector<double> caps(path.size(), pursuitTestRPM);
// Bounded capture for one 10-second run at roughly 100Hz. Print only after stop;
// these RPM values are commands, not measured motor speeds.
static aon::Drivetrain::PathSample pursuitSamples[1024];
static aon::Drivetrain::PathTrace trace{pursuitSamples, 1024};
#endif

void runPursuitHardwareTest() {
#if USING_BIG_ROBOT
  drivetrain.setPathEnabled(false);
  std::printf("PP test rejected: use the small DifferentialDrive robot.\n");
#else
  drivetrain.stop();
  if (mainController.get_digital(DIGITAL_X)) {
    drivetrain.setPathEnabled(false);
    std::printf("PP test not started: release safety X and re-enable autonomous.\n");
    return;
  }
  // Place the robot at the marked origin facing +X BEFORE enabling autonomous.
  // Keep still during IMU calibration. Resetting pose does not calibrate sensors.
  drivetrain.resetPose(0, 0, 0);
  // resetPose clears source/baselines; the 20ms sensor task needs fresh samples
  // before followPath can accept motion. Wait on readiness, not a fixed sleep.
  const uint32_t poseWaitStarted = pros::millis();
  while (!drivetrain.hasFreshPose() &&
         uint32_t(pros::millis() - poseWaitStarted) < 1000) {
    if (mainController.get_digital(DIGITAL_X)) {
      drivetrain.setPathEnabled(false);
      std::printf("PP aborted while waiting for odometry: safety X.\n");
      mainController.print(0, 0, "PP ABORTED");
      return;
    }
    pros::delay(10);
  }
  if (!drivetrain.hasFreshPose()) {
    drivetrain.setPathEnabled(false);
    std::printf("PP NOT STARTED: no fresh odometry after reset (source=%d). Check sensor task, motor ports and position sources.\n",
                drivetrain.getPoseSource());
    mainController.print(0, 0, "PP NO FRESH ODOM");
    return;
  }
  std::printf("PP START %s cap=%.1f RPM; X stops; disable to abort.\n",
              names[pursuitHardwareTest], pursuitTestRPM);
  std::printf("PP pose source=%d (1=motor encoders, 2=tracking, 3=OTOS, 4=tracking+OTOS)\n",
              drivetrain.getPoseSource());
  const auto result = drivetrain.followPath(path, 10000, caps, false, nullptr, &trace);
  drivetrain.setPathEnabled(false); // no subsequent motion until a fresh mode entry
  const auto pose = drivetrain.getPose();
  const auto& target = path.back();
  std::printf("PP RESULT %s: %s elapsed=%lu ms stop=%s maxInterval=%lu us\n",
              names[pursuitHardwareTest], pursuitStatusName(result.status),
              static_cast<unsigned long>(result.elapsedMs),
              trace.terminalStopSucceeded ? "ok" : "FAILED",
              static_cast<unsigned long>(trace.maxIntervalUs));
  std::printf("PP pose=(%.2f, %.2f, %.2f deg) target=(%.2f, %.2f) odomError=%.2f in\n",
              pose.x, pose.y, pose.theta, target.x, target.y, pose.distanceTo(target));
  std::printf("PP Measure real endpoint error with tape; no final-heading alignment.\n");
  std::printf("PP TRACE samples=%lu dropped=%lu sourceNow=%d; RPM columns are commands\n",
              static_cast<unsigned long>(trace.size),
              static_cast<unsigned long>(trace.dropped), drivetrain.getPoseSource());
  std::printf("PP t_ms,x_in,y_in,heading_deg,left_cmd_rpm,right_cmd_rpm,target_rpm,pose_in_s,progress_in,published\n");
  for (size_t i = 0; i < trace.size; ++i) {
    // About one row per second, plus the final captured observation.
    if (i % 100 != 0 && i + 1 != trace.size) continue;
    const auto& sample = trace.samples[i];
    std::printf("PP %lu,%.2f,%.2f,%.2f,%d,%d,%.2f,%.2f,%.2f,%d\n",
                static_cast<unsigned long>(uint32_t(sample.timestampUs - trace.samples[0].timestampUs) / 1000),
                sample.pose.x, sample.pose.y, sample.pose.theta,
                int(sample.leftRPM), int(sample.rightRPM), sample.diagnostics.targetRPM,
                sample.diagnostics.measuredSpeed, sample.diagnostics.progress, int(sample.published));
  }
  mainController.print(0, 0, "PP %.18s", pursuitStatusName(result.status));
  pros::delay(50);
  mainController.print(1, 0, "X %.1f Y %.1f", pose.x, pose.y);
  pros::delay(50);
  mainController.print(2, 0, "Err %.1fin %.1fs", pose.distanceTo(target), result.elapsedMs / 1000.0);
#endif
}
} // namespace

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
  if (pursuitHardwareTesting) {
    runPursuitHardwareTest();
    return; // one run per autonomous enable; reposition before enabling again
  }
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
