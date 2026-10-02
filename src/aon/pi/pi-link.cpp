#include "../../../include/aon/pi/pi-link.hpp"

#include <cmath>
#include <cstdio>
#include <memory>

#include "../../../include/aon/pi/commands.hpp"
#include "pros/apix.h"

namespace aon::pi {

namespace {

/// Joystick value (out of 127) above which the driver takes control back.
constexpr int DRIVER_OVERRIDE_DEADBAND = 20;

/// `Robot` implemented with Override's drivetrain and odometry.
class OverrideRobot : public Robot {
 public:
  OverrideRobot(Drivetrain &drivetrain, Odometry &sensors) : drivetrain(drivetrain), sensors(sensors) {}

  std::string name() override { return USING_BIG_ROBOT ? "big_robot" : "small_robot"; }

  double maxRpm() override { return MAX_RPM; }

  std::string mode() override {
    if (pros::competition::is_disabled()) return "disabled";
    if (pros::competition::is_autonomous()) return "autonomous";
    return "driver";
  }

  bool motionAllowed(std::string &why) override {
    if (pros::competition::is_disabled()) {
      why = "robot is disabled; enable it from the field controller or competition switch";
      return false;
    }
    if (pros::competition::is_autonomous()) {
      why = "autonomous is running; the Pi can only move the robot during driver control";
      return false;
    }
    if (sensors.gyroscope.is_calibrating()) {
      why = "IMU is still calibrating; wait a few seconds and try again";
      return false;
    }
    return true;
  }

  RobotPose pose() override { return {drivetrain.getX(), drivetrain.getY(), drivetrain.getTheta()}; }

  double batteryPct() override { return pros::battery::get_capacity(); }

  void tracking(double &left, double &right, double &back) override { sensors.trackingDistances(left, right, back); }

  double imuRotation() override {
    const double rotation = sensors.gyroscope.get_rotation();
    return std::isfinite(rotation) ? rotation : NAN;
  }

  void move(double inches, double rpm) override {
    double linear, angular;
    drivetrain.getMaxVelocities(linear, angular);
    drivetrain.setMaxVelocities(rpm, angular);
    drivetrain.move(inches);
    drivetrain.setMaxVelocities(linear, angular);
  }

  void turn(double degrees, double rpm) override {
    double linear, angular;
    drivetrain.getMaxVelocities(linear, angular);
    drivetrain.setMaxVelocities(linear, rpm);
    drivetrain.turn(degrees);
    drivetrain.setMaxVelocities(linear, angular);
  }

  void resetOdometry(double x, double y, double theta) override { drivetrain.resetPose(x, y, theta); }

  void spinSide(char side, double rpm, int ms, KV &motorPeaks) override {
    pros::MotorGroup *group = nullptr;
    for (auto &entry : drivetrain.motorGroups()) {
      if (entry.first[0] == side) group = entry.second;
    }

    std::vector<double> peaks;
    const std::uint32_t start = pros::millis();
    while (pros::millis() - start < static_cast<std::uint32_t>(ms) && !drivetrain.isAbortRequested()) {
      drivetrain.tank(side == 'L' ? rpm : 0, side == 'R' ? rpm : 0);
      if (group != nullptr) {
        const std::vector<double> velocities = group->get_actual_velocity_all();
        peaks.resize(velocities.size(), 0.0);
        for (std::size_t i = 0; i < velocities.size(); i++) {
          if (std::isfinite(velocities[i]) && std::fabs(velocities[i]) > std::fabs(peaks[i])) peaks[i] = velocities[i];
        }
      }
      pros::delay(10);
    }
    drivetrain.stop();

    // Let the robot coast to a stop so the next side starts from rest.
    for (int i = 0; i < 30 && !drivetrain.isAbortRequested(); i++) pros::delay(10);

    if (group != nullptr) {
      const std::vector<std::int8_t> ports = group->get_port_all();
      for (std::size_t i = 0; i < ports.size() && i < peaks.size(); i++) {
        motorPeaks.add("p" + std::to_string(std::abs(ports[i])), peaks[i], 0);
      }
    }
  }

  void stopMotors() override { drivetrain.stop(); }
  void requestAbort() override { drivetrain.requestAbort(); }
  void clearAbort() override { drivetrain.clearAbort(); }

  void registerSensors(Link &link) override {
    link.registerSensor("odom", [this](KV &kv) {
      const RobotPose p = pose();
      double left, right, back;
      tracking(left, right, back);
      kv.add("x", p.x).add("y", p.y).add("th", p.theta);
      kv.add("trk.left", left).add("trk.right", right).add("trk.back", back);
    });

    link.registerSensor("tracking", [this](KV &kv) {
      kv.add("left.installed", sensors.encoderLeft.is_installed());
      kv.add("right.installed", sensors.encoderRight.is_installed());
      kv.add("back.installed", sensors.encoderBack.is_installed());
      kv.add("left.port", static_cast<int>(sensors.encoderLeft.get_port()));
      kv.add("right.port", static_cast<int>(sensors.encoderRight.get_port()));
      kv.add("back.port", static_cast<int>(sensors.encoderBack.get_port()));
      kv.add("wheel_diameter", TRACKING_WHEEL_DIAMETER, 3);
      kv.add("offset.left", DISTANCE_LEFT_TRACKING_WHEEL_CENTER, 3);
      kv.add("offset.right", DISTANCE_RIGHT_TRACKING_WHEEL_CENTER, 3);
    });

    link.registerSensor("imu", [this](KV &kv) {
      kv.add("installed", sensors.gyroscope.is_installed());
      kv.add("port", static_cast<int>(sensors.gyroscope.get_port()));
      kv.add("calibrating", sensors.gyroscope.is_calibrating());
      kv.add("heading", sensors.gyroscope.get_heading());
      kv.add("rotation", imuRotation());
    });

    link.registerSensor("motors", [this](KV &kv) {
      for (auto &entry : drivetrain.motorGroups()) {
        pros::MotorGroup &group = *entry.second;
        const std::vector<std::int8_t> ports = group.get_port_all();
        const std::vector<double> temps = group.get_temperature_all();
        const std::vector<std::int32_t> currents = group.get_current_draw_all();
        const std::vector<double> velocities = group.get_actual_velocity_all();
        const std::vector<std::int32_t> volts = group.get_voltage_all();
        const std::vector<std::int32_t> overTemp = group.is_over_temp_all();
        const std::vector<std::int32_t> overCurrent = group.is_over_current_all();
        for (std::size_t i = 0; i < ports.size(); i++) {
          const int port = std::abs(ports[i]);
          const std::string key = std::string(entry.first) + ".p" + std::to_string(port);
          const bool connected = pros::Device::get_plugged_type(port) == pros::DeviceType::motor;
          kv.add(key + ".connected", connected).add(key + ".reversed", ports[i] < 0);
          if (!connected) continue;
          if (i < temps.size()) kv.add(key + ".temp", temps[i], 0);
          if (i < currents.size()) kv.add(key + ".ma", static_cast<int>(currents[i]));
          if (i < velocities.size()) kv.add(key + ".rpm", velocities[i], 0);
          if (i < volts.size()) kv.add(key + ".mv", static_cast<int>(volts[i]));
          if (i < overTemp.size()) kv.add(key + ".over_temp", overTemp[i] == 1);
          if (i < overCurrent.size()) kv.add(key + ".over_current", overCurrent[i] == 1);
        }
      }
    });

    link.registerSensor("battery", [](KV &kv) {
      kv.add("pct", pros::battery::get_capacity(), 0);
      kv.add("mv", static_cast<int>(pros::battery::get_voltage()));
      kv.add("ma", static_cast<int>(pros::battery::get_current()));
      kv.add("temp", pros::battery::get_temperature(), 0);
    });

    link.registerSensor("competition", [this](KV &kv) {
      kv.add("mode", mode()).add("connected", pros::competition::is_connected() != 0);
    });

    // --- Add your own sensors here -------------------------------------------
    // Each one shows up in the Pi's read_sensors tool with no Pi-side change.
    //
    // static pros::Distance frontDistance(5);
    // link.registerSensor("front_distance", [](KV &kv) {
    //   kv.add("mm", static_cast<int>(frontDistance.get_distance()));
    //   kv.add("confidence", static_cast<int>(frontDistance.get_confidence()));
    // });
  }

 private:
  Drivetrain &drivetrain;
  Odometry &sensors;
};

pros::Mutex linkMutex;
pros::Mutex writeMutex;
pros::Controller *driverController = nullptr;
std::unique_ptr<OverrideRobot> robot;
std::unique_ptr<Link> piLink;

void writeLine(const std::string &line) {
  // Never block a robot task on the USB port: drop the line instead.
  if (!writeMutex.take(20)) return;
  std::fputs(line.c_str(), stdout);
  std::fflush(stdout);
  writeMutex.give();
}

void logLine(const std::string &message) {
  writeLine(message + "\n");  // not starting with '@': the Pi keeps it as console output
  if (driverController != nullptr) driverController->set_text(2, 0, (message.substr(0, 18) + "   ").c_str());
}

}  // namespace

void start(Drivetrain &drivetrain, Odometry &sensors, pros::Controller &controller) {
  if (piLink) return;

  // Plain newline-terminated lines on stdout instead of PROS' COBS-framed
  // streams, so the Pi can read them. `pros terminal` still shows them.
  pros::c::serctl(SERCTL_DISABLE_COBS, nullptr);
  pros::c::fdctl(fileno(stdout), SERCTL_NOBLKWRITE, nullptr);

  driverController = &controller;
  robot = std::make_unique<OverrideRobot>(drivetrain, sensors);

  Link::Hooks hooks;
  hooks.millis = [] { return pros::millis(); };
  hooks.writeLine = writeLine;
  hooks.lock = [] { linkMutex.take(); };
  hooks.unlock = [] { linkMutex.give(); };
  hooks.log = logLine;
  hooks.abortMotion = [] {
    robot->requestAbort();
    robot->stopMotors();
  };
  hooks.clearAbort = [] { robot->clearAbort(); };
  hooks.heartbeatFields = [](KV &kv) {
    const RobotPose p = robot->pose();
    kv.add("mode", robot->mode()).add("x", p.x).add("y", p.y).add("th", p.theta);
    kv.add("bat", robot->batteryPct(), 0);
  };
  piLink = std::make_unique<Link>(hooks);
  registerStandardCommands(*piLink, *robot);

  // Reader: lower priority than the robot's own tasks, blocks on stdin.
  pros::Task reader([] {
    while (true) {
      const int c = std::fgetc(stdin);
      if (c == EOF) {
        std::clearerr(stdin);
        pros::delay(5);
        continue;
      }
      piLink->feed(static_cast<char>(c));
    }
  }, TASK_PRIORITY_DEFAULT - 1, TASK_STACK_DEPTH_DEFAULT, "pi_reader");

  // Worker: runs one Pi motion at a time with Override's motion functions.
  pros::Task worker([] {
    while (true) {
      if (!piLink->runPendingMotion()) pros::delay(10);
    }
  }, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "pi_worker");

  // Ticker: heartbeat to the Pi and the deadman that aborts on link loss.
  pros::Task ticker([] {
    while (true) {
      piLink->tick();
      pros::delay(20);
    }
  }, TASK_PRIORITY_DEFAULT - 1, TASK_STACK_DEPTH_DEFAULT, "pi_ticker");
}

bool hasControl() { return piLink && piLink->hasControl(); }

void checkDriverOverride() {
  if (!hasControl() || driverController == nullptr) return;
  for (pros::controller_analog_e_t axis : {pros::E_CONTROLLER_ANALOG_LEFT_X, pros::E_CONTROLLER_ANALOG_LEFT_Y,
                                              pros::E_CONTROLLER_ANALOG_RIGHT_X, pros::E_CONTROLLER_ANALOG_RIGHT_Y}) {
    if (std::abs(driverController->get_analog(axis)) > DRIVER_OVERRIDE_DEADBAND) {
      piLink->abort("driver_override");
      return;
    }
  }
}

void abort(const char *reason) {
  if (piLink) piLink->abort(reason);
}

}  // namespace aon::pi
