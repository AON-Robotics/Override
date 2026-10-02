#include "../../../include/aon/pi/commands.hpp"

#include <cmath>

// Portable on purpose: no PROS headers here (see protocol.hpp).

namespace aon::pi {

namespace {

struct Snapshot {
  RobotPose pose;
  double left = 0, right = 0, back = 0;
  double imu = NAN;
};

Snapshot snapshot(Robot &robot) {
  Snapshot s;
  s.pose = robot.pose();
  robot.tracking(s.left, s.right, s.back);
  s.imu = robot.imuRotation();
  return s;
}

/// Shortest signed difference b - a in degrees, in [-180, 180].
double headingDelta(double a, double b) {
  double d = std::fmod(b - a, 360.0);
  if (d > 180.0) d -= 360.0;
  if (d < -180.0) d += 360.0;
  return d;
}

/// Adds the before/after pose and the tracking/IMU deltas of a motion.
void addMotionFields(KV &kv, const Snapshot &a, const Snapshot &b) {
  kv.add("x0", a.pose.x).add("y0", a.pose.y).add("th0", a.pose.theta);
  kv.add("x1", b.pose.x).add("y1", b.pose.y).add("th1", b.pose.theta);
  kv.add("trk.left", b.left - a.left).add("trk.right", b.right - a.right).add("trk.back", b.back - a.back);
  kv.add("imu", b.imu - a.imu);
}

/// Speed argument `i` (RPM), defaulting to 30 % of max and capped at max.
bool readRpm(const Args &args, std::size_t i, Robot &robot, double &rpm, Result &error) {
  rpm = 0.3 * robot.maxRpm();
  if (args.size() > i && !args.number(i, rpm)) {
    error = Result::error("bad_args", "speed must be a number in RPM");
    return false;
  }
  if (rpm <= 0) {
    error = Result::error("bad_args", "speed must be positive");
    return false;
  }
  if (rpm > robot.maxRpm()) rpm = robot.maxRpm();
  return true;
}

bool checkAllowed(Robot &robot, Result &error) {
  std::string why;
  if (robot.motionAllowed(why)) return true;
  error = Result::error("refused", why);
  return false;
}

// Last legacy red_tracker packet, for the `pi_target` sensor. Written and read
// from the reader task only.
struct LegacyTarget {
  bool seen = false;
  bool visible = false;
  int inches = 0;
  std::uint32_t atMs = 0;
} legacyTarget;

}  // namespace

void registerStandardCommands(Link &link, Robot &robot, Limits limits) {
  // --- IMMEDIATE ------------------------------------------------------------

  link.registerCommand("PING", Kind::IMMEDIATE, 0, 0, [&link, &robot](const Args &) {
    KV kv;
    kv.add("proto", PROTOCOL_VERSION).add("robot", robot.name()).add("max_rpm", robot.maxRpm(), 0);
    kv.add("up", static_cast<unsigned long>(link.now()));
    return Result::ok(kv);
  });

  link.registerCommand("STATUS", Kind::IMMEDIATE, 0, 0, [&link, &robot](const Args &) {
    const RobotPose p = robot.pose();
    KV kv;
    kv.add("mode", robot.mode()).add("pi", link.hasControl()).add("busy", link.busyVerb());
    kv.add("x", p.x).add("y", p.y).add("th", p.theta).add("bat", robot.batteryPct(), 0);
    std::string why;
    kv.add("can_move", robot.motionAllowed(why));
    if (!why.empty()) kv.add("why", why);
    return Result::ok(kv);
  });

  link.registerCommand("SENSORS", Kind::IMMEDIATE, 0, 1, [&link](const Args &args) {
    KV kv;
    const std::string name = args.size() > 0 ? args.at(0) : "";
    if (!link.readSensors(kv, name)) {
      std::string known;
      for (const std::string &n : link.sensorNames()) known += (known.empty() ? "" : " ") + n;
      return Result::error("unknown_sensor", "no sensor named " + name + "; known: " + known);
    }
    return Result::ok(kv);
  });

  // STOP only ever stops motion the Pi started. Driver control and autons are
  // never touched by the Pi: it is an add-on, the brain stays in charge.
  link.registerCommand("STOP", Kind::IMMEDIATE, 0, 0, [&link, &robot](const Args &) {
    const bool wasMoving = link.hasControl();
    link.abort("stop");
    if (wasMoving) robot.stopMotors();
    return Result::ok(KV().add("was_moving", wasMoving));
  });

  // --- MOTION ---------------------------------------------------------------

  link.registerCommand("MOVE", Kind::MOTION, 1, 2, [&robot, limits](const Args &args) {
    double inches = 0;
    if (!args.number(0, inches)) return Result::error("bad_args", "distance must be a number in inches");
    if (std::fabs(inches) > limits.maxMoveIn) {
      return Result::error("bad_args", "distance exceeds the brain limit of " +
                                           std::to_string(static_cast<int>(limits.maxMoveIn)) + " in");
    }
    Result error;
    double rpm = 0;
    if (!readRpm(args, 1, robot, rpm, error) || !checkAllowed(robot, error)) return error;

    const Snapshot a = snapshot(robot);
    robot.move(inches, rpm);
    const Snapshot b = snapshot(robot);

    const double traveled = std::hypot(b.pose.x - a.pose.x, b.pose.y - a.pose.y);
    const bool reached = traveled >= std::fabs(inches) - limits.moveToleranceIn;
    KV kv;
    kv.add("target", inches).add("rpm", rpm, 0).add("traveled", traveled).add("reached", reached);
    addMotionFields(kv, a, b);
    return reached ? Result::ok(kv) : Result::timeout(kv);
  });

  link.registerCommand("TURN", Kind::MOTION, 1, 2, [&robot, limits](const Args &args) {
    double degrees = 0;
    if (!args.number(0, degrees)) return Result::error("bad_args", "angle must be a number in degrees");
    if (std::fabs(degrees) > limits.maxTurnDeg) {
      return Result::error("bad_args", "angle exceeds the brain limit of " +
                                           std::to_string(static_cast<int>(limits.maxTurnDeg)) + " deg");
    }
    Result error;
    double rpm = 0;
    if (!readRpm(args, 1, robot, rpm, error) || !checkAllowed(robot, error)) return error;

    const Snapshot a = snapshot(robot);
    robot.turn(degrees, rpm);
    const Snapshot b = snapshot(robot);

    // Prefer the continuous IMU rotation; fall back to the odometry heading.
    const double turned = std::isfinite(b.imu - a.imu) ? b.imu - a.imu : headingDelta(a.pose.theta, b.pose.theta);
    const bool reached = std::fabs(turned) >= std::fabs(degrees) - limits.turnToleranceDeg;
    KV kv;
    kv.add("target", degrees).add("rpm", rpm, 0).add("turned", turned).add("reached", reached);
    kv.add("drift", std::hypot(b.pose.x - a.pose.x, b.pose.y - a.pose.y));
    addMotionFields(kv, a, b);
    return reached ? Result::ok(kv) : Result::timeout(kv);
  });

  // Spins the left side alone, then the right side alone, and reports what
  // every motor, tracking wheel and the IMU did. Used by the diagnostic to
  // catch swapped ports and wrongly reversed motors or encoders.
  link.registerCommand("PROBE", Kind::MOTION, 0, 2, [&robot, limits](const Args &args) {
    double rpm = 100, ms = 500;
    if ((args.size() > 0 && !args.number(0, rpm)) || (args.size() > 1 && !args.number(1, ms))) {
      return Result::error("bad_args", "PROBE takes [rpm] [ms] as numbers");
    }
    if (rpm <= 0 || rpm > limits.maxProbeRpm || ms <= 0 || ms > limits.maxProbeMs) {
      return Result::error("bad_args", "PROBE needs 0 < rpm <= " + std::to_string(static_cast<int>(limits.maxProbeRpm)) +
                                           " and 0 < ms <= " + std::to_string(limits.maxProbeMs));
    }
    Result error;
    if (!checkAllowed(robot, error)) return error;

    KV kv;
    kv.add("rpm", rpm, 0).add("ms", static_cast<int>(ms));
    for (char side : {'L', 'R'}) {
      const Snapshot a = snapshot(robot);
      KV peaks;
      robot.spinSide(side, rpm, static_cast<int>(ms), peaks);
      const Snapshot b = snapshot(robot);
      const std::string s(1, side);
      KV sideKv;
      sideKv.add("trk.left", b.left - a.left).add("trk.right", b.right - a.right).add("trk.back", b.back - a.back);
      sideKv.add("imu", b.imu - a.imu).add("heading", headingDelta(a.pose.theta, b.pose.theta));
      sideKv.merge(peaks, "motors");
      kv.merge(sideKv, s);
    }
    return Result::ok(kv);
  });

  link.registerCommand("RESET_ODOM", Kind::MOTION, 0, 3, [&robot](const Args &args) {
    double x = 0, y = 0, theta = 0;
    if ((args.size() > 0 && !args.number(0, x)) || (args.size() > 1 && !args.number(1, y)) ||
        (args.size() > 2 && !args.number(2, theta))) {
      return Result::error("bad_args", "RESET_ODOM takes [x] [y] [theta] as numbers");
    }
    Result error;
    if (!checkAllowed(robot, error)) return error;
    robot.resetOdometry(x, y, theta);
    const RobotPose p = robot.pose();
    return Result::ok(KV().add("x", p.x).add("y", p.y).add("th", p.theta));
  });

  // --- Sensors --------------------------------------------------------------

  robot.registerSensors(link);

  link.registerSensor("link", [&link](KV &kv) {
    const Link::Stats s = link.stats();
    kv.add("rx_lines", static_cast<unsigned long>(s.rxLines))
        .add("bad_checksum", static_cast<unsigned long>(s.badChecksum))
        .add("too_long", static_cast<unsigned long>(s.tooLong))
        .add("unknown_verb", static_cast<unsigned long>(s.unknownVerb))
        .add("aborts", static_cast<unsigned long>(s.aborts))
        .add("deadman_trips", static_cast<unsigned long>(s.deadmanTrips));
  });

  // Distance packets from the Pi's red_tracker, if it is running.
  link.onLegacyPacket([&link](char tag, int value) {
    legacyTarget.seen = true;
    legacyTarget.visible = tag == 'R';
    legacyTarget.inches = value;
    legacyTarget.atMs = link.now();
  });
  link.registerSensor("pi_target", [&link](KV &kv) {
    kv.add("seen", legacyTarget.seen).add("visible", legacyTarget.visible).add("inches", legacyTarget.inches);
    kv.add("age_ms", legacyTarget.seen ? static_cast<long>(link.now() - legacyTarget.atMs) : -1L);
  });
}

}  // namespace aon::pi
