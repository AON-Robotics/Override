#pragma once

#ifndef AON_PI_COMMANDS_HPP_
#define AON_PI_COMMANDS_HPP_

/**
 * \file commands.hpp
 *
 * \brief The standard command set the Pi can send, written once against an
 * abstract `Robot`.
 *
 * Portable like protocol.hpp: the brain implements `Robot` with Override's
 * drivetrain and odometry (pi-link.cpp) and the laptop simulator implements it
 * with a kinematic model (RaspberryPi/sim). Argument checks, limits and result
 * field names live here, so both speak exactly the same protocol.
 *
 * Conventions (same as Override):
 *   - distances in inches, positive is forward
 *   - angles in degrees, positive is CLOCKWISE (IMU heading convention)
 *   - speeds in drivetrain motor RPM
 *
 * Adding a command: register it in `registerStandardCommands` (or anywhere
 * with `link.registerCommand`) and add a tool for it in
 * RaspberryPi/bridge/server/tools/.
 */

#include <string>

#include "./protocol.hpp"

namespace aon::pi {

struct RobotPose {
  double x = 0;      ///< inches
  double y = 0;      ///< inches
  double theta = 0;  ///< degrees, clockwise positive
};

/// Everything the standard commands need from the robot. Motion methods are
/// called from the worker task, may block, and must return early once
/// `requestAbort()` has been called.
class Robot {
 public:
  virtual ~Robot() = default;

  virtual std::string name() = 0;
  /// Drivetrain top speed in RPM; caps every speed the Pi asks for.
  virtual double maxRpm() = 0;
  /// "disabled", "autonomous" or "driver".
  virtual std::string mode() = 0;
  /// False, with a reason the user can act on, when the Pi must not move the
  /// robot right now (disabled, autonomous running, IMU calibrating, ...).
  virtual bool motionAllowed(std::string &why) = 0;

  virtual RobotPose pose() = 0;
  virtual double batteryPct() = 0;
  /// Signed tracking-wheel travel in inches, as odometry sees it.
  virtual void tracking(double &left, double &right, double &back) = 0;
  /// Continuous IMU rotation in degrees (clockwise positive), NAN if absent.
  virtual double imuRotation() = 0;

  // --- Motion (worker task, blocking) ---------------------------------------
  virtual void move(double inches, double rpm) = 0;
  virtual void turn(double degrees, double rpm) = 0;
  virtual void resetOdometry(double x, double y, double theta) = 0;
  /// Spins one side ('L' or 'R') forward at `rpm` for `ms`, stops, and lets
  /// the robot settle. Fills the peak velocity of each motor of that side as
  /// `p<port>=<rpm>`.
  virtual void spinSide(char side, double rpm, int ms, KV &motorPeaks) = 0;

  // --- Safety ---------------------------------------------------------------
  virtual void stopMotors() = 0;
  virtual void requestAbort() = 0;
  virtual void clearAbort() = 0;

  /// Registers this robot's sensors (odom, imu, motors, battery, ...).
  virtual void registerSensors(Link &link) = 0;
};

/// Hard limits enforced on the brain, independently of the Pi server's own
/// (tighter) limits. Defence in depth: a bug on the Pi cannot exceed these.
struct Limits {
  double maxMoveIn = 72.0;
  double maxTurnDeg = 720.0;
  double maxProbeRpm = 200.0;
  int maxProbeMs = 1500;
  /// A move is "reached" when it travels at least |target| - this.
  double moveToleranceIn = 1.0;
  /// A turn is "reached" when it rotates at least |target| - this.
  double turnToleranceDeg = 3.0;
};

/// Registers PING, STATUS, SENSORS, STOP, MOVE, TURN, PROBE and RESET_ODOM,
/// the robot's sensors, the `link` sensor, and the Pi's sensor packets:
/// `O` (vexpi OTOS pose, sensor `pi_otos`) and `R`/`N` (red_tracker, sensor
/// `pi_target`).
void registerStandardCommands(Link &link, Robot &robot, Limits limits = Limits());

/// An OTOS pose as vexpi sends it: `O,<x>,<y>,<heading>`, inches forward,
/// inches right, degrees clockwise (Override's convention).
struct OtosPose {
  double x = 0;
  double y = 0;
  double heading = 0;
};

/// Parses the fields of an `O` packet (after the tag). False if malformed.
bool parseOtos(const std::vector<std::string> &fields, OtosPose &pose);

/// The latest OTOS pose received on `link` and its age. False if none yet or
/// the last one was malformed. Treat it as stale after ~300 ms.
bool latestOtos(const Link &link, OtosPose &pose, std::uint32_t &ageMs);

}  // namespace aon::pi

#endif  // AON_PI_COMMANDS_HPP_
