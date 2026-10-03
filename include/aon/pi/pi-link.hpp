#pragma once

#ifndef AON_PI_PI_LINK_HPP_
#define AON_PI_PI_LINK_HPP_

/**
 * \file pi-link.hpp
 *
 * \brief Brain side of the Raspberry Pi link (LLM debugging bridge).
 *
 * The Pi sends commands over the brain's USB user port; this module answers
 * them using Override's own drivetrain and odometry. The Pi is an add-on:
 *   - With no Pi attached nothing changes: `hasControl()` stays false and no
 *     heartbeat is printed.
 *   - The Pi can only move the robot during driver control, never while
 *     disabled or in autonomous.
 *   - Touching a joystick, pressing X (`aon::autonSafety`) or losing the Pi
 *     for 1 s aborts the Pi's motion and hands control back to the driver.
 *
 * It is the ONLY reader of stdin. The Pi's other programs (vexpi's OTOS
 * stream, red_tracker) share the same USB port, and their packets reach the
 * rest of Override through this module (`latestOtosPose`, the `pi_target`
 * sensor). Do not add another fgetc(stdin) loop: two readers split the bytes
 * between them and both see garbage.
 *
 * Protocol: include/aon/pi/protocol.hpp and RaspberryPi/docs/serial-protocol.md
 */

#include "../drivetrain/drivetrain.hpp"
#include "../odometry/odometry.hpp"
#include "pros/misc.hpp"

namespace aon::pi {

/// Starts the reader, worker and heartbeat tasks. Call once from
/// `initialize()`, after odometry is running. Safe with no Pi attached.
/// @param drivetrain The drivetrain the Pi commands move
/// @param sensors An odometry object on the same ports, used for raw sensor reads
/// @param controller The driver's controller, for joystick takeover and messages
void start(Drivetrain &drivetrain, Odometry &sensors, pros::Controller &controller);

/// True while a Pi motion is queued or running. `opcontrol()` must not drive
/// while this is true.
bool hasControl();

/// Call from `opcontrol()` while `hasControl()`: aborts the Pi motion as soon
/// as the driver moves a joystick.
void checkDriverOverride();

/// Aborts the Pi motion, if any. Safe to call any time, from any task.
void abort(const char *reason);

/// How old an OTOS pose can be and still count as current (vexpi sends 50 Hz).
constexpr std::uint32_t OTOS_TIMEOUT_MS = 300;

/// The latest OTOS pose from the Pi's vexpi program, for odometry fusion.
/// @param pose Inches forward, inches right, degrees clockwise (OTOS frame,
///             zeroed when vexpi calibrates)
/// @param ageMs How long ago it arrived; stale after OTOS_TIMEOUT_MS
/// @return false if start() was not called or no valid pose has arrived yet
/// @note Safe from any task. This replaces a separate stdin reader for OTOS.
bool latestOtosPose(Pose &pose, std::uint32_t &ageMs);

}  // namespace aon::pi

#endif  // AON_PI_PI_LINK_HPP_
