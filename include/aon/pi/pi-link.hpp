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

}  // namespace aon::pi

#endif  // AON_PI_PI_LINK_HPP_
