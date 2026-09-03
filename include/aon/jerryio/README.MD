# AON JerryIO

This folder contains AON's native integration for paths exported by
[PATH.JERRYIO](https://path.jerryio.com/). PATH.JERRYIO is the offline path
editor; decoding, motion profiling, odometry feedback, pure pursuit, and motor
control remain part of the AON codebase.

The integration does not link or call LemLib. The supported export currently
uses PATH.JERRYIO's **LemLib v0.5** text layout only because it provides the
required sampled `x, y, speed` rows. That label describes the file grammar, not
the robot's path-following library.

## Folder responsibilities

- `path.hpp` defines the reusable AON path and speed-aware waypoint model.
- `path-jerryio.hpp` declares the strict PATH.JERRYIO decoder.
- `asset.hpp` declares assets embedded by the AON build rules.
- `path-follower.hpp` and `path-following.hpp` provide controller and runtime
  APIs.
- `routines.hpp` exposes the hardware validation routine.
- The corresponding implementations live under `src/aon/jerryio/`.

## Supported data section

Each exported point is a comma-separated row:

```text
x, y, speed
```

The numeric data section must contain at least two points and end with
`endData`. Coordinates must be finite. Speed is preserved on PATH.JERRYIO's
`0` through `127` scale. Editor configuration and control-point metadata after
`endData` are intentionally ignored by the robot decoder.

Malformed, incomplete, non-finite, and out-of-range paths are rejected as a
whole; callers never receive a partially decoded path.

## Coordinate convention

Coordinates are expressed in inches using the same field frame as AON
odometry. Robot heading is supplied by the autonomous routine rather than the
sample rows and follows the AON/VEX convention: zero degrees faces field `+Y`,
with positive angles clockwise.

## PATH.JERRYIO editor configuration

Use these settings when creating an Override path:

- Field image: `V5RC 2027 - Override`.
- Unit of length: inches.
- Coordinate system: VEX Gaming Positioning System.
- Robot type: non-holonomic for the current differential drivetrain.
- Output format: `LemLib v0.5` (text grammar only).
- Point density: no more than 2 inches between samples; smaller spacing gives
  smoother geometric lookahead at the cost of a larger asset.
- Speed range: `0` through `127`; end every stopping leg at speed `0`.

Keep the path's starting `x`, `y`, and heading beside the autonomous routine.
PATH.JERRYIO's sample rows do not contain a usable robot heading, so the
routine must reset AON odometry explicitly before starting.

## Safety

Decoding never commands hardware. The drivetrain execution layer is
responsible for validating options, observing competition-disabled and timeout
conditions, limiting motor output, and stopping both drive sides on every exit.

## AON motion API

`Drivetrain::followPath()` accepts a decoded `Path` and `FollowPathOptions`.
The options control lookahead, drivetrain width, RPM ceiling, acceleration,
deceleration, terminal tolerance, direction, timeout, loop period, optional
final heading, and an optional cancellation callback. It returns a
`MotionResult` whose status distinguishes completion, invalid input, timeout,
competition disable, and cancellation.

```cpp
const aon::PathDecodeResult decoded = aon::PathJerryIO::decode(data, size);
if (!decoded) {
  drivetrain.stop();
  return;
}

aon::FollowPathOptions options;
options.lookaheadDistance = 8.0;
options.timeoutMs = 5000;
options.finalHeading = 90.0;

const aon::MotionResult result = drivetrain.followPath(decoded.path, options);
if (!result) {
  // The drivetrain has already been stopped safely.
}
```

For a single destination, `Drivetrain::moveToPose(target, options)` creates a
profiled AON path from the current odometry pose and aligns to `target.theta`
after reaching the target position.

## Embedded path assets

Save exported files in `static/` with the suffix `.jerryio.txt`. The AON
Makefile rules convert static files to a linked asset archive, so an SD card is
not required at competition time. Declare an asset by replacing punctuation in
its filename with underscores:

```cpp
#include "aon/jerryio/asset.hpp"

AON_JERRYIO_ASSET(my_auton_first_leg_jerryio_txt);

const auto path = aon::PathJerryIO::decode(
    reinterpret_cast<const char*>(my_auton_first_leg_jerryio_txt.data),
    my_auton_first_leg_jerryio_txt.size);
```

`aon::jerryio::RunPathJerryIOValidation(drivetrain)` demonstrates the complete
AON-only workflow using `static/path-jerryio-validation.jerryio.txt`. It resets
odometry to the path's documented starting pose, follows the speed profile,
aligns its final heading, and returns `1` only after successful completion. It
accepts the drivetrain explicitly so the subsystem does not depend on one
robot's globals. It is intentionally not assigned to a match GUI slot
automatically; wrap or register it only during controlled physical validation.

## Creating an autonomous routine

1. Create and simulate the route in PATH.JERRYIO with the settings above.
2. Split the route into separate files wherever an intake, piston, scoring, or
   other mechanism action must run between movements.
3. Export each leg into `static/` as
   `<routine-name>-<leg-name>.jerryio.txt`.
4. Declare each file with `AON_JERRYIO_ASSET`, decode it, and stop immediately
   if decoding fails.
5. Reset AON odometry to the first leg's documented starting pose.
6. Follow each leg with `drivetrain.followPath()` and check its `MotionResult`
   before starting the next movement or mechanism action.
7. Stop all drivetrain and mechanism outputs when any required phase fails.

An autonomous wrapper can keep the same convenient shape as the former
LemLib-facing routines while using only AON functions:

```cpp
AON_JERRYIO_ASSET(red_left_to_goal_jerryio_txt);

int RedLeftPathAuton() {
  const auto decoded = aon::PathJerryIO::decode(
      reinterpret_cast<const char*>(red_left_to_goal_jerryio_txt.data),
      red_left_to_goal_jerryio_txt.size);
  if (!decoded) {
    drivetrain.stop();
    return 0;
  }

  drivetrain.resetPose(-60.0, -36.0, 90.0);

  aon::FollowPathOptions options;
  options.lookaheadDistance = 8.0;
  options.timeoutMs = 6000;
  options.positionTolerance = 1.5;
  options.finalHeading = 270.0;

  if (!drivetrain.followPath(decoded.path, options)) return 0;

  intake.move();
  pros::delay(500);
  intake.stop();
  return 1;
}
```

For multiple legs, decode every required asset before commanding the first
movement. This prevents a malformed later file from leaving the robot halfway
through an autonomous sequence.

## Tuning guide

- Start with `maximumRpm` between `100` and `200`, then increase it only after
  the robot tracks the geometry repeatedly.
- Increase `lookaheadDistance` for smoother, wider turns. Decrease it for
  tighter tracking, watching for steering oscillation.
- Set `trackWidth` to the effective left-to-right drive width used by AON.
- Keep `maximumAcceleration` at or below the measured no-slip limit.
- Keep `maximumDeceleration` low enough to avoid tipping or wheel slip.
- Use `positionTolerance` around 1.5 to 2 inches for initial tests.
- Give `timeoutMs` enough margin for the exported speed profile, but never use
  an unbounded timeout.
- Set `forwards = false` only when the path coordinates progress in the travel
  direction while the robot faces 180 degrees away from the path tangent.
- Supply `finalHeading` only when heading alignment is required after the
  positional path completes.

## Verification

Run the host suite after changing the decoder, follower, options, or a checked
in path:

```powershell
./tools/run-host-tests.ps1
```

Then compile the complete firmware with the PROS extension or `pros make`.
The host suite validates parsing, speed preservation, lookahead geometry,
progress, profiling, reverse travel, normalized motor commands, execution
status policy, and the checked-in validation asset.

Before enabling a new routine in the match GUI:

1. Put the robot on blocks and confirm forward/reverse motor signs.
2. Confirm odometry `x`, `y`, and heading change in the documented directions.
3. Run at a low `maximumRpm` with a spotter ready to disable the robot.
4. Validate one path leg at a time, including its timeout and final heading.
5. Repeat each leg at least five times from the marked starting pose.
6. Add mechanism actions only after every required motion leg is repeatable.
7. Run the complete routine from the competition switch before assigning its
   GUI slot.

Software tests cannot certify wheel diameter, tracking offsets, motor reversal,
field placement, or traction. Those values must be validated on the physical
robot before match use.
