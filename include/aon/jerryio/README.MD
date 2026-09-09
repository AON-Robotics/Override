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
- `path-transform.hpp` rebases exported field coordinates into a robot-relative
  route.
- `asset.hpp` declares assets embedded by the AON build rules.
- `path-follower.hpp` and `path-following.hpp` provide controller and runtime
  APIs.
- `path-telemetry.hpp` defines per-loop samples and constant-space run metrics.
- `path-actions.hpp` defines mechanism callbacks bound to zero-speed markers.
- `routines.hpp` exposes the team's PATH.JERRYIO autonomous routine.
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

Exported coordinates are expressed in inches, but the checked-in routine
converts them to a robot-relative frame before driving. The first point becomes
`(0, 0)`, and the first sampled travel direction becomes AON heading `0°`
(`+X`). AON uses clockwise-positive headings, so relative `+Y` is to the
robot's right at heading zero. The conversion also flips the export's lateral
axis to preserve physical left/right turns. Length, speed, and zero-speed
action markers are preserved. The final heading is derived from the transformed
final segment (about 182 degrees clockwise for the checked-in right U-turn).

This means an exported route may be drawn anywhere in PATH.JERRYIO. On the
robot, place it at the desired physical starting location and point it in the
desired initial travel direction; the routine resets odometry to `(0, 0, 0°)`.
This removes editor-origin dependence, but it does not replace correct tracking
wheel calibration or compensate automatically for severe wheel slip.

The host regression suite checks the exported right-turn motor direction and
runs the complete checked-in route through the production odometry and follower
with simulated tracking wheels, IMU, and motor slew. Odometry tests cover both
directions across the 180-degree heading boundary and arc integration from the
starting orientation. These tests catch software geometry errors; they do not
validate real sensor signs, mounting, traction, or task scheduling on the robot.
Run `tools/run-host-tests.ps1` for all tests, or pass `-Test odometry-test` for
the odometry/route simulation alone. After uploading, select Red/Blue 4 (JIO)
to test JerryIO; Red 3 (BAS) is the startup selection.

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

The first two sampled points must be distinct because they define the relative
starting direction. The last two decoded points must also be distinct because
they define the finishing heading.

## Safety

Decoding never commands hardware. The drivetrain execution layer is
responsible for validating options, observing competition-disabled and timeout
conditions, limiting motor output, and stopping both drive sides on every exit.
The follower plans speed over the whole path before motion begins: exported
speed remains the upper bound, while curvature and available braking distance
can lower that bound. This avoids entering a corner or stop point too fast.

## AON motion API

`Drivetrain::followPath()` accepts a decoded `Path` and `FollowPathOptions`.
The options control lookahead, drivetrain width, wheel diameter, motor-to-wheel
ratio, RPM ceiling, acceleration, deceleration, lateral acceleration,
terminal recovery speed, projection search window, terminal tolerance,
direction, timeout, loop period, monitored final-heading control, and an
optional cancellation callback. It returns a
`MotionResult` whose status distinguishes completion, invalid input, timeout,
competition disable, and cancellation. The result also contains summary path
metrics on successful and interrupted runs.

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

## Adaptive lookahead

Fixed lookahead remains the default. Adaptive mode increases lookahead as the
planned speed rises and reduces it where path curvature rises:

```text
speedFraction = clamp(plannedRpm / maximumRpm, 0, 1)
normalizedCurvature = abs(pathCurvature) * trackWidth
lookahead = clamp(
    baseLookahead * (1 + speedWeight * speedFraction)
        / (1 + curvatureWeight * normalizedCurvature),
    minimumDistance,
    maximumDistance)
```

Enable it explicitly after measuring suitable bounds on the robot:

```cpp
options.adaptiveLookahead.enabled = true;
options.adaptiveLookahead.minimumDistance = minimumTestedLookahead;
options.adaptiveLookahead.maximumDistance = maximumTestedLookahead;
options.adaptiveLookahead.speedWeight = testedSpeedWeight;
options.adaptiveLookahead.curvatureWeight = testedCurvatureWeight;
```

All distances are inches. Both weights are dimensionless and non-negative.
Invalid bounds or weights reject the motion before the drivetrain moves.

## Path telemetry

Every `MotionResult` includes sample count, RMS and maximum cross-track error,
endpoint error, final-heading error, saturation count, loop-overrun count, and
elapsed time. These metrics use constant memory and are available even after a
timeout, disable, or cancellation.

An optional callback exposes detailed loop samples for logging or a debug UI:

```cpp
options.telemetryEveryNLoops = 5;
options.telemetry = [&](const aon::PathTelemetrySample& sample) {
  telemetryBuffer.push(sample);  // Use a bounded, non-blocking buffer.
};
```

The callback runs synchronously inside the motion-control loop. Keep it short,
avoid blocking I/O, and increase `telemetryEveryNLoops` to reduce logging load.
Leaving the callback empty disables sample delivery without disabling summary
metrics.

For a single destination, `Drivetrain::moveToPose(target, options)` creates a
profiled AON path from the current odometry pose and aligns to `target.theta`
after reaching the target position. If the target position is already reached,
it performs only the monitored heading alignment. `turnToHeadingMonitored()`
provides that same timeout-, disable-, and cancellation-aware rotation directly.

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

`aon::jerryio::RunPathJerryIOAuton(drivetrain, actions)` runs the team's checked-in
`static/path.jerryio.txt`. It rebases the exported route, resets odometry to
`(0, 0, 0°)`, and follows the complete speed profile with a `500` RPM ceiling,
a 10-inch base lookahead, and a 60 in/s² lateral-acceleration cap. Adaptive
lookahead varies from 5 to 14 inches using speed weight 0.6 and curvature
weight 1.2. It
aligns with the final path segment and returns `1` only after successful
completion. The GUI
exposes the same wrapper as Red 4 and Blue 4 (`JerryIO Path`) and preselects
Red 4 at startup.
It accepts the drivetrain explicitly so the JerryIO module does not depend on
one robot's globals.

## Zero-speed action markers

An internal point with speed `0` is a numbered stop marker. Markers are
one-based in path order; the final zero-speed point is only the route endpoint
and is not a marker. At a marker, `followPathWithActions()` stops the drive,
runs every matching action in declaration order, waits for each duration,
always calls its cleanup callback, and then resumes the path. An unbound marker
still produces an immediate stop and resume.

The current checked-in competition path contains no internal zero-speed
markers. Red 4 and Blue 4 therefore follow it as one continuous motion-only
leg, stop at `(-66.429, -17.710)`, and do not run the intake or outtake. The
competition wrapper calls `intake.stop()` after the runner returns, including
decode and path failures.

To add mechanism work to a future export, put an internal zero-speed sample at
each desired stop and pass one `PathAction` for each marker. Action durations
count toward the same 30-second overall timeout as path motion.

```cpp
std::vector<aon::PathAction> actions{
    {1, 750,
     [&] { intake.activateScan(); },
     [&] { intake.stopScan(); }},
    {2, 0,
     [&] { intake.dropCart(); },
     {}},
};

const aon::MotionResult result =
    drivetrain.followPathWithActions(decoded.path, actions, options);
```

Use duration `0` for an instantaneous piston/state change. The one overall
`FollowPathOptions::timeoutMs` covers both driving and action waits. Disable,
cancellation, or timeout stops the drivetrain, invokes cleanup for the active
action, and returns a readable failure status. An action referencing a marker
that does not exist returns `InvalidOptions` before motion starts.

## Creating an autonomous routine

1. Create and simulate the route in PATH.JERRYIO with the settings above.
2. Put an internal speed-`0` point wherever the robot must pause for an intake,
   piston, scoring, or other mechanism action. Keep the final point at speed
   `0` when the route should settle at its endpoint.
3. Export the route into `static/` as `<routine-name>.jerryio.txt`.
4. Declare the file with `AON_JERRYIO_ASSET`, decode it, and stop immediately
   if decoding fails.
5. Convert the decoded route with `makePathRelative()` and reset AON odometry
   to `(0, 0, 0°)`.
6. Bind `PathAction` callbacks by marker ordinal and call
   `drivetrain.followPathWithActions()`.
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

  std::vector<aon::PathAction> actions{
      {1, 500,
       [&] { intake.activateScan(); },
       [&] { intake.stopScan(); }},
  };
  return drivetrain.followPathWithActions(decoded.path, actions, options)
             ? 1
             : 0;
}
```

Separate files remain useful for unrelated routes. Decode every required asset
before commanding the first movement so a malformed later file cannot leave
the robot halfway through an autonomous sequence.

## Tuning guide

- Start with `maximumRpm` between `250` and `500`, then increase it only after
  the robot tracks the geometry repeatedly.
- Increase `lookaheadDistance` for smoother, wider turns. Decrease it for
  tighter tracking, watching for steering oscillation.
- Tune adaptive lookahead bounds first, then add speed weight, then curvature
  weight. Change one value at a time and compare the returned path metrics.
- Set `trackWidth` to the effective left-to-right drive width used by AON.
- Keep `maximumAcceleration` at or below the measured no-slip limit.
- Keep `maximumDeceleration` low enough to avoid tipping or wheel slip.
- Set `driveWheelDiameter` and `motorToWheelRatio` to the real drivetrain;
  braking distances are wrong when either value is wrong.
- Start `maximumLateralAcceleration` around `40` in/s². Lower it if corners
  slide or oscillate; raise it gradually if curved sections are needlessly slow.
- Use `positionTolerance` around 1.5 to 2 inches for initial tests.
- Keep `terminalRecoveryRpm` just high enough to correct residual endpoint
  error after an exported zero-speed sample; set it to `0` only after proving
  the robot settles reliably without recovery.
- Keep `projectionWindowSegments` large enough for normal motion between
  updates but small enough that crossing or nearby return legs cannot steal
  progress from the active leg.
- Give `timeoutMs` enough margin for the exported speed profile, but never use
  an unbounded timeout.
- Set `forwards = false` only when the path coordinates progress in the travel
  direction while the robot faces 180 degrees away from the path tangent.
- Supply `finalHeading` only when heading alignment is required after the
  positional path completes. Its proportional gain, tolerance, and minimum
  turn speed are controlled by `headingKp`, `headingTolerance`, and
  `minimumTurnRpm` under the same overall timeout.

## Verification

Run the host suite after changing the decoder, follower, options, or a checked
in path:

```powershell
./tools/run-host-tests.ps1
```

Then compile the complete firmware with the PROS extension or `pros make`.
The host suite validates parsing, speed preservation, lookahead geometry,
adaptive lookahead, progress, profiling, reverse travel, normalized motor
commands, telemetry metrics, execution status policy, and the checked-in
autonomous asset.

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
