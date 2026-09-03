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
