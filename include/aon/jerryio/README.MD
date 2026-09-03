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

The complete export and autonomous-call workflow will be documented here as
the public follower API is completed.
