# Using a static PATH.JERRYIO export with existing AON

## Current baseline

The separate JerryIO decoder, coordinate transform, follower, execution loop,
actions, telemetry, embedding rules, and GUI routine have been removed.
`static/path.jerryio.txt` is preserved unchanged as input data; it is not
currently embedded, uploaded, or executed by this firmware. Red 3 runs the
working ordinary `move(33)`, `driveAngleOfArc(8.5, 180)`, `move(33)` sequence.
The tracking-sensor reversal and tested odometry arc/heading-wrap fixes remain.

## What was duplicated

| Existing AON | Removed parallel implementation |
| --- | --- |
| `controls/pure-pursuit.hpp`: `PurePursuit::follow()` / `go()` | JerryIO `PathFollower` |
| `drivetrain/drivetrain.hpp`: `Drivetrain::follow(vector<Pose>)` | `followPath()` and its execution loop |
| `controls/s-curve-profile.hpp`: `MotionProfile` | Separate acceleration/deceleration and speed-planning logic |
| Existing autonomous routines and mechanism commands | Marker-action routines and callback execution layer |
| Existing odometry and drivetrain | Not duplicated; both controllers consumed these |

The new implementation did not call the old follower at the same time. This
was architectural duplication, not evidence that two followers fought over
the motors. The physical success of move/arc/move also does not validate the
existing `Drivetrain::follow()` or prove that all sensor behavior is correct.

## Smallest proposed connection

Use one build-time converter and the existing AON motion stack:

`static/path.jerryio.txt -> generated vector<Pose> -> Drivetrain::follow() -> PurePursuit -> tank()`

No LemLib runtime, second path follower, action framework, or new odometry is
needed. The `LemLib v0.5` export label describes the file format only.
Generating a C++ header during the build avoids both a runtime parser and
binary-asset linker plumbing. Regenerate it whenever the source file changes.
This converter and its build hook are proposed work, not implemented here.

The converter must:

1. Parse finite `x, y, speed` rows up to `endData`, validate speeds in 0..127,
   and ignore editor metadata after `endData`. Reject malformed or empty input.
2. Handle the checked-in export's duplicated zero-speed endpoint and trailing
   editor point without extending the actual route. This file has 48 runnable
   points and approximately 92.85 inches of travel. Do not blindly drop the last
   two rows for every file.
3. Convert inches and coordinate handedness exactly once. AON heading zero is
   +X, heading increases clockwise, and local +Y points to the robot's right.
   For initial export tangent angle `a` and displacement `(dx, dy)`, use
   `x = dx*cos(a) + dy*sin(a)`, `y = dx*sin(a) - dy*cos(a)`.
4. Derive the final heading from the last distinct segment; intermediate `Pose`
   headings are ignored by the existing follower.
5. Choose an explicit speed policy. `Pose` contains no speed, so AON currently
   cannot consume the exported speed column. A first geometry-only version can
   use AON's configured profile and reject internal zero-speed action markers.
   Honoring sampled speeds later belongs in the existing controller/profile,
   not in another follower.

At runtime, place the normalized route at one captured starting odometry pose,
including its heading, before calling `follow()`. This can avoid re-taring the
IMU while the odometry task is updating. If pose reset is used instead, make
reset/update synchronization part of the implementation and its tests.

## Existing follower gaps to address first

These are code findings, not a diagnosis from a captured robot trace:

- `Drivetrain::follow()` dereferences `path.back()` without checking empty input.
- `PurePursuit::follow()` searches every point each iteration and retains no
  progress. Nearby return lanes or crossings can select the wrong segment.
- Its lookahead is five sample indices, not a physical distance; export density
  changes its behavior. It also copies the whole path on each controller call.
- A zero output at an intermediate lookahead target ends the outer loop, even
  though that does not necessarily mean the final endpoint was reached.
- The runtime returns `void` and aligns final heading after its loop even if
  the loop ended by timeout. It has no in-loop disable/cancel checks.
- `go()` adds linear and turning outputs without jointly limiting the motor
  pair; motor clamping can change the intended curvature.

Improve these behaviors in the existing AON controller/runtime before enabling
the static route. Keep the normal autonomous as a physical baseline.

## Verification for the next implementation

Test malformed/empty files, terminal rows, and left/right coordinate conversion.
Then exercise the existing follower with a straight route, right quarter-turn,
and the complete U-turn using simulated drivetrain/odometry feedback. Include
nearby lanes, timeout, and disable behavior. Finally repeat the same sequence
on the robot from a marked start. A normal autonomous working is useful evidence,
but does not replace this follower-specific check.
