# Static PATH.JERRYIO files through AON

## Running the robot test

Upload the newly built firmware. Red 4 is preselected as **AON Static Path (PTH)**;
Blue 4 runs the identical right-hand route. Red/Blue 3 keeps the working ordinary
move/arc/move U-turn for comparison. No intake actions run in either path test.

Place the robot at the marked start facing the desired initial direction.
The generated route is placed at its current odometry pose; no pose reset or
IMU tare occurs. `Drivetrain::getPose()` now reads live odometry rather than the
old stored constructor pose. The static route has 48 poses and about 92.85 inches
of travel, ending approximately 17 inches right of the starting position and
facing about 182 degrees clockwise from the initial direction.

The screen says AON STATIC PATH and shows progress, pose, commanded left/right
RPM, and Path or Heading phase. Controller B cancels; competition disable is
checked throughout motion and heading alignment. The 30-second timeout covers
both phases. Final status distinguishes Completed, Timed out, Cancelled,
Disabled, Invalid path, and Invalid options. A motor-stop settling interval
follows every exit.

Repeat from the same marked start and compare with BAS. Record the final status
and the phase where motion diverges. These routines have different controllers
and speed profiles, so matching their exact timing is not expected.

## One execution stack

`static/path.jerryio.txt -> build-time converter -> vector<Pose> -> Drivetrain::follow() -> PurePursuit -> AON MotionProfile / tank()`

The previous separate JerryIO follower, runtime parser, linker asset archive,
marker-action framework, and telemetry framework remain removed. The exporter
label LemLib v0.5 is only a text format; there is no LemLib runtime dependency.

`tools/generate-static-path.py` validates and converts the file into
`src/aon/paths/static-path.cpp`. The Makefile regenerates it when the
export or converter changes, including on a fresh checkout. Python 3 must be
on PATH (`PYTHON` can be overridden when invoking make). Do not edit the generated
C++ file; paste the complete export into `static/path.jerryio.txt` and rebuild. The C++ hub contains only route poses and
the start-pose transform, with no exported speeds or editor/version metadata.

The converter rejects malformed/non-finite rows, invalid speeds, duplicate
consecutive positions, missing endData, and unsupported internal zero-speed
markers. It recognizes this export variant's duplicate zero-speed endpoint plus
editor-only trailer. Coordinates are inches; local +X is forward, +Y is right,
and headings increase clockwise. The conversion changes handedness once and
derives final heading from the final distinct segment.

## Explicit speed policy

This first version follows geometry at an AON profile limit of **200 RPM**.
The exported speed column is validated but does not set motor speed. `Pose` has
no speed field. The converter requires a zero-speed endpoint and rejects internal
stops instead of silently skipping mechanism actions. Sampled speed support, if
needed later, belongs in the existing controller/profile.

## Changes to the existing AON follower

- Route points are passed by reference; cumulative distances are cached per run.
- Projection advances monotonically within a bounded forward window, reducing
  jumps to nearby return lanes. Lookahead is six inches, not five sample indices.
- Geometric steering uses the existing AON linear motion profile. Motor pairs
  are scaled together to preserve curvature when limiting RPM.
- Only arrival near the final endpoint can start heading alignment. Position is
  reacquired if alignment/braking carries the robot outside the two-inch tolerance.
- The original one-argument `follow(path)` remains available; the overload
  `follow(path, timeoutMs, maximumRpm)` returns a result. The robot routine uses
  this overload on the differential drivetrain. Alternate drivetrain-specific
  one-argument overrides remain legacy code and are not validated by this test.
- Timeout/cancel/disable never triggers a subsequent final-heading maneuver.
  Completion requires heading within two degrees for 150 ms.

Each PurePursuit instance follows one immutable route per run. The forward
projection window cannot guarantee recovery from arbitrary teleports, large
tracking errors, or every self-intersecting route; the checked-in U-turn is the
physical test target. There is no reverse-travel or internal-action support in
this static-path routine.

## Verification and limits

`tools/run-host-tests.ps1` runs converter validation, selector tests, real odometry
math with fake hardware, the existing follower on straight/right-turn/U-turn
routes (also from a rotated start), and the real execution loop under completion,
timeout, invalid-input, cancellation, and disable conditions. Use `-Test` with a
test name for a focused run. Simulations include configured motor slew.

Passing host tests and building firmware do not establish physical tracking
accuracy. Real sensor signs, mounting, traction, latency, and task scheduling
still require the robot test. Keep BAS available as the known working comparison.
