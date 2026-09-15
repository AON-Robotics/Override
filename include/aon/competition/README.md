# Competition

Code related to the driver controlled portion of the match and the autonomous routines should go here.

 - [Operator Control](./operator-control.hpp)
 - [Autonomous Routines](./autonomous-routines.hpp)

## Working autonomous baseline

Select **Red 3 or Blue 3: Basic U-turn (BAS)** for the normal autonomous.
**Red 4: AON Static Path (PTH)** is preselected for the JerryIO test.
Blue 4 runs the same static route. Both BAS selections
run the same right-hand routine, without mirroring or intake actions.

BAS uses the ordinary AON autonomous commands:

```cpp
drivetrain.move(24);
drivetrain.driveAngleOfArc(8.5, 180);
drivetrain.move(24);
drivetrain.stop();
```

It pauses briefly at each transition. The 8.5-inch-radius right semicircle
produces a nominal 17-inch lane spacing and 180-degree final heading, with
about 74.7 inches of travel. The user reported that the earlier 33-inch first
leg worked physically; both straight legs are now 24 inches.

Unlike the earlier timed BAS test, `move()` uses odometry position and
`driveAngleOfArc()` uses the tracking wheels directly. It uses AON's ordinary
motion profiles, without a JerryIO path file or follower. Odometry is not reset.
The brain/controller shows Forward, Right U-turn, Return, then Sequence ended.
The existing move/arc calls are blocking; B/disable is checked between commands
and during the settling pauses, not inside those calls. Sequence ended means
the calls returned; those legacy commands do not report whether they timed out.

The static export is converted during the build and executed through the existing
AON follower. Paste each complete export into `static/<name>.jerryio.txt`, then
build and upload. All matching files are discovered automatically. Choose the
route in `src/aon/competition/static-path.cpp` with
`generated::staticPathAt(drivetrain.getPose(), "name")`. A missing name returns an invalid empty route rather than another path.
The current action test below uses `"testing"`.
Each route is anchored independently to the supplied start pose. The generated header is disposable; do not edit it. Exported
speeds are omitted; this routine uses a 200 RPM limit and a 30-second timeout.
B or competition disable cancels following. Internal stop markers are unsupported.

Latest physical test: the user reports the automatic JerryIO route only drove
straight. Red 3 (Basic U-turn / BAS) remains available for comparison.
The cause of the static-path failure is not yet established.

## Current JerryIO action test

Red 4 / PTH now runs `static/testing.jerryio.txt`, stopping at these editor poses:

| X | Y | Heading | Action after arrival |
| --- | --- | --- | --- |
| -30.842 | -14.325 | 0 | Intake forward for 2 seconds, then stop |
| -35.223 | 8.090 | 270 | Reverse intake for 2 seconds, then stop |
| -57.133 | -13.483 | 270 | Extend Arrow (port C) |

All legs and stop headings use one transform from the robot's initial live pose;
intermediate stops do not reset or rebase odometry. The first sampled segment
must align with the robot's initial forward direction, as with the existing path
conversion. Stop targets use the follower's existing 2-inch/2-degree tolerances.
The intake scanner is disabled for this test so it does not override timed actions.
B, disable, or a failed leg stops the intake and prevents remaining actions.
The whole sequence has a 30-second deadline. Arrow stays extended after success.
Re-exporting a different test shape also requires updating the three stop poses
in `src/aon/competition/static-path.cpp`; unmatched stops reject the routine.
