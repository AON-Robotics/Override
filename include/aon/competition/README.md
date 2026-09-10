# Competition

Code related to the driver controlled portion of the match and the autonomous routines should go here.

 - [Operator Control](./operator-control.hpp)
 - [Autonomous Routines](./autonomous-routines.hpp)

## Working autonomous baseline

Select **Red 3 or Blue 3: Basic U-turn (BAS)** for the normal autonomous.
**Red 4: AON Static Path (PTH)** is preselected for the static-file test.
Blue 4 runs the same static route. Both BAS selections
run the same right-hand routine, without mirroring or intake actions.

BAS uses the ordinary AON autonomous commands:

```cpp
drivetrain.move(33);
drivetrain.driveAngleOfArc(8.5, 180);
drivetrain.move(33);
drivetrain.stop();
```

It pauses briefly at each transition. The 8.5-inch-radius right semicircle
produces a nominal 17-inch lane spacing and 180-degree final heading, with
about 92.7 inches of travel. The user reports that this routine works physically.

Unlike the earlier timed BAS test, `move()` uses odometry position and
`driveAngleOfArc()` uses the tracking wheels directly. It uses AON's ordinary
motion profiles, without a JerryIO path file or follower. Odometry is not reset.
The brain/controller shows Forward, Right U-turn, Return, then Sequence ended.
The existing move/arc calls are blocking; B/disable is checked between commands
and during the settling pauses, not inside those calls. Sequence ended means
the calls returned; those legacy commands do not report whether they timed out.

The static export is converted during the build and executed through the existing
AON follower. Paste the complete export into `static/path.jerryio.txt`, then
build and upload. The generated header is disposable; do not edit it. Exported
speeds are omitted; this routine uses a 200 RPM limit and a 30-second timeout.
B or competition disable cancels following. Internal stop markers are unsupported.
