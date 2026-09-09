# Competition

Code related to the driver controlled portion of the match and the autonomous routines should go here.

 - [Operator Control](./operator-control.hpp)
 - [Autonomous Routines](./autonomous-routines.hpp)

## Working autonomous baseline

Select **Red 3 or Blue 3: Basic U-turn (BAS)**. Red 3 is the startup selection.
The previous JIO option 4 and its integration have been removed. Both BAS selections
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

The original static export is preserved as source data, but this firmware does
not load or run it. See [AON static-path requirements](./static-path-requirements.md)
for the findings and the smallest proposed integration with the existing AON follower.
