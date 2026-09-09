# Competition

Code related to the driver controlled portion of the match and the autonomous routines should go here.

 - [Operator Control](./operator-control.hpp)
 - [Autonomous Routines](./autonomous-routines.hpp)

## JerryIO comparison test

Select **Red 3 or Blue 3: Basic U-turn (BAS)**. Red/Blue 4 still runs
JerryIO; Red 4 (JIO) is the startup selection. Both BAS alliance selections
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
about 92.7 inches of travel. It approximates the exported curve.

Unlike the earlier timed BAS test, `move()` uses odometry position and
`driveAngleOfArc()` uses the tracking wheels directly. It uses AON's ordinary
motion profiles, without a JerryIO path file or follower. Odometry is not reset.
The brain/controller shows Forward, Right U-turn, Return, then Sequence ended.
The existing move/arc calls are blocking; B/disable is checked between commands
and during the settling pauses, not inside those calls. Sequence ended means
the calls returned; those legacy commands do not report whether they timed out.

Upload the new firmware, explicitly select BAS, and repeat from the same marked
start position on the same surface. Compare several BAS and JIO runs and record
the physical shape, endpoint spread, and stage where either deviates. Repeatable
BAS with erratic JIO narrows attention to the follower, route conversion,
pose reset, or different motion profiles. Both now depend on tracking sensors,
so erratic BAS can still be caused by odometry or sensing as well as drivetrain
commands, motor configuration, traction, and mechanical consistency. Neither
result alone proves a software or mechanical cause.
