# Competition

Code related to the driver controlled portion of the match and the autonomous routines should go here.

 - [Operator Control](./operator-control.hpp)
 - [Autonomous Routines](./autonomous-routines.hpp)

## JerryIO comparison test

Select **Red 3 or Blue 3: Basic U-turn (BAS)**. Red/Blue 4 still runs
JerryIO; Red 4 remains the startup selection. Both BAS alliance selections
run the same right-hand routine, without mirroring or intake actions.

BAS uses timed AON tank RPM commands, without reading the path file, pose,
tracking wheels, or IMU to control movement. It approximates the current
route with 33 inches forward, an 8.5-inch-radius right semicircle, then
33 inches forward along the return lane. Ideal lane spacing is 17 inches,
final heading is 180 degrees, and travel is about 92.7 inches. This is an
approximation of the exported curve, not an exact replay. Timing is derived
from the configured wheel diameter and gear ratio at 100 center RPM;
acceleration, slip, and real geometry affect the actual endpoint.

The brain/controller shows Forward, Right U-turn, Return, then Timing finished.
Controller B or competition disable aborts. Timing finished means the timed
commands ended, not that a measured endpoint was reached. Final odometry is
displayed only for comparison and is not reset by this routine.

Upload the new firmware, explicitly select BAS, and repeat from the same marked
start position on the same surface. Compare several BAS and JIO runs and record
the physical shape, endpoint spread, and stage where either deviates. Repeatable
BAS with erratic JIO narrows attention to sensing, pose estimation, follower,
or the different speed profiles. Erratic BAS also warrants checking drivetrain
commands, motor configuration, traction, and mechanical consistency. Neither
result alone proves a software or mechanical cause. If BAS only drives straight
during Right U-turn, its left/right RPM commands are unequal by construction;
check motor-side response and configuration before blaming odometry.
