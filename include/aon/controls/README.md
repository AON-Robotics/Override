# Controls

Control algorithms such as Trapezoid Profile, PID and path planning live here.
Encapsulate behavior in classes and other data structures for reuse.

`PurePursuit` prepares geometry and calculates commands; `Drivetrain` owns execution,
cancellation, stopping and optional final-heading alignment. `MotionProfile` supplies
speed shaping; safety limits can override smoothness. PID supports measured timestep
updates for the optional heading stage.

See the [competition README](../competition/README.md) for experimental support,
units, results, compatibility and callback-lifetime limits. Source comments explain
numerical invariants beside their implementation. Software checks do not establish
physical tracking accuracy or competition readiness.
