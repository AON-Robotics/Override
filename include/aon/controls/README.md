# Controls

Control algorithms such as Trapezoid Profile, PID, Path planning, etc. should go here.

It's best to encapsulate their behavior in classes and other data structures to make future use of them much easier.

## Experimental Pure Pursuit (Issue 18)

`PurePursuit` owns prepared geometry and command calculation; `Drivetrain` owns
movement execution, cancellation, stopping and optional final-heading alignment.
Positions and lookahead use inches, headings use clockwise degrees in the AON
frame, and speed ceilings use equivalent motor RPM.

- `setPath()` validates and prepares owned geometry once per request.
- `update()` advances that request without allocation, formatting or device I/O.
- `motorCommands()` converts the coupled wheel pair while retaining its limits.
- `MotionProfile` supplies existing speed/acceleration shaping; final safety limits
  can override smoothness. Global wheel acceleration and jerk are not guaranteed.
- PID supports measured timestep updates for the separately owned heading stage.

The [competition README](../competition/README.md) is the canonical contract for
results, supported configurations, settlement, cancellation, API compatibility,
telemetry, remaining blockers and the PR #31 integration boundary. Source
comments document the numerical invariants and units beside their implementation.
Software tests do not establish physical tracking accuracy or competition safety.
