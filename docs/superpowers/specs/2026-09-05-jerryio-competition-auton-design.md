# JerryIO Competition Autonomous Design

## Goal

Turn the checked-in robot-relative PATH.JERRYIO route into a faster autonomous
routine with three deterministic intake actions and a final drivetrain stop.
The implementation remains entirely within AON motion and mechanism APIs.

## Route markers

The PATH.JERRYIO export does not place samples at every control point exactly,
so each requested control point maps to its closest decoded sample:

| Marker | Requested point | Decoded sample | Distance | Action |
|---|---:|---:|---:|---|
| 1 | `(-46.086, -13.517)` | `(-46.040, -13.527)` | 0.047 in | Intake forward for 2000 ms |
| 2 | `(-38.844, -46.383)` | `(-39.078, -46.661)` | 0.363 in | Intake reverse for 2000 ms |
| 3 | `(-55.927, -24.101)` | `(-56.063, -23.987)` | 0.178 in | Intake forward for 2000 ms |
| Final | `(-61.497, -59.566)` | `(-61.497, -59.566)` | 0 in | Stop and complete immediately |

The three internal samples receive speed `0`, making them one-based action
markers in the existing `PathActionPlan`. The existing final speed-0 point
remains terminal-only and does not receive a callback or extra delay.

## Mechanism behavior

The AON competition wrapper owns the robot-specific callbacks and passes them
into the generic JerryIO routine:

- Intake: `intake.move(INTAKE_VELOCITY)`; cleanup: `intake.stop()`.
- Outtake: `intake.move(-INTAKE_VELOCITY)`; cleanup: `intake.stop()`.
- Every action lasts 2000 ms.

The drivetrain is stopped before each callback. Cleanup runs after the delay
and also on disable, cancellation, or overall timeout. Actions run in marker
order. This keeps the generic JerryIO module independent of the `Intake` type.

## Motion tuning

The routine uses a 500 RPM ceiling. Because the export peaks at speed 100 on
PATH.JERRYIO's 0-127 scale, the nominal path-speed peak is about 394 motor RPM
before curvature and braking limits. Maximum lateral acceleration increases
from 40 to 60 in/s^2.

Adaptive lookahead is enabled with:

- base lookahead: 10 in
- minimum: 5 in
- maximum: 14 in
- speed weight: 0.6
- curvature weight: 1.2

Fast straight sections therefore look farther ahead, while tight bends reduce
lookahead and speed. The 30-second overall timeout includes all driving and the
six seconds of mechanism activity.

## Data flow and failure handling

1. Decode the embedded path.
2. Convert it to the robot-relative `(0,0,0)` frame.
3. Validate that it contains exactly the three internal markers required by
   the configured actions.
4. Reset AON odometry and execute the path with actions.
5. Stop drivetrain and intake on every terminal path.
6. Report completion, decode/configuration failure, disable, cancellation, or
   timeout on the brain and controller.

## Verification

- Asset test asserts the three marker samples are zero speed and the final
  point remains terminal-only.
- Action-plan test asserts the embedded route produces three markers/four legs.
- Existing path, profile, adaptive-lookahead, execution-policy, and relative
  transform host tests remain green.
- Full PROS firmware build confirms callback wiring and embedded asset linkage.
- Physical test begins with the robot on blocks, then on the field with a
  spotter ready to disable it.
