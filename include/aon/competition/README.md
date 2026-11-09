# Competition

- [Operator control](./operator-control.hpp)
- [Autonomous routines](./autonomous-routines.hpp)
- [Competition callbacks](../../../src/main.cpp)
- [Robot hardware and profiles](../globals.hpp)

## Experimental motion for Issues 18 and 33

This is AI-assisted reference work. Students must understand, adapt, validate and
attribute it before competition use. Issue 18 remains open.

[`followPath` and result types](../drivetrain/drivetrain.hpp) define the synchronous
request API. Only forward DifferentialDrive tracking is supported. Reverse and
holonomic requests return `UNSUPPORTED_CONFIGURATION` and attempt all-group stops.
An inner wheel can reverse during a tight turn.

- **Geometry.** XY uses inches with +X forward at zero heading, +Y right and
  clockwise degrees. The controller uses fixed 5-inch lookahead, ordered progress,
  corner gates and bounded recovery. Paths contain 1–4096 finite poses. Adjacent
  duplicate positions retain the stricter cap and last heading.
- **Speed.** Optional caps match the input waypoints and use center motor-equivalent
  RPM. Only the terminal cap may be zero. Braking, wheel, turn and curvature
  limits can override profile smoothness. Fractional caps are never raised.
- **Completion.** Translation needs remaining arc and endpoint error within 2.5
  inches and 150 ms of consecutive qualifying intervals at no more than 1 in/s
  and 5 deg/s. Zero RPM alone does not establish arrival. Update intervals must
  be positive and no more than 50 ms. Frozen odometry can time out even when
  timestamps remain fresh.
- **Heading.** Intermediate headings are ignored. The result exposes the requested
  final heading. Optional `HeadingCorrection` runs after successful translation
  and a zero-command handoff. It has a separate timeout and requires fresh pose
  within endpoint tolerance and low XY/angular motion throughout dwell. Review
  the gains and limits on the robot. Check final `status` before the next action.

| Compatibility API | Current behavior |
| --- | --- |
| Differential `goToPose` | Translation only, 15 s budget. Use `followPath` for the result and opt-in heading alignment. |
| `follow` | Discards the result and skips automatic final turns. Holonomic `goToPose` retains its legacy behavior. |
| `stop` / `cancelPath` | Both revoke the active generation. Only `stop` attempts immediate zeros. |
| `setPathEnabled` | Disabling closes admission, revokes the active generation and attempts zeros. Enabling permits fresh requests. |
| `PurePursuit::follow` | Prepares from IDLE only. Call `setPath` to start another request. |

Orientation-dependent sequences must opt in to final alignment after translation.

## Execution and verification limits

Two permanent tasks own execution storage and command publication because PROS
4.2.2 can delete a competition callback without C++ unwinding. Callbacks copy
bounded request data and wait without application locks. The executor owns its
vectors and capture storage without borrowing callback stack pointers. Mode-entry epochs
reclaim abandoned callback requests and wait for old execution before re-enabling.
Failed startup stays closed to motion. Calls without the service require a
surviving caller. Driver and legacy movement callers must serialize ownership.
Legacy loops with direct odometry access need separate lifetime and cancellation
review.

An in-flight motor pair can finish before stop returns. A revoked request cannot
publish afterward. A surviving stop, cancel or disable waits for publication even
across a mode transition. Terminal stops attempt all groups. Transport failure takes
precedence over cancellation, and cancellation takes precedence over success.
Translation's deadline includes queueing, preparation and stop, and wins success
ties. Blocking sensor or transport calls can delay a stop. Optional `PathTrace`
is bounded by caller storage and read after return. Sample publication flags describe transport
acknowledgments. Check `terminalStopSucceeded` for the separate stop result.

Check actual ports, gearing, placeholder sensors, odometry and tuning on the
robot. Loaded callback timing, memory margin, stopping and repeated-run accuracy
require physical validation. Very low quantized commands can still stall.

## PR #31 compatibility

[PR #31](https://github.com/AON-Robotics/Override/pull/31) owns parsing, frame
conversion, route legs and actions. Issue 18 exposes no importer. Adapt each
validated leg to native poses and caps in AON inches and clockwise degrees. For
declared normalized speed `s` in [0,127], use `(double(s)/127)*referenceRPM`
without rounding up, using the export's reference RPM. Preserve restrictive caps, split interior
stops, and resolve fallback headings before enabling alignment. Adaptive
lookahead, configurable settlement RPM, profile scales, lateral-limit overrides
and action hooks need explicit migration or rejection. Reuse the native
execution lifecycle and its single final-heading stage.
