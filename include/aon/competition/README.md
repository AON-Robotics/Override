# Competition

Driver control and autonomous routines live here. Framework callbacks are in
[src/main.cpp](../../../src/main.cpp); hardware and profiles are in
[globals.hpp](../globals.hpp).

- [Operator Control](./operator-control.hpp)
- [Autonomous Routines](./autonomous-routines.hpp)

## Experimental motion: Issues 18 and 33

This is AI-assisted experimental reference work. Students must understand, adapt,
validate and attribute it before competition use. Software verification does not
establish physical accuracy, reliable stopping or loaded V5 timing. Issue 18 remains
open; test evidence and historical decisions live in local session records.

### Native request contract

`followPath(path, timeoutMs = 15000, caps = {}, reversed = false,
headingCorrection = nullptr, trace = nullptr)` returns a synchronous `PathResult`.
Only forward DifferentialDrive tracking is supported. Holonomic or reversed
requests reject with UNSUPPORTED_CONFIGURATION and attempt all-group stops.
Tight turns may reverse the inner wheel.

Positions and lookahead use inches: +X forward at zero heading, +Y right, clockwise
degrees. Caps restrict center speed in equivalent motor RPM. Wheel/turn/profile and
curvature limits also apply. Validate geometry, gearing and tuning against the robot.
Paths contain 1–4096 finite poses; optional caps match their count. Consecutive
duplicate positions retain the stricter cap and last heading. Interior zero caps
reject; terminal zero is allowed. Fractional caps are never raised to force motion.

`PurePursuit::setPath` prepares owned geometry; `update` performs no allocation or
I/O. Progress follows ordered contiguous segments and corner gates, with bounded
pose-displacement credit and recovery. Lookahead is fixed at 5 inches. Translation
requires remaining arc and endpoint error within 2.5 inches, pose speed <=1 inch/s
and heading rate <=5 degrees/s for 150ms. Only intervals between consecutive
qualifying observations earn dwell. Zero wheel commands do not imply success.
Observed update intervals must be positive and <=50ms; overruns fail and stop.
Actuator resolution can reject active motion as unsupported. Frozen but freshly
timestamped odometry can instead time out. Safety limits override profile smoothness;
global acceleration/jerk bounds and convergence for every path are not guaranteed.

Intermediate headings are ignored; the final heading is returned in `finalHeading`.
Final alignment requires an explicit, reviewed `HeadingCorrection` with finite PID
configuration, RPM ceiling, tolerance, dwell and timeout. No robot gains are supplied.
It runs only after successful translation and a zero-command handoff. It also
requires fresh pose, endpoint tolerance and low XY/angular motion throughout dwell.
`translationStatus` describes translation; `status` describes the whole request.
Check SUCCESS before any subsequent autonomous action.

### API compatibility

| API | Current behavior / required migration |
| --- | --- |
| Differential `goToPose(pose)` | Translation-only wrapper; fixed 15000ms budget, discards result. Use `followPath` for failure handling and explicitly tuned final alignment. |
| `follow(path)` | Discards result; no automatic final turn. Use the explicit result before subsequent actions. |
| Holonomic `follow` / `followPath` | Unsupported; all motor groups receive stop attempts. Independent holonomic `goToPose` retains its legacy contract. |
| `stop()` | Revokes the current generation and attempts zeros; admission is unchanged. |
| `cancelPath()` | Revokes generation and waits for publication; does not itself write zeros. |
| `setPathEnabled(false)` | Revokes, closes admission and attempts zeros; enabling permits only fresh requests. |
| `PurePursuit::follow(path, pose, dt)` | Prepares only from IDLE. Use `setPath` for every new request. |

Removing the unconditional final turn after failed translation is intentional.
Successful legacy final alignment also now requires explicit opt-in; source
compatibility does not preserve orientation-dependent routine behavior.

### Callback lifetime and remaining limits

The application starts two permanent owners before competition callbacks: path
execution and manual/configuration/pose publication. PROS 4.2.2 deletes old callback
tasks on mode changes without C++ scope cleanup. Callbacks transfer bounded trivial
data and wait without application locks. The executor owns vectors, heading state,
results and capture storage; it never borrows a callback stack pointer. Mode-entry
epochs reclaim abandoned callback requests, revoke old generations and await cleanup
before re-enabling motion. Removing only the executor or publication owner restores
stranded ownership or mutex risks. Failed startup stays closed to motion.

Publication and terminal stops share the command barrier. An in-flight pair can
finish before stop returns; the cancelled generation cannot publish afterward.
Every owned terminal return attempts zeros for all groups. Transport/stop failure
takes precedence over cancellation; cancellation takes precedence over success.
Deadlines include preparation/queue time and win success ties; heading has a separate
budget. Blocking sensor/transport calls prevent a hard stop-latency guarantee.

Driver/path ownership still requires caller serialization. Legacy PID/profile and
holonomic movement loops that access odometry directly need separate lifetime and
cancellation review. Direct drivetrain objects without the service require permanent
caller lifetime. Existing zero motor ports / ADI 'z' placeholders need confirmed
hardware configuration. Real callback scheduling, loaded timing, heap/stack margin,
sensor behavior, tracking and repeated-run accuracy remain physically unverified.

Optional `PathTrace` records bounded observations without loop formatting or I/O.
Provide live caller storage for the synchronous request; read it after return.
Overflow increments `dropped`; nonzero capacity with null storage rejects. Initial
intervals are zero; compute timing covers controller/PID only, publication timing
includes its lock and transport. Samples omit terminal stops; inspect final status
and `terminalStopSucceeded`. API publication success is not measured motor motion.

### PR #31 compatibility

[PR #31](https://github.com/AON-Robotics/Override/pull/31) has separate PathView/
PathRoute and FollowOptions interfaces. Issue 8 owns parsing, export-frame conversion,
route splitting/actions and tuning. No importer is exposed by Issue 18. Its future
adapter must copy a validated ordered leg into native poses/caps and use this
execution lifecycle, without a duplicate follower or final-heading loop.

Convert declared normalized speed s in [0,127] as `(double(s)/127)*referenceRPM`
without rounding upward; referenceRPM must match the export contract. Preserve
restrictive caps and split intentional interior stops. Supply calibrated AON inches
and clockwise headings; theta never carries speed. Resolve fallback headings before
an explicitly configured heading stage. PR31 adaptive lookahead, configurable RPM
settlement, profile scales, lateral-limit overrides and action hooks have no direct
native mapping: migrate explicitly or reject unsupported requested semantics.
