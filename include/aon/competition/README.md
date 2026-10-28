# Competition

Driver control and autonomous routine definitions live here. Competition lifecycle
callbacks are in [src/main.cpp](../../../src/main.cpp); configured drivetrain,
odometry and motion profiles are in [globals.hpp](../globals.hpp).

 - [Operator Control](./operator-control.hpp)
 - [Autonomous Routines](./autonomous-routines.hpp)

## Experimental motion execution: Issues 18 and 33

### Scope and validation limits

This is experimental forward DifferentialDrive reference code. The application
starts persistent path-execution and motor-publication owners before competition
callbacks. Native requests and shipped driver transport do not depend on a
deleted callback unwinding C++ scopes. Real PROS scheduling, target memory and
stopping latency still require supervised validation before deployment.

Tests and audit evidence remain local; a public clone alone cannot reproduce the
private suite. Verification checkpoints belong in the PR and local records, not
in duplicated readiness declarations. Host tests establish neither physical
tracking accuracy nor loaded V5 timing, memory margin or stopping performance.
Issue 18 remains open. Issue 8's JerryIO adapter and broader Issue 33 legacy
integration remain separate work.

AI-assisted reference changes require student explanation, adaptation, validation
and attribution before competition use. Host tests and compilation establish no
physical accuracy. These execution contracts supersede the earlier motion notes
where timing, geometry recovery or heading ownership differ.

`followPath(path, translationTimeoutMs, caps, reversed, headingCorrection, trace)` is a
synchronous request. DifferentialDrive alone supports its forward wheel model;
H-drive, mecanum, X-drive and reversed paths return UNSUPPORTED_CONFIGURATION
and attempt stopping. Tight curves can reverse an inner wheel. Intermediate
waypoint headings are ignored; the last heading is returned unchanged in
`finalHeading`. Consecutive duplicate coordinates retain the stricter speed cap
and last heading. Empty paths, nonfinite data and paths above 4096 samples reject.

Coordinates, lookahead, wheel diameter and track width are inches. +X is forward
at zero heading, +Y is right, heading increases clockwise in degrees. Motor RPM
is converted by `math::linearSpeed`, including configured wheel circumference
and gearing; verify those constants against the physical drive before tuning.
Caps restrict center speed in equivalent motor RPM, not individual wheels.
Wheel magnitude is limited by both MAX_RPM and the linear profile maximum;
turn magnitude also respects the angular profile maximum. Acquisition turns
have zero center speed, so a center cap does not restrict their wheel RPM.

Progress is ordered, not a global nearest-point lookup. Its cumulative allowance
is at most one initial lookahead plus twice total measured pose displacement.
Corners cannot replenish it. Noncollinear branch transitions require directed
crossings of corner gates in order; a reversal uses the incoming endpoint plane.
The forward horizon stops at folds relative to both the active direction and
adjacent future directions. If a horizon spanning the first outstanding gate
returns strictly before that gate's plane, it instead targets the gate vertex.
This retains ordinary corner cutting on the forward side and grants no progress.
Fold-aware lookahead and active-branch recovery avoid coincident targets. Capped
acquisition retains a fixed earned reference until arrival. A missed gate requires
returning strictly before its plane, then crossing again. Recovery retreat uses
the contiguous incoming straight run, retaining all waypoint speed restrictions.
Each preceding direction is compared with the corner's incoming direction using
the gate classification threshold, dot product > `1-1e-6`. Retreat is half the smaller of lookahead and run length,
floored at `2*r`, where `r=2*linearSpeed(1 RPM)*maxPathInterval` uses the longest
supported following hold rather than the preceding measured interval. Its arrival
radius is at most half the retreat, so the entire recovery ball remains strictly
before the gate. This internal actuator floor does not enlarge final arrival
tolerance. Only an earned noncollinear crossing invalidates an obsolete recovery
target; redundant samples retain the latch. The backward run scan occurs only
when creating a recovery target and is bounded by the prepared path size.
Collinear resampling preserves recovery geometry; exact RPM equivalence is not
guaranteed because the existing per-waypoint braking envelope can change speeds.
A corner target within
`max(1e-6 inches, linearSpeed(1 RPM)*dt)` uses an incoming-tangent extension by one
lookahead to cross the existing gate through measured displacement. This avoids
zero-center integer-command oscillation near a vertex, without adding progress.
The 1e-6-inch numerical floor matches the squared-chord guard and is approximately
25 nanometers; gate equality uses 1e-9. Neither is sensor resolution. This
is a conservative traversal policy, not proof of physical visitation: odometry
noise/jumps are still measured displacement. Severe shortcuts can require
reacquisition or timeout. Frozen but freshly timestamped odometry is not detected
as a sensor fault; unreachable endpoints rely on timeout, with no automatic retry.

Translation SUCCESS requires both remaining arc length and endpoint error within
2.5 inches, measured pose speed <=1 inch/s and heading rate <=5 degrees/s for
150ms. Dwell counts only intervals between consecutive qualifying observations;
the interval entering tolerance and the initial pose earn no dwell. Zero commands
alone never imply success. A singleton requests arrival
and settlement at that point. `translationStatus` records the translation result;
`status` records the whole request, including optional heading alignment and stop
errors. Failures must stop the autonomous sequence; the example path callers now
inspect results. Differential `goToPose` is a void compatibility wrapper using
the same path execution, a 15-second translation timeout and no automatic heading
turn. Use the explicit API for results and optional heading correction. Legacy
holonomic goToPose implementations and unrelated PID/profile motion APIs retain
their old contracts and require caller serialization and separate safety review.

### API compatibility and migration

These interfaces remain callable, but source compatibility does not preserve all
previous behavior. Existing in-repository path examples use explicit result checks.
External routines must be reviewed before integration.

| API | Current behavior | Migration requirement |
| --- | --- | --- |
| Differential `goToPose(pose)` | Translation only, fixed 15000ms budget; discards result | Use `followPath` and inspect `PathResult`. If final orientation matters, supply explicitly reviewed `HeadingCorrection`; no arbitrary default gains are provided. |
| Drivetrain `follow(path)` | Discards result; final alignment is opt-in through `followPath` | Inspect the explicit result before subsequent autonomous actions. |
| Holonomic `follow` / `followPath` | Reject unsupported kinematics and attempt every group's stop | Do not depend on the former differential fallback as validated holonomic following. Independent holonomic `goToPose` is unchanged. |
| `stop()` | Revokes active path generation and attempts immediate zero commands | A cancelled request must return before starting another motion owner. Admission itself is unchanged. |
| Differential `tank()` / SmartMotorGroup | Manual pairs are serialized; zero velocity bypasses slew; path publication bypasses duplicate slew | Serialize manual/path ownership. Locking alone does not make a deletable competition task safe. |
| PurePursuit `follow(path, pose, dt)` | Prepares only from IDLE; later path arguments do not replace a request | Use `setPath` explicitly for each new path and after terminal status. |

Removing an unconditional final turn after failed translation is intentional.
Successful final alignment also requires explicit opt-in; this is a compatibility
decision, not a claim that legacy orientation-dependent routines behave unchanged.

The movement layer owns one opt-in final-heading stage. PurePursuit geometry
never owns it. Pass a `Drivetrain::HeadingCorrection` pointer only with explicitly
reviewed tuning; nullptr disables it. Configuration contains an existing `PID`,
maximum turn RPM, tolerance in degrees, settlement milliseconds and correction
timeout milliseconds. Gains are RPM/degree, RPM/(degree*second) and
RPM*second/degree. Use finite nonnegative gains and integral activation/bounds;
the default PID infinity bounds must be replaced with finite values. At least
one gain must be positive, maximum RPM must be >=1 and within the angular/linear
profile and MAX_RPM ceilings, tolerance must be between 0 and 180 degrees, and the
nonzero dwell must be shorter than the timeout. Measured timestep is supplied to
PID without resetting its history. No robot-specific gains are supplied here.

For example, after students choose and validate `headingPID`, `turnRPM`,
`headingTolerance`, `settleMs` and `correctionTimeoutMs`:

```cpp
Drivetrain::HeadingCorrection heading{
    headingPID, turnRPM, headingTolerance, settleMs, correctionTimeoutMs};
const auto result = drivetrain.followPath(path, 15000, {}, false, &heading);
if (result.status != PurePursuit::Status::SUCCESS) return;
// Only now execute the next autonomous action.
```

Correction starts only after verified translation SUCCESS and a zero-command
handoff. It uses the exact final heading, shortest wrapped error and the existing
PID. It requires endpoint position still within 2.5 inches, valid fresh pose,
heading within the configured tolerance and measured heading rate <=5 degrees/s
and translational speed <=1 inch/s through the entire configured dwell. Both
rates use the measured pose-to-pose interval; the first sample establishes a
baseline and earns no dwell. Intermittent motion resets the full dwell. It reports
failure instead of correcting
X/Y drift. Heading SUCCESS sets `headingCorrected`; unsuccessful translation,
cancellation or invalid configuration cannot authorize this stage. One request
guard covers translation and correction. Reentrant path requests fail without
disturbing the active request; legacy unrelated movement entry points must still
be serialized by their callers. Correction tuning, overshoot and physical heading
accuracy remain unverified. Start with proportional feedback and a conservative
turn ceiling on raised wheels; add integral/derivative only with measured need.

The translation/preparation timeout and optional correction timeout are separate
cooperative budgets; total elapsed time can exceed their sum because pose reads,
motor I/O and scheduling can block. Odometry uses TIMEOUT_MAX mutex acquisition.
The 10ms delay is a requested polling delay, not a cancellation-latency guarantee.
`maxUpdateUs` measures only path-controller compute; it excludes pose access,
heading PID, motor writes, locks and delay. Host timing is not V5 timing.

Path translation updates and optional final-heading correction require positive
measured intervals <=50ms (`PurePursuit::maxPathInterval`); an observed overrun
returns CONTROL_FAILURE through normal stop/error/cancellation handling. This
explicitly replaces the former 250ms path acceptance. Longer command holds are
outside the supported recovery and corner-target scheduling envelope. This is
not general polyline convergence or a real-time guarantee.
An overrun is detected at the next observation; motion during a blocked hold is
not bounded by that detection. Legacy single-target `go`/`turn` keep their prior
250ms input domain and separate, unvalidated scheduling assumptions.

Cancellation revokes the request generation before waiting on the command mutex.
Generation revalidation and paired wheel publication use the same mutex as stop.
An in-flight pair finishes before stop returns; after that boundary the cancelled
request cannot publish stale nonzero commands. `cancelPath` establishes the same
publication barrier but does not itself send zeros; use `stop` for immediate zero
attempts. Lock order is drivetrain command mutex then one motor-group state
mutex. No pose read, control computation or delay holds the command mutex. The
finite number of writes is bounded; scheduler and device wall time are not.

### Competition callback lifetime

`initialize()` starts `initializeMotionService()` before configuring motion. Each
framework callback first calls `beginCompetitionMode()`: it revokes the preceding
callback epoch, waits for the stop barrier and old path cleanup, then permits the
new mode's explicit admission decision. These waits hold no application mutex.
PROS may delete a waiting callback; permanent execution/publication tasks retain
their scopes and release their locks normally. Do not delete those service tasks,
destroy their drivetrain or call the mode-entry method from a surviving GUI task.

Path submission uses fixed service-owned storage for at most 4096 poses/caps.
The executor owns its prepared vectors, heading configuration, result and optional
capture staging. It never retains callback pointers. A live caller receives the
result and copies completed samples to its original buffer after acknowledgement;
a deleted caller's buffer is never written by the worker. Copy/queue time consumes
the translation budget. Overlapping path requests still reject without stopping
the current owner. Cancellation retains the generation captured at submission.
This does not unwind caller-owned C++ objects when PROS deletes their task.
Prepare allocation-owning input containers outside deletable scopes when their
cleanup must survive mode changes; service ownership covers its own request data.

Manual wheel commands, configuration, stopping and pose snapshots pass through
eight bounded command slots. Only persistent owners take the existing drivetrain,
motor-group or forwarded pose-snapshot locks. Mode epochs reclaim abandoned
callback slots, never executing slots; other submitters must survive and consume
their requests. Epoch identity assumes fewer than 2^28 callback transitions during
one outstanding request. A fresh epoch's revocation precedes its command replies.
Startup failure rejects motion without falling back to callback-owned transport.
Two default PROS task stacks, fixed transfer storage and optional capture staging
add memory and scheduling cost; measure loaded target margins and driver latency.

This protects native path execution, differential `goToPose`, forwarded pose
accessors and shipped manual publication. It is not a full legacy-motion audit:
older PID/profile/holonomic movement loops still access odometry directly, and
their callback deletion/cancellation behavior needs separate review. Unrelated
mechanism tasks and existing GUI/driver ownership rules are unchanged. A manually
constructed drivetrain without service startup still requires a caller that
survives its complete direct request; it is not safe in deletable callbacks.

The actual `disabled()` callback then calls `setPathEnabled(false)`, which persistently
rejects new path requests and revokes active translation/heading before waiting
for the publication barrier and attempting zeros. Autonomous and operator-control
entry explicitly re-enable path admission. Disable revokes the generation again
inside the command mutex before publishing disabled/zeros: a request admitted
during an overlapping enable is invalidated at that completed boundary. The
pre-lock revocation remains immediate. Enabling never revives a cancelled
generation. Direct API instances start enabled for existing manual/offline callers.
This gates `followPath` and its heading stage; raw driver commands and unrelated
legacy movement routines remain caller-owned. Host callback tests do not verify
PROS field-notification delivery, task scheduling or physical motor stopping.

Every normal terminal return attempts zeros for all motor groups, even after a
write error (both sides on DifferentialDrive, all three/four on holonomic drives).
Terminal status is published after those attempts complete. CONTROL_FAILURE
overrides other results on a motor-write/stop error; otherwise cancellation wins
over the current result, and a translation deadline wins a success tie at the
loop's check. Terminal status proves completed stop attempts, never physical
braking or successful zero delivery after a transport error. A success candidate
is also checked against its active stage budget after all terminal stop attempts;
blocking I/O can overrun a deadline but cannot preserve SUCCESS once expiry is
observed there. Settlement describes sampled pose evidence; it is not an atomic
guarantee about sensor updates or physical motion after the last pose observation.
A reentrant rejected call does not stop another request. Each new accepted request resets controller
state and accepts the current cancellation generation, enabling explicit restart
after any terminal state; there is no automatic restart or recovery.

Motor-group state and transport writes are synchronized. Path commands bypass
the manual wheel slew limiter. Paired integer conversion starts with truncation
toward zero, then decreases positive wheel magnitude if needed so conversion
does not increase requested center speed or exceed the angular ceiling. Reducing
individual magnitudes alone does not preserve these coupled limits. Geometry
feasibility uses this same conversion as DifferentialDrive publication. Diagnostics
contain normalized floating requests; final integer commands may be smaller.
The final pair also enforces the 30 inches/s^2 lateral ceiling. With
`u=linearSpeed(1 RPM)`, its ideal commanded lateral acceleration is
`u^2*abs(L^2-R^2)/(2*DRIVE_WIDTH)`. If necessary, conversion reduces the larger
signed wheel toward the other wheel's absolute magnitude; for forward motion this
decreases center, angular and lateral magnitudes together. Work is fixed, with
at most one square root and a rounding postcheck, without a physical safety margin
or a raised limit. This can change curvature and is not a traction guarantee.
Low-speed quantization changes curvature; it cannot preserve every floating
wheel ratio or establish physical tracking accuracy.

If both bounded integer tracking commands are zero while heading error exceeds
acquisition tolerance, the controller latches a stationary acquisition turn.
Targets behind the robot also acquire. Exit tolerance is the configured angular
deadband, floored by one +/-1RPM heading step `2*linearSpeed(1)*dt/DRIVE_WIDTH`,
and capped at 45 degrees. The floor avoids chasing a tolerance smaller than an
integer yaw step; the cap guarantees a forward cone with cosine at least sqrt(.5).
The latch prevents repeated entry/exit near perpendicular targets. These are
representation/geometry rules, not robot tuning or physical dead-zone claims.
No minimum RPM is injected and no speed ceiling is raised. An active translation
ceiling below 1 RPM, or required stationary acquisition with angular ceiling below
1 RPM, returns zero/UNSUPPORTED_CONFIGURATION. Arrival settlement takes precedence,
so already-arrived singleton/tiny terminal paths remain supported. Positive caps
remain accepted at preparation; unsupported resolution is detected when active
motion requires it. Limits below 0.5 RPM cannot express any nonzero integer wheel
difference; such requests can reject acquisition or time out without steering.

Center profiling does not globally constrain final wheel acceleration/deceleration or
jerk: geometry, proportional wheel normalization, curvature/lateral and waypoint
speed ceilings, acquisition turns, settlement and fault/cancel stops can override
smoothness. Safety ceilings and stopping take priority over ramp continuity;
student testing must assess traction/current and use conservative speeds. Do not
describe this implementation as globally acceleration- or jerk-constrained.

Recovery uses a fixed earned reference and caps commanded center travel over
any supported following hold to half its current distance. With
`u=linearSpeed(1 RPM)` and `T=maxPathInterval`, its internal arrival radius is at
least `r=2*u*T`; a gate recovery retreats at least `2*r` so the whole arrival ball
remains before the unearned gate. These rules address integer recovery overshoot
without granting progress or enlarging the requested final tolerance. Active
translation requires linear tolerance >=r (about 0.010799 inches with the current
wheel/gearing); smaller active requests return zero/UNSUPPORTED_CONFIGURATION.
Already sampled terminal arrival is checked first and can still settle with a
smaller tolerance. This is a conservative supported resolution, not a theorem
that every accepted path converges or that smaller physical tolerances are
impossible. The original 600RPM/0.1-inch/49-50ms E37 cases remain accepted.

### PR #31 compatibility

Issue #18 owns native translation through `PurePursuit::setPath(path, caps,
reversed)` and `update(pose, dtSeconds)`, and synchronous movement through:

```cpp
Drivetrain::PathResult followPath(
    const std::vector<Pose>& path, uint32_t translationTimeoutMs = 15000,
    const std::vector<double>& caps = {}, bool reversed = false,
    const HeadingCorrection* correction = nullptr, PathTrace* trace = nullptr);
```

`setPath` copies/validates preparation data and returns RUNNING or a failure;
`update` uses the prepared path without allocation or I/O. Direct controller use
requires caller serialization. `followPath` owns the command guard, freshness,
timeouts, stop attempts and optional heading stage. Existing vector callers and
`follow(vector)` remain source compatible; use `PathResult` for failure handling.
Status values are IDLE, RUNNING, SUCCESS, TIMEOUT, CANCELLED, INVALID_PATH,
UNSUPPORTED_CONFIGURATION and CONTROL_FAILURE. Rejected overlapping requests
return CONTROL_FAILURE without stopping the active request.

[PR #31](https://github.com/AON-Robotics/Override/pull/31), reviewed at head
`4ab4a14ee15296e486ecb93c435d6cad87e0211b`, has its own PathView/PathRoute,
FollowOptions, legacy PurePursuit and Drivetrain::follow implementations. Its
future Issue #8 adapter must use this followPath lifecycle; do not merge a second
follower or duplicate final-heading loop. The local JerryIO branch was inspected
read-only. Parsing, metadata/frame conversion, generated routes, leg selection,
actions, diagnostics and tuning workflows remain Issue #8 responsibilities.
No importer is exposed by the Issue #18 API.

Construction without odometry skips sensor initialization so followPath can
reject CONTROL_FAILURE and attempt zero commands safely. This does not provide
manual-pose operation: legacy pose getters/setters and initialize still require
configured odometry and remain outside this path-execution contract.

| PR #31 input | Required future adapter contract |
| --- | --- |
| PathView / PathRoute | Validate pointer/count/cap storage and lifetime; copy the selected ordered leg into vector<Pose> and vector<double> before motion. Reject more than 4096 samples. Never use theta to carry speed. |
| XY / heading | Supply calibrated AON inches (+X forward at zero, +Y right) and clockwise degrees. Intermediate headings are ignored; final heading is explicit, finite and stored in the last Pose. Issue #8 owns export transforms. |
| Normalized speed s in [0,127] | Convert without integer rounding: capRPM = (double(s)/127.0)*referenceRPM. referenceRPM must match the export's declared full-scale motor RPM, not an inferred cartridge rating. For PR31's current runtime semantics it is FollowOptions.maximumRpm. Example: 63.5/127*180 = 90 RPM. Validate before conversion; clamp only downward under additional limits. |
| Null speed pointer | Explicitly unrestricted by export; native profile/wheel/curvature limits still apply. If maximumRpm is requested, preserve its center ceiling in the caps. |
| Duplicate XY / zero speeds | Native duplicate tolerance is 1e-9 inches; retain the stricter cap and last heading. Interior zero is INVALID_PATH; Issue #8 must split intentional stops into settled legs. Only optional terminal zero is accepted; native braking always ends at zero. Never silently discard a restrictive point. |
| timeoutMs | Maps to the cooperative translation/preparation budget. Optional heading has a separate budget; a total sequence deadline must budget both stages. No hard real-time guarantee. |
| finalHeading / headingTolerance | Resolve PR31's NaN fallback before the call; configure finite HeadingCorrection only when requested and tuned. Failed translation skips it. PurePursuit never performs final alignment. |
| lookahead / lookaheadAtSpeed | Native drivetrain uses fixed 5-inch lookahead. Adaptive lookahead has no mapping. Reject unsupported requested semantics rather than silently accepting them. |
| positionTolerance / settleMs / settledRpm | Native translation requires 2.5 inches, 150ms and pose speed <=1 inch/s plus heading rate <=5 degrees/s. PR31's configurable RPM settlement is different; explicit migration is required. |
| maximumRpm / lateralAcceleration / profile scales | Caps constrain center RPM; they do not substitute for requested wheel/turn limits. Native wheel limits use configured profiles/MAX_RPM and lateral limit 30 inches/s^2. Arbitrary per-request profile changes and lateral-limit disabling have no mapping; retain stricter constraints or reject until separately reviewed. |
| FollowHooks | There is no in-loop action callback. Return statuses govern travel/arrive/act; optional PathTrace is read after return. Online action hooks require a separately reviewed integration design. |
| reverse / holonomic | Unsupported: reject safely. Differential forward tracking may reverse the inner wheel. |

Integer publication may reduce commands further. Never round a fractional cap
up to a minimum RPM. Positive caps below actuator resolution can reject active
motion as UNSUPPORTED_CONFIGURATION or time out; they must not be raised.
The cap is a center-speed bound, not a promise that either individual wheel is
below that cap in a turn. The normalized conversion's rounding error must never
be used to loosen the source ceiling.

Repository policy keeps tests, simulations, benchmarks and audit records local.
They are not reproducible from the public production repository alone. Preserve
source-identified local packages and execution evidence; no public test release,
issue attachment or testing workflow is authorized by this work.

### Optional buffered measurements (Stage 2)

`followPath` accepts a sixth optional `Drivetrain::PathTrace*`, default nullptr.
This only captures measurements; no autonomous routine enables it automatically.
Provide caller-owned `PathSample` storage outside the control task's stack, keep
it alive and unmodified for the synchronous request, then read/export after return.
There is no concurrent-reader contract. A full buffer drops later samples and
increments `dropped`; zero capacity is allowed, but nonzero capacity with null
storage rejects. Choose capacity using `sizeof(PathSample)` and a measured memory
budget; input preparation and sensor tasks also require memory. A truncated trace
cannot establish whole-run error or timing percentiles.

Samples include pose observation timestamp, preceding interval, compute and
publication duration, stage, geometry diagnostics, floating requests and converted
integer RPM. First sample of each stage has interval zero because no preceding
stage interval was measured; control keeps its existing synthetic startup timestep.
Compute duration means controller update for translation and PID calculation only
for heading. Publication duration includes mutex wait/transport; terminal stop
attempts are separate. Heading samples retain the last translation diagnostics;
use their pose and wheel fields for heading-stage motion. Timestamp is not the
sensor acquisition time. `published` means both command APIs returned success,
not physical application or measured wheel velocity. A failed publication may
have partially affected hardware. Terminal handoff/stop pairs are not update
samples; inspect the final result and `terminalStopSucceeded` separately.

Capture adds fixed work when enabled but performs no formatting, allocation,
logging or additional sensor reads. Measure its overhead on V5 before relying on
traces. `maxIntervalUs` survives buffer exhaustion, including observed overruns.
Terminal `status` and `translationStatus` are retained even when there are no
samples. Rejected overlapping requests do not reset another request's capture.
Independent measured wheel speeds, physical final pose and stop distance require
external measurements; no such evidence is generated by this API.
