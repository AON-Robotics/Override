# Controls

Control algorithms such as Trapezoid Profile, PID, Path planning, etc. should go here.

It's best to encapsulate their behavior in classes and other data structures to make future use of them much easier.

## Experimental Pure Pursuit (Issue 18)

As of 2026-10-24, the audited native forward DifferentialDrive software scope is
complete with documented limitations and ready for supervised hardware validation.
Local verification passed 52 host suites normally and under AddressSanitizer,
15 Python checks and forced PROS builds for both robot configurations. No physical
tracking accuracy, stopping performance or loaded V5 timing is established.
The [competition README](../competition/README.md) is the canonical execution,
heading, cancellation, telemetry and future JerryIO integration contract.

`PurePursuit::setPath` prepares a bounded polyline once (up to 4096 poses).
Coordinates are inches; heading is clockwise degrees, with +X forward at zero
heading and +Y right. The legacy constructor's lookahead argument now means
inches rather than waypoint count. Intermediate headings do not steer the path.
Repeated positions collapse while preserving the strictest speed ceiling and
the last heading. A single point is a supported endpoint request.

`Drivetrain::followPath(path, timeoutMs, caps, reversed, headingCorrection, trace)`
executes synchronously
and returns a `PathResult`. Only DifferentialDrive opts into the tested wheel
kinematics. H-drive, mecanum, X-drive and reverse paths return
`UNSUPPORTED_CONFIGURATION` and stop. Their independent holonomic goToPose methods are
unchanged. Differential goToPose uses the native path lifecycle without automatic
final-heading correction. Existing `follow(path)` calls remain source compatible, but callers
must inspect `getPathStatus()` or migrate to the explicit result API.

The default deadline is 15000ms, including preparation. Serialize movement
requests. A second concurrent path request returns `CONTROL_FAILURE` without
interrupting the first. Another task can call `cancelPath()`; `stop()` also
latches cancellation on DifferentialDrive. The polling loop checks cancellation
and deadline before command publication, normally every 10ms. This is not a
hard real-time latency guarantee and cannot prevent a task from issuing an
unrelated motion command. A completed cancellation before a new request does not cancel
that new request. Competition disable persistently closes path admission until
explicitly re-enabled; enabling never revives an already-cancelled generation.
Positive measured path-update and heading-stage intervals must be <=50ms; an
observed overrun returns CONTROL_FAILURE. Legacy go/turn retain their separate
250ms input domain. The polling delay does not guarantee target scheduling.

| Result | Meaning |
| --- | --- |
| SUCCESS | Settled translation and, when requested, successful optional heading correction and terminal stop attempts |
| TIMEOUT | Cooperative stage budget expired before success, including stalls; blocking I/O can overrun it |
| CANCELLED | Cancellation/stop observed during this request |
| INVALID_PATH | Empty/non-finite path, invalid cap data, or size/coordinate bounds exceeded |
| UNSUPPORTED_CONFIGURATION | Unvalidated drivetrain or reverse request |
| CONTROL_FAILURE | Missing/invalid profiles, stale/non-finite pose, invalid timestep, or unusable steering geometry |

`translationStatus == SUCCESS` means remaining progress and endpoint error are
within 2.5 inches, with pose speed <=1 inch/s and heading rate <=5 degrees/s for
at least 150ms. `status` covers the whole request, including optional correction
and terminal stop errors. **Issue 33's movement layer owns final heading
alignment.** Pass a finite, explicitly tuned `HeadingCorrection` to enable the
single guarded post-translation stage; nullptr disables it. `finalHeading` carries
the requested heading. Failed translation skips correction. The PurePursuit
geometry controller never invokes a final-heading loop. Broader Issue 33 legacy
movement integration and physical tuning remain separate work.

Path geometry uses local contiguous projection, monotonic arc-length progress
and interpolated distance lookahead. Steering uses clockwise curvature
`2*lateral/chordSquared`, then differential wheel factors
`1 +/- curvature*DRIVE_WIDTH/2`. Center velocity never reverses. An inner wheel
may reverse on a tight turn; targets behind the robot use a stationary acquisition
turn. Forward speed scales by nonnegative cosine of heading error. Curvature
limits speed with `sqrt(30/abs(curvature))` in inches/s; 30in/s^2 is an experimental
lateral acceleration assumption, not a measured traction limit. Wheel RPM is
normalized proportionally to the lesser of MAX_RPM and the configured linear
profile ceiling.

Optional `caps` are per-waypoint center-speed ceilings expressed in equivalent
motor RPM, positive except optional
terminal zero, with the same length as the input path. They remain independent
of any exporter. The outgoing segment uses its start ceiling and a backward
braking envelope toward subsequent ceilings. The endpoint always requires zero
velocity. Immediate safety/curvature restrictions take priority over acceleration
and jerk smoothness; this is not a globally jerk-constrained trajectory.
PATH.JERRYIO decoding is outside Issue 18; the unused decoder was removed from
this branch. Generic speed-cap metadata remains supported. The future Issue 8
adapter must convert normalized speeds into equivalent motor RPM and pass these
ceilings through followPath, without adding another follower or movement lifecycle.
See the canonical PR #31 compatibility table for conversion and unsupported options. Do not encode speed in Pose.theta or assume the legacy
Pose vector preserves exporter metadata. Individual wheels and stationary
acquisition turns may exceed center-speed caps while staying within configured
wheel and angular profile ceilings. DifferentialDrive stop bypasses motor-group
slew and resets its velocity state, so a single fault-stop cannot leave a
partially ramped nonzero setpoint active.
Path commands bypass that second slew limiter because the controller already
profiles them; failed motor command writes return CONTROL_FAILURE.

The controller allocates during setup only. Its update has no formatting or
I/O. `PathResult.diagnostics` contains projected reference, lookahead, cross-track
distance, progress, curvature, target center RPM, pose-derived speed and angular
command. The result also includes elapsed milliseconds and maximum controller
compute microseconds, excluding pose access, motor I/O, locks and delay. Optional
PathTrace captures bounded caller-owned samples, read only after return; converted
RPM is a request, never measured motor velocity. Diagnostics are returned snapshots; controller internals
are not safe for concurrent inspection. No new background logging task is added.
The former per-cycle LCD/controller formatting has been removed from follow.

Construction without odometry now skips sensor reset so followPath can reject
CONTROL_FAILURE and attempt stops. Other legacy pose accessors and initialize
still require configured odometry. Active speed ceilings below integer actuator
resolution can reject as UNSUPPORTED_CONFIGURATION or time out; no minimum RPM
is injected. Command conversion preserves center/angular/lateral ceilings but
can change curvature. Safety limits and stops do not guarantee globally bounded
final wheel acceleration, deceleration or jerk.

### Validation and remaining work

Host regression cases cover sparse/dense/duplicate/single-point paths, straight,
arc, S, tight corners, a closed loop, large initial pose/heading errors, numerical
inputs, speed restrictions and settlement. A PROS timing/odometry model exercises
timeout, cancellation, stale pose, reentrant execution and timer rollover using
the production drivetrain class body. Local host tests and analysis records are
excluded from feature commits per repository policy.

Before integration, student reviewers must inspect the explicit result/heading
handoff and audit callers that previously relied on automatic alignment or
holonomic fallback. Fixed lookahead remains the reference: no adaptive law is
adopted without a measured benefit. Stalls and excessive deviation are bounded
by timeout; geometric gate/heading reacquisition exists, but there is no automatic
fault retry or dedicated stall/deviation detector.
Pose freshness does not prove sensor accuracy or detect a frozen-but-timestamped
source. Motor API errors and drivetrain dynamics still need hardware validation.

Before robot testing, calibrate track width, wheel ratio, odometry, profile
ceilings and lateral acceleration; verify cancellation on blocks with wheels
raised. Record intended and measured trajectory, cross-track error, velocity,
progress, completion result and controller time at controlled sampling intervals
outside the control loop. For offline analysis, project each sample onto every
reference segment to compute independent cross-track mean/P95/max; plot XY,
error/time, velocity/time and progress/time. Retain failed trials.

Before competition use, run at least 20 trials for representative straight,
curved, S and tight paths with controlled start pose, battery, field conditions
and robot configuration. Report endpoint error, success rate, completion time
and compute-time distribution. Repeat timeout, cancellation and sensor faults.
Optional future lookahead experiments require equivalent measured conditions;
no adaptive lookahead is required for the completed native software scope. None of these physical measurements have
been supplied or established by the software checks.
