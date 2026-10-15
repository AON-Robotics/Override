# Controls

Control algorithms such as Trapezoid Profile, PID, Path planning, etc. should go here.

It's best to encapsulate their behavior in classes and other data structures to make future use of them much easier.

## Experimental Pure Pursuit (Issue 18)

AI-assisted reference work requires student understanding, adaptation, testing
and attribution before competition use. Compilation and ideal-drive simulation
do not establish physical tracking accuracy.

`PurePursuit::setPath` prepares a bounded polyline once (up to 4096 poses).
Coordinates are inches; heading is clockwise degrees, with +X forward at zero
heading and +Y right. The legacy constructor's lookahead argument now means
inches rather than waypoint count. Intermediate headings do not steer the path.
Repeated positions collapse while preserving the strictest speed ceiling and
the last heading. A single point is a supported endpoint request.

`Drivetrain::followPath(path, timeoutMs, caps, reversed)` executes synchronously
and returns a `PathResult`. Only DifferentialDrive opts into the tested wheel
kinematics. H-drive, mecanum, X-drive and reverse paths return
`UNSUPPORTED_CONFIGURATION` and stop. Their independent goToPose methods are
unchanged. Existing `follow(path)` calls remain source compatible, but callers
must inspect `getPathStatus()` or migrate to the explicit result API.

The default deadline is 15000ms, including preparation. Serialize movement
requests. A second concurrent path request returns `CONTROL_FAILURE` without
interrupting the first. Another task can call `cancelPath()`; `stop()` also
latches cancellation on DifferentialDrive. The polling loop checks cancellation
and deadline before command publication, normally every 10ms. This is not a
hard real-time latency guarantee and cannot prevent a task from issuing an
unrelated motion command. A cancellation before a new request does not cancel
that new request.

| Result | Meaning |
| --- | --- |
| SUCCESS | End progress and endpoint position within 2.5in, pose speed <=1in/s and heading rate <=5deg/s for at least 150ms |
| TIMEOUT | Hard deadline expired before success, including stalls |
| CANCELLED | Cancellation/stop observed during this request |
| INVALID_PATH | Empty/non-finite path, invalid cap data, or size/coordinate bounds exceeded |
| UNSUPPORTED_CONFIGURATION | Unvalidated drivetrain or reverse request |
| CONTROL_FAILURE | Missing/invalid profiles, stale/non-finite pose, invalid timestep, or unusable steering geometry |

Success is translation settlement only. **Issue 33's movement layer owns final
heading alignment.** `PathResult.finalHeading` carries the requested heading;
the movement layer may align it only after `status == SUCCESS`. The path follower
never invokes `turnToHeading`. Timeout, cancellation and failures never authorize
heading correction. Final-heading correction itself is outside Issue 18.

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
and jerk smoothness; this is not a globally jerk-constrained trajectory. Active
PATH.JERRYIO import and speed metadata are absent from this checkout (earlier
integration was reverted); an importer must explicitly convert its speed units
and pass these ceilings. Do not encode speed in Pose.theta or assume the legacy
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
compute microseconds. Diagnostics are returned snapshots; controller internals
are not safe for concurrent inspection. No new background logging task is added.
The former per-cycle LCD/controller formatting has been removed from follow.

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
by timeout; there is no automatic recovery or dedicated stall/deviation detector.
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
Compare fixed, velocity-dependent, curvature-dependent and combined lookahead
under equivalent conditions; retain fixed lookahead if added complexity does
not improve measured reliability. None of these physical measurements have
been supplied or established by the software checks.
