# Competition

Code related to the driver controlled portion of the match and the autonomous routines should go here.

 - [Operator Control](./operator-control.hpp)
 - [Autonomous Routines](./autonomous-routines.hpp)

## Experimental motion execution: Issues 18 and 33

AI-assisted reference changes require student explanation, adaptation, validation
and attribution before competition use. Host tests and compilation establish no
physical accuracy. These execution contracts supersede the earlier motion notes
where timing, geometry recovery or heading ownership differ.

`followPath(path, translationTimeoutMs, caps, reversed, headingCorrection)` is a
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
Fold-aware lookahead and active-branch recovery avoid coincident targets. Capped
acquisition retains a fixed earned reference until arrival. A missed gate requires
returning strictly before its plane, then crossing again. Near-coincidence
1e-6-inch and gate equality 1e-9 thresholds address numerical singularities,
not sensor resolution; the former matches the square root of the 1e-12-inch²
chord floor and is approximately 25 nanometers. This
is a conservative traversal policy, not proof of physical visitation: odometry
noise/jumps are still measured displacement. Severe shortcuts can require
reacquisition or timeout. Frozen but freshly timestamped odometry is not detected
as a sensor fault; unreachable endpoints rely on timeout, with no automatic retry.

Translation SUCCESS requires both remaining arc length and endpoint error within
2.5 inches, measured pose speed <=1 inch/s and heading rate <=5 degrees/s for
150ms. Zero commands alone never imply success. A singleton requests arrival
and settlement at that point. `translationStatus` records the translation result;
`status` records the whole request, including optional heading alignment and stop
errors. Failures must stop the autonomous sequence; the example path callers now
inspect results. Differential `goToPose` is a void compatibility wrapper using
the same path execution, a 15-second translation timeout and no automatic heading
turn. Use the explicit API for results and optional heading correction. Legacy
holonomic goToPose implementations and unrelated PID/profile motion APIs retain
their old contracts and require caller serialization and separate safety review.

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
through the entire configured dwell. It reports failure instead of correcting
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

Cancellation revokes the request generation before waiting on the command mutex.
Generation revalidation and paired wheel publication use the same mutex as stop.
An in-flight pair finishes before stop returns; after that boundary the cancelled
request cannot publish stale nonzero commands. `cancelPath` establishes the same
publication barrier but does not itself send zeros; use `stop` for immediate zero
attempts. Lock order is drivetrain command mutex then one motor-group state
mutex. No pose read, control computation or delay holds the command mutex. The
finite number of writes is bounded; scheduler and device wall time are not.

Every normal terminal return attempts zeros for all motor groups, even after a
write error (both sides on DifferentialDrive, all three/four on holonomic drives).
Terminal status is published after those attempts complete. CONTROL_FAILURE
overrides other results on a motor-write/stop error; otherwise cancellation wins
over the current result, and a translation deadline wins a success tie at the
loop's check. Terminal status proves completed stop attempts, never physical
braking or successful zero delivery after a transport error. A reentrant rejected
call does not stop another request. Each new accepted request resets controller
state and accepts the current cancellation generation, enabling explicit restart
after any terminal state; there is no automatic restart or recovery.

Motor-group state and transport writes are synchronized. Path commands bypass
the manual wheel slew limiter and quantize toward zero to integer RPM. Center
profiling does not globally constrain final wheel acceleration/deceleration or
jerk: geometry, proportional wheel normalization, curvature/lateral and waypoint
speed ceilings, acquisition turns, settlement and fault/cancel stops can override
smoothness. Safety ceilings and stopping take priority over ramp continuity;
student testing must assess traction/current and use conservative speeds. Do not
describe this implementation as globally acceleration- or jerk-constrained.

PATH.JERRYIO restoration is an **open dependency of Issue 18's exported-speed
requirement**, separate from these safety fixes. There is no active importer.
Optional RPM caps do not verify exported-speed preservation. Before closing that
requirement, inspect an actual export grammar/fixture and speed units, implement
separate parsing/conversion, and test order/coordinates, duplicate samples,
terminal zero, malformed/invalid speeds and explicit supported reversal policy.
Do not encode speed in Pose.theta or add a LemLib dependency.

Repository policy excludes local tests and audit records from commits. Another
clone can build production sources but cannot reproduce this session's host,
mutation, sanitizer, timing or simulation evidence from Git alone. A reviewed
versioned test-artifact release attached to Issue 18, or a separately maintained
regression repository with source-commit identifiers, is an acceptable proposed
shared mechanism; publishing either needs an explicit follow-up. This change
does not silently alter exclusion policy. Do not close Issue 18/33 from host checks.
