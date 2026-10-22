# Odometry audit: issues #14 and #15

Audited `main` at `330077c1b41d68aa2abc610231ce2f88b3c74a69` on 2026-10-21.
The branch was created from that fetched revision. No physical robot validation
was performed. Both issues must remain open until the changes are merged and the
hardware checks below are recorded.

## Architecture and merged work

- [PR #12](https://github.com/AON-Robotics/Override/pull/12), merge `c59dfb7`,
  supplied signed-port encoder reversal. The current implementation still applies
  each sign exactly once, after reading the absolute port's centidegrees.
- [PR #29](https://github.com/AON-Robotics/Override/pull/29), merge `cb9240f`,
  replaced the old update loop with tracking/OTOS fusion, IMU heading smoothing,
  motor fallback, freshness APIs and source-switch baselines. It explicitly
  replaced the old GPS getter with `return getPosition()`; no GPS read or startup
  call remained. That was a placeholder, not GPS initialization.
- Later main changes (#32 driver feedback and #35 joystick scaling) do not change
  these odometry calculations.
- `src/aon/odometry/odometry.cpp` owns the live pose, stdin OTOS receiver, 20 ms
  update loop, source selection and mutex. Priority remains tracking with OTOS
  correction, tracking alone, OTOS (with motor prediction where available), then
  differential-drive motors. Source codes remain 0/1/2/3/4.
- `heading-fusion.hpp` supplies continuous clockwise heading, preferring the IMU.
  It aligns returning IMUs and rejects jumps over 90 degrees. Smoothing is retained.
- `pose-estimator.cpp` previously contained incorrect, unused wheel calculations
  (direction-dependent offset signs, lateral averaging and double arc rotation).
  It now supplies the same midpoint-frame wheel calculation used by live Odometry.
- `sensor-measurements.hpp` contains wheel baseline and GPS gate utilities;
  `ekf.cpp` is a separate retained estimator, not the live Odometry filter. The
  GPS gate/freshness helpers are also not wired into the live runtime. This change
  does not introduce another filter or enable continuous GPS fusion. The audited
  EKF prediction Jacobian had opposite cross-covariance signs; these are corrected
  and tested against forward/lateral propagation.
- `DifferentialDrive` alone registers its motor groups; its encoder-unit setter
  already invalidates baselines after taring. Holonomic drives do not acquire an
  invented two-motor fallback. Debug GUI/autonomous freshness gates remain intact.

## Issue #14: requirement-by-requirement audit

| Requirement | State on audited main | Change / evidence | Remaining |
| --- | --- | --- | --- |
| 1. Three-wheel forward, lateral and rotation | Partial: live calculation worked for straight motion with symmetric offsets; helper was incorrect | `localMotion()` removes each wheel's rotational arc and uses `sinc(turn/2)` midpoint chords; `Odometry::update()` uses it | Measured robot validation |
| 2. Reversal and sensor direction | Implemented by #12 and retained by #29 | Negative port signs retained; tests exercise all three reversed wheels through actual sensor conversion | Verify physical mounting signs |
| 3. Diameter, offsets and geometry | Partial: one diameter; forward mean assumed symmetric offsets | Per-wheel diameter overrides default to existing diameter; unequal offsets and both turn signs tested | Measure actual wheel diameters/offsets |
| 4. Rotation compensation | Partial: back compensation used smoothed heading, causing phantom lateral travel during turns | Compensation uses simultaneous wheel rotation; fused heading still orients translation | CW/CCW turn and IMU smoothing tests |
| 5. Units and coordinates | Tracking centidegrees to inches existed; GPS was not read | Inches, clockwise continuous degrees, +X forward at zero, +Y right preserved; GPS meters/east/north converted explicitly | Check field alignment |
| 6. GPS initialization | Missing: `gpsPosition()` returned the current pose | Opt-in startup via `initializeFromGps()`; finite, RMS-error, field-bound and heading checks; failed reads preserve all pose components | Locked/unlocked GPS startup checks |
| 7. Straight/reverse/lateral/arc/in-place behavior | No repeatable production tests | Temporary host tests executed `Odometry::update()`, actual conversion, shared helper and fusion | Surface/slip accuracy |
| 8. Tests exercise production | No test harness on main | Temporary harness compiled real odometry, pose-estimator and EKF sources, substituting only PROS hardware/RTOS calls; removed at the user’s request after successful runs | Hardware is not simulated physics |

The original issue's request to replace commented three-wheel code was outdated:
#29 already replaced it. The defects above, rather than that old implementation,
are addressed here.

## Issue #15: requirement-by-requirement audit

| Requirement | State on audited main | Change / evidence | Remaining |
| --- | --- | --- | --- |
| 1. Accessible total distance | Missing | Mutex-protected `getTraveledDistance()` returns path inches | Compare measured distance |
| 2. Exclude in-place rotation | No distance API; smoothed-heading compensation also introduced translation | Accumulate compensated local arc length, not raw wheel travel or pose corrections | Quantization/slip tolerance on robot |
| 3. Distance reset | Missing | Every successful pose reset/setter, including GPS startup, clears distance; failed GPS or invalid setters do not | None in software |
| 4. Invalid/disconnected/stale/implausible detection | Partial: wheel error/step guards, OTOS timeout/parser, IMU jump guard existed | Reject partial motor groups/unsupported units, OTOS teleports, nonfinite setters; report input health; update gaps invalidate baselines | Limits of timestamp-free sensors below |
| 5. Supported fallback | Implemented by #29 | Retained tracking → OTOS → differential motor priority; failure/recovery tested; unavailable back wheel reports degraded lateral coverage | Physical unplug/reconnect tests |
| 6. Continuous transitions | Partial: rebasing existed, but OTOS-only corrections were ungated and heading deltas could span a switch | Gate OTOS in every mode; rebase after outages and loss of wheel heading; reject late accumulated wheel travel after update stalls | Robot logs during source switches |
| 7. Source and failed inputs | Source API already implemented; individual input health missing | Retain `getPoseSource()`, `hasFreshPose()`, `hasFreshOtos()`, `isImuFusing()`; add `getHealth()` with left/right/back, left/right motor groups, aggregate motors, IMU, OTOS and GPS | Record actual failure diagnostics |
| 8. Normalized public heading | `getDegrees()` was continuous and needed by relative-turn callers | Add `getHeading()` in [0,360); preserve continuous `getDegrees()`, `getRadians()` and drivetrain `getTheta()` | Verify wraparound on robot |
| 9. External raw-sensor access | One autonomous filter diagnostic read `odometry.gyroscope` | Diagnostic and absolute-heading turn use public API; raw sensor members become private; relative/multi-turn routines stay continuous | External downstream code using raw members must migrate |
| 10. Recovery clears diagnostics | No per-input diagnostics | Baseline/healthy states replace failure states on successful recovery; returning IMU no longer discards that interval's fallback turn | Recovery logs on robot |

## Public contracts and limits

- `getTraveledDistance()` accumulates absolute translational **path length** from
  the selected wheel source, including curved travel. OTOS-only distance uses
  accepted incremental displacement. Rebaselines and fusion corrections are not
  travel. It is estimated distance, not a slip-free ground-truth measurement.
- `resetCurrent`, `resetInitial`, `SetPosition`, `setDegrees`, `setRadians`, and a
  successful GPS initialization reset distance and sensor baselines. Invalid
  numbers are ignored. Failed GPS initialization preserves the configured pose
  and distance. Reconnect/tare baseline changes alone do not reset distance.
- `getHeading()` normalizes degrees without changing continuous stored pose.
  `getTheta()` and `getDegrees()` remain continuous for compatibility.
- `getHealth()` is a mutex-protected snapshot. Enum codes printed by `debug()`:
  0 unavailable, 1 baselining, 2 healthy, 3 invalid, 4 implausible, 5 stale.
  Motor groups are explicitly identified; one invalid member disables its whole
  group so changes in the averaging population cannot create a false delta.
  Degree encoder units are supported. Other units disable motor fallback rather
  than silently interpreting rotations/counts as degrees. Normal drivetrain setup
  already selects degrees. Back-wheel loss preserves forward/heading tracking but
  cannot measure sideways travel.
- Pose/OTOS freshness deadlines remain 300 ms. Encoder/IMU polls have **no source
  timestamp** in the used PROS APIs. A polling outage is detected and rebaselined;
  an unchanged encoder value cannot distinguish a stationary robot from a frozen
  sensor. The existing `O,x,y,heading` Pi protocol likewise has no sequence or
  acquisition timestamp: arriving repeated stationary packets remain valid, while
  missing/malformed packets do not extend the last valid packet's deadline.
  Acquisition-stale but syntactically valid replays require a producer protocol
  change to detect reliably; no unsupported guarantee or movement heuristic is
  claimed. GPS startup is a one-shot validated read, not timestamped GPS fusion.
- A valid OTOS packet may still disagree with wheel prediction. The retained
  six-inch innovation gate rejects that correction and rebases its offset;
  `getHealth().otos` reports implausible. Packet freshness and correction acceptance
  are distinct. An OTOS jump is also rejected before heading fusion. The generous
  input bound is at least six inches per packet (300 in/s for longer intervals),
  and 90 degrees per packet. These existing/new bounds require robot confirmation.
- Source recovery intentionally discards unobserved movement while establishing
  baselines; it does not invent the robot's path during an outage.
- All live reads, baselines, source changes, setters and diagnostics serialize on
  the existing mutex. No new dependencies or background tasks were added.

## GPS configuration

`GPS_INITIALIZATION_ENABLED` defaults to `false`, preserving configured starts.
When enabled, `initialize()` tries GPS after IMU calibration and before launching
updates. `initializeFromGps()` is also available for an explicit retry while the
robot is stationary. It does not wait in a retry loop or keep overwriting pose.

`GPS_MAX_ERROR_METERS` defaults to 0.05 and `GPS_FIELD_HALF_SIZE_METERS` to 1.83.
The hardware's existing offsets and seeded GPS coordinates stay in meters. PROS
reports +X east, +Y north and clockwise heading from north; initialization maps
`pose.x = gps.y / 0.0254`, `pose.y = gps.x / 0.0254`, preserving heading. This makes
heading zero align with the runtime's +X direction. GPS must be aligned to this
field frame before use. See the [PROS GPS API](https://pros.cs.purdue.edu/v5/pros-4/group__cpp-gps.html)
and the checked-in `include/pros/gps.hpp` for units and sensor error semantics.
`gpsPosition()` now returns an actual validated field position in inches, with
NaN coordinates on failure, rather than returning the current estimated pose.

## Validation performed

A temporary C++17 host harness compiled production `.cpp` files with simulated
PROS inputs, AddressSanitizer and UndefinedBehaviorSanitizer. Leak detection was
disabled for the existing unrelated `Vector` unmanaged `Angle` allocation.
The user intentionally removed the test harness after the successful runs and
requested that it remain excluded. No test files or dependent CI workflow are
included in this change; the recorded host results are historical local runs,
not a regression suite available in the final checkout.

The PROS build remains repeatable with `make -j4`. It uses the project's ARM
compiler; on this machine that compiler is installed under the VS Code PROS
extension's `pros-toolchain-macos/bin`, which must be added to `PATH`.

Recorded results on 2026-10-21:

- Standard and unequal-diameter configurations: **433 checks passed each**.
- PROS `make -j4`: **passed**, producing `bin/hot.package.bin`. Existing warnings
  concern deprecated `ADIPotentiometer` and vendored JSON `std::is_pod`.
- No robot was connected or driven. Automated results do not verify wiring,
  mounting, field alignment, serial transport behavior, slip, or physical accuracy.

## Reading the active robot's diagnostics

The drivetrain owns a copy of the global `odometry` object. Read the active state
through `drivetrain`; the original global object does not run the update loop.
Use `drivetrain.getPose()`, `getHeading()`, `getTraveledDistance()`,
`getOdometryHealth()`, `getPoseSource()`, `hasFreshOtos()` and `isImuFusing()`.

For temporary terminal telemetry, add this task in `src/main.cpp::initialize()`
after the existing three-second delay. It does not change drive commands:

```cpp
pros::Task odometryTelemetry([] {
  while (true) {
    const auto p = drivetrain.getPose();
    const auto h = drivetrain.getOdometryHealth();
    std::printf("POSE x=%.2f y=%.2f heading=%.1f distance=%.2f source=%d "
                "health L=%d R=%d B=%d M=%d IMU=%d OTOS=%d GPS=%d\n",
                p.x, p.y, drivetrain.getHeading(),
                drivetrain.getTraveledDistance(), drivetrain.getPoseSource(),
                static_cast<int>(h.left), static_cast<int>(h.right),
                static_cast<int>(h.back), static_cast<int>(h.motors),
                static_cast<int>(h.imu), static_cast<int>(h.otos),
                static_cast<int>(h.gps));
    pros::delay(100);
  }
});
```

Observe stdout using your existing serial/terminal setup. When the Pi owns the
USB serial link for OTOS streaming, capture the robot's output on that link;
keep the OTOS producer connected for the OTOS tests. Do not start a second
odometry update loop for telemetry. Reset between independent trials with
`drivetrain.resetPose(0, 0, 0)` while stationary. Do not reset between the forward
and reverse legs when checking accumulated distance.

GPS port is currently `0` in both robot configurations in `globals.hpp`. Before
GPS tests, configure the real GPS smart port in the `Odometry(...)` constructor
and enable `GPS_INITIALIZATION_ENABLED` in `constants.hpp`. Leave GPS disabled
for the initial wheel/IMU tests.

## Hardware procedure and acceptance records (pending)

Use a clear field, a measured center reference point and an operator able to stop
the robot. Record firmware commit, robot configuration, measured geometry, surface,
source code, health snapshot, start/end pose and distance. Keep the robot still
through IMU calibration. Suggested acceptance limits below are explicit initial
criteria; any team-approved different limits must be recorded before testing.

1. **Signs/geometry:** move each tracking wheel independently by one marked
   revolution. Confirm configured reversal and inch conversion. Measure left/right
   offsets from the center of turning and signed back offset. Verify both robot
   configurations if both are deployed.
2. **Translation:** reset at a surveyed origin; travel 24 inches forward and back,
   then move 24 inches laterally each way with the wheels contacting the floor.
   Repeat three times. Each endpoint should be within 1 inch of the measured
   position, heading within 3 degrees and distance within 1 inch of actual travel.
   A forward/back pair should accumulate about 48 inches, not zero.
3. **Rotation:** rotate +360 and -360 degrees about the measured center, three
   trials each, with IMU enabled and then unavailable. Require center drift and
   accumulated translation each under 1 inch and heading error under 3 degrees.
   Cross 359/0 in both directions; normalized heading must stay in [0,360), while
   continuous heading and relative turns must remain continuous.
4. **Arcs/combined motion:** trace a measured 12-inch-radius quarter circle in
   both directions (18.85 inches of center travel), plus an S path and a diagonal
   lateral/forward move. Repeat three times; require endpoint and traveled-distance
   errors under 1 inch and heading error under 3 degrees. Include reversed ports.
5. **GPS:** with initialization disabled, confirm a chosen configured pose is
   preserved. Enable it on a marked field with visible GPS strips and a good lock;
   verify north/east conversion and heading within 2 inches/3 degrees of survey.
   Repeat with GPS unplugged, obstructed/low confidence and still calibrating;
   failed reads must preserve all configured pose components and distance.
6. **Fallback/recovery:** at rest, disconnect/reconnect left, right and back
   tracking sensors separately; stop/restart the Pi stream for >300 ms; disconnect
   each motor group member and the IMU. Record expected source codes and the named
   failing input. Returning sensors must first baseline, then clear diagnostics.
   No stationary transition may change position by >0.25 inch or heading by >1
   degree. Repeat during slow measured travel, allowing only travel since the last
   trusted interval; verify there is no replay of the full outage distance. Verify
   back loss reports unavailable lateral coverage and motor loss is visible.
7. **Transport/stall/reset:** pause the odometry task >300 ms, restore it, and
   confirm stale diagnostics and baseline recovery without accumulated catch-up.
   Exercise Pi reboot/reset, IMU reset and motor tare, then run existing autonomous
   relative/multi-turn routines. Confirm no new discontinuity and freshness stop
   behavior. The host suite covers malformed/NaN/Inf and implausible samples;
   repeat packet injection with the real transport where feasible. Do not equate
   identical stationary packets with an acquisition timestamp guarantee.

Attach the measurements and logs to issues #14/#15. Leave failed criteria open;
close only after merging the software and satisfying the recorded hardware criteria.
