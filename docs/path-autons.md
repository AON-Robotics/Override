# JerryIO autonomous guide

## Run the diagnostics

With `TESTING_AUTONOMOUS` enabled, open the debug registered-function list:

1. **Path: straight** — 24 inches, including a slower final approach.
2. **Path: gentle curve** — a 24-inch-radius quarter-circle.
3. **Path: U-turn** — the existing `static/path.jerryio.txt` route.

They run separately at a 120 RPM cap and do not activate mechanisms. Place the
robot consistently before each run. B or competition disable cancels motion.
The first exported segment is aligned with the robot's live starting heading.
The curve's sampled first chord, rather than its ideal tangent, defines that
alignment. The code never resets the running odometry during a route.

Red/Blue 4 still run the `testing` route: intake forward for 2 seconds after leg
one, reverse for 2 seconds after leg two, then extend Arrow after leg three.
The three legs share one initial transform and one route allocation. Its 30-second
budget includes action waits; final braking can extend return time by 300 ms.
The routine always stops the intake on exit. Arrow stays extended on success.

## Read the evidence

The screen shows commanded/measured RPM for each side. Diagnostics save
`aon-straight.csv`, `aon-curve.csv`, or `aon-uturn.csv` on the SD card, plus a
matching `-runs.csv` summary. In testing builds, the action routine also writes
`aon-testing.csv` and `aon-testing-runs.csv`.

Traces contain elapsed milliseconds, leg, pose, pursuit target, progress,
distance from the projected route, endpoint error, commanded and measured RPM,
and alignment state. RPM is motor RPM; on a differential drive the measurement
is the representative motor reported by each PROS motor group.

Recording is bounded at 320 samples (about 19 KB, allocated only when recording).
No SD operations occur in the motion loop. Samples arrive at 10 Hz with a final
sample after braking. Overflow is marked `truncated`; a missing/full SD card
reports that saving failed and does not change the motion result. Logs append;
the `run` column joins traces to summaries. Archive logs when changing routes or
tuning so comparisons use the same setup.

Copy the summary files to the PC and run:

```powershell
python tools/path-report.py path/to/aon-straight-runs.csv path/to/aon-uturn-runs.csv
```

The report includes completion rate, successful-run timing, maximum position
and heading errors, failure counts, and truncated logs. Errors come from
odometry: measure physical endpoints too, because incorrect odometry can report
a perfect arrival at the wrong real position.

For the reported straight-only failure:

| Trace observation | What to inspect next |
| --- | --- |
| Target stays straight through the expected curve | Selected export, generated geometry, route progress |
| Target is lateral but left/right commands are nearly equal | Pose heading, steering calculation and coordinate conventions |
| Commands differ but measured RPM does not | Motor grouping, signs, gearing, motor response |
| Wheel RPM differs but reported heading stays fixed | IMU configuration/calibration and odometry updates |
| Reported turn differs from the physical turn | Wheel slip, mechanical condition, odometry calibration |

These are diagnostic clues, not proof of the physical cause. Host simulations
cannot reproduce disconnected sensors, reversed wiring, slip, or collisions.

## Tune a route

Paste a complete inches-based LemLib-style `x,y,speed` export into
`static/<name>.jerryio.txt`. Building regenerates the disposable header.

```cpp
auto route = aon::generated::staticRouteAt(drivetrain.getPose(), "path");
aon::FollowOptions options;
options.maximumRpm = 200;
options.lookahead = 4;
options.lookaheadAtSpeed = 7;
options.positionTolerance = 0.75;
options.headingTolerance = 2;
options.timeoutMs = 15000; // choose the budget for this routine
auto result = drivetrain.follow(route.view(), options);
```

| Option | Default | Meaning |
| --- | --- | --- |
| `maximumRpm` | 200 | Maximum motor command on either side |
| `lookahead` / `lookaheadAtSpeed` | 6 / 6 inches | Target distance at rest / at the commanded speed cap; equal values disable adaptation |
| `positionTolerance` | 2 inches | Arrival radius |
| `headingTolerance` | 2 degrees | Allowed final angular error |
| `finalHeading` | NaN | Use endpoint heading; a finite value overrides it in native absolute degrees |
| `lateralAcceleration` | 35 inches/s² | Curve speed limit; zero disables it |
| `settleMs` / `settledRpm` | 150 ms / 5 RPM | Time within heading/position tolerance and below measured wheel-speed threshold |
| `timeoutMs` | 30000 | Budget for one following call, before final braking |

Export speed is a normalized cap: **127 means `maximumRpm`, 64 means roughly
half**. It is not interpreted as physical inches/second or an exact voltage.
Speeds are rounded to the nearest byte; positive values below one stay one.
AON's motion profile controls acceleration and final braking. The follower also
previews reduced speed caps and curve limits, and scales center speed so neither
wheel exceeds its RPM cap. Physical deceleration and grip still require tuning.

The terminal zero is a stop marker, not a command to approach asymptotically at
zero speed. Internal zero markers remain rejected: split the routine into legs
instead. The legacy `staticPathAt()` returns geometry only; use `staticRouteAt()`
and `.view()` to preserve speed caps.

Start with fixed lookahead, then change one setting at a time. Smaller lookahead
can track tighter curves but oscillate; larger lookahead can smooth motion but
cut corners. The default 35 inches/s² curve limit is a starting value, not a
measured grip limit for this robot.

## Sequence travel, approach, alignment and mechanisms

Use `aon/competition/path-sequence.hpp`. A `PathStep` borrows a path slice, holds
its own `FollowOptions`, and can run actions during travel and after arrival.
`slice(first,last)` includes both endpoints. Adjacent legs should share their
boundary point. Keep the owning route and callback contexts alive for the entire
blocking call; do not resize or edit its vectors during execution.

```cpp
auto route = aon::generated::staticRouteAt(drivetrain.getPose(), "path");
const auto middle = route.points.size()/2;
aon::PathStep steps[2];
steps[0].path = route.view().slice(0,middle);
steps[1].path = route.view().slice(middle,route.points.size()-1);
steps[1].options.maximumRpm = 100;
steps[1].options.positionTolerance = 0.75;
// Explicitly set each intermediate leg's desired final heading.
steps[0].options.finalHeading = 90; // example native absolute heading
auto result = aon::runPathSequence(drivetrain,steps,2,15000);
```

- `events` points to a sorted array of `PathEvent{distance, fire, context}`.
  Distances are inches from the beginning of that leg; each fires once.
  Endpoint events still fire after successful arrival within tolerance.
- `arrived(context)` runs only after successful position/heading settling.
- `waitMs` holds the stopped robot for that duration.
- With `ready(context)`, `waitMs` becomes the maximum sensor wait; success
  continues early, timeout aborts the sequence. A sensor wait requires a
  nonzero timeout.
- `afterWait(context)` can stop an intake started at arrival; it also runs when
  that wait is interrupted. The routine owner must clean up mechanisms activated
  by progress events if following fails before arrival.
- `hooks.update(context, progress)` can inspect sensors during following;
  returning false cancels. `hooks.sample` receives observations for recording.
- All callbacks must be short and nonblocking. The sequencer handles waits and
  checks B, disable and its shared deadline every 10 ms.

The existing action routine in `src/aon/competition/static-path.cpp` is a working
example. Its stop positions are matched to export samples before any motion;
changing that route also requires updating the three editor stop poses there.
`generated::staticWaypointAt(start, editorPose, name)` converts editor GPS heading
and coordinates into the same anchored frame as the route.

## Calibrate and establish repeatability

1. Check wheel diameter, gearing, tracker offsets and motor signs against the
   installed robot. Turn clockwise by hand: the reported heading must increase.
   Move forward: reported motion must agree with the heading.
2. Use the straight diagnostic and measure actual travel. For a known physical
   travel distance, a distance scale correction is `physical / reported`.
   Apply it to the relevant tracking-wheel conversion only after repeating the
   measurement in both directions. Do not compensate bad odometry using path
   coordinates.
3. Verify 90° and 180° physical turns against the IMU. With reliable wheel travel
   measurements, effective track width is `(left travel - right travel) / angle`
   using signed inches and clockwise radians. Average several slow turns in both
   directions; slip makes a single estimate unreliable.
4. Use a consistent starting jig or field reference. Anchoring to the live pose
   does not correct inconsistent physical placement. Do not re-anchor subsequent
   legs after contact or a mechanism action.
5. In JerryIO, show the full robot dimensions, use smooth tangent transitions,
   and leave clearance for the swept robot and mechanisms. Sample spacing should
   resolve the tightest curve; adding points to a straight does not add accuracy.
6. Repeat each diagnostic at least ten times under representative battery/load
   conditions. Record physical endpoint error, heading, elapsed time and result.
   Choose acceptable tolerances before increasing speed. Then test the full
   mechanism routine repeatedly with its intended time budget.

No calibration constants were guessed or changed by this implementation.

## Footprint and verification

Export coordinates occupy 8 bytes per point in static float tables, plus one
byte per speed and one final heading per route. Calculations and anchored poses
remain double precision. Legs borrow shared storage; the controller allocates
its distance/speed preview once per run and searches only forward from its
current segment. No new runtime libraries, background logging tasks, or
per-control-tick allocations were introduced.

The build already uses `-Os` and section garbage collection. The optional log
buffer is freed after each run and absent from ordinary non-testing action runs.
Do not compare ELF debug-file length to flashed size; use `arm-none-eabi-size`
and the produced `.bin` instead.

Run `./tools/run-host-tests.ps1` for numerical, runtime, action, generator,
recording and report checks. Run `make -j4` with the PROS toolchain on PATH for the
ARM build. Host simulations check ideal motion and cancellation, not real grip
or scoring reliability.

### Verification snapshot (2026-09-15)

All 7 Python tests and 6 C++ host test executables passed; the ARM build passed
with existing ADIPotentiometer and JSON is_pod deprecation warnings.
The final hot image is 858,092 bytes. Code plus initialized data is 858,045
bytes, 4,954 bytes below the starting firmware artifact; BSS is unchanged.
This comparison includes the existing workspace and is not proof of an absolute
minimum. Physical calibration and repeated robot trials remain necessary.
