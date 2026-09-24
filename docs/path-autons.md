# JerryIO autonomous guide

## Autonomous selections

Red/Blue 3 (**BAS**) run the in-tree `basicUTurn` test routine: move 24 inches, drive a
right semicircle of radius 8.5 inches, then move 24 inches. Both alliances use
the same route. The routine brakes between commands and checks B/disable during
those pauses. The legacy move/arc calls are blocking and do not report timeout
success, so cancellation is checked between commands.

Red/Blue 4 (**PTH**) run the generated `testing` route. The intake scanner is
disabled while this routine controls intake actions. Unknown route names return
empty routes and are rejected before motion. Internal zero-speed markers are
unsupported.

## Run the diagnostics

With `TESTING_AUTONOMOUS` enabled, open the debug registered-function list:

1. **Path: straight** — 24 inches, including a slower final approach.
2. **Path: gentle curve** — a 24-inch-radius quarter-circle.
3. **Path: U-turn** — the existing `static/path.jerryio.txt` route.

They run separately with a default 120 RPM cap and do not activate mechanisms.
An SD trial profile can override diagnostic tuning, as described below. Place the
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
`aon-straight-v2.csv`, `aon-curve-v2.csv`, or `aon-uturn-v2.csv` on the SD card, plus a
matching `-runs.csv` summary. In testing builds, the action routine also writes
`aon-testing-v2.csv` and `aon-testing-v2-runs.csv`.

Traces contain elapsed milliseconds, leg, pose, pursuit target, progress,
distance from the projected route, endpoint error, commanded and measured RPM,
and alignment state. Version 2 summaries also record the profile ID, route revision,
robot geometry, base motion limits, and every applied tuning field. Profile 0
means the routine's built-in defaults. RPM is motor RPM; on a differential drive the measurement
is the representative motor reported by each PROS motor group.

Recording is bounded at 320 samples (about 19 KB, allocated only when recording).
No SD operations occur in the motion loop. Samples arrive at 10 Hz with a final
sample after braking. Overflow is marked `truncated`; a missing/full SD card
reports that saving failed and does not change the motion result. Logs append;
the `run` column joins traces to summaries. Preserve both files together.
During one tuning campaign, keep appending to the same SD logs so run IDs remain
unique. Archive and start a new campaign after changing geometry, routes, or
firmware. Version 1 logs remain readable by the report tool but cannot be used
by the tuning tool.

Copy the summary files to the PC and run:

```powershell
python tools/path-report.py path/to/aon-straight-v2-runs.csv path/to/aon-uturn-v2-runs.csv
```

The report includes completion rate, successful-run timing, maximum position
and heading errors, failure counts, and truncated logs. Errors come from
odometry: measure physical endpoints too, because incorrect odometry can report
a perfect arrival at the wrong real position.

When a route drives straight through an expected turn:

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
| `accelerationScale` / `decelerationScale` | 1 / 1 | Per-run linear profile multipliers; acceleration also scales jerk |
| `turnAccelerationScale` / `turnDecelerationScale` | 1 / 1 | Per-run final-alignment profile multipliers |

Export speed is a normalized cap: **127 means `maximumRpm`, 64 means roughly
half**. It is not interpreted as physical inches/second or an exact voltage.
Speeds are rounded to the nearest byte; positive values below one stay one.
AON's motion profile controls acceleration and final braking. The follower also
previews reduced speed caps and curve limits, and scales center speed so neither
wheel exceeds its RPM cap. Physical deceleration and grip still require tuning.

The terminal zero is a stop marker, not a command to approach asymptotically at
zero speed. Internal zero markers remain rejected: split the routine into legs
instead. Use `staticRouteAt()` and `.view()` to preserve speed caps.

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
std::vector<aon::PathStep> steps(route.stops.size());
std::size_t first = 0;
for (std::size_t i = 0; i < steps.size(); ++i) {
  const auto& stop = route.stops[i];
  steps[i].path = route.view().slice(first, stop.index);
  steps[i].options.finalHeading = stop.heading;
  first = stop.index;
}
auto result = aon::runPathSequence(drivetrain,steps.data(),steps.size(),15000);
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

The action routine in `src/aon/competition/static-path.cpp` uses the generated
`route.stops`: each entry supplies a segment's endpoint index and anchored heading.
The export's JerryIO JSON owns those endpoints and headings. Include every
segment endpoint in the exported samples, rounded to the finest decimal place
present in the coordinate rows (scientific notation is supported). Metadata must
contain one continuous path of line segments (two controls) or cubic segments
(four controls), with finite coordinates and endpoint headings. The editor start
and final endpoint must agree with the samples. When declared, the format must
be `LemLib v0.5` and `gc.uol` must be `2.54` (inches). The final stop always uses
the terminal sample, even when that position was visited earlier.
The generator resolves them in order and rejects missing boundary samples or
ambiguous earlier crossings at build time; it does not infer or insert stops. No coordinate matching or route
edits happen on the robot. Re-export the complete file to change the stops. The testing routine
requires three segments, corresponding to its three actions; other segment
counts reject the routine before motion. Exports without editor metadata can
still be followed as whole routes, but do not provide segment stops.

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

## Guided automatic tuning

`tools/path-tune.py` performs calibration calculations, generates bounded trial
settings, schedules unfinished trials, and ranks passing candidates. It uses
Python's standard library. The robot runs the existing three diagnostics;
one operator-started run executes at a time. Reposition the robot between runs.
No new robot movement starts merely by copying a file or running the PC tool.

The search adjusts speed, low/high-speed lookahead, corner acceleration,
linear/turn acceleration and braking multipliers, and stopping criteria.
Arrival position/heading requirements stay fixed. Exported speed caps and
headings remain authoritative. Motion tuning affects copies of the path
follower's profiles, leaving legacy move/turn routines and driver control alone.
The full action sequence retains its shared 30-second budget, including waits.

### 1. Establish a physical reference

Measure at least three straight distances and turns independently (a tape and
angle reference, or a calibrated external positioning system). Record magnitudes
in inches/degrees in `measurements.csv` with this header:

```csv
reported_distance,actual_distance,reported_turn,actual_turn
```

Use actual odometry distance change, not the requested route length: the arrival
tolerance permits stopping short. Include both driving directions and clockwise/
counterclockwise turns across trials. Enter the configured tracking-wheel diameter:

```sh
python tools/path-tune.py calibrate measurements.csv --tracking-diameter 2 --output calibration.json
```

The report calculates `actual / reported` distance scale and a corrected tracking
wheel diameter. Inconsistent distance scales fail calibration. If `ready` is
false, apply the appropriate correction in the active robot section of
`include/aon/constants.hpp`, rebuild, then measure again. The tool never edits
geometry automatically. A turn error requires checking IMU conventions and
calibration; changing drivetrain width cannot repair a faulty IMU measurement.

### 2. Collect baseline response

Start without `aon-path-tuning.csv` on the SD card. Run each of the three
diagnostics at least three times. Copy their six version 2 CSV files into
`baseline/` on the PC. Keep a separate copy of the calibration measurements.

Choose the arrival and tracking accuracy required by the task. This example
requires a one-inch arrival radius, two-degree heading error, and at most one
inch of odometry cross-track error:

```sh
python tools/path-tune.py plan baseline --calibration calibration.json --position 1 --heading 2 --cross-track 1 --max-rpm 120 --output tuning-plan.json
```

The plan contains candidate IDs, full settings, and observed linear/turn response
estimates. Effective drive width is an estimate from wheel RPM and IMU change;
it is informational and is not applied automatically. Insufficient motion samples
produce `null` estimates. These measurements describe response under the existing
controller, not maximum acceleration, tire grip, or a validated friction model.
The search uses conservative measured motion settings and changes one setting
at a time around the baseline. It is a bounded local search, not a guarantee of
the globally fastest settings. It can be repeated around a later measured baseline.

### 3. Run scheduled candidates

Generate the first candidate and copy the resulting file to the SD card root:

```sh
python tools/path-tune.py trial tuning-plan.json --output aon-path-tuning.csv
```

The diagnostic screen shows the loaded profile ID. Run the scheduled diagnostics,
copy the accumulated six log files to `trials/`, then request the next unfinished
candidate:

```sh
python tools/path-tune.py trial tuning-plan.json --logs trials --output aon-path-tuning.csv
```

Copy that file to the card before the next run. The command reports how many runs
of each diagnostic remain. It skips candidates with a failed accuracy/completion
trial and does not repeat them automatically. To rerun a specific candidate, use
`--profile ID`. Defaults apply only when the SD file is absent; malformed,
out-of-range, or hardware-mismatched settings reject motion visibly.

After the scheduled trials finish:

```sh
python tools/path-tune.py select tuning-plan.json trials --output selection.json
```

A candidate must pass every recorded trial and have enough repeats on all three
routes. Cancelled, disabled, timed-out, truncated, inaccurate, or incomplete
trials cannot establish a passing candidate. Among passing candidates, the tool
minimizes the sum of each route's mean time, giving each route equal weight.
It rejects mismatched geometry, route revisions, profile settings, duplicate
run IDs, and incomplete trace/summary pairs. If none pass, no settings are selected.
Investigate the logs or revise the route; do not relax task accuracy merely to
make the optimizer report success.

### 4. Validate on new runs, then enable the action routine

Use `trial --profile ID` to load the top-ranked profile. Collect at least three
additional runs per diagnostic, including at least two representative battery/
load conditions. Continue appending to the same SD files and copy them into
`validation/`. Record physical endpoint errors for each new run in
`physical-endpoints.csv`:

```csv
route,run,position_error,heading_error,condition
```

`route` is `straight`, `curve`, or `uturn`; `run` comes from the summary CSV.
Position error is physical distance from the intended exported endpoint,
heading error is the smallest angular error in degrees, and `condition` describes
the battery/load setup. These are independent measurements, not copied odometry
errors. The approval command ignores runs already used for candidate selection:

```sh
python tools/path-tune.py approve tuning-plan.json selection.json validation --measurements physical-endpoints.csv --output aon-path-approved.csv
```

Only after all validation gates pass does it write the approved file. Copy
`aon-path-approved.csv` to the SD root to apply the same settings to all legs of
Red/Blue 4. The diagnostic trial file does not affect autonomous selection.
Remove the approved file to restore the built-in autonomous settings. Revalidate
after hardware, route, or firmware changes, and test the full mechanism sequence
under its shared deadline: passing the three diagnostics does not prove scoring
or mechanism reliability.

The SD protocol is versioned and generated by the PC tool. Do not hand-edit or
rename a trial file into an approved one. Profile loading and saving occur outside
the control loop. The candidate search never changes route geometry, segment
headings, action ordering, deadlines, or your requested accuracy.

## Storage and development checks

Export coordinates occupy 16 bytes per point in static double tables, plus one
byte per speed and one final heading per route. Generated segment stops add
endpoint indices and headings. Calculations and anchored poses remain double
precision. Legs borrow shared storage; the controller allocates
its distance/speed preview once per run and searches only forward from its
current segment. No new runtime libraries, background logging tasks, or
per-control-tick allocations are needed.

The build already uses `-Os` and section garbage collection. The optional log
buffer is freed after each run and absent from ordinary non-testing action runs.
Do not compare ELF debug-file length to flashed size; use `arm-none-eabi-size`
and the produced `.bin` instead.

Run `python tools/run-host-tests.py` for numerical, runtime, action, generator,
recording and report checks. The runner uses `CXX` if set, otherwise discovers
clang++, g++, or MSVC (via Visual Studio Installer on Windows). It builds one
C++17 test executable with shared hardware stubs in a temporary directory.
Python-only checks are available with `--python-only`. Run `make -j4` with the
PROS toolchain on PATH for the ARM build. Host simulations check ideal motion and cancellation, not real grip
or scoring reliability.
