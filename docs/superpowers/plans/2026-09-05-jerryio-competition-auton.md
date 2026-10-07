# JerryIO Competition Autonomous Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the checked-in JerryIO route faster and execute two-second intake, outtake, and intake actions at the three requested control points before stopping at the final point.

**Architecture:** Encode stops as zero-speed samples in the PATH.JERRYIO asset, then pass generic `PathAction` callbacks from the competition wrapper into the JerryIO runner. Keep intake hardware dependencies outside the JerryIO module, and tune speed plus adaptive lookahead through existing AON options.

**Tech Stack:** PROS C++17, AON `PathFollower`, AON `PathAction`, AON `Intake`, PowerShell host-test runner.

**Spec:** `docs/superpowers/specs/2026-09-05-jerryio-competition-auton-design.md`

## Global Constraints

- Use only AON motion and mechanism APIs; add no LemLib runtime dependency.
- Marker actions are one-based and each lasts exactly 2000 ms.
- Marker order is intake, outtake, intake; the final point only stops and completes.
- The existing 30000 ms overall timeout includes all driving and actions.
- Preserve the robot-relative `(0,0,0)` transformation.
- Keep unrelated user changes and Superpowers documents out of commits.

---

### Task 1: Encode and verify the three route markers

**Files:**
- Modify: `static/path.jerryio.txt`
- Modify: `tests/path-jerryio-asset-test.cpp`

**Interfaces:**
- Consumes: `PathJerryIO::decode()` and `buildPathActionPlan(const Path&)`.
- Produces: an embedded path containing three internal zero-speed markers and one terminal zero-speed endpoint.

- [ ] **Step 1: Write the failing asset assertions**

Add assertions after decoding that samples 12, 32, and 65 (zero-based) have speed `0`, the endpoint has speed `0`, and `buildPathActionPlan(decoded.path)` returns `markerCount == 3` with four legs.

```cpp
CHECK(decoded.path[12].speed == 0.0);
CHECK(decoded.path[32].speed == 0.0);
CHECK(decoded.path[65].speed == 0.0);
const aon::PathActionPlan plan = aon::buildPathActionPlan(decoded.path);
CHECK(plan.valid);
CHECK(plan.markerCount == 3);
CHECK(plan.legs.size() == 4);
```

- [ ] **Step 2: Run the asset test to verify RED**

Run the targeted host compile for `tests/path-jerryio-asset-test.cpp` with `path-jerryio.cpp`, `path-follower.cpp`, `path-transform.cpp`, and `path-actions.cpp`.

Expected: FAIL because the three samples currently have speeds `44.903`, `100`, and `43.831`.

- [ ] **Step 3: Set the marker sample speeds to zero**

Change only these exported data rows before `endData`:

```text
-46.04, -13.527, 0
-39.078, -46.661, 0
-56.063, -23.987, 0
```

Leave `-61.497, -59.566, 0` as the terminal-only stop.

- [ ] **Step 4: Run the asset and action-planner tests**

Expected: both pass, with three markers and four drivable legs.

- [ ] **Step 5: Commit**

```bash
git add static/path.jerryio.txt tests/path-jerryio-asset-test.cpp
git commit -m "feat: add JerryIO mechanism markers"
```

### Task 2: Bind real intake actions and faster motion tuning

**Files:**
- Create: `include/aon/jerryio/routine-actions.hpp`
- Create: `src/aon/jerryio/routine-actions.cpp`
- Create: `tests/routine-actions-test.cpp`
- Modify: `include/aon/jerryio/routines.hpp`
- Modify: `src/aon/jerryio/routines.cpp`
- Modify: `include/aon/competition/autonomous-routines.hpp`
- Modify: `tools/run-host-tests.ps1`
- Modify: `include/aon/jerryio/README.MD`

**Interfaces:**
- Produces: `std::vector<PathAction> makePathJerryIOActions(std::function<void()> intake, std::function<void()> outtake, std::function<void()> stop)`.
- Produces: `int RunPathJerryIOAuton(Drivetrain&, const std::vector<PathAction>&)`.
- Consumes: global AON `intake` only inside the competition wrapper.

- [ ] **Step 1: Write the failing action-schedule test**

Assert that `makePathJerryIOActions()` returns markers `{1,2,3}`, durations `{2000,2000,2000}`, uses intake callbacks for markers 1 and 3, outtake for marker 2, and uses the supplied stop callback as cleanup for all three.

- [ ] **Step 2: Run the action-schedule test to verify RED**

Expected: FAIL because `routine-actions.hpp/.cpp` do not exist.

- [ ] **Step 3: Implement the hardware-independent action builder**

Return three `PathAction` values without including or referring to `Intake`:

```cpp
return {
    {1, 2000, intake, stop},
    {2, 2000, outtake, stop},
    {3, 2000, intake, stop},
};
```

- [ ] **Step 4: Pass actions into the JerryIO runner**

Change the runner signature to accept `const std::vector<PathAction>& actions`, validate them through `followPathWithActions`, and keep its decode/relative-path/failure reporting behavior.

- [ ] **Step 5: Bind the AON intake in the competition wrapper**

```cpp
const auto actions = aon::jerryio::makePathJerryIOActions(
    [] { intake.move(INTAKE_VELOCITY); },
    [] { intake.move(-INTAKE_VELOCITY); },
    [] { intake.stop(); });
return aon::jerryio::RunPathJerryIOAuton(drivetrain, actions);
```

- [ ] **Step 6: Apply faster adaptive tuning**

Set `maximumRpm = 500`, `maximumLateralAcceleration = 60`, and configure adaptive lookahead as enabled with minimum 5, maximum 14, speed weight 0.6, and curvature weight 1.2. Retain base lookahead 10 and timeout 30000 ms.

- [ ] **Step 7: Document the concrete routine**

Update the JerryIO README with the marker coordinates, actions, 500 RPM ceiling, adaptive values, six seconds of mechanism time, and terminal-only fourth stop.

- [ ] **Step 8: Verify and commit**

Run the complete host suite, run `pros make`, inspect the diff for LemLib runtime calls, and commit:

```bash
git add include/aon/jerryio/routine-actions.hpp src/aon/jerryio/routine-actions.cpp tests/routine-actions-test.cpp include/aon/jerryio/routines.hpp src/aon/jerryio/routines.cpp include/aon/competition/autonomous-routines.hpp tools/run-host-tests.ps1 include/aon/jerryio/README.MD
git commit -m "feat: run JerryIO intake autonomous"
```
