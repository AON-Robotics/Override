# PATH.JERRYIO Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Load PATH.JERRYIO path assets and follow them with a speed-aware, fully AON-native autonomous motion stack.

**Architecture:** A strict decoder converts the external text format into an AON path model. A host-testable pure-pursuit core calculates speed-aware tank commands, while `Drivetrain` supplies odometry, motion profiles, PROS timing, cancellation, and safe motor shutdown.

**Tech Stack:** C++17, PROS 4, AON odometry/drivetrain/motion profiles, PowerShell host-test runner, PATH.JERRYIO LemLib-v0.5 text export format (format only).

**Spec:** `docs/superpowers/specs/2026-09-03-pathjerry-io-integration-design.md`

## Global Constraints

- Do not add or call LemLib.
- Preserve PATH.JERRYIO per-point speed values.
- Preserve source compatibility for `Drivetrain::follow(std::vector<Pose>)`.
- Stop motors on every terminal and error path.
- Commit after each independently verified task.

---

### Task 1: PATH.JERRYIO path model and strict decoder

**Files:**
- Create: `include/aon/jerryio/path.hpp`
- Create: `include/aon/jerryio/path-jerryio.hpp`
- Create: `include/aon/jerryio/README.MD`
- Create: `src/aon/jerryio/path-jerryio.cpp`
- Create: `tests/path-jerryio-test.cpp`
- Create: `tools/run-host-tests.ps1`

**Interfaces:**
- Produces: `aon::PathPoint`, `aon::Path`, `aon::PathDecodeError`, and bounded-buffer `aon::PathJerryIO::decode` overloads.

- [ ] Write decoder tests for valid upstream-format data, trailing editor metadata, CRLF, malformed rows, missing terminator, fewer than two points, non-finite values, and speeds outside 0-127.
- [ ] Run the focused host test and confirm the missing API fails compilation.
- [ ] Implement the minimal path model and strict non-throwing decoder.
- [ ] Re-run the focused host test and the PROS build.
- [ ] Commit with `feat: add PATH.JERRYIO path decoder`.

### Task 2: Speed-aware AON pure-pursuit core

**Files:**
- Create: `include/aon/controls/path-follower.hpp`
- Create: `src/aon/controls/path-follower.cpp`
- Create: `tests/path-follower-test.cpp`
- Modify: `tools/run-host-tests.ps1`

**Interfaces:**
- Consumes: `const aon::Path&`, current `aon::Pose`, elapsed seconds, and `PathFollowerConfig`.
- Produces: `PathFollower::step(...) -> PathFollowerOutput` containing left/right RPM, progress, target point, remaining distance, and completion state.

- [ ] Write tests for monotonic progress, geometric lookahead, straight and curved output, 0-127 speed-to-RPM mapping, per-point speed limiting, acceleration/deceleration, reverse following, normalization, and terminal tolerance.
- [ ] Run the focused test and confirm it fails before implementation.
- [ ] Implement continuous segment projection, lookahead selection, remaining path length, heading/curvature control, motion-profile limiting, and normalized tank outputs.
- [ ] Run both host test executables and the PROS build.
- [ ] Commit with `feat: add speed-aware AON path follower`.

### Task 3: Safe drivetrain execution API

**Files:**
- Create: `include/aon/controls/path-following.hpp`
- Create: `src/aon/drivetrain/path-following.cpp`
- Modify: `include/aon/drivetrain/drivetrain.hpp`
- Modify: `include/aon/api.hpp`
- Create: `tests/path-execution-policy-test.cpp`
- Modify: `tools/run-host-tests.ps1`

**Interfaces:**
- Consumes: decoded `Path`, `FollowPathOptions`, drivetrain odometry and tank commands.
- Produces: `MotionResult Drivetrain::followPath(...)`, `MotionResult Drivetrain::moveToPose(...)`, and a compatible legacy `follow(...)` adapter.

- [ ] Write policy/core tests for invalid options, timeout classification, disabled/cancelled classification, final-heading policy, and guaranteed stopping decisions.
- [ ] Run the focused test and confirm it fails before implementation.
- [ ] Implement the result/options types and drivetrain loop with a single safe-stop exit path.
- [ ] Adapt legacy `follow(std::vector<Pose>)` to the new AON path execution without changing its call sites.
- [ ] Run all host tests and the PROS build.
- [ ] Commit with `feat: expose AON autonomous motion API`.

### Task 4: Embedded asset example and autonomous integration

**Files:**
- Create: `static/path-jerryio-validation.jerryio.txt`
- Create: `include/aon/competition/path-jerryio-routines.hpp`
- Create: `src/aon/competition/path-jerryio-routines.cpp`
- Modify: `include/aon/competition/autonomous-routines.hpp`
- Create: `tests/path-jerryio-asset-test.cpp`
- Modify: `tools/run-host-tests.ps1`

**Interfaces:**
- Produces: a reusable `RunPathJerryIOValidation()` routine that embeds, decodes, initializes pose, follows, reports success, and stops safely without occupying a GUI competition slot by default.

- [ ] Write an asset-validation test for point count, endpoint, speed range, spacing, and final zero speed.
- [ ] Run it and confirm it fails before the asset/routine exists.
- [ ] Add the validation asset and AON-native routine using PROS `ASSET`.
- [ ] Run all host tests and the PROS build.
- [ ] Commit with `feat: add PATH.JERRYIO autonomous example`.

### Task 5: Team workflow documentation and final verification

**Files:**
- Create: `docs/PATH_JERRYIO.md`
- Modify: `MIGRATION_GUIDE.md`

**Interfaces:**
- Produces: the documented create/export/name/embed/decode/follow/tune/validate workflow.

- [ ] Document PATH.JERRYIO editor settings, supported external format, inches and heading convention, `static/` naming, API example, option tuning, and staged physical checks.
- [ ] Document that “LemLib v0.5” identifies only the exported text grammar and adds no LemLib dependency.
- [ ] Run all host tests, a clean PROS build, and searches proving no LemLib include/symbol was introduced.
- [ ] Review the complete diff and address only issue-scope findings.
- [ ] Commit with `docs: add PATH.JERRYIO workflow`.
