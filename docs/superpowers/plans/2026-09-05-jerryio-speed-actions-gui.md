# JerryIO Speed, Actions, and GUI Implementation Plan

> **For Codex:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Make JerryIO paths smoother and faster, support safe zero-speed action markers, and expose the routine as preselected Red 4 plus Blue 4 without replacing existing autonomous routines.

**Architecture:** Extend `PathFollower` with a path-wide motor-RPM profile built from exported speed, curvature, and backward braking constraints. Add a generic action-aware executor that splits a path at internal zero-speed markers and runs callbacks while stopped. Keep robot mechanisms outside JerryIO through callbacks, and make GUI selection limits depend on the selected autonomous list.

**Tech Stack:** PROS C++17, AON drivetrain/motion APIs, LVGL GUI, PowerShell host-test runner.

---

### Task 1: Plan JerryIO velocity across the full path

**Files:**
- Modify: `include/aon/jerryio/path-follower.hpp`
- Modify: `src/aon/jerryio/path-follower.cpp`
- Modify: `tests/path-follower-test.cpp`

- [ ] Keep the existing failing tests for braking, curvature limiting, and invalid physical configuration as the red phase.
- [ ] Validate wheel diameter, motor-to-wheel ratio, and lateral acceleration.
- [ ] Build a per-waypoint RPM limit from PATH.JERRYIO speed values.
- [ ] Apply an optional curvature limit using `sqrt(maximumLateralAcceleration / curvature)`.
- [ ] Run a backward braking pass using `v² = u² + 2as` and convert between wheel linear speed and motor RPM.
- [ ] Interpolate the planned RPM by path distance and use it in `step()` while retaining runtime slew limiting.
- [ ] Run the host tests and commit `feat: plan JerryIO path velocity`.

### Task 2: Add safe zero-speed action markers

**Files:**
- Create: `include/aon/jerryio/path-actions.hpp`
- Create: `src/aon/jerryio/path-actions.cpp`
- Modify: `include/aon/drivetrain/drivetrain.hpp`
- Modify: `src/aon/drivetrain/path-following.cpp`
- Create: `tests/path-actions-test.cpp`
- Modify: `tools/run-host-tests.ps1`

- [ ] Write failing host tests showing that internal zero-speed points split a path, the final zero is terminal-only, marker ordinals are stable, missing actions are allowed, and nonexistent marker references are rejected.
- [ ] Define a generic `PathAction` with marker ordinal, duration, start callback, and cleanup callback; do not reference intake or piston classes.
- [ ] Implement a pure path-action planner that creates valid drive legs and gives resumed legs a nonzero launch speed derived from the next point.
- [ ] Add `Drivetrain::followPathWithActions()` that stops at each marker, runs matching actions in declaration order, polls disable/cancel/overall timeout during waits, always runs cleanup, and resumes the next leg.
- [ ] Ensure final-heading correction is applied only to the final leg and the single overall timeout includes driving and actions.
- [ ] Run host tests and commit `feat: add JerryIO path actions`.

### Task 3: Add Red 4 and Blue 4 GUI slots

**Files:**
- Create: `include/aon/tools/gui/auton-selection.hpp`
- Modify: `include/aon/tools/gui/gui.hpp`
- Modify: `include/aon/tools/gui/ui/gui-layout.hpp`
- Modify: `src/aon/tools/gui/gui.cpp`
- Modify: `src/aon/tools/gui/ui/gui-displays.cpp`
- Create: `tests/auton-selection-test.cpp`
- Modify: `tools/run-host-tests.ps1`

- [ ] Write failing tests for list-dependent index clamping, including alliance option 4 and the three-option Skills list.
- [ ] Restore Black Beard as Red 1 and preserve every existing Red, Blue, and Skills entry.
- [ ] Add `JerryIO Path` as Red 4 and Blue 4, both calling the same routine wrapper.
- [ ] Render alliance autonomous choices as a 2x2 grid while leaving Skills at three options.
- [ ] Make selection bounds derive from the active list and preselect Red 4.
- [ ] Verify selecting Blue 4 sets the alliance to Blue.
- [ ] Run host tests and commit `feat: add JerryIO red and blue GUI slots`.

### Task 4: Tune the routine and document the workflow

**Files:**
- Modify: `include/aon/jerryio/path-following.hpp`
- Modify: `src/aon/jerryio/path-following.cpp`
- Modify: `src/aon/jerryio/routines.cpp`
- Modify: `include/aon/jerryio/README.MD`
- Modify: `include/aon/tools/gui/GUIDE_GUI.md`

- [ ] Carry physical-profile settings through `FollowPathOptions` into `PathFollowerConfig`.
- [ ] Tune the JerryIO routine to 350 motor RPM, a 10-inch lookahead, and a conservative lateral-acceleration limit while retaining timeout diagnostics.
- [ ] Route the routine through the action-aware executor; keep its action list empty until the current path contains an intentional internal zero-speed marker.
- [ ] Document how to add internal zero-speed markers and bind intake/piston callbacks, including cleanup and timeout behavior.
- [ ] Document Red 4/Blue 4 selection and Red 4 preselection.
- [ ] Run all host tests, perform a clean PROS build, inspect the diff for LemLib runtime references, and commit `feat: tune JerryIO autonomous workflow`.
