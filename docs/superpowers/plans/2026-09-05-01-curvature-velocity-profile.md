# Curvature Velocity Profile Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Finish and harden AON's JerryIO curvature, acceleration, and braking velocity planner.

**Architecture:** Keep planning inside `PathFollower` construction so autonomous execution only interpolates a precomputed RPM envelope. Exported JerryIO speed is always an upper bound; curvature, forward acceleration reachability, and backward braking reachability may only reduce it.

**Tech Stack:** PROS 4 C++17, AON `PathFollower`, MSVC host tests, PROS ARM build.

**Spec:** Approved in-chat controls design from 2026-09-05.

## Global Constraints

- Use only AON code; do not add LemLib or another path library.
- Preserve inches, RPM, and seconds as the planner units.
- Do not invent physical constants; zero lateral acceleration keeps the curvature cap disabled.
- Preserve `static/path.jerryio.txt` and unrelated working-tree changes.

### Task 1: Forward acceleration reachability

**Files:** Modify `src/aon/jerryio/path-follower.cpp`; test `tests/path-follower-test.cpp`.

**Interfaces:** Consumes existing `velocityProfileRpm` and `cumulativeDistance`; preserves `double plannedSpeedRpm(double distance) const`.

- [ ] Add a failing test with speed samples `100, 20, 127` and assert the point after the slow sample is capped by `sqrt(v0^2 + 2*a*ds)` converted back to RPM.
- [ ] Run `./tools/run-host-tests.ps1` and confirm only the new acceleration-reachability assertion fails.
- [ ] Add a left-to-right pass using `maximumAcceleration * inchesPerSecondPerRpm`; clamp each point to the reachable speed from its predecessor.
- [ ] Run the full host suite and confirm exit code 0.
- [ ] Commit `fix: complete JerryIO velocity constraints`.

### Task 2: Numerical and behavioral verification

**Files:** Modify `tests/path-follower-test.cpp`; update `include/aon/jerryio/README.MD` only if behavior text is inaccurate.

- [ ] Add literal tests for a straight path, a three-point corner, disabled lateral limiting, reverse mode, short finite segments, and zero-speed endpoint braking.
- [ ] Run the full host suite.
- [ ] Run a clean PROS ARM build with `make.exe clean` then `make.exe all`.
- [ ] Confirm the checked-in routine's lateral acceleration is physically validated before raising it; otherwise leave the feature configurable and document the field gate.

