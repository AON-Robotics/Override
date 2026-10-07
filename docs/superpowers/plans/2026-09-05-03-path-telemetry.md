# Path Telemetry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Measure JerryIO tracking quality with fixed-cost samples and summary metrics.

**Architecture:** `PathFollowerOutput` exposes pure geometric/control observations. A small telemetry accumulator computes count, maximum error, RMS error, saturation, and overruns. The PROS drivetrain adapter optionally emits decimated samples through a callback and returns metrics in `MotionResult`.

**Tech Stack:** PROS 4 C++17, `std::function`, AON host-test harness.

**Spec:** Approved in-chat controls design from 2026-09-05.

## Global Constraints

- Telemetry is disabled when no callback is supplied.
- No file I/O, allocation, or formatting occurs inside the follower step.
- Metrics are returned even when motion times out, is disabled, or is cancelled.

### Task 1: Observable follower output

**Files:** Modify `include/aon/jerryio/path-follower.hpp`, `src/aon/jerryio/path-follower.cpp`, and `tests/path-follower-test.cpp`.

- [ ] Write failing tests for cross-track error, effective lookahead, planned RPM, path curvature, steering curvature, and output saturation.
- [ ] Refactor projection internally to return both monotonic progress and distance error.
- [ ] Populate the new fields without changing wheel commands.
- [ ] Run host tests and commit `feat: expose JerryIO tracking observations`.

### Task 2: Metrics accumulator

**Files:** Create `include/aon/jerryio/path-telemetry.hpp`, `src/aon/jerryio/path-telemetry.cpp`, `tests/path-telemetry-test.cpp`; modify `tools/run-host-tests.ps1`.

**Interfaces:** Add `PathTelemetrySample`, `PathMetrics`, `PathMetricsAccumulator::observe`, `finish`, and `mergePathMetrics`.

- [ ] Write literal failing tests for RMS, maximum error, saturation count, overrun count, empty input, and merging action-leg metrics.
- [ ] Implement constant-space accumulation using a squared-error sum and integer counters.
- [ ] Run the full host suite and commit `feat: add path telemetry metrics`.

### Task 3: PROS runtime integration

**Files:** Modify `include/aon/jerryio/path-following.hpp`, `src/aon/jerryio/path-following.cpp`, `src/aon/drivetrain/path-following.cpp`, `tests/path-execution-policy-test.cpp`, and `include/aon/jerryio/README.MD`.

- [ ] Add `telemetryEveryNLoops` and `std::function<void(const PathTelemetrySample&)> telemetry` with disabled defaults.
- [ ] Validate decimation, sample current pose once, include commanded RPM and measured average drive RPM, and count loop overruns.
- [ ] Preserve path metrics across final-heading alignment and merge all action legs.
- [ ] Return endpoint and final-heading errors for every terminal status.
- [ ] Run host tests, clean PROS build, and commit `feat: integrate JerryIO path telemetry`.

