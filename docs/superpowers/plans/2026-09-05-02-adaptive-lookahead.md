# Adaptive Lookahead Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add optional speed- and curvature-aware lookahead to AON's JerryIO follower without changing fixed-lookahead defaults.

**Architecture:** Precompute geometric curvature beside the velocity profile. At each step, calculate a bounded lookahead from normalized planned speed and dimensionless curvature-times-track-width, then use that distance for target sampling and terminal logic.

**Tech Stack:** PROS 4 C++17, AON `PathFollower`, host tests.

**Spec:** Approved in-chat controls design from 2026-09-05.

## Global Constraints

- Adaptive behavior is disabled by default.
- Fixed-lookahead callers retain identical output.
- Every distance and coefficient must be finite and validated.

### Task 1: Configuration and pure calculation

**Files:** Modify `include/aon/jerryio/path-follower.hpp`, `src/aon/jerryio/path-follower.cpp`, and `tests/path-follower-test.cpp`.

**Interfaces:** Add `AdaptiveLookaheadConfig { enabled, minimumDistance, maximumDistance, speedWeight, curvatureWeight }`; expose effective lookahead and path curvature through `PathFollowerOutput`.

- [ ] Write failing tests proving disabled mode returns the existing fixed distance, speed increases lookahead, curvature decreases it, and results remain within bounds.
- [ ] Verify the tests fail because adaptive fields/calculation do not exist.
- [ ] Precompute point curvature and interpolate it at current progress.
- [ ] Implement `L = clamp(base * (1 + speedWeight*speedFraction) / (1 + curvatureWeight*abs(kappa)*trackWidth), min, max)`.
- [ ] Reject enabled configurations with non-positive bounds, `min > max`, or negative/non-finite weights.
- [ ] Run the host suite and commit `feat: add adaptive JerryIO lookahead`.

### Task 2: Runtime options and documentation

**Files:** Modify `include/aon/jerryio/path-following.hpp`, `src/aon/jerryio/path-following.cpp`, `src/aon/drivetrain/path-following.cpp`, `tests/path-execution-policy-test.cpp`, and `include/aon/jerryio/README.MD`.

- [ ] Add failing option-validation tests for each adaptive field.
- [ ] Map options into `PathFollowerConfig` without enabling adaptive behavior in existing routines.
- [ ] Document the formula, disabled default, and field comparison procedure.
- [ ] Run host tests and a clean PROS build.
- [ ] Commit `feat: expose adaptive lookahead options`.

