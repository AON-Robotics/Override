# Nonlinear Move To Pose Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a convergent polar-coordinate controller for differential-drive `moveToPose` using AON localization and constraints.

**Architecture:** A pure controller converts pose error into linear and angular commands; an adapter converts them to left/right RPM and applies existing speed constraints and motion safety policy.

**Tech Stack:** C++17, AON Pose, JerryIO motion status policy, host tests.

**Spec:** Approved in-chat controls design from 2026-09-05.

## Global Constraints

- Requires corrected SE(2) odometry and characterized drivetrain behavior.
- Gains and reverse-selection thresholds remain explicit configuration.

### Task 1: Polar controller

**Files:** Create `include/aon/controls/nonlinear-pose-controller.hpp`, matching source/test; modify host runner.

- [ ] Test targets ahead, behind, beside, at zero distance, across heading wrap, with reverse enabled, and with non-finite input.
- [ ] Implement wrapped `rho`, `alpha`, and `beta` errors; calculate bounded `v` and `omega`; convert to differential wheel RPM.
- [ ] Run host tests and commit `feat: add nonlinear pose controller`.

### Task 2: Monitored drivetrain integration

**Files:** Modify drivetrain declarations/implementation and execution-policy tests.

- [ ] Reuse timeout, disabled, cancellation, minimum-output, settle, and telemetry behavior.
- [ ] Keep current JerryIO two-point `moveToPose` selectable until field trials approve replacement.
- [ ] Run host tests, clean PROS build, and staged robot validation.

