# Feedforward Characterization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Measure and apply AON drivetrain `kS`, `kV`, and `kA` without fabricated constants.

**Architecture:** Add an abortable characterization routine that emits raw voltage, velocity, and acceleration samples. Fit constants outside the real-time loop, review residuals, then add disabled-by-default voltage feedforward plus PID correction.

**Tech Stack:** PROS motor voltage/telemetry, C++17 CSV serial output, host math tests.

**Spec:** Approved in-chat controls design from 2026-09-05.

## Global Constraints

- No default feedforward constants other than zero.
- Disabled state and controller abort always command zero voltage.

### Task 1: Characterization and fitting

**Files:** Create `include/aon/controls/feedforward.hpp`, `src/aon/controls/feedforward.cpp`, `include/aon/competition/drivetrain-characterization.hpp`, matching source/tests, and documentation.

- [ ] Test `V = kS*sign(v) + kV*v + kA*a`, zero behavior, reversal, clamps, and non-finite rejection.
- [ ] Implement quasistatic and step tests with configurable voltage ceilings and fixed sample periods.
- [ ] Record forward/reverse and left/right datasets and calculate regression residuals before accepting constants.

### Task 2: Shadow and active integration

- [ ] Log predicted voltage without commanding it, compare residuals, then enable voltage control only after lead review and low-speed robot tests.

