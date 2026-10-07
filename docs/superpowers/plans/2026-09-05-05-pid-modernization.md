# PID Modernization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Provide a deterministic AON PID controller with measured time, filtering, saturation-aware integration, and settle reporting.

**Architecture:** Introduce a pure PID calculation module while keeping the legacy class until each caller is migrated. Migrate JerryIO heading alignment first and legacy movements only after robot validation.

**Tech Stack:** C++17, AON host tests, PROS timer adapter.

**Spec:** Approved in-chat controls design from 2026-09-05.

## Global Constraints

- Gains remain explicit configuration; do not invent tuning values.
- Invalid `dt`, error, or configuration returns an invalid zero output.

### Task 1: Pure PID module

**Files:** Create `include/aon/controls/pid/controller.hpp`, `tests/pid-controller-test.cpp`; modify `tools/run-host-tests.ps1`.

- [ ] Test proportional output, trapezoidal integral, derivative suppression on first sample, derivative low-pass filtering, output clamps, conditional anti-windup, reset, and settle duration.
- [ ] Implement `PidOutput update(double error, double elapsedSeconds)` exposing P/I/D terms, saturation, settled, and valid flags.
- [ ] Run host tests and commit `feat: add robust AON PID controller`.

### Task 2: Controlled migration

**Files:** Modify JerryIO heading-alignment files and tests, then legacy controllers in separate commits.

- [ ] Migrate final heading, verify host and robot turn tests, then decide whether legacy drive/turn migration is justified.

