# EKF Shadow Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Repair the dormant AON EKF and evaluate it in telemetry-only shadow mode before it can control autonomy.

**Architecture:** Correct prediction math against the approved SE(2) estimator, require measured covariance configuration, and feed validated IMU/GPS observations through NIS gates. Active odometry remains authoritative throughout evaluation.

**Tech Stack:** C++17 fixed 3x3 matrices, PROS IMU/GPS, replay tests.

**Spec:** Approved in-chat controls design from 2026-09-05.

## Global Constraints

- Requires the SE(2) odometry project first.
- Zero/unmeasured covariance configuration cannot activate fusion.

### Task 1: Mathematical repair

**Files:** Modify `include/aon/odometry/ekf.hpp`, `src/aon/odometry/ekf.cpp`; create `tests/ekf-test.cpp`; modify host runner.

- [ ] Test the prediction Jacobian by finite differences, Joseph covariance symmetry, positive diagonals, wrapped heading innovation, GPS NIS acceptance/rejection, and singular measurement rejection.
- [ ] Correct the Jacobian and configuration validation; run host tests and commit.

### Task 2: Shadow evaluation

- [ ] Add timestamped wheel/IMU/GPS adapters and log active pose, EKF pose, covariance, innovation, and NIS without feeding EKF pose to controllers.
- [ ] Activate only after repeated field trials show lower measured endpoint error.

