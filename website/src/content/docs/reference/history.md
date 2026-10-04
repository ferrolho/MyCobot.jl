---
title: History
description: What was done when, and the decisions on the way.
---

## 2024–2025

- **2024-04:** repository started.
- **2025-02:** the Julia package implements the stock ATOM protocol from scratch. It shows that pymycobot is not the bottleneck.
- **2025-02:** TORA.jl branch `hf/mycobot` solves a circle and plays it at 20 Hz through the stock ATOM.
- **2025-04:** [elephantrobotics/myCobot#53](https://github.com/elephantrobotics/myCobot/issues/53) reports 20 ms per `get_angles` (50 Hz). No replies.
- **2025-04 to 11:** a byte-by-byte frame reader and retries (workarounds for the latency). Never committed; kept in a git stash.
- **2025-11:** transponder mode is not supported. A plan to find the servo bus on the base pins; no results recorded.

## 2026-10-03

- The FT232R **latency timer** (16 ms) causes most of the 20 ms. 1 ms gives 8.5 ms.
- The servos are **Feetech STS**. The laptop can **read and write the servo bus directly**.
- Velocity mode on J1 works.
- The goal-position rule is corrected: goal = present, not present + offset. Goal speed 0 means no motion.
- A 300 Hz closed loop from the laptop. Smooth multi-joint motion. Lag of 30–120 ms per joint.
- Kinematics moves from hand-written Python to **RigidBodyDynamics.jl** in Julia (the hand-written version was deleted).
- The circle: 12.5 mm → 5.1 mm RMS with lag compensation.
- Offline analysis: the circle's error peaks are J2/J3 sticking after reversals; a delay + first-order model per joint; the load register looks like PWM duty.
- The servo bus layer and the player move to **Julia**, with tests on a simulated bus.
- Iterative learning control is implemented.

## 2026-10-04

- The Julia player at full rate loses SYNC WRITEs. A 1 ms gap fixes it: 5.0 mm at 284 Hz.
- **ILC on the robot:** 5.0 → 2.5 → 1.3 → 0.8 mm RMS in three runs.
- With the ATOM removed, the servos still answer: **the base bridges the bus**.
- The stock ATOM firmware is **backed up** (4 MB, verified).
- The **bus probe** firmware finds the ATOM's bus pins (G19 RX, G22 TX) and measures 1.26 ms per read on the ATOM.
- The **controller firmware** plays the circle at **500 Hz** onboard: 5.0 mm, then 1.0 mm after three ILC runs. The IMU shows vibration bursts at joint reversals that ILC does not change.
- This documentation site replaces the Markdown notes in `docs/`.

## Decisions

| Decision | Reason |
| --- | --- |
| Kinematics only through RigidBodyDynamics.jl | No hand-written kinematics. One model (the URDF) for planning, tracing and the viewer. |
| Planning stays on the laptop; the ATOM plays and records | The ATOM has the timing; the laptop has the compute. |
| Plans are stored on the ATOM, not streamed | A WiFi drop cannot interrupt a run. Streaming is on the roadmap for MPC. |
| The ATOM holds the pose at power-up with goal speed 0 | No motion until a plan sets a speed. |
| The button is not an emergency stop | It moves with the end effector. |
| Firmware backups and WiFi credentials stay outside the repository | Proprietary firmware and secrets. |
