---
title: Roadmap
description: Open work, grouped by topic. Done items move to the results and history pages.
---

## Tracking accuracy

- [ ] More ILC iterations: does the error continue to go down, or does it stop near 1 mm? Try ILC on a faster circle.
- [ ] **Feedforward through the servo model** instead of a pure time shift: command `q(t+τ) + T·q̇(t+τ)` with the fitted τ and T per joint.
- [ ] Step and chirp tests per joint: delay, bandwidth and overshoot in position mode.
- [ ] Repeat with other PID gains (registers 21–23), acceleration (41) and speed caps (46–47). Can the ~120 ms lag of J1–J3 be smaller?
- [ ] A friction model (Coulomb, viscous, and the sticking after reversals).

## Vibration

- [ ] Set the servo acceleration register to 50 (as the stock firmware does) instead of 0. Compare the IMU vibration.
- [ ] Use the IMU (vibration, jerk) as a cost in planning or learning.

## ATOM firmware

- [ ] A watchdog for streamed commands, before PWM mode (plans that are stored onboard do not need the laptop during the run).
- [ ] Higher loop rates (600 Hz or more, reads of positions only) if a controller needs them.
- [ ] Streaming mode: setpoints from the laptop in real time, for replanning and MPC.
- [ ] Optional: a subset of the stock protocol (`FE FE` frames), so that myStudio and pymycobot basics work.

## Julia

- [ ] An allocation-free laptop loop (7.3 KB per cycle now, which causes 30–80 ms GC stalls). Preallocate buffers, use fixed-size arrays, and optionally `GC.enable(false)` during a run. See Koolen & Deits, ICRA 2019.
- [ ] Use the SYNC WRITE gap for work instead of a busy wait. Check if 0.5 ms is enough.
- [ ] Live MeshCat view from the telemetry.
- [ ] Connect [TORA.jl](/mycobot-280-lab/software/tora/) to the players.

## Beyond position control

- [ ] PWM mode (mode 2) on J1 (no gravity load), with a watchdog. It is the closest mode to torque control.
- [ ] Find what the load register measures, with a known load. The data suggests PWM duty.
- [ ] Real inertial data for the URDF (weigh the links, or identify them).

## Hardware

- [ ] Measure the logic level of base pins 13/14 and the `IOR` pin.
- [ ] Check if the base `TRVG` port carries the same serial line as pins 13/14.
- [ ] An emergency stop on the 12 V line.
- [ ] Connect the [gripper](/mycobot-280-lab/system/gripper/). Check the side port and the cable, then PING ID 7 and read its registers.
- [ ] Add the gripper to the Julia model (mass at the flange, and the `mimic` finger joints).

## Loose ends

- [ ] Why did the stock ATOM ignore `send_angle` before it froze?
- [ ] Is a mode change (EEPROM area, lock = 1) lost at power-off?
- [ ] Identify the servo models (label, Feetech's FD software, or Feetech).
- [x] Read the factory registers 80–86 (2026-10-04, [register map](/mycobot-280-lab/reference/registers/#factory-registers-8086)).
- [ ] Find what acceleration the servos use when register 41 is 0 (register 86 differs per servo type: 1 on J1–J3, 4–5 on J4–J6). Step tests with 41 = 0, 50 and 254.
- [ ] Record the stock firmware's bus traffic at power-up and during `send_angles` (registers 19 and 41, gripper ID 7). This is easier than reverse engineering the binary.
- [ ] Post the findings on [elephantrobotics/myCobot#53](https://github.com/elephantrobotics/myCobot/issues/53). A draft is in `~/myCobot/issue-53-reply-draft.md`.
