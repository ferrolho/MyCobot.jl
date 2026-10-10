---
title: Roadmap
description: Open work, grouped by topic. Done items move to the results and history pages.
---

## Control page and website (handed over 2026-10-04)

- [x] Confirm moving the real arm from the Control page (2026-10-05: GitHub Pages in Chrome on a laptop, `mycobot.local`).
- [ ] Away from home: a TCP forward on the Pi (Tailscale :8282 → ATOM :80), so the page reaches the ATOM over Tailscale.
- [x] Browser flasher with Improv WiFi setup and a Search button (Setup page, firmware 4.3.0). Tested on the real ATOM on 2026-10-05: install with Erase, WiFi set up over USB, Search found 4.3.0 at `mycobot.local`, and the Control page moved the arm.
- [ ] Test the CI firmware build (it runs on the first push to GitHub) and an update over 4.3.0 that keeps the WiFi network.
- [ ] An address sweep when `mycobot.local` does not resolve.
- [x] End-effector jog: [gamepad teleoperation](/mycobot-280-lab/software/gamepad/) on the Control page (2026-10-09). A first version (the controller in the browser, TRACK) moved the real arm.
- [x] End-effector JOG in the firmware (JOG frames 1 and 2, firmware 5.1): the twist integrated on the ATOM at 500 Hz (2026-10-09, tested on the laptop and the simulator).
- [x] Flash 5.1 and test end-effector JOG on the arm with `twist_ws_test.py` (2026-10-10: 40.15 mm for 40, a 10.11° turn with the TCP within 0.06 mm; `twist_us` about 450 µs).
- [ ] Test the gamepad on the arm with 5.1 (straight lines, turns about the TCP, the stops before J3 = 0° and J5 = ±90°, a WiFi stall).
- [ ] Live mode with TRACK (firmware 4.4): used on the real arm on 2026-10-05, up to 90 °/s; a blocked joint stops the arm with a following error. Still to check: the joints settle on the goal without hunting.
- [x] Test Chrome's local-network prompt for real users on GitHub Pages (2026-10-05: works from the public page; the laptop webcam works too).
- [ ] The page assumes the real robot until the first status-log line arrives (the simulator's starts with "atom-sim").

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

## State estimation

- [ ] **Better joint velocity estimates.** The servo speed register moves in steps of 50 steps/s (≈ 4.4°/s), and differentiating the position at 500 Hz turns one count (0.088°) into ~44°/s of noise. Estimate the velocity with a filter (e.g. a Kalman filter) that combines position, servo speed, the command and the servo model (`src/servo_model.jl`), and the IMU gyro for the wrist joints. Needed for MPC and learning.

## Accuracy and calibration

See [IMU calibration and encoder errors](/mycobot-280-lab/results/imu-encoders/) (2026-10-06).

- [x] Check the pitch zero offsets with the IMU (2026-10-06: δ2 + δ3 + δ4 = 0.2–0.3°, no correction needed).
- [x] One robot description for the firmware, the Control page, the simulator and Julia ([Robot description](/mycobot-280-lab/software/robot-description/)).
- [x] **J5 encoder correction** in the Julia package (2026-10-06): 3.7° → 0.18° peak to peak over ±148°.
- [ ] **Store the calibration on each ATOM** (NVS) and apply it in the firmware and the Control page. The public firmware and the GitHub Pages site serve every user's arm, so the values cannot be built in.
- [ ] J1 (0.56° peak to peak) and J4 (1.0°): add their corrections if a task needs them. J4's axis is always horizontal, so its play is in every J4 sweep: separate the play from the encoder error first.
- [ ] **Gravity model for the play of J2–J4 and the bending of J2:** identify the masses and centres of mass (`inertials.yaml`) from the IMU and the encoders, not from the load register. Then predict which side of the play each link rests on.
- [ ] J2 and J3 oscillate at rest in some poses with the integral gains (9 of 60 holds, 0.3–0.5°). Find gains or a deadband that stop it. Check whether this is the hunting in Live mode.
- [ ] J4 and J5 settle up to 1° and 2.4° from their goal (no integral action). Try integral action on J4–J5.
- [ ] Calibration file per robot: the IMU mounting and accelerometer calibration, encoder corrections, play.
- [ ] Separate δ2, δ3 and δ4 with a second IMU (a phone) on link 2 or 3, if a tool needs it.
- [ ] Measure the flange accuracy with an external reference (pen on paper, dial gauge, a cone plate milled on the CNC).

## Beyond position control

- [ ] PWM mode (mode 2) on J1 (no gravity load), with a watchdog. It is the closest mode to torque control.
- [ ] Find what the load register measures, with a known load. The data suggests PWM duty.
- [ ] Real inertial data for the URDF (weigh the links, or identify them). Elephant's URDFs have placeholders only (checked 2026-10-06). See *Gravity model* above.

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
