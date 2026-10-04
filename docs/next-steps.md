# Next steps

Roadmap for high-rate control of the myCobot 280 (for Arduino), as of 2026-10-03. Background and measurements are in [fast-communication.md](fast-communication.md).

## Where things stand

- The laptop drives all six servos directly over the Feetech bus at **~300 Hz**: SYNC WRITE goals + SYNC READ position/speed/load each cycle. No failures were seen in 4 runs (~8,300 cycles in total).
- **Smooth multi-joint demo** (`tools/python/smooth_motion_demo.py`): all joints follow sine waves (±20–45°, 4 s period) from the zero pose. Recordings are in `tools/python/recordings/`. Two runs gave nearly identical results:

  | Joint | RMS error | Lag |
  | --- | --- | --- |
  | J1 | 3.9° | ~120 ms |
  | J2 | 2.4° | ~113 ms |
  | J3 | 3.2° | ~120 ms |
  | J4 | 1.1° | ~54 ms |
  | J5 | 1.3° | ~38 ms |
  | J6 | 1.4° | ~28 ms |

- Tracking error is almost entirely a **repeatable delay**. The big servos (J1–J3, P=32/D=8 on J1–J2) lag ~120 ms, the small ones (J4–J6, P=10/I=1) 30–55 ms.
- The servo's speed reading is quantised to ~4.4°/s (50 steps/s).
- Joints stick briefly at each reversal (static friction). The load reading flips sign with velocity, so friction dominates it.

## 1. Quick wins

- [x] **Lag compensation:** send each joint its target early by its measured lag (~120 ms for J1–J3, ~30–55 ms for J4–J6). On the circle demo it cut Cartesian error from 12.5 to 5.2 mm RMS.
- [x] **Kinematics in Julia with RigidBodyDynamics.jl** (`src/kinematics.jl`): FK, IK (damped least squares on RBD's Jacobian) and joint limits from the URDF. Matches the ATOM's `get_coords` within ~1 cm.
- [x] **ILC implemented** (`src/ilc.jl`, `scripts/ilc_step.jl`), tested on a model plant (RMS error halves in 5 iterations). First correction for the circle is in `tools/python/plans/circle_ilc1.csv` (≤ 0.9° per joint). **Not run on the robot yet.**
- [x] **ILC on the robot:** 3 iterations took the circle from 5.0 to **0.8 mm RMS** (max 12.4 → 2.2 mm).
- [ ] More ILC iterations (does it keep improving or plateau?), and ILC on a faster circle.
- [x] Error spikes on the circle: **J2 and J3 sticking after reversals** (J2 7–9 mm, J3 2–5 mm per spike), found with `scripts/attribute_error.jl`.
- [ ] **Feedforward through the servo model** instead of a pure time shift: command `q(t+τ) + T·q̇(t+τ)` with the fitted τ, T per joint.
- [x] Velocity estimators compared offline: finite differences + 10 Hz low-pass beat the speed register (2.2 vs 3.1°/s RMS), but the gain is small. A model-based (Kalman) estimator is the next step if needed.
- [x] Add the smooth-demo results to `fast-communication.md`.

## 2. Characterise the servos (model for TO / MPC / RL)

- [x] Delay + first-order response fitted per joint from the sine recordings (see `fast-communication.md`).
- [ ] **Step and chirp tests per joint:** delay, bandwidth and overshoot in position mode.
- [ ] Repeat with different **PID gains** (registers 21–23), **acceleration** (41; the ATOM uses 50) and **speed cap** (46–47). Can the ~120 ms lag on J1–J3 be reduced?
- [ ] **Friction model** (Coulomb + viscous, plus the stick at reversals) from the load and velocity recordings.
- [ ] Velocity mode: why it takes ~0.25 s to start moving at low speed, and how that changes with speed.
- [ ] Check what the load register actually measures against a known load. The offline fit suggests **PWM duty** (~0.53 % per °/s on J1, mostly back-EMF), not torque.

## 3. Fast-control layer in MyCobot.jl

- [x] Bus layer and player ported to Julia (`src/feetech.jl`, `src/ftdi.jl`, `src/player.jl`, `scripts/play_plan.jl`), with tests on a simulated bus. Checked read-only on the robot (`read_state` 2.02 ms).
- [x] **First playback with `scripts/play_plan.jl`:** ~285–300 Hz and 5.0 mm RMS on the circle (Python: ~240 Hz, 5.1 mm), after fixing dropped SYNC WRITEs (1 ms gap after each).
- [ ] **Allocation-free player loop.** Each cycle allocates 7.3 KB (`read_state` 5.2 KB, `write_goals` 1.8 KB), ~2.2 MB/s at 300 Hz, so the GC runs every so often and stalls the loop 30–80 ms. Preallocate packet and reply buffers (the pointer form of `sp_blocking_read`), fixed-size arrays instead of `Dict`s, then optionally `GC.enable(false)` for the duration of a run. RigidBodyDynamics' authors ran 1 kHz control on Atlas in Julia this way (Koolen & Deits, ICRA 2019).
- [ ] Use the SYNC WRITE gap for work instead of busy-waiting (compute the next command or log while waiting), and check whether 0.5 ms is enough (0 drops in 1000 at 0.5 ms; the 1 ms gap costs only ~0.2 ms per cycle on the robot because it overlaps USB frame timing).
- [x] Safety in the player: plan checks (starts/ends at zero, joint-limit margin, joint speed), a speed cap, and a tracking-error abort that holds every joint.
- [ ] A watchdog that holds position if the loop stalls (matters once PWM mode is used; in position mode the servos already hold the last goal).
- [ ] Hook it into the MeshCat viewer for live visualisation at full rate.
- [ ] Run trajectory optimisation in Julia, and play the result on the robot with the recorder. [TORA.jl](https://github.com/ferrolho/TORA.jl) is the natural tool: see **[tora-integration.md](tora-integration.md)** for what already exists on `hf/mycobot`, the missing inertial data, and the export steps.

## 4. Beyond position control

- [ ] **Inner loop on a microcontroller at the base** (ESP32 or RPi on the base UART pins instead of the FT232): no USB frames or latency timer, so a write + read cycle is limited by the ~110 bytes on the 1 Mbaud bus (~1.2 ms), roughly 700–800 Hz instead of ~300 Hz, with steady timing. The laptop would stream references at a lower rate. Most useful with PWM mode; in position mode the servos' own 30–120 ms lag dominates.

- [ ] **PWM mode** (mode 2) carefully on J1 (no gravity load), with a watchdog. It's the closest thing to torque control.
- [ ] **Custom firmware**, for a watchdog close to the servos and steady ~1 kHz loops:
  - The ATOM (ESP32) is the main option: back up the original with `esptool.py read_flash` first, then find the servo-bus pin with a multimeter.
  - The base board (Arduino Mega) is the alternative.

## 5. Loose ends

- [ ] Why did the ATOM refuse `send_angle` before it froze? It moves normally after a reboot.
- [ ] Is a mode change (EEPROM area, lock = 1) really lost at power-off?
- [ ] Identify the servo models (label on the back, Feetech's FD software, or ask Feetech).
- [ ] Post the findings on [elephantrobotics/myCobot#53](https://github.com/elephantrobotics/myCobot/issues/53), which still has no replies.
