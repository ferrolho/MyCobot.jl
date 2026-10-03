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
- [ ] **Learn the repeatable error:** two lag-compensated circle runs (one Python-planned, one Julia-planned) have almost the same error curve (5.2 vs 5.1 mm RMS, same spikes at the same moments). Iterative learning control (correct the next run's commands using this run's error) should remove most of it.
- [ ] Find out where the error spikes come from (~4, 8 and 12.5 s on the circle): probably sticking at reversals, maybe IK/model mismatch.
- [ ] **Velocity estimate from position:** differentiate the 0.088° position readings at 300 Hz with a filter (or a Kalman filter), instead of using the coarse speed register.
- [ ] Add the smooth-demo results to `fast-communication.md`.

## 2. Characterise the servos (model for TO / MPC / RL)

- [ ] **Step and chirp tests per joint:** delay, bandwidth and overshoot in position mode.
- [ ] Repeat with different **PID gains** (registers 21–23), **acceleration** (41; the ATOM uses 50) and **speed cap** (46–47). Can the ~120 ms lag on J1–J3 be reduced?
- [ ] **Friction model** (Coulomb + viscous, plus the stick at reversals) from the load and velocity recordings.
- [ ] Velocity mode: why it takes ~0.25 s to start moving at low speed, and how that changes with speed.
- [ ] Check what the load register actually measures (current? PWM duty?) against a known load.

## 3. Fast-control layer in MyCobot.jl

- [ ] Port `tools/python/mycobot_bus.py` and `play_trajectory.py` to Julia. The Python loop drops to ~236–250 Hz with occasional 20–30 ms hiccups while interpolating plans, so a Julia loop should help:
  - set the FT232R latency timer on connect (libusb control transfer),
  - `read_state` (SYNC READ),
  - `write_goals` (SYNC WRITE),
  - angle ↔ position conversion (`angle = sign × (pos − 2048) × 360/4096`, signs `[−1, −1, +1, −1, −1, −1]`).
- [ ] Safety: joint limits, speed caps, a tracking-error abort, and a watchdog that holds position if the loop stalls.
- [ ] Hook it into the MeshCat viewer for live visualisation at full rate.
- [ ] Run trajectory optimisation in Julia, and play the result on the robot with the recorder. [TORA.jl](https://github.com/ferrolho/TORA.jl) (RigidBodyDynamics + Ipopt) is the natural tool; there's an older `WIP with mycobot` commit in `~/git/TORA.jl`.

## 4. Beyond position control

- [ ] **PWM mode** (mode 2) carefully on J1 (no gravity load), with a watchdog. It's the closest thing to torque control.
- [ ] **Custom firmware**, for a watchdog close to the servos and steady ~1 kHz loops:
  - The ATOM (ESP32) is the main option: back up the original with `esptool.py read_flash` first, then find the servo-bus pin with a multimeter.
  - The base board (Arduino Mega) is the alternative.

## 5. Loose ends

- [ ] Why did the ATOM refuse `send_angle` before it froze? It moves normally after a reboot.
- [ ] Is a mode change (EEPROM area, lock = 1) really lost at power-off?
- [ ] Identify the servo models (label on the back, Feetech's FD software, or ask Feetech).
- [ ] Post the findings on [elephantrobotics/myCobot#53](https://github.com/elephantrobotics/myCobot/issues/53), which still has no replies.
