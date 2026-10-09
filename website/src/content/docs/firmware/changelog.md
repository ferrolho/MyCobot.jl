---
title: Firmware changelog
description: Versions of the ATOM controller firmware (atom_controller), with what changed in each.
---

The controller firmware uses [semantic versioning](https://semver.org/):

- **MAJOR** for protocol changes that break old clients,
- **MINOR** for added commands (old clients still work),
- **PATCH** for fixes only.

PING reports the version (`MyCobot.atom_ping(link).version`), and the status log on
UDP port 5005 shows it with the git commit of the build, for example
`atom_controller v4.0.0 (bff4529)`. Each version has a git tag
`atom-controller-vX.Y.Z` (local until the repository is pushed).

## 5.0.1 — 2026-10-09

- **Smoother slow motions:** at power-up the ATOM also writes each joint's minimum starting force
  (register 24) and dead zone (registers 26/27) from `servos.yaml`: 20 and 1 on J2–J4 (factory: 0
  and 3). In a 4 mm/s descent J2 then follows its plan within ±0.4° instead of moving in steps of
  about 1°. See [Servos](/mycobot-280-lab/system/servos/#slow-motions-starting-force-and-dead-zone).

## 5.0.0 — 2026-10-09

Breaking: STREAM, STATE, MOVE_TO, JOG and TRACK change; GRIPPER is removed. Update the
clients with the firmware (the Control page reads both STREAM layouts).

- **The gripper is joint J7.** While the ATOM finds it, every joint message has 7 joints
  (J1–J6, then J7) in the same units (degrees, same formula; limits −51.5° closed to 0°
  open, the end stops). Without it, 6. See [J7](/mycobot-280-lab/comms/websocket-api/#joints-j1j6-and-j7-50).
- J7 is in every bus read and write of the joints (the group read takes about 0.2 ms
  more), so it streams at the same rate as J1–J6 (it was read 10 times a second), with
  speed, temperature, voltage and status.
- **MOVE_TO, JOG and TRACK take 6 or 7 values** (the length tells). With 6, J7 holds its
  goal; 7 without a gripper is refused with −7. J7 follows the same minimum-jerk and
  TRACK profiles as the other joints (90 °/s, 2000 °/s²), so it arrives with them.
- **J7 has no margin:** its goals, JOG and TRACK use the whole range, end stop to end stop
  (J1–J6 keep 2° inside their limits). Grasps drive the jaws against the object or the stops,
  and a closed gripper at rest (about −50.9°) can hold its goal.
- **GRIPPER (`0x11`) is removed**, and with it the torque-off value (`0xFFFF`).
- **No sag after a run, JOG, TRACK or HOLD.** Runs start from the goals the joints hold,
  not from the present position (a goal more than 3° from the position, after the arm was
  moved by hand, starts from the position). A JOG or TRACK that stops normally keeps its last
  goals, and HOLD writes the held goals again. Only power-up and errors hold at the present
  position. Before: each reset of a goal to the present position let a joint that gravity
  loads sag by its position error (on 2026-10-09: J4 and J5 about 1° per jog, 3–4° over a
  few tests). Clients should also start plans from the held goals (Julia `atom_move_to` does).
- **LED progress as a cyan spiral:** over a dark matrix, the pixels come on one at a time
  from the centre out to the top-left corner (25 steps, each halfway through its part of the
  motion; full during the 0.5 s of settling; 0 at the start of each run). Before: 5 white
  pixels in the bottom row, behind the motion. See [LED matrix signals](/mycobot-280-lab/firmware/led-signals/).
- **Grasps:** J7 has no following-error check, and a hold (HOLD, STOP, end or abort of a
  run) keeps its goal. Runs start J7 from its goal. A grasp stays closed.
- **STREAM** has per-joint arrays of n and the goal of each joint: 21 + 11·n bytes (87 or
  98), control byte at offset 7, n at offset 8. **STATE** has `u8 n` after `ok`.
- Temperature, voltage and status: one joint every 1/n s (each joint once a second),
  also during runs (before: idle only, all joints at once). While idle the ATOM reads
  them only while someone subscribes or J7 is there (for the thermal derating).
- JOG and TRACK move a joint that starts outside its range (moved by hand, or J7 on its
  open end stop) back in smoothly. Before, it jumped onto the range edge in one step.
- When it finds the gripper, the ATOM also writes its position-loop gains (150/150/0, the
  values found on it) and holds it where it is (before: torque off until the first
  GRIPPER).
- Status log: `joints=6|7` (and `j7_derated` while derated) instead of `gripper_c=N`
  (J7's temperature is in the STREAM).
- **Plans** have 6 or 7 joints: PLAN_BEGIN has an optional `u8 joints` (default 6), and PLAN_DATA
  samples have that many. PLAY of a 7-joint plan without a gripper is refused with −7. With a
  6-joint plan or PLAY_SIGNAL (J1–J6), J7 holds its goal.
- **TELEM** samples have the joints of the run's command (17 + 10·n bytes): a 6-joint plan or
  MOVE_TO gets the 77-byte samples as before; 7 joints give 87 bytes (16 per packet).
- **STATE** has the goal of each joint after the positions (as the STREAM).

## 4.7.0 — 2026-10-06

- **Gripper at full torque.** When the ATOM finds the gripper (at power-up or when it is
  plugged in), it writes registers 16 (max torque), 28 (protection current) and 48 (torque
  limit) to `torque` in `servos.yaml` (1000 = 100 %). Elephant's values (140 / 300 / 300)
  limited it to 30 %, and register 28 caps register 48. Soft objects held only at 100 %.
- **Thermal derating.** The ATOM reads the gripper temperature about once a second. Above
  70 °C the torque limit drops to 500, and below 60 °C it comes back to 1000. The status log
  has `gripper_c=N` (and `gripper_derated` while derated).

## 4.6.1 — 2026-10-06

- **WiFi watchdog.** After a drop, the ATOM joins the saved network again every 15 s
  until it works. Before, it relied on the ESP32's own auto-reconnect: after a router
  restart on 2026-10-06 the ATOM stayed off the network until a power cycle.
- mDNS (`mycobot.local`) starts again after each reconnect (before: only once).
- The status log has `wifi_drops=N` (drops since power-up). The Control page shows it
  in the footer when it is not 0.

## 4.6.0 — 2026-10-06

- **Gripper.** The ATOM finds the adaptive gripper (servo ID 7, model `0x070A`) at
  power-up and once a second after, and reads it 10 times a second while it is there.
  New command **GRIPPER** (`0x11`): u16 opening in 0.1 % (0 closed, 1000 open), or
  `0xFFFF` for torque off. It needs control and works also during MOVE_TO, JOG and
  TRACK.
  Removed in 5.0 (J7 moves with MOVE_TO, JOG and TRACK).
- STREAM has 5 more bytes (79): gripper found, opening (0.1 %), load (0.1 %).
  Older clients read the first 74 bytes and still work.
- The opening maps to the servo end stops in `servos.yaml` (key `gripper`; generated
  into `robot_params.h`).

## 4.5.1 — 2026-10-06

- J6 limits **−225° to +135°** (was −220°): one full turn, so the gripper can face
  every direction.
- **Power-up guard:** at ±135° the one-turn reading cannot tell the turn. If J6 is
  within 1.5° of it at power-up, J6 goes limp and the ATOM stays in ERROR. The status
  log says `TURN UNKNOWN`. Turn J6 a few degrees by hand, then send HOLD: it finds the
  turn again. Under control the arm never stops there (all motions stop 2° inside the
  limits).

## 4.5.0 — 2026-10-06

- **J6 past one turn.** With the gripper on, its cable lets J6 turn from −237° to +140°,
  so the J6 limits are now **−220° to +135°**. At power-up the firmware sets J6's phase
  bit 4 (multi-turn reading) and angle limits 0/0 (goals past one turn), in RAM, for
  each joint marked `multi_turn` in `servos.yaml`. Tested on J6 on 2026-10-06: it read
  −231.8° and moved to −209.3° and back. See [Servos](/mycobot-280-lab/system/servos/#multi-turn-j6).
- After a servo power-up the servo counts from its one-turn reading. The limits span
  less than one turn, so a reading in the gap between them (past its middle, +137.5° on
  J6) is one turn off; the firmware corrects it (`turn_offset`).
- Positions in STATE, STREAM and plans are steps as before (2048 + sign × angle × 4096 / 360).
  On J6 they now go past 0–4095 (−220° is 4551). REG_READ and REG_WRITE stay raw.
- **Limits need not be symmetric:** `robot_params.h` has `LIMIT_MIN_DEG` and
  `LIMIT_MAX_DEG` (was `LIMIT_DEG`, ±). MOVE_TO, JOG, TRACK and PLAY_SIGNAL use both.
- PONG `gains_ok` is 1 when the gains and the multi-turn setup both took.
- No change in the messages: 4.4 clients work, but a client that clamps positions to
  0–4095 shows J6 wrong past −180°.

## 4.4.1 — 2026-10-06

- **Fix:** the J6 limit is ±135° (was ±180°). With the gripper on, its cable stops
  J6 at −237.1° and +139.9°. See [Gripper](/mycobot-280-lab/system/gripper/#on-this-arm-2026-10-06).
  Only `robot_params.h` changed (from `tools/gen_robot.py`); no protocol change.

## 4.4.0 — 2026-10-05

- **TRACK** (`0x10`): the client sends a goal pose and a speed cap (at most 90 °/s).
  The ATOM moves each joint there at its acceleration limit (400 or 2000 °/s², as
  MOVE_TO) and brakes to stop exactly on the goal (`motion::track_step`, with C++
  tests). 200 ms deadman; STOP and HOLD brake the joints. New state 8, **tracking**
  (LED as jogging).
- The following-error limit in TRACK grows with the joint's recent peak speed (it
  decays over 0.3 s): 20° + 0.15 s × speed. With a fixed 20°, fast fader swings at 90 °/s stopped the arm (J1 overshot to
  105 °/s and lagged by more than 20°). JOG and TRACK report a stop for a following
  error with DONE (result 1, the joint and its error). See [TRACK](/mycobot-280-lab/comms/websocket-api/#track-live-mode).
- Why: the Control page's Live mode sent JOG velocities from the browser: at most
  30 °/s and 200 °/s², with a slow approach (2.5 × the error, about 1.5 s for the last
  degrees). Live mode now uses TRACK: J1 moves 120° in 1.6 s instead of more than 4 s,
  and 10° in under 0.4 s (host tests).

## 4.3.1 — 2026-10-05

- **Fix:** a MOVE_TO from a WebSocket client sends no TELEM. Over TCP, the 500 Hz
  telemetry (about 28 packets/s) blocked the network task: the STREAM paused for
  0.1–0.5 s many times per move (17–30 packets/s instead of 50), and replies came
  0.3–0.7 s late. Found with the Control page's session log on 2026-10-05. The
  STREAM shows the motion; DONE still ends the move. UDP clients and PLAY and
  PLAY_SIGNAL keep the telemetry.

## 4.3.0 — 2026-10-05

Set up the robot from the browser: see [Set up the robot](/mycobot-280-lab/start/setup/).
Tested on the real ATOM on 2026-10-05: install from the Setup page, Improv WiFi setup,
Search, and control from the Control page.

- **Improv WiFi** over the USB serial port (`improv.h`, with C++ tests): the Setup
  page's installer sends the WiFi network and password. The ATOM saves them in NVS
  (namespace `wifi`) only after it connects, and uses them at power-up.
- WiFi at power-up: the saved network first, then the network compiled in from
  `wifi_secrets.h` (lab builds). Without either, the LED matrix **blinks white**.
- **Public build** (`-DPUBLIC_BUILD`, `tools/build-public-firmware.sh`): no WiFi
  secrets and no OTA. GitHub Actions builds it for the Setup page.
- The status log shows the build: `atom_controller v4.3.0 (<git>, public)` or `lab`.
- No protocol change: UDP and WebSocket clients of 4.2 work unchanged.

## 4.2.0 — 2026-10-04

A browser can now see and control the arm directly; see the
[WebSocket API](/mycobot-280-lab/comms/websocket-api/).

- **WebSocket** `ws://<ATOM>/ws` on port 80: the same binary messages as UDP, and the
  status log as text frames. **mDNS** name `mycobot.local`.
- **Control**: one client at a time may move the robot or write registers
  (CONTROL `0x0D`). If nobody has control, such a command takes it for its sender,
  so old clients keep working. Control ends on release, on WebSocket close, or 2 s
  after the holder's last message (not during its run). HOLD and STOP work for
  every client.
- **MOVE_TO** (`0x0E`): minimum-jerk move to a pose; the shortest duration within
  90°/s and the servo acceleration limits.
- **JOG** (`0x0F`): joint velocities, ≤ 30°/s and 200°/s², stops 2° inside the
  limits; **deadman** 200 ms with a ramped stop.
- STREAM has a control byte (74 bytes); states 6 (moving) and 7 (jogging).
- One limits table (`motion_limits.h`); MOVE_TO/JOG logic in `motion.h` with C++
  property tests. Smoke test on the robot: `tools/firmware-tests/ws_smoke_test.py`
  (all checks passed: control hand-over between UDP and WebSocket, JOG and deadman,
  MOVE_TO).
- Fixed during testing: UDP replies went to the last WebSocket client.

## 4.1.0 — 2026-10-04

Several clients can now use the ATOM at the same time.

- **Replies go to the sender** of each request (its IP address and UDP port).
  Before, all replies went to the last sender's IP, port 5007, so a second client
  took the telemetry away from a running test. `MyCobot.AtomLink` (sends from port
  5007) sees no change.
- **PLAY and PLAY_SIGNAL telemetry and DONE go to the client that started the run.**
- **SUBSCRIBE** (`0x0C`): a state stream at 1–100 Hz to up to 4 clients, also while
  a plan plays (`0x88` STREAM, 73 bytes). Idle: the ATOM reads the state at the
  stream rate, and temperatures, voltages and status once a second.
- Tested with two clients: one played a chirp (7250 of 7250 samples received, 0 late
  cycles), the other received the stream at 20 Hz and pings during the run.

## 4.0.0 — 2026-10-04

**Breaking:** the telemetry sample is now 77 bytes (was 53).

- Telemetry samples include the **command and the reference** that the ATOM used
  (`u16 cmd[6], u16 ref[6]` after the time stamp), 18 samples per packet. The
  recordings show what ran onboard, so the host does not compute them.
- Test signals: smooth fade in/out (minimum jerk) and a smooth amplitude limit. The
  3.1 chirp had speed steps of up to ~20°/s at the ends of its linear fades and
  ~1.6°/s where the amplitude limit changed. The generator has C++ property tests
  (`tools/firmware-tests/`).
- The host no longer has its own copy of the test signals.

## 3.1.0 — 2026-10-04

- **PLAY_SIGNAL** (`0x0B`): chirp and step test signals computed onboard
  (`test_signal.h`). The ATOM checks the parameters and the start pose; nothing is
  uploaded, so plan memory does not limit the duration.
- **Cubic plans**: PLAN_BEGIN takes an optional byte, 1 = Catmull-Rom between
  samples. 50 Hz cubic reproduces our plans to within half a servo step, with 5× less
  memory than 250 Hz linear.
- Semantic version: PING adds `u8 minor, u8 patch`; the status log shows the git
  commit (`FW_GIT`, set by the build).

## 3.0.0 — 2026-10-04

- Writes **our position-loop gains** (`GAINS`: 32/4/16 on J1–J3) at power-up,
  verified; PING reports `gains_ok`.
- **REG_READ** (`0x09`) and **REG_WRITE** (`0x0A`): servo registers over WiFi.
  Writes to registers 0–8, 55 and 80+ are refused.

## 2.0.0 — 2026-10-04

- Setup writes (goal = present, acceleration, speed cap) are verified by reading
  them back. In 1.0.0 back-to-back writes were sometimes lost.
- The LED matrix is updated only by the network task, and replies go through a
  queue (the UDP sockets are not safe across cores).

## 1.0.0 — 2026-10-04

- First version: 500 Hz onboard player (core 1), plan upload with CRC-32C, IMU
  telemetry, UDP commands, status log and OTA updates (core 0).
