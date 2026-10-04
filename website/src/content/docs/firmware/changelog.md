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
