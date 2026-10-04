---
title: ATOM link (WiFi)
description: The UDP protocol between the laptop and the controller firmware, and the Julia API that uses it.
---

The laptop talks to the [controller firmware](/mycobot-280-lab/firmware/controller/) over
WiFi with UDP. All values are little-endian.

| Port | Direction | Content |
| --- | --- | --- |
| 5006 | laptop → ATOM | Commands |
| 5007 | ATOM → laptop | Replies and telemetry, sent to the address of the last command |
| 5005 | ATOM → broadcast | Status log, one text line per second |

## Messages

From 4.2 the same messages also go over a WebSocket (`ws://<ATOM>/ws`), and commands that move the robot need control; see the [WebSocket API](/mycobot-280-lab/comms/websocket-api/) and the [changelog](/mycobot-280-lab/firmware/changelog/).

| Code | Message | Content | Reply |
| --- | --- | --- | --- |
| `0x01` | PING | — | `0x81` PONG: u16 version, u8 state, u32 plan samples, u16 plan rate, u8 IMU ok, u8 gains ok (3.0+), u8 minor, u8 patch (3.1+) |
| `0x02` | STATE | — | `0x82` STATE: u8 ok, u16 position[6], u16 speed[6], u16 load[6], i16 acc[3], i16 gyro[3] |
| `0x03` | HOLD | — | `0x83` ACK |
| `0x04` | PLAN_BEGIN | u32 samples, u16 rate (Hz), optional u8 interpolation (0 linear, 1 cubic; 3.1+) | ACK (`-3` = not enough memory) |
| `0x05` | PLAN_DATA | u32 offset, u16 count, count × {u16 cmd[6], u16 ref[6]} | ACK with the offset |
| `0x06` | PLAN_END | u32 CRC-32C of all samples | ACK (`-1` = CRC mismatch) |
| `0x07` | PLAY | u16 rate (Hz), u16 speed cap, u16 max error (steps), u16 start tolerance (steps) | ACK, then TELEM packets, then DONE |
| `0x08` | STOP | — | ACK |
| `0x09` | REG_READ (v3+) | u8 servo id (1–7), u8 address, u8 length (1–32) | `0x86`: u8 id, u8 address, u8 length, i8 status, data |
| `0x0A` | REG_WRITE (v3+) | u8 servo id, u8 address, u8 length, data | `0x87`: u8 id, u8 address, i8 status, u8 servo error |
| `0x0B` | PLAY_SIGNAL (3.1+) | the PLAY parameters, then `sig::Params` (38 bytes: u8 joint, u8 kind, f32 amp, f0, f1, duration, vmax, amax, i16 base[6] in 0.01°) | ACK (−11…−18 invalid parameters, −29/−30 bad start pose), then TELEM and DONE |
| `0x0D` | CONTROL (4.2+) | u8 action: 0 release, 1 take, 2 take over | ACK: 0, −2 another client has control, −1 robot moving |
| `0x0E` | MOVE_TO (4.2+) | i16 goal[6] (0.01°), u16 duration (ms, 0 = shortest) | ACK, TELEM, DONE |
| `0x0F` | JOG (4.2+) | u8 frame (0 = joints), i16 velocity[6] (0.1°/s); 200 ms deadman | ACK only if refused |
| `0x0C` | SUBSCRIBE (4.1+) | u16 rate (Hz, 1–100; 0 = stop), renew at least once a second | `0x88` STREAM packets to the sender; see [Clients](/mycobot-280-lab/comms/clients/#firmware-41) |

REG_READ and REG_WRITE are refused while a plan plays. The ATOM reads every write
back. It refuses writes to registers 0–8 (ID, baud rate and other comms settings),
55 (EEPROM lock) and 80+ (factory). Status: 0 ok, −1 busy or bad request, −4 no
reply, −5 not allowed, −6 the read-back differs. Writes to the EEPROM area last
until the next power cycle.
| `0x84` | TELEM | u32 first sequence number, u8 n, n × sample | — |
| `0x85` | DONE | u8 result, u32 cycles, u32 max period (µs), u32 late cycles, u32 telemetry dropped, u8 joint, i16 error (steps) | — |

ACK (`0x83`) is: u8 message code, i8 status (0 = OK), u32 value.

### Plans

A plan is a list of samples at a fixed rate (250 Hz by default). Each sample has
two sets of six servo positions, in steps:

- **cmd**: the goals the ATOM sends (lag-compensated or learned).
- **ref**: the wanted positions. The ATOM uses them for the start check and the tracking check.

The ATOM interpolates linearly between samples at the control rate. The circle
(15 s) is 3751 samples, about 90 KB.

### Telemetry sample (53 bytes)

`u32 t_us, u16 position[6], u16 speed[6], u16 load[6], i16 acc[3], i16 gyro[3], u8 ok`

The values are raw register values. `src/atom.jl` converts them to degrees, °/s,
%, g and °/s. The ATOM sends 20 samples per packet.

### DONE results

| Code | Result |
| --- | --- |
| 0 | done |
| 1 | tracking error (the ATOM holds the pose) |
| 2 | stopped (STOP received) |
| 3 | not at the start pose |
| 4 | bus error |

## Telemetry sample (4.0+)

77 bytes: `u32 t_us, u16 cmd[6], u16 ref[6], u16 pos[6], u16 speed[6], u16 load[6], i16 acc[3], i16 gyro[3], u8 ok`.
Up to 18 samples per TELEM packet. Firmware before 4.0 sent 53-byte samples without `cmd` and `ref`;
`decode_telemetry` reads both.

## Julia API

```julia
import MyCobot
link = MyCobot.AtomLink("192.168.1.107")
MyCobot.atom_ping(link)                     # version, state, plan, IMU
s = MyCobot.atom_state(link)                # q (°), dq (°/s), load (%), imu (g, °/s)
MyCobot.atom_move_to(link, zeros(6))        # MOVES THE ROBOT: minimum-jerk move
rec, done = MyCobot.atom_play_trajectory(link, t, q_plan)   # MOVES THE ROBOT
MyCobot.write_atom_recording_csv("rec.csv", rec)
MyCobot.atom_read_reg(link, 1, 62, 2)       # servo 1: voltage (0.1 V), temperature (°C)
MyCobot.atom_write_reg(link, 1, 21, [32, 4, 16])   # servo 1: P, D, I
MyCobot.atom_gains(link)                    # (P, D, I) of the six servos
MyCobot.atom_set_gains!(link, MyCobot.GAINS)
rec, done = MyCobot.atom_play_signal(link, MyCobot.SignalParams(1, "chirp"; amp=10.0))   # MOVES THE ROBOT
close(link)
```

`atom_play_trajectory` has the same contract as the laptop player: it checks the
plan, applies lag compensation (or uses `q_cmd`), uploads, plays and returns the
recording with the IMU columns `acc_x … gyro_z`.
