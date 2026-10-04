---
title: Clients and the bridge
description: Who talks to the ATOM, the state stream and per-client replies of firmware 4.1, the proposed bridge API, and the replay tool for development without the robot.
---

Several programs want the robot at the same time: test scripts, a web control
centre, browser tabs that only watch. This page is the contract between them.

## Rules

1. **One controller at a time.** Only one client may move the robot or write
   servo registers. Today the user and the sessions agree on this by hand. With
   the bridge it is a **control lease** (see below).
2. **Watching is free.** Any number of clients may receive the state.
3. **The laptop is only an interface.** Programs run on the Raspberry Pi
   (`raspberrypi5`) or in a browser.
4. Network services listen **only on the Tailscale interface** (100.69.15.110),
   not on the home network.

## The ATOM (UDP, firmware 4.x)

The full protocol is in [ATOM link](/mycobot-280-lab/comms/atom-link/). Firmware
4.0 and older sent every reply and all telemetry to the address of the **last
packet received**, so a second client took the telemetry away from a running test.
Firmware 4.1 fixes this.

### Firmware 4.1

- **Replies go to the sender** of each request (its IP address and UDP port).
  Clients that bind port 5007 and send from it (as `MyCobot.AtomLink` does) see no
  change.
- **PLAY telemetry and DONE go to the client that started the run** (PLAY or
  PLAY_SIGNAL), whatever other packets arrive during the run.
- **State stream** (new):

| Code | Command | Data | Reply |
| --- | --- | --- | --- |
| `0x0C` | SUBSCRIBE | u16 rate (Hz, 1–100; 0 = stop) | `0x88` STREAM packets to the sender at that rate |

  A subscription ends 2 s after the last SUBSCRIBE: send it again at least once
  a second. Up to 4 subscribers. The stream also runs while a plan plays (it
  then uses the samples of the control loop, so it adds no bus traffic).

  `0x88` STREAM (73 bytes with the type byte): `u32 t_ms, u8 controller state, u8 ok, u16 pos[6],
  u16 speed[6], u16 load[6], u8 temperature[6] (°C), u8 voltage[6] (0.1 V),
  u8 servo status[6] (register 65), i16 acc[3], i16 gyro[3]`. Temperature,
  voltage and status are read once a second while idle, and keep their last
  values while a plan plays.

## The bridge (proposed)

A service on the Raspberry Pi is the ATOM's **only** client. It owns the camera,
records every run, and shares everything with any number of clients. The session
that builds it (the web control centre) owns this section; the shapes below are
a first proposal.

```text
ATOM ◀─UDP─▶ bridge (Pi) ◀─WebSocket/HTTP (Tailscale)─▶ browser tabs, scripts, control centre
                │
camera ─────────┘ (/stream.mjpg, /snapshot.jpg)
```

| Item | Proposal |
| --- | --- |
| State to viewers | WebSocket messages at 20–50 Hz: joint angles (°), speeds, loads, temperatures, voltages, IMU, controller state |
| Full-rate telemetry | Opt-in: every 500 Hz sample of a run, with `cmd` and `ref` |
| Control lease | `request` → `granted` (holder, expiry) or `denied` (holder). Renew at least every 1 s; the lease ends 2 s after the last renewal. Motion and write commands without the lease are refused. |
| Commands | The UDP commands of the ATOM (PLAY, PLAY_SIGNAL, HOLD, STOP, REG_READ, REG_WRITE, plan upload), forwarded only for the lease holder. STOP is accepted from anyone. |
| Recordings | Every run is saved on the Pi, with who started it. A list and a download endpoint. |
| Camera | MJPEG stream and single frames. The bridge is the only process that opens `/dev/video0`. |

## Replay tool

`tools/atom_replay.py` behaves like an ATOM (protocol 4.1) and plays saved
recordings instead of moving the robot. Use it to develop clients without the
robot:

```bash
~/venvs/mycobot/bin/python tools/atom_replay.py tools/python/recordings/tuning/<recording>.csv --port 5106
# --host 127.0.0.1 (default) or the Tailscale address 100.69.15.110
```

It answers PING, STATE, SUBSCRIBE (state stream from the recording, looped),
REG_READ (gains, temperature, voltage), REG_WRITE (gains only, stored in memory),
HOLD, STOP, and PLAY and PLAY_SIGNAL (it streams the recording's 500 Hz telemetry
and DONE). Point the client at the Pi's address and the chosen port.
