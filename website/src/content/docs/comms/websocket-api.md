---
title: WebSocket API (browser)
description: How a web page talks to the ATOM directly — the same messages as the UDP protocol over a WebSocket, plus control, move and jog commands (firmware 4.2+).
---

Controller firmware **4.2.0** and later implement this API (tested on the robot on
2026-10-04). The [simulated ATOM](#simulated-atom) implements it for development.

The [Control page](/mycobot-280-lab/control/) on this site talks to the ATOM
directly. The user needs only the arm and a browser: no Raspberry Pi and no laptop
software.

```text
browser (GitHub Pages, HTTPS) ──WebSocket ws://<ATOM>/ws──▶ ATOM (controller firmware)
```

## Transport

| Item | Value |
| --- | --- |
| URL | `ws://<ATOM address>/ws` (port 80). The address is an IP address (`192.168.1.107`) or an mDNS name (`mycobot.local`). |
| Binary frames | One message per frame. The bytes are the same as a [UDP message](/mycobot-280-lab/comms/atom-link/#messages): the code byte, then the data, little-endian. |
| Text frames | ATOM → browser only: the status log line, once a second (the same text as the UDP broadcast on port 5005). |
| Connections | At least 4 at the same time. |

The firmware has **one** message handler for both transports. A reply goes to the
connection that sent the request. The [state stream](/mycobot-280-lab/comms/clients/#firmware-41)
(SUBSCRIBE `0x0C`, STREAM `0x88`) is the same on both transports. A subscription
also ends when its connection closes.

The UDP protocol stays for the lab tools (Julia, Python). UDP and WebSocket clients
can be connected at the same time. They share the control rules below.

### Browser access to the home network

The page is served over HTTPS, and the ATOM serves plain `ws://`. Chrome 147 and
later allows this connection if the address is a private IP address or a `.local`
name. The first time, Chrome asks the user for **local network access**. Other
browsers are not tested yet.

## Control

Many clients can watch. **One client at a time has control.** Only that client can
move the robot or write servo registers.

| Code | Command | Data | Reply |
| --- | --- | --- | --- |
| `0x0D` | CONTROL | u8 action: 0 release, 1 take, 2 take over | ACK: 0 = you have control, −2 = another client has control, −1 = the robot moves (take over refused) |

1. **Take** (1) succeeds if no client has control, or if the sender has control already.
2. **Take over** (2) takes control from another client. The ATOM refuses it while the
   robot moves (states playing, moving, jogging, tracking).
3. **Control ends** when the client releases it, when its WebSocket closes, or **2 s
   after the last message** from that client. A client that wants to keep control
   sends any message at least once a second (CONTROL 1 is enough).
4. **Control does not expire during the holder's run** (PLAY, PLAY_SIGNAL,
   MOVE_TO), because a client that only receives telemetry sends nothing. The 2 s
   timer starts when the run ends. A lost WebSocket still ends control at once.
5. **STOP and HOLD work for every client.** They are safety commands.

:::note
The API has no authentication. Every device on the home network can connect and
take control. This is a known limit, accepted for home use.
:::

These commands need control: PLAN_BEGIN, PLAN_DATA, PLAN_END, PLAY, PLAY_SIGNAL,
REG_WRITE, MOVE_TO, JOG and TRACK. If **nobody** has control, such a command takes control
for its sender first (as CONTROL 1). So older UDP clients that never send CONTROL
still work. If another client has control, the ATOM replies ACK with status **−2**.

## New commands (4.2)

Angles use the joint convention of the docs (degrees, 0° = the zero pose). The ATOM
converts them to servo steps.

| Code | Command | Data | Reply |
| --- | --- | --- | --- |
| `0x0E` | MOVE_TO | i16 goal[6] (0.01°), u16 duration (ms; 0 = the ATOM chooses it from the speed limit) | ACK: 0 started, −1 busy, −2 no control, −10−j goal of joint j outside the limits. Then DONE (`0x85`) when the move ends. No TELEM over WebSocket (4.3.1+): watch the STREAM. |
| `0x0F` | JOG | u8 frame (0 = joints), i16 velocity[6] (0.1 °/s) | ACK only if refused: −1 busy, −2 no control |
| `0x10` | TRACK (4.4+) | i16 goal[6] (0.01°), u16 vmax (0.1 °/s, at most 90 °/s) | ACK only if refused: −1 busy, −2 no control, −10−j goal of joint j outside the limits. See [TRACK](#track-live-mode). |

### MOVE_TO

The ATOM moves all joints from the current pose to the goal along a minimum-jerk
path, as `atom_move_to` does. A minimum-jerk move of distance *d* in time *T* has a
peak speed of 1.875·*d*/*T* and a peak acceleration of 5.77·*d*/*T*². If the
duration is 0, the ATOM uses the shortest duration that keeps every joint inside
both limits:

*T* = max over the joints of max(1.875·*d*/90, √(5.77·*d*/*a*ₘₐₓ))

with 90 °/s, and *a*ₘₐₓ = 400 °/s² on J1–J3 and 2000 °/s² on J4–J6. A duration
that is too short for these limits is refused (−1). A STOP or a HOLD ends the move,
and the robot holds the pose. DONE reports result 0 (done), 1 (tracking
error) or 2 (stopped).

### JOG and the deadman

JOG sets a joint velocity. The ATOM moves the goal positions at that velocity:

1. **Deadman:** if no JOG arrives for **200 ms**, the ATOM ramps the velocity down
   to zero at the acceleration limit, then holds the pose. The ramp makes a WiFi
   stall (200–300 ms happen) a short smooth stop, not a jerk. Send JOG every 50 ms
   while the user jogs.
2. **Limits:** the speed is limited to 30 °/s per joint and the acceleration to
   200 °/s². The ATOM stops each joint 2° inside its [joint limit](#joint-limits).
3. **Stop:** a JOG with all velocities zero, STOP, HOLD, or a lost connection stops
   the jog.

Frame 0 is the joint space. Other frames (end-effector jogging) are for a later version.

### TRACK (Live mode)

TRACK (firmware 4.4+) sets a goal pose. The ATOM moves each joint to its goal at up to
`vmax` and the joint's acceleration limit (400 °/s² on J1–J3, 2000 °/s² on J4–J6, as
MOVE_TO), and brakes to stop exactly on it. A new TRACK changes the goal at once. The
Control page's Live mode sends the fader goals with TRACK.

1. **Deadman:** send TRACK again at least every 200 ms (the page sends it every 50 ms
   and at once when a goal changes). After 200 ms without TRACK, the joints brake to
   zero at their acceleration limits and the ATOM holds the pose.
2. **Limits:** `vmax` is at most 90 °/s (the MOVE_TO limit). Goals must be 2° inside
   the [joint limits](#joint-limits); a joint never passes that margin, also when its
   goal jumps back.
3. **Stop:** STOP, HOLD (from any client) and a lost connection brake the joints to
   zero.
4. **Following error:** if a joint's measured position is too far from its goal
   position, the ATOM holds and goes to state error, and sends DONE (result 1,
   tracking error, with the joint and its error in steps). The limit is 20°, plus
   0.15 s × the joint's recent peak speed in TRACK (the peak decays over 0.3 s, so a
   fast reversal keeps it; about 33° at 90 °/s): at speed the servos lag by about
   0.11 s, and up to twice that in fast reversals with the integral gain on J1–J3. JOG
   uses 20° and also sends this DONE (4.4+). On 2026-10-05 a finger that blocked J4
   stopped the arm with this error (20–34°) after about 0.5 s, five times.

The motion runs on the ATOM at 500 Hz, so WiFi delays do not change the path. Before
4.4, Live mode sent JOG velocities from the page (at most 30 °/s and 200 °/s², with a
slow approach to the goal).

## State in the STREAM packet (4.2 additions)

Firmware 4.2 adds one byte at the end of STREAM (74 bytes). Clients that read only
73 bytes still work.

| Offset | Field | Values |
| --- | --- | --- |
| 73 | u8 control | 0 nobody, 1 **you** (the receiver), 2 another client |

The controller state byte gets two new values:

| Value | State |
| --- | --- |
| 0 | booting |
| 1 | holding |
| 2 | ready (plan loaded) |
| 3 | playing |
| 4 | error |
| 5 | OTA update |
| 6 | moving (MOVE_TO) — new |
| 7 | jogging — new |
| 8 | tracking (TRACK, 4.4+) |

## Units

The STREAM packet has raw servo values. Convert them as `src/atom.jl` does.
`sign` is `(−1, −1, +1, −1, −1, −1)` for J1–J6.

| Value | Raw | Conversion |
| --- | --- | --- |
| Angle (°) | u16 position, 0–4095 | `sign × (pos − 2048) × 360 / 4096` |
| Speed (°/s) | u16, sign in bit 15 | `sign × (±(v & 0x7FFF)) × 360 / 4096` |
| Load (%) | u16, sign in bit 10 | `sign × (±(v & 0x3FF)) / 10` |
| Temperature (°C) | u8 | as is |
| Voltage (V) | u8 | `/ 10` |
| Acceleration (g) | i16 | `/ 4096` |
| Angular rate (°/s) | i16 | `/ 16.4` |

### Joint limits

From the URDF (`mycobot_description/urdf/mycobot_280_arduino/`). The firmware keeps
one table (`limits.h`) for all its checks. JOG stops 2° inside these limits;
test signals (PLAY_SIGNAL) stay 10° inside them.

| Joint | J1 | J2 | J3 | J4 | J5 | J6 |
| --- | --- | --- | --- | --- | --- | --- |
| Limit (°) | ±165 | ±140 | ±150 | ±150 | ±160 | ±180 |

## Example

```js
const ws = new WebSocket('ws://192.168.1.107/ws');
ws.binaryType = 'arraybuffer';
ws.onopen = () => {
  const sub = () => ws.send(new Uint8Array([0x0c, 50, 0]));   // SUBSCRIBE at 50 Hz
  sub();
  setInterval(sub, 1000);                                      // renew once a second
};
ws.onmessage = (e) => {
  if (typeof e.data === 'string') return console.log(e.data);  // status log
  const v = new DataView(e.data);
  if (v.getUint8(0) === 0x88) {
    const pos1 = v.getUint16(7, true);                         // J1 position (steps)
    console.log('J1', (-(pos1 - 2048) * 360) / 4096, '°');
  }
};
```

## Simulated ATOM

`tools/atom_sim.py` implements this API without the robot. It simulates the arm
(each joint follows its goal with a 0.12 s lag and a 90 °/s speed limit) and uses
`tools/atom_replay.py` to play a recording for PLAY and PLAY_SIGNAL. Run it on the
Raspberry Pi:

```bash
~/venvs/control/bin/python tools/atom_sim.py --host 100.69.15.110 --port 8281 \
    [--recording tools/python/recordings/<recording>.csv]
```

Then connect the Control page to `100.69.15.110:8281`.
