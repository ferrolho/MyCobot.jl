---
title: WebSocket API (browser)
description: How a web page talks to the ATOM directly — the same messages as the UDP protocol over a WebSocket, plus control, move and jog commands (firmware 4.2+), with the gripper as joint J7 (5.0+).
---

Controller firmware **4.2.0** and later implement this API (tested on the robot on
2026-10-04). Firmware **5.0.0** adds the gripper as a seventh joint, [J7](#joints-j1j6-and-j7-50).
The [simulated ATOM](#simulated-atom) implements it for development.

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

## Joints: J1–J6 and J7 (5.0+)

The [gripper](/mycobot-280-lab/system/gripper/#control) is a bus servo of the same
family as the joint servos. Firmware 5.0 uses it as a seventh joint, **J7**:

- The ATOM looks for it (bus ID 7, model `0x070A`) at power-up and then once a second
  while the robot is idle, so you can connect it later. It counts as removed after 5
  missed reads in a row.
- While the ATOM finds it, every joint message has **n = 7** joints: J1–J6, then J7.
  Without it, n = 6. The STREAM tells n.
- J7 uses the same units as the other joints: degrees, from the same formula. Its
  limits are the end stops: −51.5° (closed) to 0° (open). The opening in % is a
  display value of the Control page only.
- The commands MOVE_TO, JOG and TRACK take 6 or 7 values. The message length tells
  the number. With 6 values, J7 holds its goal. With 7 values and no gripper, the
  ATOM refuses the command with status **−7**.
- Plans (PLAY) have 6 or 7 joints (PLAN_BEGIN tells the number). With a 6-joint plan
  or a test signal (PLAY_SIGNAL, J1–J6), J7 holds its goal. TELEM has the joints of the
  command (the plan or the MOVE_TO): a 6-joint client gets the 77-byte samples as before.
  See [ATOM link](/mycobot-280-lab/comms/atom-link/#telemetry-sample-40).
- STATE has the goal of each joint (as the STREAM).

Two rules apply only to J7, because it grasps:

1. **No following-error check.** On an object J7 stops short of its goal, as intended.
2. **A hold keeps its goal.** HOLD, STOP, the end of a run and an abort set J1–J6 to
   their measured positions, but J7 keeps its goal and speed. So a grasp stays closed.
   Runs start J7 from its goal, not from its measured position.

The ATOM also sets the gripper's torque (100 %) and its thermal derating when it finds
it (see [Gripper](/mycobot-280-lab/system/gripper/#control)).

## New commands (4.2)

Angles use the joint convention of the docs (degrees, 0° = the zero pose). The ATOM
converts them to servo steps. n is 6, or 7 with [J7](#joints-j1j6-and-j7-50) (5.0+).

| Code | Command | Data | Reply |
| --- | --- | --- | --- |
| `0x0E` | MOVE_TO | i16 goal[n] (0.01°), u16 duration (ms; 0 = the ATOM chooses it from the speed limit) | ACK: 0 started, −1 busy, −2 no control, −7 J7 given but no gripper, −10−j goal of joint j outside the limits. Then DONE (`0x85`) when the move ends. No TELEM over WebSocket (4.3.1+): watch the STREAM. |
| `0x0F` | JOG | u8 frame (0 = joints), i16 velocity[n] (0.1 °/s). Frames 1 and 2 (5.1+): [end-effector JOG](#end-effector-jog-51) | ACK only if refused: −1 busy or bad length, −2 no control, −7 J7 given but no gripper |
| `0x10` | TRACK (4.4+) | i16 goal[n] (0.01°), u16 vmax (0.1 °/s, at most 90 °/s) | ACK only if refused: −1 busy, −2 no control, −7 J7 given but no gripper, −10−j goal of joint j outside the limits. See [TRACK](#track-live-mode). |

Firmware 4.6 and 4.7 had a separate GRIPPER command (`0x11`). Firmware 5.0 removes it:
J7 moves with MOVE_TO, JOG and TRACK.

### MOVE_TO

The ATOM moves all joints from their goals (the current pose; J7 from its goal) to
the new goal along a minimum-jerk path, as `atom_move_to` does. A minimum-jerk move of distance *d* in time *T* has a
peak speed of 1.875·*d*/*T* and a peak acceleration of 5.77·*d*/*T*². If the
duration is 0, the ATOM uses the shortest duration that keeps every joint inside
both limits:

*T* = max over the joints of max(1.875·*d*/90, √(5.77·*d*/*a*ₘₐₓ))

with 90 °/s, and *a*ₘₐₓ = 400 °/s² on J1–J3 and 2000 °/s² on J4–J7. A duration
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
   200 °/s². The ATOM stops each joint 2° inside its [joint limit](#joint-limits) (J7, the gripper: on its end stops).
3. **Stop:** a JOG with all velocities zero, STOP, HOLD, or a lost connection stops
   the jog.

Frame 0 is the joint space. Frames 1 and 2 (firmware 5.1) move the TCP: see
[End-effector JOG](#end-effector-jog-51).

### End-effector JOG (5.1)

JOG with frame 1 or 2 sets a **twist of the TCP** (the tool point): a linear and an angular
velocity. The ATOM turns it into joint goals at 500 Hz (`twist.h`). The
[gamepad](/mycobot-280-lab/software/gamepad/) on the Control page uses it.

| Offset | Field | Unit |
| --- | --- | --- |
| 0 | u8 code `0x0F` | |
| 1 | u8 frame: 1 = base frame, 2 = tool frame | |
| 2 | i16 linear velocity x, y, z | 0.1 mm/s |
| 8 | i16 angular velocity about x, y, z, through the TCP | 0.1 °/s |
| 14 | i16 J7 velocity (0 without the gripper) | 0.1 °/s |
| 16 | u16 joint speed cap (at most 90 °/s) | 0.1 °/s |

The message has 18 bytes. Another length gives ACK −1. A nonzero J7 velocity without the
gripper gives −7.

- **Frames.** Base frame: the frame of the URDF (x, y, z of the base). Tool frame: the flange
  axes (z points along the fingers). The rotations turn about the TCP in both frames.
- **TCP.** With the gripper (n = 7): between the finger pads, (−0.6, 8.2, 100) mm in the flange
  frame (`tcp_mm` in `servos.yaml`). Without the gripper: the flange.
- **Limits.** The TCP accelerates at most 300 mm/s² and 230 °/s². The joints stay below the
  speed cap and 80 % of their acceleration limits, and brake to stop 2.5° inside their
  limits. One factor scales all joints, so the TCP keeps its direction when a limit slows it.
- **Singular poses.** The ATOM stops the TCP 10° before J3 = 0° (the arm straight) and
  J5 = ±90° (J4 and J6 parallel). A joint that starts nearer (the zero pose has J3 = 0°) can
  move away on either side. If the joints cannot give the twist (more than 20 % off), the arm
  brakes and stays: it does not move in another direction.
- **Deadman.** Send the message at least every 200 ms (the page sends it every 20 ms, also
  with a zero twist). The run continues while the messages come. After 200 ms without one, the
  TCP brakes along its path, then the ATOM holds the pose. Until then the last twist stays in
  effect: send a zero twist to stop at once. STOP and HOLD brake at once.
- **Following error.** As TRACK: 20° plus 0.15 s × the joint's recent peak speed.
- **Time.** The status log reports `twist_us`: the longest controller step (µs) in the last
  second: about 450 µs on the ATOM (2026-10-10). The ATOM integrates the measured time
  between cycles, so a late cycle does not shorten the motion.

### Switching modes (5.1)

JOG (any frame) and TRACK share one run. A message of the other mode switches the running JOG
or TRACK without a stop: the joint goals and speeds carry over. The state byte shows 7
(jogging) for JOG and 8 (tracking) for TRACK. The Control page uses this: X on the gamepad
sends TRACK to the ready pose, and the next end-effector JOG continues from there.

### TRACK (Live mode)

TRACK (firmware 4.4+) sets a goal pose. The ATOM moves each joint to its goal at up to
`vmax` and the joint's acceleration limit (400 °/s² on J1–J3, 2000 °/s² on J4–J7, as
MOVE_TO), and brakes to stop exactly on it. A new TRACK changes the goal at once. The
Control page's Live mode sends the fader goals with TRACK.

1. **Deadman:** send TRACK again at least every 200 ms (the page sends it every 50 ms
   and at once when a goal changes). After 200 ms without TRACK, the joints brake to
   zero at their acceleration limits and the ATOM holds the pose.
2. **Limits:** `vmax` is at most 90 °/s (the MOVE_TO limit). Goals must be 2° inside (J7: within its end stops)
   the [joint limits](#joint-limits); a joint never passes that margin, also when its
   goal jumps back. A joint that starts outside the margin (moved by hand, or J7 on
   its open end stop at 0°) moves back in smoothly and never further out (5.0+).
3. **Stop:** STOP, HOLD (from any client) and a lost connection brake the joints to
   zero.
4. **Following error** (J1–J6, not J7): if a joint's measured position is too far from its goal
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

## The STREAM packet (5.0)

Firmware 5.0 sends the joints as arrays of n (6, or 7 with J7): 21 + 11·n bytes (87 or
98). It also sends the goal of each joint: the goal that the joint holds or follows.

| Offset | Field | Values |
| --- | --- | --- |
| 0 | u8 type | `0x88` |
| 1 | u32 t_ms | ATOM time (ms) |
| 5 | u8 state | see below |
| 6 | u8 ok | 1 if J1–J6 replied to the last read |
| 7 | u8 control | 0 nobody, 1 **you** (the receiver), 2 another client |
| 8 | u8 n | joints: 6, or 7 with J7 |
| 9 | u16 pos[n] | position (steps) |
| 9 + 2n | u16 goal[n] | goal (steps). Equal to pos while the robot holds, except J7 on an object (its grasp goal). |
| 9 + 4n | u16 speed[n] | sign in bit 15 |
| 9 + 6n | u16 load[n] | sign in bit 10 |
| 9 + 8n | u8 temp[n] | °C |
| 9 + 9n | u8 volt[n] | 0.1 V |
| 9 + 10n | u8 status[n] | servo status (register 65) |
| 9 + 11n | i16 acc[3], i16 gyro[3] | IMU |

The ATOM reads the temperature, voltage and status of one joint at a time (each joint
about once a second), also during runs. Before 5.0, STREAM had J1–J6 at fixed offsets:
73 bytes (4.1), 74 bytes with the control byte at offset 73 (4.2), 79 bytes with 5
gripper bytes (4.6). The Control page reads both layouts.

The controller state byte (4.2 adds 6 and 7):

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
`sign` is `(−1, −1, +1, −1, −1, −1, +1)` for J1–J7.

| Value | Raw | Conversion |
| --- | --- | --- |
| Angle (°) | u16 position or goal, 0–4095 (J6: past one turn) | `sign × (pos − 2048) × 360 / 4096` |
| Speed (°/s) | u16, sign in bit 15 | `sign × (±(v & 0x7FFF)) × 360 / 4096` |
| Load (%) | u16, sign in bit 10 | `sign × (±(v & 0x3FF)) / 10` |
| Temperature (°C) | u8 | as is |
| Voltage (V) | u8 | `/ 10` |
| Acceleration (g) | i16 | `/ 4096` |
| Angular rate (°/s) | i16 | `/ 16.4` |

### Joint limits

From `mycobot_description/config/mycobot_280_arduino/` (J1–J6: `joint_limits.yaml`, J7:
the gripper's end stops in `servos.yaml`). The firmware keeps one table
(`motion_limits.h`) for all its checks. Goals, JOG and TRACK stay 2° inside these (J7: no margin)
limits; test signals (PLAY_SIGNAL) stay 10° inside them.

| Joint | J1 | J2 | J3 | J4 | J5 | J6 | J7 (gripper) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Limit (°) | ±165 | ±140 | ±150 | ±150 | ±160 | −225 to +135 | −51.5 (closed) to 0 (open) |

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
    const n = v.getUint8(8);                                   // 6, or 7 with the gripper (J7)
    const pos1 = v.getUint16(9, true);                         // J1 position (steps)
    console.log(n, 'joints; J1', (-(pos1 - 2048) * 360) / 4096, '°');
  }
};
```

## Simulated ATOM

`tools/atom_sim.py` implements this API (5.0) without the robot. It simulates the arm
(each joint follows its goal with a 0.12 s lag and a 90 °/s speed limit), with the
gripper as J7, and uses `tools/atom_replay.py` to play a recording for PLAY and
PLAY_SIGNAL. `--no-gripper` simulates the arm without the gripper (6 joints).
`--object DEG` puts an object between the fingers: J7 stops at DEG when it closes, with
load, as in a grasp. Run it on the Raspberry Pi:

```bash
~/venvs/control/bin/python tools/atom_sim.py --host 100.69.15.110 --port 8281 \
    [--recording tools/python/recordings/<recording>.csv] [--no-gripper] [--object -30]
```

Then connect the Control page to `100.69.15.110:8281`.
