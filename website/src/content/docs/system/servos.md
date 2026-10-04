---
title: Servos
description: The Feetech STS servos, their modes, units and settings, and how goals and torque behave.
---

The six joints use **Feetech STS** bus servos with the standard STS register map.
The full register dump is in the [register map](/mycobot-280-lab/reference/registers/).

## Identity

| Joints | Model number (registers 3–4) | Firmware | Supply (measured) |
| --- | --- | --- | --- |
| J1–J3 | `0x0809` (2057) | 3.9 | ~7.6 V |
| J4 | `0x0709` (1801) | 3.9 | ~6.5 V |
| J5–J6 | `0x0209` (521) | 3.9 | ~6.4 V |

The common STS3215 reports `0x0309` and the STS3250 `0x0B09`. These three model
numbers are not in any public table found so far. They can be custom versions made
for Elephant Robotics.

### Replacement servos

The servos are **Feetech STS-series** bus servos: they use Feetech's protocol and
register map, and register 3 is 9 as on other STS models. There are **three
types**. Their settings are also different:

| Type | Joints | Model | Max voltage (14) | Min voltage (15) | Phase (18) | Protection current (28–29) |
| --- | --- | --- | --- | --- | --- | --- |
| A | J1, J2, J3 | `0x0809` | 13.0 V (J1, J2), 24.0 V (J3) | 6.0 V | 4 | 300 |
| B | J4 | `0x0709` | 24.0 V | 4.0 V | 100 | 1000 |
| C | J5, J6 | `0x0209` | 24.0 V | 4.0 V | 100 | 1000 |

The Feetech product name of each type is **not known yet**. To find it:

1. Read the label on the servo. This needs the joint covers off.
2. Connect Feetech's FD software (Windows) through the FT232R at 1 Mbaud. It reads
   the model number and shows the name.
3. Ask Elephant Robotics for a replacement servo for that joint and give the model
   number. Elephant sells parts for the arm (not checked for these servos).

A replacement must report the same model number. Before you use it, set its ID,
the PID gains (21–23) and the other settings in the
[register map](/mycobot-280-lab/reference/registers/) for that joint, and
calibrate its zero (stock command `SET_SERVO_CALIBRATION`, `0x54`, or the offset
in 31–32).

## Units and encodings

- Multi-byte values are little-endian.
- **Position:** 12-bit, 4096 steps per turn (0.088° per step).
- **Speed:** steps/s, sign-magnitude with bit 15 as the sign. The present speed is quantised to 50 steps/s (4.4°/s).
- **Load:** 0.1 % units, bit 10 as the sign. It behaves like PWM duty, not torque: on J1 it is ~0.53 % per °/s plus ~3 % static friction.
- **Voltage:** 0.1 V. **Temperature:** °C.
- **Calibration offset** (registers 31–32): sign-magnitude, bit 11 as the sign.

## Modes

Register 33 sets the mode.

| Mode | Command register | Status |
| --- | --- | --- |
| 0 — position (default) | Goal position, 42–43 | Used for all the results. |
| 1 — velocity | Goal speed, 46–47 | Tested on J1: 102–103 steps/s for 100 commanded, with ~0.25 s of dead time at that speed. No joint limits. |
| 2 — PWM (open loop) | Goal time / PWM, 44–45 | Not tested. The closest mode to torque control. |
| 3 — step | relative position | Not used. |

## How goals and torque behave

These rules come from measurements. They are also in
[Known problems and rules](/mycobot-280-lab/reference/gotchas/).

1. **Goal position uses the same units as present position.** To hold a joint, write goal = present.
2. **Goal speed 0 means "do not move"** in position mode. A nonzero goal speed is a speed cap.
3. **Goal position and goal speed read 0 after power-up.** Set every goal to the present position before you set a nonzero speed.
4. **Writing a goal position turns torque on.** Writing the mode (register 33) turns it off.
5. **In velocity mode, the present position is raw** (without the calibration offset). It looks like a jump of the offset size when the mode changes. The joint does not move.
6. **Register 55 (EEPROM lock) is 1.** Changes to the EEPROM area (for example the mode) are probably lost at power-off. This is not verified.

## Response in position mode

The servos follow a stream of goals with a repeatable lag and smoothing. Fitted
on the sine recordings (see [Servo response](/mycobot-280-lab/results/servo-response/)):

| Joint | Delay | Time constant | Pure-delay fit | PID (21/22/23) |
| --- | --- | --- | --- | --- |
| J1 | 40 ms | 80 ms | 120 ms | 32 / 8 / 0 |
| J2 | 88 ms | 25 ms | 114 ms | 32 / 8 / 0 |
| J3 | 44 ms | 75 ms | 118 ms | 10 / 0 / 1 |
| J4 | 4 ms | 50 ms | 54 ms | 10 / 0 / 1 |
| J5 | 0 ms | 40 ms | 40 ms | 10 / 0 / 1 |
| J6 | 4 ms | 25 ms | 28 ms | 10 / 0 / 1 |

The joints also stick for 0.2–0.5 s after a reversal (static friction). The dead
zone (registers 26/27) is 3 steps on every joint.

:::caution[Register names]
Elephant's documentation calls register 22 "I" and register 23 "D". This is wrong:
22 is D and 23 is I, as in Feetech's table. See
[Stock protocol commands](/mycobot-280-lab/reference/stock-protocol/#problems-in-elephants-documentation).
:::

:::note[PID defaults]
An older note in the repository listed J4–J6 with P = 25, D = 25, I = 1. The
values read from the servos on 2026-10-03 are in the table above.
:::
