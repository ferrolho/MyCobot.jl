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
| J7 (the [gripper](/mycobot-280-lab/system/gripper/#control)) | `0x070A` (1802) | 3.40 | not measured |

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

## Multi-turn (J6)

The STS servos can read and move past one turn (tested on J6 on 2026-10-06, through
the ATOM's REG_READ and REG_WRITE, with J6 turned by hand and then moved):

| Setting | Present position (56–57) | Goal position (42–43) |
| --- | --- | --- |
| As shipped: phase 100, angle limits 0/4095 | One turn: wraps at 0/4095 (±180°) | Clamped to 0–4095: J6 stopped at −179.3° for a −190° goal |
| Phase bit 4 set (100 → 116), angle limits 0/4095 | **Past one turn:** read −231.8° (4685) | Clamped as above |
| Phase bit 4 set, angle limits **0/0** | Past one turn | **Past one turn:** J6 moved to −209.3° and back across ±180° the correct way |

- Values past one turn are sign-magnitude: bit 15 is the sign.
- The EEPROM lock (55) stays 1, so these settings last until the servo's next power-off.
  Our firmware (4.5+) writes them at each power-up for the joints marked `multi_turn`
  in `servos.yaml` (J6).
- After a power-up the servo counts from its one-turn reading. The firmware finds the
  turn from the J6 limits: they span one turn (−225° to +135°), so a reading above
  +135° is one turn lower. It keeps a `turn_offset` for the joint. Within 1.5° of ±135°
  the turn is not known: J6 goes limp and the ATOM stays in the error state until a
  HOLD finds the turn (the status log says `TURN UNKNOWN`).
- The positions in STATE, STREAM and plans go past 0–4095 on J6: −225° is step 4608.
- This also explains an older problem: after many turns by hand, J6 turned away from
  its goal when motion was enabled (see [Known problems](/mycobot-280-lab/reference/gotchas/#servos)).
  The servo counts turns inside, even when it reports one turn only.

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

The joints also stick for 0.2–0.5 s after a reversal (static friction). The factory dead
zone (registers 26/27) is 3 steps on every joint, and the minimum starting force (register 24)
is 0.

### Slow motions: starting force and dead zone

At slow speed, J2 and J3 move in steps. The servo does nothing until its error passes the
dead zone and the friction, then it jumps. Test on 2026-10-09: a vertical descent of the
fingertips at 4 mm/s in free air (J2 about 0.5°/s), recorded at 500 Hz.

| J2–J4 registers 24 / 26–27 | J2: measured − plan | J3: measured − plan | J2 motion |
| --- | --- | --- | --- |
| 0 / 3 (factory) | −0.62 to +0.70° | −1.76 to −0.97° | steps of about 1° |
| 0 / 1 | −0.53 to +0.53° | −1.67 to −0.79° | smaller steps |
| 20 / 1 | −0.35 to +0.44° | −1.41 to −0.53° | smooth |

Our firmware (5.0.1+) writes `start_force` (register 24) and `dead_zone` (26/27) from
`servos.yaml` at power-up, with the gains: 20 and 1 on J2–J4, the factory values on J1, J5 and
J6. A starting force of 40 made the arm vibrate more (gyro, 99th percentile: 19 °/s instead of
13 °/s). The EEPROM lock (register 55) stays 1, so the servos go back to the factory values at
power-off.

:::caution[Register names]
Elephant's documentation calls register 22 "I" and register 23 "D". This is wrong:
22 is D and 23 is I, as in Feetech's table. See
[Stock protocol commands](/mycobot-280-lab/reference/stock-protocol/#problems-in-elephants-documentation).
:::

The PID column was read on 2026-10-03 with the stock ATOM firmware. See the next
section for the gains in use now.

## Position-loop gains

Registers 21, 22, 23 are P, D, I of the servo's position loop (Elephant's
documentation names 22 and 23 the other way round; see
[Stock protocol commands](/mycobot-280-lab/reference/stock-protocol/#problems-in-elephants-documentation)).

| Joint | **Ours** (`MyCobot.GAINS`, written at power-up by the controller firmware v3+) | Stored in the servos | Stock ATOM firmware writes |
| --- | --- | --- | --- |
| J1 | **32 / 4 / 16** | 32 / 8 / 0 | 32 / 8 / 0 |
| J2 | **32 / 4 / 16** | 32 / 8 / 0 | 32 / 8 / 0 |
| J3 | **32 / 4 / 16** | 32 / 8 / 0 | 10 / 0 / 1 |
| J4 | **32 / 8 / 0** | 32 / 8 / 0 | 10 / 0 / 1 |
| J5 | **32 / 8 / 0** | 32 / 8 / 0 | 10 / 0 / 1 |
| J6 | **32 / 8 / 0** | 32 / 8 / 0 | 10 / 0 / 1 |

- **Ours** adds integral action on J1–J3. On the circle it halves the flange error
  (5.3 → 2.6 mm) with almost the same end-effector vibration. See
  [Servo dynamics](/mycobot-280-lab/results/servo-dynamics/#servo-gains-pid-and-the-imu).
- **Stored**: what the servos use if nothing writes the gains (read on 2026-10-04
  with the controller firmware v2, which did not write them).
- **Stock**: read on 2026-10-03 with Elephant's ATOM firmware, which writes these at
  power-up.
- With the EEPROM lock (register 55) at 1, written gains last until the next power
  cycle. The controller firmware writes ours again at each power-up (`gains_ok` in PING).
- Change them for a test with `scripts/set_gains.jl ours|stored|stock --atom=IP`.
