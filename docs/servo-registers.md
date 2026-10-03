# Servo registers (Feetech STS)

The six joint servos use the standard **Feetech STS** register map. This dump was taken on 2026-10-03 through the ATOM (`GET_SERVO_DATA 0x53`, one byte at a time) with [`tools/python/dump_servo_registers.py`](../tools/python/dump_servo_registers.py). Register names come from the Feetech STS memory map and are confirmed where noted below.

## Encoding

- Multi-byte values are **little-endian**: low byte at the lower address. For example, present position = `reg[56] | reg[57] << 8`.
- Position is 12-bit, 0–4095, with 4096 steps per turn (0.088° per step).
- **Joint angle ↔ position:** `angle = sign × (position − 2048) × 360 / 4096`, with sign = `[−1, −1, +1, −1, −1, −1]` for J1–J6. 0° is position 2048 on every joint (the ATOM writes 2048 for `send_angles` to zero). The signs come from comparing direct positions with the ATOM's `get_angles` at a non-zero pose. They agree within 0.1°.
- **Speed** (58–59, 46–47): bit 15 is the direction (sign-magnitude), in steps/s.
- **Load** (60–61): bit 10 is the direction, the magnitude is in 0.1 % units. For example, J4 `0x0421` = 3.3 % in the negative direction.
- **Offset** (31–32): bit 11 is the sign (sign-magnitude), so J3 `0x0F7C`, J4 `0x0D1D` and J6 `0x0D4C` are negative. In position mode, present position has the offset applied, and **goal position uses those same units**: to hold a joint, write `goal = present`. In velocity mode, present position is reported raw, without the offset (see `fast-communication.md`, gotchas 1–5).
- Voltage (62) is in 0.1 V. Temperature (63) is in °C.

## Confirmed by observation

| Register | Evidence |
| --- | --- |
| 5 ID | Reads 1–6 on J1–J6. |
| 6 baud rate | 0 means 1 Mbaud; direct packets at 1 Mbaud work. |
| 33 mode | Writing 1 put J1 into velocity mode. |
| 40 torque enable | Turned off by writing the mode (33). Writing a goal position (42–43) sets it back to 1. |
| 46–47 goal speed | Drives J1 in velocity mode. In position mode, 0 means no motion and nonzero is the speed cap. |
| 42–43 goal position | Same units as present position. Used in the 300 Hz SYNC WRITE loop. |
| 56–57 present position | Matches the ATOM's `get_angles`. |
| 58–59 present speed | Tracked the commanded speed. |
| 62 voltage | J1–J3 ≈ 7.6 V, J4–J6 ≈ 6.4–6.8 V. |
| 67–68 | Tracks present position within a few steps (meaning unknown). |

## Model numbers

| Joints | Model (reg 3–4) | Firmware (reg 0–1) | Notes |
| --- | --- | --- | --- |
| J1–J3 | `0x0809` (2057) | 3.9 | larger servo, P=32/D=8 (J1–J2) |
| J4 | `0x0709` (1801) | 3.9 | speed loop I = 10, unlike the others |
| J5–J6 | `0x0209` (521) | 3.9 | |

For comparison, the common STS3215 reports `0x0309` (777).

## Full dump (2026-10-03)

J1's goal position (2159) was written during the tests, at a time when the goal units were misunderstood (see gotcha 1). On the other joints it reads 0, meaning never written since power-up. Present position, load, voltage and temperature are live values.

| Addr | Name | J1 | J2 | J3 | J4 | J5 | J6 |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | firmware major | 3 | 3 | 3 | 3 | 3 | 3 |
| 1 | firmware minor | 9 | 9 | 9 | 9 | 9 | 9 |
| 2 |  | 0 | 0 | 0 | 0 | 0 | 0 |
| 3 | model (L) | 9 | 9 | 9 | 9 | 9 | 9 |
| 4 | model (H) | 8 | 8 | 8 | 7 | 2 | 2 |
| 5 | ID | 1 | 2 | 3 | 4 | 5 | 6 |
| 6 | baud rate (0 = 1M) | 0 | 0 | 0 | 0 | 0 | 0 |
| 7 | return delay | 0 | 0 | 0 | 0 | 0 | 0 |
| 8 | response level | 1 | 1 | 1 | 1 | 1 | 1 |
| 9 | min angle (L) | 0 | 0 | 0 | 0 | 0 | 0 |
| 10 | min angle (H) | 0 | 0 | 0 | 0 | 0 | 0 |
| 11 | max angle (L) | 255 | 255 | 255 | 255 | 255 | 255 |
| 12 | max angle (H) | 15 | 15 | 15 | 15 | 15 | 15 |
| 13 | max temperature | 70 | 70 | 70 | 70 | 70 | 70 |
| 14 | max voltage | 130 | 130 | 240 | 240 | 240 | 240 |
| 15 | min voltage | 60 | 60 | 60 | 40 | 40 | 40 |
| 16 | max torque (L) | 232 | 232 | 232 | 232 | 232 | 232 |
| 17 | max torque (H) | 3 | 3 | 3 | 3 | 3 | 3 |
| 18 | phase | 4 | 4 | 4 | 100 | 100 | 100 |
| 19 | unloading condition | 44 | 44 | 44 | 38 | 38 | 38 |
| 20 | LED alarm | 47 | 47 | 47 | 38 | 38 | 38 |
| 21 | P | 32 | 32 | 10 | 10 | 10 | 10 |
| 22 | D | 8 | 8 | 0 | 0 | 0 | 0 |
| 23 | I | 0 | 0 | 1 | 1 | 1 | 1 |
| 24 | min startup force (L) | 0 | 0 | 0 | 0 | 0 | 0 |
| 25 | min startup force (H) | 0 | 0 | 0 | 0 | 0 | 0 |
| 26 | CW dead zone | 3 | 3 | 3 | 3 | 3 | 3 |
| 27 | CCW dead zone | 3 | 3 | 3 | 3 | 3 | 3 |
| 28 | protection current (L) | 44 | 44 | 44 | 232 | 232 | 232 |
| 29 | protection current (H) | 1 | 1 | 1 | 3 | 3 | 3 |
| 30 | angular resolution | 1 | 1 | 1 | 1 | 1 | 1 |
| 31 | offset (L) | 146 | 102 | 124 | 29 | 192 | 76 |
| 32 | offset (H) | 0 | 4 | 15 | 13 | 0 | 13 |
| 33 | MODE | 0 | 0 | 0 | 0 | 0 | 0 |
| 34 | protective torque | 20 | 20 | 20 | 20 | 20 | 20 |
| 35 | protection time | 200 | 200 | 200 | 200 | 200 | 200 |
| 36 | overload torque | 80 | 80 | 80 | 80 | 80 | 80 |
| 37 | speed loop P | 10 | 10 | 10 | 10 | 10 | 10 |
| 38 | overcurrent protection time | 200 | 200 | 200 | 0 | 200 | 200 |
| 39 | speed loop I | 200 | 200 | 200 | 10 | 200 | 200 |
| 40 | torque enable | 1 | 1 | 1 | 1 | 1 | 1 |
| 41 | acceleration | 0 | 0 | 0 | 0 | 0 | 0 |
| 42 | goal position (L) | 111 | 0 | 0 | 0 | 0 | 0 |
| 43 | goal position (H) | 8 | 0 | 0 | 0 | 0 | 0 |
| 44 | goal time / PWM (L) | 0 | 0 | 0 | 0 | 0 | 0 |
| 45 | goal time / PWM (H) | 0 | 0 | 0 | 0 | 0 | 0 |
| 46 | goal speed (L) | 0 | 0 | 0 | 0 | 0 | 0 |
| 47 | goal speed (H) | 0 | 0 | 0 | 0 | 0 | 0 |
| 48 | torque limit (L) | 232 | 232 | 232 | 232 | 232 | 232 |
| 49 | torque limit (H) | 3 | 3 | 3 | 3 | 3 | 3 |
| 50 |  | 0 | 0 | 0 | 0 | 0 | 0 |
| 51 |  | 0 | 0 | 0 | 0 | 0 | 0 |
| 52 |  | 0 | 0 | 0 | 0 | 0 | 0 |
| 53 |  | 0 | 0 | 0 | 0 | 0 | 0 |
| 54 |  | 0 | 0 | 0 | 0 | 0 | 0 |
| 55 | EEPROM lock | 1 | 1 | 1 | 1 | 1 | 1 |
| 56 | present position (L) | 224 | 53 | 206 | 209 | 35 | 86 |
| 57 | present position (H) | 7 | 14 | 14 | 14 | 4 | 9 |
| 58 | present speed (L) | 0 | 0 | 0 | 0 | 0 | 0 |
| 59 | present speed (H) | 0 | 0 | 0 | 0 | 0 | 0 |
| 60 | present load (L) | 0 | 0 | 74 | 37 | 1 | 0 |
| 61 | present load (H) | 0 | 0 | 4 | 4 | 4 | 0 |
| 62 | present voltage | 76 | 75 | 76 | 68 | 64 | 64 |
| 63 | present temperature | 25 | 26 | 27 | 33 | 29 | 31 |
| 64 | async write flag | 0 | 0 | 0 | 0 | 0 | 0 |
| 65 | status | 0 | 0 | 0 | 0 | 0 | 0 |
| 66 | moving | 0 | 0 | 0 | 0 | 0 | 0 |
| 67 | ? position-like (L) | 221 | 52 | 206 | 208 | 36 | 86 |
| 68 | ? position-like (H) | 7 | 14 | 14 | 14 | 4 | 9 |
| 69 | present current (L) | 0 | 0 | 1 | 0 | 0 | 0 |
| 70 | present current (H) | 0 | 0 | 0 | 0 | 0 | 0 |
