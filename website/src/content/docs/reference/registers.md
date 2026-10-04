---
title: Servo register map
description: Registers 0–70 of the six servos, read on 2026-10-03, with the encodings.
---

The servos use the Feetech STS register map. This dump was read on 2026-10-03
through the stock ATOM (`GET_SERVO_DATA`, one byte at a time) with
`tools/python/dump_servo_registers.py`. Present position, load, voltage and
temperature are live values. J1's goal position was written during the tests; on
the other joints it reads 0 (not written since power-up).

## Encodings

- Little-endian: low byte at the lower address. Present position = `reg[56] | reg[57] << 8`.
- Speed (46–47, 58–59): bit 15 is the sign. Load (60–61): bit 10 is the sign, 0.1 % units.
- Offset (31–32): bit 11 is the sign. J3 `0x0F7C`, J4 `0x0D1D` and J6 `0x0D4C` are negative.
- Goal position (42–43) uses the same units as present position (56–57) in position mode.
- Voltage (62): 0.1 V. Temperature (63): °C.
- Register 19 (unloading condition) read 44/38 with the stock firmware and 0 with the custom firmware. The stock firmware probably writes it at power-up.

## Registers confirmed by observation

| Register | Observation |
| --- | --- |
| 5 ID | 1–6 on J1–J6. |
| 6 baud rate | 0 = 1 Mbaud. |
| 33 mode | Writing 1 put J1 in velocity mode. Writing the mode turns torque off. |
| 40 torque enable | A goal write sets it to 1. |
| 41 acceleration | The stock firmware writes 50; the players write 0 (no ramp). |
| 42–43 goal position | Same units as present position. |
| 46–47 goal speed | 0 = no motion in position mode; otherwise the speed cap. Drives the joint in velocity mode. |
| 56–57, 58–59 | Present position and speed; match the stock firmware's `get_angles`. |
| 62 voltage | J1–J3 ≈ 7.6 V, J4–J6 ≈ 6.4–6.8 V. |
| 67–68 | Follows the present position within a few steps. Meaning not known. |

## Full dump

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
