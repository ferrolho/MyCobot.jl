---
title: Known problems and rules
description: Every trap found so far, with the rule that avoids it.
---

Read this page before you write code that talks to the servos.

## Servos

| # | Problem | Rule |
| --- | --- | --- |
| 1 | Goal position uses the same units as present position. An earlier note said "present + offset". That was wrong. | Write goal = present to hold a joint. |
| 2 | Goal speed 0 means "do not move" in position mode. | Set a nonzero speed cap to move. Set 0 again when you stop. |
| 3 | Goal position and goal speed read 0 after power-up. | Set every goal to the present position before you set a nonzero speed. |
| 4 | Writing a goal turns torque on. Writing the mode turns torque off. | After a mode change, write goal = present to hold the joint. |
| 5 | In velocity mode, the present position does not include the calibration offset. | Do not compare positions across modes. |
| 6 | The servos do not enforce the URDF joint limits. | Check the limits in software (the players do). |
| 7 | After a reversal, J1–J3 can stick for 0.2–0.5 s. | Expect it. ILC reduces its effect. |
| 8 | Elephant's documentation names register 22 "I" and register 23 "D". | 22 is D, 23 is I (Feetech's table). |
| 9 | After J6 was turned by hand about 9 turns while limp, it turned away from its goal (+45° in 0.4 s) as soon as motion was enabled. The servo probably counts turns internally, although it reports one turn only. A power cycle cleared it (2026-10-04). | After you turn J6 more than half a turn by hand, power-cycle the arm before the next motion. |

## Servo bus

| # | Problem | Rule |
| --- | --- | --- |
| 10 | Laptop path: a SYNC WRITE followed by a request within ~0.3 ms is lost (24 % with no gap). SYNC WRITE has no reply. | Wait ≥ 0.5 ms after each SYNC WRITE (`SYNC_WRITE_GAP` = 1 ms), or drain the port. |
| 11 | ATOM: back-to-back SYNC WRITEs lost the second write on some servos. | Verify setup writes by reading them back. |
| 12 | Feetech checksums can be `0xFE`, which looks like a stock-ATOM frame header. | Check every candidate header for length, footer and command. |
| 13 | There is no bus arbitration. | Only one device drives the bus at a time. |
| 14 | Unsupported stock-ATOM queries (for example `0x19`) froze the stock ATOM. | Use only known commands. |
| 15 | Two-byte reads through the stock ATOM (`0x53` with a mode byte) gave wrong values. | Read one byte at a time, or read the servos directly. |

## Laptop and USB

| # | Problem | Rule |
| --- | --- | --- |
| 16 | The FT232R latency timer is 16 ms by default and resets on unplug and on robot power-cycle. | Set it to 1 ms each time you connect. The scripts do this. |
| 17 | The FT232R and the ATOM's USB chip have the same USB ID (`0403:6001`). | Select the FT232R by serial (`B00033ZX`). |
| 18 | The Julia loop allocates memory, so the garbage collector stalls it for 30–80 ms at times. | Use the ATOM path for deterministic timing. |

## ATOM

| # | Problem | Rule |
| --- | --- | --- |
| 19 | With the ATOM in the arm and the 12 V supply off, the ATOM's USB powers the servos. | Flash the ATOM out of the arm. Then use OTA. |
| 20 | With the 12 V supply on, the ATOM's USB does not connect. | Use OTA updates. |
| 21 | The ATOM's USB corrupts data above 115 200 baud. | Flash and read at 115 200 baud. |
| 22 | Without firmware that does it, the servos start with torque off (limp arm). | The controller firmware holds the pose at power-up. Park the arm before you change firmware. |
| 23 | The LED matrix blocks for ~0.75 ms per update. | Update it from core 0, never inside the control loop. |
| 24 | `WiFiUDP` is not safe across cores. | Only the network task uses the sockets. |
| 25 | The ATOM button is out of reach when the arm moves. | Do not use it as an emergency stop. |
| 26 | Elephant's documentation lists G19 and G22 (function interface group 6) as general-purpose I/O. They carry the servo bus. | Do not use G19 or G22 as I/O. |

## Power

| # | Problem | Rule |
| --- | --- | --- |
| 27 | A power cycle resets servo goals and speeds, and the arm can sag while the power is off. | Support the arm. Expect a new start pose. |
