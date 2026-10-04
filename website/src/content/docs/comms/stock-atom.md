---
title: Stock ATOM protocol
description: The FE FE protocol of Elephant's stock ATOM firmware, and what it does on the servo bus.
---

This page describes Elephant's stock ATOM firmware (v7.2). It is not installed
now, but the [backup](/mycobot-280-lab/firmware/stock-backup/) can restore it. pymycobot,
myStudio and ROS use this protocol.

## Frame format

```text
FE FE <LEN> <CMD> <data...> FA        LEN = number of data bytes + 2
```

## Commands that were used

| Code | Command | Notes |
| --- | --- | --- |
| `0x02` | SOFTWARE_VERSION | Reply `0x48` = v7.2 |
| `0x12` | IS_POWER_ON | Fast reply, no bus traffic |
| `0x20` | GET_ANGLES | int16 big-endian, angle × 100 |
| `0x22` | SEND_ANGLES | 6 × int16 + speed (%) |
| `0x23` | GET_COORDS | x, y, z (0.1 mm), rx, ry, rz (0.01°) |
| `0x52` / `0x53` | SET / GET_SERVO_DATA | One servo register, one byte. The two-byte mode gave wrong values. |
| `0x6A` | SET_COLOR | LED matrix, no bus traffic |

## What the stock firmware does on the bus

- **GET_ANGLES** and **GET_COORDS**: a PING to ID 7 (the gripper, which gets no
  reply without a gripper), then one SYNC READ of 15 bytes at register 56 from
  servos 1–6. The ATOM then converts the result. `get_coords` runs the forward
  kinematics on the ATOM.
- **SEND_ANGLES**: one SYNC WRITE at register 41 for all servos: acceleration 50,
  goal position, goal time 0, and a goal speed proportional to each joint's distance
  (≈3.34 steps/s per degree at speed 30, minimum 300). The servos then move by
  themselves. The firmware does not interpolate.
- When idle, the stock firmware does not use the bus.

## Problems seen

- After a series of status queries starting at `GET_ROBOT_STATUS` (`0x19`), which
  this firmware does not support, the ATOM stopped replying to all commands. The
  servos were not affected. A power cycle cleared it.
- Before that, the ATOM acknowledged `send_angle` but wrote nothing to the bus. The
  cause is not known.

## LED colours

See [LED matrix signals](/mycobot-280-lab/firmware/led-signals/#stock-firmware).
