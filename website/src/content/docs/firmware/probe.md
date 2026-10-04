---
title: Bus probe firmware
description: atom_probe, the passive diagnostic firmware that found the bus pins and measured the bus speed.
---

`firmware/atom_probe/` is diagnostic firmware (version 3). Use it to check the
bus pins, WiFi, OTA, the IMU, the button and the LED matrix. It does not
command servos 1–6 unless it gets the `bench` command.

## What it does

- Counts every Feetech packet on G19, per servo ID, and counts bad checksums.
- Answers as **servo ID 7** on G22:
  - PING → empty status packet.
  - READ address 0, up to 32 bytes → its counters (bytes received, valid packets, bad checksums, requests to ID 7, packets per ID).
- Sends a status line every second (UDP broadcast, port 5005).
- Shows its state on the [LED matrix](/mycobot-280-lab/firmware/led-signals/#bus-probe-firmware).
- Accepts OTA updates.

## The `bench` command

Send the text `bench` to UDP port 5006. The probe then:

1. Sets goal speed 0 on all servos and verifies it. Nothing can move after this.
   Torque comes on.
2. Does 2000 SYNC READs of 6 bytes and reports the time and the failures.
3. Does 1000 cycles of SYNC WRITE goals (present position ± 1 step) + optional gap
   + SYNC READ for gaps of 0, 100, 200, 300, 500 and 1000 µs. It reads the goals
   back after each cycle to count lost writes.
4. Sets the goals back to the present positions.

Results on 2026-10-04:

| Test | Result |
| --- | --- |
| SYNC READ ×2000 | 1.259 ms mean (1.248–1.279), 0 failures |
| Write + read, no gap | 1.541 ms (~650 Hz), 0 lost writes in 1000 |
| Write + read, 1 ms gap | 2.539 ms (~390 Hz), 0 lost writes |

## How it found the bus pins

With the ATOM in the arm and the FT232R on the base, the laptop sent PINGs to
servos 1–6 and to ID 7, and read the counters. The probe saw all the traffic
(4254 bytes, 382 valid packets, 0 bad checksums), and its replies reached the
laptop. Its own replies did not appear in its counters, so the ATOM does not
receive its own transmissions.

The pin assignment (G19 RX, G22 TX) came first from the source code of
[lewpar/myCobot280](https://github.com/lewpar/myCobot280). Their README says
G32/G26, but the code uses G19/G22.
