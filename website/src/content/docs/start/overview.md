---
title: Overview
description: What this project is, what works now, and how the pages are organised.
---

This project gives a laptop fast, direct control of an Elephant Robotics
**myCobot 280 for Arduino**. It replaces the stock control path, which reads the
joint angles at about 50 Hz, with two faster paths:

- **Laptop path.** The laptop talks to the servo bus directly through an FT232R
  USB-serial adapter. The loop runs at about 300 Hz on the laptop.
- **ATOM path.** Custom firmware on the ATOM (the ESP32 board at the end of the
  arm) runs the loop at 500 Hz. The laptop sends plans and receives telemetry
  over WiFi.

Both paths use the same plans, the same safety checks and the same recording
format. Trajectory planning and analysis stay on the laptop, in Julia.

## Status

| Area | Status |
| --- | --- |
| Direct servo-bus access from the laptop | Works. ~300 Hz write + read loop. |
| Onboard 500 Hz player on the ATOM | Works. 0 late cycles in all runs so far. |
| WiFi plan upload, telemetry, OTA updates | Works. |
| End-effector IMU telemetry | Works. Shows vibration bursts at joint reversals. |
| Kinematics (RigidBodyDynamics.jl) | Works. Matches the stock firmware's `get_coords` to ~1 cm. |
| Lag compensation and iterative learning control | Works. Circle error 12.5 mm → 0.8 mm RMS. |
| PWM (torque-like) servo mode | Not tried. |
| Trajectory optimisation (TORA.jl) | Not connected yet. See [TORA](/mycobot-280-lab/software/tora/). |
| Emergency stop | None. Disconnect the 12 V supply. See [Safety](/mycobot-280-lab/start/safety/). |

## The main findings

1. The 20 ms latency of `get_angles` came mostly from the FT232R's **16 ms
   latency timer**, not from the ATOM. Set it to 1 ms. See
   [Laptop link](/mycobot-280-lab/comms/laptop-link/).
2. The **servo bus is reachable directly** from the laptop's serial port. The
   base passes the bytes through. See [Architecture](/mycobot-280-lab/system/architecture/).
3. The servos are **Feetech STS** servos with the standard register map. See
   [Servos](/mycobot-280-lab/system/servos/).
4. In position mode, the servos follow commands with a **repeatable lag** of
   30–120 ms and stick briefly after each reversal. A faster loop does not
   change this. Lag compensation and learning control do. See
   [Results](/mycobot-280-lab/results/circle/).

## How to read this site

- **System** describes the hardware: the robot, the wiring, the servos and the ATOM.
- **Communication** describes the protocols on each link.
- **Firmware** describes the ATOM firmware and how to build and flash it.
- **Software** describes the Julia package, the scripts and the Python tools.
- **Results** records each experiment with its numbers and plots.
- **Reference** collects the rules and known problems, the register map, the
  roadmap and the history.
