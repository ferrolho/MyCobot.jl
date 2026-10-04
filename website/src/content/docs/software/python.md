---
title: Python tools
description: The Python tools for the laptop path, plotting and offline analysis.
---

The Python tools are in `tools/python/`. They need `pyserial` and `pyusb`
(`requirements.txt`), and `numpy` and `matplotlib` for the plots.

```bash
brew install libusb
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt numpy matplotlib
```

| Script | Moves the robot | Function |
| --- | --- | --- |
| `ftdi_latency.py [ms]` | no | Read or set the FT232R latency timer (selects the FT232R by serial). |
| `benchmark.py` | no | Times stock-ATOM commands against direct Feetech reads. |
| `sniff_atom_command.py <cmd> [args]` | depends | Sends one stock-ATOM command and prints the bus traffic it causes. |
| `dump_servo_registers.py` | no | Prints registers 0–70 of J1–J6. |
| `j1_velocity_mode_test.py` | **yes (J1)** | Velocity-mode test with a safe return to position mode. |
| `sync_write_sine_test.py` | **yes (J1)** | 300 Hz closed loop on J1. |
| `smooth_motion_demo.py` | **yes** | Multi-joint sine trajectory from the zero pose, recorded. |
| `play_trajectory.py <plan.csv>` | **yes** | The Python laptop player. |
| `plot_recording.py <rec.csv>` | no | Plots a recording and prints error, lag, speed and load per joint. |
| `plot_circle.py <rec>_path.csv …` | no | Plots planned against traced flange paths. |
| `analyze_servo_response.py` | no | Fits the servo response, friction, and velocity estimators. |
| `mycobot_bus.py` | — | Shared helpers. |

Kinematics is not in Python. Use the Julia scripts for planning and for the
flange path.
