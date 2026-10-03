# Python tools for the myCobot 280 servo bus

Investigation scripts behind [docs/fast-communication.md](../../docs/fast-communication.md). They're written in Python so they can run without loading the Julia package, and they need only `pyserial` and `pyusb`.

## Setup

```bash
brew install libusb
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

The serial port is set in `mycobot_bus.py` (`/dev/tty.usbserial-B00033ZX`, 1 Mbaud).

**Run `ftdi_latency.py 1` after every plug-in.** Otherwise every reply waits ~16 ms.

## Scripts

| Script | Moves the robot? | What it does |
| --- | --- | --- |
| `ftdi_latency.py [ms]` | no | Read or set the FT232R USB latency timer |
| `benchmark.py` | no | Times ATOM commands against direct Feetech reads |
| `sniff_atom_command.py <cmd> [args]` | depends on the command | Sends one ATOM command and prints the Feetech traffic it causes |
| `dump_servo_registers.py` | no | Prints registers 0–70 of J1–J6 as a markdown table |
| `j1_velocity_mode_test.py` | **yes (J1)** | Velocity-mode test with a safe return to position mode (direct writes) |
| `sync_write_sine_test.py` | **yes (J1)** | 300 Hz closed loop: SYNC WRITE goals + SYNC READ state, J1 follows a ±5° sine |
| `smooth_motion_demo.py` | **yes (all joints)** | Smooth multi-joint sine trajectory from the zero pose at ~300 Hz, recorded to `recordings/*.csv` |
| `play_trajectory.py <plan.csv> [--no-lag-comp]` | **yes (all joints)** | Streams a joint trajectory planned in Julia (e.g. `scripts/plan_circle.jl` → `plans/circle.csv`) at ~250–300 Hz with per-joint lag compensation, and records it |
| `plot_circle.py <rec>_path.csv ...` | no | Plots planned vs traced flange paths computed by `scripts/trace_recording.jl` |
| `plot_recording.py <csv>` | no | Plots a recording and prints tracking error, lag, velocity and load per joint |
| `mycobot_bus.py` | – | Shared helpers: ATOM frames, Feetech packets, sync read |

Kinematics (FK/IK) lives in Julia (`src/kinematics.jl`, RigidBodyDynamics.jl). Python only streams and records.

Circle workflow:
```bash
julia --project=. scripts/plan_circle.jl                                 # plan + checks -> tools/python/plans/circle.csv
python play_trajectory.py plans/circle.csv                               # run on the robot (from tools/python)
julia --project=. scripts/trace_recording.jl tools/python/recordings/<rec>.csv
python plot_circle.py recordings/<rec>_path.csv
```

The scripts that move the robot were last run successfully on 2026-10-03. Read them before running, and keep a hand near the power switch.
