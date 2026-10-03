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
| `j1_velocity_mode_test.py` | **yes (J1)** | Velocity-mode test with a safe return to position mode |
| `mycobot_bus.py` | – | Shared helpers: ATOM frames, Feetech packets, sync read |

`j1_velocity_mode_test.py` was tidied after its successful run on 2026-10-03, and that tidied version hasn't been run on the robot yet. Read it before running it, and keep a hand near the power switch.
