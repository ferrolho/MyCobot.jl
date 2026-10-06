# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This repository (`mycobot-280-lab` on GitHub, renamed from `MyCobot.jl`; docs at https://ferrolho.github.io/mycobot-280-lab/) holds the Julia package `MyCobot`, the ATOM firmware, Python tools and the documentation site for the myCobot 280 (for Arduino). The Julia package controls the arm over serial and WiFi. It provides a high-level interface built on LibSerialPort.jl for sending commands and receiving data from the robot using a custom binary protocol.

## Development Commands

### Testing
```bash
julia --project=. test/runtests.jl
```

### Running Examples
All example scripts are in the `scripts/` directory. Run them with:
```bash
julia --project=. scripts/example.jl
```

Key examples:
- `example.jl` - Comprehensive demo of robot control features
- `viewer.jl` - Real-time 3D visualization using MeshCat
- `example_gripper.jl` - Gripper control
- `read_all_servo_data.jl` - Read PID parameters from all servos

### Serial Port Configuration
The robot connects via USB serial at **1000000 baud**. Port names are typically:
- macOS: `/dev/tty.usbserial-XXXXXXXX`
- Linux: `/dev/ttyUSB0` or `/dev/ttyACM0`

## Architecture

### Serial Communication Protocol

The core of this package is the **binary frame-based protocol** implemented in `src/serial/ProtocolCode.jl`. All communication follows this structure:

**Frame Format:**
```
[0xFE, 0xFE, length, command, data..., 0xFA]
```
- Two header bytes (0xFE, 0xFE)
- Length byte (excludes headers and footer)
- Command byte (defined in `ProtocolCodeEnum`)
- Variable data payload (0-16 bytes)
- Footer byte (0xFA)

**Key Functions:**
- `prepare_frame(command, data)` - Constructs a frame to send
- `extract_all_frames(response)` - Parses incoming byte stream into frames
- `wait_for_command_response(sp, expected_command)` - Handles request-response pattern with timeout

### Module Organization

The package is organized by functional domains in `src/serial/`:

- **ProtocolCode.jl** - Protocol definitions and frame handling utilities
- **mdi_mode.jl** - Manual Data Input mode (joint angle control)
  - `get_angles()` - Read current joint positions
  - `send_angles()` - Command joint positions with speed
- **robot_status.jl** - Power management and motion modes
  - `power_on()`, `power_off()`, `release_all_servos()`
  - `set_fresh_mode()` - Toggle between "latest" and "queue" motion modes
- **servo_control.jl** - Low-level servo operations
  - Individual servo power control
  - PID parameter reading/writing (see website/src/content/docs/system/servos.md)
  - Servo calibration
- **gripper_control.jl** - End effector control
- **atom_io_control.jl** - ATOM controller I/O (LED, digital pins)
- **system_status.jl** - Version queries and system info
- **utils.jl** - Servo diagnostics (`print_servo_data()`)

### Communication Pattern

Most functions follow this pattern:
1. Prepare request frame using `prepare_frame()`
2. Clear serial buffers (`sp_flush`)
3. Write frame to serial port
4. Wait for response using `wait_for_command_response()`
5. Parse response data from frame

Critical timing notes:
- Use `yieldsleep()` instead of `sleep()` to allow Julia task scheduling
- Commands often retry 3-5 times with buffer flushing
- Default timeout is 0.2s for responses

### 3D Visualization

The package supports real-time robot visualization:
- Uses RigidBodyDynamics.jl + MeshCat.jl
- URDF files in `mycobot_description/urdf/mycobot_280_arduino/`
- `scripts/viewer.jl` demonstrates live visualization by polling `get_angles()` and updating the model

## Where things run

- The Raspberry Pi 5 next to the robot (`ssh raspberrypi5`) hosts the lab service (site, camera, session logs) and runs scripts that use the robot: repository `~/myCobot/mycobot-280-lab`, Julia `~/.juliaup/bin/julia` (not on `PATH` for non-interactive SSH), Python `~/venvs/mycobot/bin/python`, `~/bin/arduino-cli`. Recordings stay on the Pi.
- The Pi's USB-C supply gives only 3 A (`/proc/device-tree/chosen/power/max_current` = 3000): heavy jobs (Chromium/Playwright, big `npm` builds, dev servers) crashed it on 2026-10-05. Until it has a 5 A supply, keep its load light.
- Browser and site work run on the laptop: a temporary dev server and `tools/atom_sim.py` on `127.0.0.1` only, stopped afterwards (the laptop does not keep serving the docs). Build the site on the laptop and copy `website/dist/` to `raspberrypi5:myCobot/lab-services/website/dist/` to update the lab site.
- The laptop can build and flash the ATOM over USB (`arduino-cli`, `esptool`; see firmware/build-flash.md). OTA updates work from either machine.
- The Pi hub (`raspberrypi5:git/mycobot-280-lab.git`, remote `pi`) is git storage; push branches there. Push to GitHub (`origin`) only when the user says so.
- Control page sessions served by the lab service are logged on the Pi in `~/myCobot/lab-logs/*.jsonl` (software/control-page.md, "Session log"). Read them to see what the user did and what the robot did.
- The ATOM (controller firmware, UDP protocol: website/src/content/docs/comms/atom-link.md) is the bus master. The FT232R on the Pi is for occasional register access and debugging only.

## Documentation site (source of truth)

- All prose documentation lives in `website/src/content/docs/` (Astro + Starlight). There is no `docs/` directory any more.
- The site is the source of truth. When code, firmware or wiring changes, update the matching page in the same commit. Record measurements with dates and numbers; mark what is not verified.
- Write the pages in a style based on ASD-STE100 Simplified Technical English (`reference/writing-style.md`): short sentences, active voice, imperative steps, one term for one thing.
- Prefer visual representations where they help (SVG diagram components in `website/src/components/`, e.g. `LedMatrix.astro` for LED signals), with a text description alongside for accessibility.
- Run the site: `cd website && npm run dev`. Check it: `npm run build` (fails on broken internal links).
- The dev server does not see new or renamed pages: restart it after you add or rename a page (otherwise every page fails with "slug ... does not exist"). `npm run build` does not catch this.
- After you change a diagram, run `npm run check:diagrams` with the dev server running. It reports SVG text that leaves the drawing, overlaps other text, or is crossed by a line or shape edge.

## Important Context

### Fast communication (read the documentation site first: website/src/content/docs/)
- The FT232R USB latency timer defaults to 16 ms and resets on every unplug. Set it to 1 ms (`tools/python/ftdi_latency.py 1`), otherwise every reply takes ≥16 ms.
- The Feetech STS servo bus (IDs 1–6, 1 Mbaud, `FF FF` packets) is reachable directly on the same serial port. A sync read of all servos takes ~3 ms, and a SYNC WRITE + SYNC READ loop runs at ~300 Hz. Never mix ATOM commands with a direct-bus loop.
- Goal position uses the same units as present position (no offset). Goal speed 0 means no motion in position mode, so set a nonzero speed cap to move. Writing a goal switches torque on; writing the mode switches it off.
- Before setting a nonzero goal speed, set every goal to the joint's present position.
- A SYNC WRITE followed by another request within ~0.3 ms is silently dropped by all servos; `ft_sync_write` waits `SYNC_WRITE_GAP` (1 ms) afterwards. Keep that gap in any new code.
- ATOM reply parsers must skip interleaved Feetech bytes (a checksum byte can be 0xFE).
- The robot description has one source: `mycobot_description/urdf/mycobot_280_arduino/mycobot_280_arduino.urdf.xacro` and `mycobot_description/config/mycobot_280_arduino/*.yaml`. `uv run tools/gen_robot.py` writes the URDFs and the parameter tables (`robot_params.*` in the firmware, the Control page, tools/ and src/). Never edit a generated file.
- Kinematics lives in `src/kinematics.jl` (RigidBodyDynamics.jl, URDF in `mycobot_description/`): `load_mechanism`, `flange_transform`, `flange_goal`, `inverse_kinematics`, `joint_limits_deg`. Don't hand-write FK/IK elsewhere. Planning is in Julia (`scripts/plan_circle.jl`).
- Direct servo bus in Julia: `src/feetech.jl` (packets, `read_state`, `write_goals`, `enable_motion`/`disable_motion`), `src/ftdi.jl` (`set_latency_timer`), `src/player.jl` (`play_trajectory`, plan/recording CSV I/O), `src/ilc.jl` (`ilc_update`). Scripts: `play_plan.jl` (moves the robot), `ilc_step.jl`, `trace_recording.jl`, `attribute_error.jl`. All transport goes through `transport_write`/`transport_read`/`transport_discard_input`; `test/simulated_bus.jl` implements them for tests without the robot.
- The Python player (`tools/python/play_trajectory.py`) is the version proven on the robot; the Julia player hasn't moved the robot yet.

### Motion Modes
The robot has two motion command queuing modes:
- **Queue mode** (fresh_mode=false): Commands are queued and executed sequentially
- **Latest mode** (fresh_mode=true): New commands replace previous ones immediately

### PID Tuning
Servo PID parameters vary by joint (J1-J6). Defaults are in `website/src/content/docs/system/servos.md`. Use `read_servo_parameter()` and `set_servo_parameter()` to adjust. Parameter indices:
- 21: Proportional gain (P)
- 22: Derivative gain (D)
- 23: Integral gain (I)
- 26/27: Insensitive zones

### Serial Communication Reliability
The current implementation includes retry logic and buffer flushing because:
- Frames can be missed or incomplete
- Multiple responses may arrive in one read
- The `extract_all_frames()` function handles parsing multiple frames from buffers
