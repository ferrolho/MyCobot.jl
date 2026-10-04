---
title: Julia package
description: The modules of the MyCobot Julia package and their main functions.
---

The Julia package `MyCobot` is in `src/`. Load it with `import MyCobot` from the
repository root (`julia --project=.`).

## Modules

| File | Function |
| --- | --- |
| `src/kinematics.jl` | Forward and inverse kinematics from the URDF, with RigidBodyDynamics.jl. |
| `src/feetech.jl` | Feetech packets, SYNC READ/WRITE, joint-level helpers. Transport-independent. |
| `src/ftdi.jl` | FT232R latency timer, through libusb. |
| `src/player.jl` | The laptop player, plan and recording files, `move_to`. |
| `src/ilc.jl` | Iterative learning control. |
| `src/atom.jl` | The WiFi link to the ATOM controller firmware. |
| `src/serial/` | The older stock-ATOM-protocol functions (`get_angles`, `send_angles`, …). |

## Kinematics

| Function | Use |
| --- | --- |
| `load_mechanism()` | Parse the URDF. |
| `flange_transform(state, q_deg)` | Flange pose (metres) for joint angles in degrees. |
| `flange_position_mm(state, q_deg)` | Flange position in millimetres. |
| `flange_goal(mechanism, R, p_mm)` | A target pose for IK. |
| `inverse_kinematics(state, goal, q_init)` | Damped least squares on RBD's Jacobian, with `log` of the pose error. About 1 ms per warm-started solve. |
| `joint_limits_deg(mechanism)` | Joint limits from the URDF. |

Do not write kinematics by hand. Use RigidBodyDynamics.jl through these functions.

## Servo bus

| Function | Use |
| --- | --- |
| `ft_packet`, `parse_status_packets` | Build and parse Feetech packets. |
| `ft_read`, `ft_write`, `ft_sync_read`, `ft_sync_write` | One request each. `ft_sync_write` waits `SYNC_WRITE_GAP` (1 ms) after the write. |
| `read_state(io)` | Position (°), speed (°/s) and load (%) of all six servos, ~2 ms. |
| `write_goals(io, q_deg)` | Goal positions of all six servos. |
| `hold_position(io)` | Goal = present on all servos. |
| `enable_motion(io; speed_cap)` / `disable_motion(io)` | Hold, then set a speed cap / set goal speed 0. |
| `angle_to_position`, `position_to_angle` | Degrees ↔ servo steps. |

All bus I/O goes through `transport_write`, `transport_read` and
`transport_discard_input`. They have methods for `LibSerialPort.SerialPort`. The
tests add methods for a simulated bus.

## Players

| Function | Moves the robot | Use |
| --- | --- | --- |
| `play_trajectory(io, t, q; lag, q_cmd, rate, …)` | yes | Laptop player. Returns `(recording, aborted)`. |
| `atom_play_trajectory(link, t, q; …)` | yes | Same contract, played on the ATOM. Returns `(recording, done)`. |
| `move_to(io, q_goal)` / `atom_move_to(link, q_goal)` | yes | Minimum-jerk move from the present pose. |
| `check_plan(t, q, mechanism)` | no | Start/end at zero, limit margin, speed limit. |
| `read_plan_csv`, `write_plan_csv`, `write_recording_csv`, `write_atom_recording_csv` | no | Files. |

`DEFAULT_LAG` holds the measured lag per joint: `[0.120, 0.113, 0.120, 0.054, 0.038, 0.028]` s.

## Learning control

`ilc_update(t, q_ref, q_cmd, t_meas, q_meas; gain=0.5, lead=DEFAULT_LAG, smooth=0.08, ramp=0.3)`
returns the next commands. See [Iterative learning control](/mycobot-280-lab/results/ilc/).

## Dependencies

LibSerialPort, RigidBodyDynamics, MeshCat, MeshCatMechanisms, libusb_jll, and the
standard libraries LinearAlgebra, DelimitedFiles, Dates, Sockets and CRC32c.
