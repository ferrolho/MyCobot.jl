---
title: Scripts
description: The Julia scripts that plan, play, trace and learn.
---

Run the scripts from the repository root with `julia --project=. scripts/<name>.jl`.

| Script | Moves the robot | Function |
| --- | --- | --- |
| `plan_circle.jl` | no | Plans the 100 mm vertical circle with IK, checks it, and writes `tools/python/plans/circle.csv`. |
| `play_plan.jl <plan.csv>` | **yes** | Plays a plan and writes a recording. See the options below. |
| `trace_recording.jl <rec.csv>` | no | Forward kinematics of the planned and measured angles. Writes `<rec>_path.csv` and prints the Cartesian error. |
| `ilc_step.jl <plan.csv> <rec.csv>` | no | One learning step. Writes `<plan>_ilc<k>.csv` with explicit commands. |
| `attribute_error.jl <rec.csv>` | no | Splits the Cartesian error into one contribution per joint. |
| `viewer.jl` | no | MeshCat 3D view of the robot. |
| `static_poses.jl [--n=48] [--dry-run]` | **yes** | Random static poses inside the lab workspace; holds of 6 s with the joint state, the IMU and the servo voltage, temperature and current. |
| `fit_imu_gravity.jl <samples.csv>` | no | Fits the IMU gravity model (zero offsets, base tilt, IMU calibration) to static poses. |
| `encoder_sweep.jl [1,2,3,4,5] [--dry-run]` | **yes** | Moves one joint at a time through its safe range, played on the ATOM with gyro telemetry. |
| `fit_encoder_sweep.jl <J*_sweep.csv …> [--plot=DIR]` | no | Encoder angle against the integrated gyro: encoder error and play per joint. |
| `example*.jl`, `read_all_servo_data.jl` | some | Older examples for the stock ATOM protocol. |

## `play_plan.jl` options

| Option | Effect |
| --- | --- |
| *(none)* | Laptop path through the FT232R, as fast as the bus allows, with lag compensation. |
| `--atom=<ip>` | Upload and play on the ATOM at 500 Hz. The recording has IMU columns. |
| `--rate=<Hz>` | Cap the laptop loop rate, or set the ATOM loop rate. |
| `--no-lag-comp` | Send the plan as it is, without lag compensation. |

If the plan has `cmd_1 … cmd_6` columns (from `ilc_step.jl`), the player sends those
commands as they are.

`static_poses.jl` and `encoder_sweep.jl` choose poses on their own. They keep every
pose and path inside the lab workspace (`scripts/lab_workspace.jl`): link origins
from joint4 on at least 120 mm above the table, within the reach used on the Control
page before 2026-10-06 (per 30° of azimuth, minus 20 mm), and the wrist away from
the base column.

Recordings go to `tools/python/recordings/` with names like
`<date>-<time>_<plan>_<lagcomp|nolag|ilc>[_<rate>hz]_<jl|atom>.csv`.

## File formats

- **Plan:** `t, q_1 … q_6` (s, degrees), and optionally `cmd_1 … cmd_6`.
- **Recording:** `t, q_plan_1…6, q_cmd_1…6, q_1…6, dq_1…6, load_1…6`, and for the
  ATOM also `acc_x, acc_y, acc_z` (g) and `gyro_x, gyro_y, gyro_z` (°/s).
