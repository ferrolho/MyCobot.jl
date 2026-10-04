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

Recordings go to `tools/python/recordings/` with names like
`<date>-<time>_<plan>_<lagcomp|nolag|ilc>[_<rate>hz]_<jl|atom>.csv`.

## File formats

- **Plan:** `t, q_1 … q_6` (s, degrees), and optionally `cmd_1 … cmd_6`.
- **Recording:** `t, q_plan_1…6, q_cmd_1…6, q_1…6, dq_1…6, load_1…6`, and for the
  ATOM also `acc_x, acc_y, acc_z` (g) and `gyro_x, gyro_y, gyro_z` (°/s).
