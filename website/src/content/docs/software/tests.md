---
title: Tests
description: What the Julia tests check, and the simulated servo bus.
---

Run the tests from the repository root:

```bash
julia --project=. -e 'import Pkg; Pkg.test()'
```

They do not need the robot.

| Test set | Checks |
| --- | --- |
| Frame handling, motion mode, colour, gripper, servo control | Stock-ATOM-protocol frames. |
| Kinematics | Joint limits; forward kinematics against two `get_coords` readings of the stock firmware (< 12 mm); IK recovers a known pose. |
| Feetech packets | Packets built byte for byte against packets captured on the robot, including the stock firmware's `send_angles` SYNC WRITE. |
| Parser robustness | Garbage, `FE FE` frames, bad checksums, truncated and repeated packets. |
| Encodings | Speed and load sign bits; degrees ↔ steps. |
| Simulated bus | Reads, writes, "goal speed 0 = no motion", `enable_motion`, missing servos. |
| Player | A plan plays to the end; a stuck joint stops the run and every joint holds; bad plans are refused. |
| Learning control | On a model plant with delay, lag and sticking, the error goes down in every iteration and halves in five. |
| Robot description | The generated URDFs and parameter tables are up to date (`uv run tools/gen_robot.py --check`; skipped without `uv`). |

## The simulated bus

`test/simulated_bus.jl` implements the transport functions for a fake bus with
six servos. It keeps a register map per servo, answers PING, READ, WRITE, SYNC
READ and SYNC WRITE like the real servos, and moves each servo toward its goal at
its goal speed. It copies the measured behaviour:

- goal speed 0 means no motion;
- a goal write turns torque on, a mode write turns it off;
- servos can be marked as stuck, to test the tracking check.

`RecordingIO` is a transport that only records what is written. The packet tests use it.
