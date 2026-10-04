---
title: Circle and lag compensation
description: The 100 mm circle demo, how it is planned, and how lag compensation halves the error.
---

## The task

A vertical circle with a radius of 50 mm, 170 mm in front of the base, centred at
a height of 310 mm. The flange keeps the orientation it has at the zero pose
(pointing forward). The plan has:

1. A 3 s minimum-jerk move from the zero pose to the circle.
2. Two laps of 4 s each, with the speed eased in and out (1 s ramps).
3. A 3 s move back to the zero pose.

`scripts/plan_circle.jl` solves IK for each sample (residual ≤ 0.001 mm) and checks
the plan: joint limits with a 10° margin, joint speeds ≤ 90°/s, flange height ≥
120 mm. J4 reaches 120° (limit 145°). The first try, at a height of 250 mm, needed
J4 at 137° and the check refused it.

## Lag compensation

Each joint gets its target early, by its measured lag
(`DEFAULT_LAG = [120, 113, 120, 54, 38, 28]` ms).

| Run | Error on the circle (RMS) | Max |
| --- | --- | --- |
| No lag compensation (Python) | 12.5 mm | 21.0 mm |
| Lag compensation (Python, IK in Python) | 5.2 mm | 12.3 mm |
| Lag compensation (Python player, IK in Julia/RBD) | 5.1 mm | 12.6 mm |
| Lag compensation (Julia player, 284 Hz) | 5.0 mm | 12.4 mm |
| Lag compensation (ATOM, 500 Hz) | 5.0 mm | 12.5 mm |

"Error" is the distance between the planned flange position and the forward
kinematics of the measured joint angles. It does not include gear backlash or
link flex.

![Planned and traced flange paths, with and without lag compensation.](../../../assets/plots/circle-lagcomp.png)

## Where the remaining error comes from

`scripts/attribute_error.jl` moves one joint at a time to its measured angle:

| | J1 | J2 | J3 | J4 | J5 | J6 |
| --- | --- | --- | --- | --- | --- | --- |
| RMS contribution, no lag compensation | 6.7 mm | 9.7 mm | 12.3 mm | 2.3 mm | 0.8 mm | 0 |
| RMS contribution, lag compensation | 1.2 mm | 3.7 mm | 2.0 mm | 0.6 mm | 0.5 mm | 0 |

The peaks (≈4.1, 8.1 and 12.6 s) are J2 and J3 sticking just after they reverse:
J2 contributes 7–9 mm and J3 2–5 mm at each peak. J6 does not move the flange
point, so its constant 0.7° offset contributes nothing.
