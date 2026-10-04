---
title: Servo response
description: How the servos follow a stream of position goals — delay, smoothing, sticking, friction and velocity estimates.
---

These results come from the smooth multi-joint sine runs
(`tools/python/smooth_motion_demo.py`, 2026-10-03) and the offline analysis
(`tools/python/analyze_servo_response.py`).

## Smooth multi-joint motion

All six joints followed sine waves (±20–45°, 4 s period) from the zero pose at
~300 Hz. Two runs gave nearly the same result:

| Joint | RMS error | Lag (best-fit delay) |
| --- | --- | --- |
| J1 | 3.9° | ~120 ms |
| J2 | 2.4° | ~113 ms |
| J3 | 3.2° | ~120 ms |
| J4 | 1.1° | ~54 ms |
| J5 | 1.3° | ~38 ms |
| J6 | 1.4° | ~28 ms |

![Smooth motion: target and measured position, velocity and load for each joint.](../../../assets/plots/smooth-motion.png)

## Response model

A model with a delay and a first-order lag, `T q̇ = q_cmd(t − τ) − q`, fits better
than a pure delay:

| Joint | τ | T | RMS fit | Validation (second run / circle) |
| --- | --- | --- | --- | --- |
| J1 | 40 ms | 80 ms | 0.22° | 0.23° / 0.37° |
| J2 | 88 ms | 25 ms | 0.28° | 0.26° / 0.79° |
| J3 | 44 ms | 75 ms | 0.20° | 0.19° / 0.69° |
| J4 | 4 ms | 50 ms | 0.31° | 0.31° / 0.38° |
| J5 | 0 ms | 40 ms | 0.54° | 0.52° / 0.55° |
| J6 | 4 ms | 25 ms | 0.29° | 0.29° / 0.70° |

A pure time shift (what lag compensation does) is correct at one frequency only.
Feedforward through the model, `q(t + τ) + T·q̇(t + τ)`, is on the roadmap.

## Sticking after reversals

After a reversal, J1 and J2 stay still for 0.2–0.5 s while the plan moves. The
model does not capture this. It is the main cause of the circle's error spikes
(see [Circle](/mycobot-280-lab/results/circle/)).

![J2 on the circle: the model does not follow the sticking after reversals. Bottom: velocity estimators on J1.](../../../assets/plots/servo-analysis.png)

## Friction and the load register

- The jump of the load at each reversal gives Coulomb friction of 5–8 % on all joints.
- On J1 (no gravity load): load ≈ 2.8 % · sign(v) + 0.53 %/(°/s) · v.

The term proportional to speed is large (27 % at 50°/s). This fits a register that
shows **PWM duty** (back-EMF), not torque.

## Velocity estimates

Against an offline reference (smoothed slope of the position):

| Estimator | RMS error | Lag |
| --- | --- | --- |
| Speed register (50-step quantisation) | 3.1°/s | ~19 ms |
| Finite difference + 10 Hz low-pass | 2.2°/s | ~16 ms |
| Finite difference + 20 Hz low-pass | 2.9°/s | ~9 ms |
| Least-squares slope, last 20 samples | 2.3°/s | ~19 ms |

The 0.088° position resolution limits all of them. A model-based estimator is the
next step if a controller needs better velocity.

## Velocity mode

J1 in velocity mode tracked 102–103 steps/s for 100 commanded, with ~0.25 s of
dead time before it started to move.
