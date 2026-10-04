---
title: Trajectory optimisation (TORA)
description: What exists in TORA.jl for the myCobot, the missing inertial data, and how to connect it to the players.
---

[TORA.jl](https://github.com/ferrolho/TORA.jl) (in `~/git/TORA.jl`) is a trajectory
optimiser built on RigidBodyDynamics.jl and Ipopt. It is not connected to the
players yet. This page records what exists and the plan to connect it.

## What exists

- **Branch `hf/mycobot`** (`c182ce0`, 2025-02-18):
  - `create_robot_mycobot("mycobot_280_arduino", vis)` loads the URDF from
    `~/myCobot/mycobot-280-lab/mycobot_description` (hard-coded path). The end effector
    is `joint6_flange`, the same frame as `src/kinematics.jl`.
  - `notebooks/mycobot.ipynb` solves a circle: end-effector position constraints at
    81 knots (dt = 1/20 s), zero velocity at both ends, `use_inv_dyn=true`,
    `minimise_velocities=true`. Orientation constraints are written but disabled.
  - The notebook played the result through the stock ATOM at 20 Hz.
- **Branch `hf/mycobot-280`** (2024-03-17): an older robot loader.
- The working tree is on `hf/dev`, with uncommitted changes.

## No inertial data

The URDF has no `<inertial>` tags, so every link has zero mass. With inverse
dynamics, TORA then forces all torques to 0. The integration constraints between
knots stay, so positions and velocities stay consistent. Torque costs and torque
limits have no meaning.

This is acceptable for now: the joints are position-controlled servos, and the
useful objectives are kinematic (smooth paths, velocity and acceleration limits,
end-effector constraints, minimum time). For real dynamics later:

1. Weigh the links and get the centres of mass and inertias from the meshes.
2. Or identify the inertial parameters from data, when the relation between the load register and torque is known.

Elephant's ROS URDFs have placeholder masses (0.2 kg per link). Do not use them.

## Connect TORA to the players

1. Make the plan start and end at the zero pose: `fix_joint_positions!` at the
   first and last knots, or add minimum-jerk moves like `scripts/plan_circle.jl`.
2. Export: `qs, vs, τs = TORA.unpack_x(x, robot, problem)`, then
   `MyCobot.write_plan_csv(path, (0:K-1) .* problem.dt, rad2deg.(qs'))`.
3. Use knots of about 10 ms, or resample with cubic Hermite interpolation from
   `qs` and `vs`. The players interpolate linearly.
4. Keep joint speeds below 90°/s (the plan check).
5. Play, trace and learn with the usual scripts.

## Ideas

- **Kinematic transcription for TORA:** decision variables q, v, a (or jerk),
  with `q[k+1] = q[k] + dt·v[k] + dt²/2·a[k]` and `v[k+1] = v[k] + dt·a[k]`, and
  bounds on v, a and jerk. No masses are necessary. This is kinematic trajectory
  optimisation (for example Drake's `KinematicTrajectoryOptimization`).
- **Optimise through the servo model:** use `T q̇ = u(t − τ) − q` per joint
  (see [Servo response](/mycobot-280-lab/results/servo-response/)) as the dynamics, so the
  optimiser plans the commands directly. Learning control then removes what the
  model does not capture.

## Suggested order

1. Merge `hf/mycobot` into a clean branch. Make the URDF path a parameter.
2. Export the notebook's circle and play it. Compare with the IK circle (5.0 mm RMS).
3. Finish the orientation constraints.
4. Add the kinematic transcription or the servo model.
