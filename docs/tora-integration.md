# Trajectory optimisation with TORA.jl

How to connect [TORA.jl](https://github.com/ferrolho/TORA.jl) (`~/git/TORA.jl`) to the
300 Hz pipeline in this repository. Written 2026-10-03 from a read-only review; nothing
in TORA.jl was changed.

## What already exists in TORA.jl

- **Branch `hf/mycobot`** (last commit `c182ce0`, 2025-02-18, "WIP with mycobot"):
  - `create_robot_mycobot("mycobot_280_arduino", vis)` in `src/robot.jl` loads the URDF directly from `~/myCobot/MyCobot.jl/mycobot_description` (hard-coded path; the artifact-based loading is commented out). The end effector is `joint6_flange`, the same frame as `src/kinematics.jl` here.
  - `notebooks/mycobot.ipynb`:
    - Solves a circle: `constrain_ee_position!` at each of 81 knots (`dt = 1/20` s, so 4 s), along a circle with cubic time scaling, zero velocity at both ends.
    - Ipopt with `use_inv_dyn=true`, `minimise_velocities=true`, HSL `ma97`.
    - Orientation constraints (`add_constraint_body_orientation!`) are written but commented out ("[WIP]", and `ipopt.jl` has the body name temporarily disabled).
    - Played the result through the ATOM with `MyCobot.send_angles` at 20 Hz.
- **Branch `hf/mycobot-280`** (2024-03-17): an older, smaller version of the robot loader. Superseded by `hf/mycobot`.
- The working tree is on `hf/dev` with uncommitted changes to `Project.toml` and two notebooks. `stash@{1}` is an auto-stash from `hf/mycobot`.

## The catch: no inertial data

`mycobot_280_arduino.urdf` has no `<inertial>` tags, so every link has zero mass in
RigidBodyDynamics. TORA's dynamics constraints are then satisfied with zero torques, and
torque costs or limits mean nothing. The notebook works because its objective is
kinematic (minimise velocities). Elephant's ROS URDFs have placeholder masses (10 kg base,
0.2 kg per link, no real inertias), so they don't help.

This matters less than it seems. The joints are **position-controlled servos with
their own PID**: we command positions, not torques. The servo "load" register behaves
like PWM duty (about 0.53 % per °/s on J1, mostly back-EMF), not torque. Until PWM mode
is in use, the useful TORA objectives are kinematic: smooth paths, velocity and
acceleration limits, end-effector position and orientation constraints, minimum time.

Options for real dynamics later:
1. Weigh the links and estimate centres of mass and inertias from the meshes (CAD), and add `<inertial>` tags.
2. Identify base inertial parameters from data, once the load ↔ torque relation is known (see `next-steps.md`, "what the load register measures").

## Plugging TORA into the 300 Hz pipeline

```
TORA (Ipopt)  ─ qs, vs at knots ─▶  plan CSV  ─▶  scripts/play_plan.jl (300 Hz, lag-compensated)
                                                 ─▶ recording ─▶ trace_recording.jl / ilc_step.jl
```

1. **Start and end at the zero pose.** `play_trajectory` refuses plans that don't. Either add `fix_joint_positions!(problem, robot, 1, zeros(6))` and the same at the last knot, or prepend and append a minimum-jerk approach like `scripts/plan_circle.jl` does.
2. **Export.** `qs, vs, τs = TORA.unpack_x(x, robot, problem)` then
   `MyCobot.write_plan_csv(path, (0:K-1) .* problem.dt, rad2deg.(qs'))`.
   Same angle convention: both use the URDF, and it matches the ATOM's angles.
3. **Knot spacing.** The player interpolates linearly. At `dt = 1/20` s that gives velocity steps every 50 ms, which the servos smooth out but which aren't what was optimised. Either use finer knots (`dt` ≈ 0.01 s), or resample with cubic Hermite interpolation using TORA's `vs` before writing the plan (a small addition to `write_plan_csv` or the export script).
4. **Limits from what we measured.** Joint limits come from the URDF (TORA already uses them). Add velocity limits well below the speed cap the player sets (2000 steps/s ≈ 176°/s); the demos so far used ≤ 90°/s.
5. **Play, trace, learn.** `scripts/play_plan.jl`, `scripts/trace_recording.jl`, and `scripts/ilc_step.jl` all work on any plan in this format.

## A better fit: optimise through the servo model

The servos follow commands with a repeatable delay and smoothing. Fitted per joint (`tools/python/analyze_servo_response.py`):

| Joint | Delay τ | Time constant T |
| --- | --- | --- |
| J1 | 40 ms | 80 ms |
| J2 | 88 ms | 25 ms |
| J3 | 44 ms | 75 ms |
| J4 | 4 ms | 50 ms |
| J5 | 0 ms | 40 ms |
| J6 | 4 ms | 25 ms |

TORA could treat that as the robot's dynamics: the state is the measured joint angle, the control is the commanded angle, `T q̇ = u(t − τ) − q`. Then the optimiser plans the **commands** directly, so that the measured motion follows the path, instead of planning angles and compensating afterwards. That's model-based feedforward. ILC then removes what the model misses, mainly the sticking after reversals.

## TORA.jl feature idea: kinematic trajectory optimisation

Today the only way to get position/velocity-consistent trajectories without inertial data is the massless-URDF trick (inverse dynamics forces τ = 0, and the knots stay linked by the integration constraints). A cleaner option in TORA would be a **double-integrator (or triple-integrator) transcription**: decision variables q, v, a (or jerk) per knot, constraints `q[k+1] = q[k] + dt·v[k] + dt²/2·a[k]`, `v[k+1] = v[k] + dt·a[k]`, bounds on v, a, jerk, and the existing end-effector constraints and costs. No masses needed, smooth and consistent by construction, and limits that match what the servos can actually do. This is often called kinematic trajectory optimisation (e.g. Drake's `KinematicTrajectoryOptimization`); "kinodynamic" usually means planning under velocity/acceleration (or torque) limits, which this is too.

## Suggested order

1. Merge `hf/mycobot` into a clean branch off `main`. Make the URDF path configurable (keyword argument or `MyCobot.URDF_PATH`) instead of hard-coded.
2. Re-run the notebook's circle, export with steps 1–3 above, and play it with `play_plan.jl`. Compare with the IK-planned circle from `scripts/plan_circle.jl` (5.1 mm RMS).
3. Finish the orientation constraints, so TORA can hold the flange orientation like the IK planner does.
4. Then the servo-model dynamics, or real inertial parameters if PWM mode is next.
