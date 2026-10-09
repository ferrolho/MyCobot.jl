---
title: Robot description
description: The one source of the URDF, the joint limits and the servo tables, the formats it uses, and the generator that writes the copies.
---

The robot description has one source in `mycobot_description/`. A generator writes
every copy that the firmware, the Control page, the simulator and the Julia package
use. Do not edit a generated file: edit the source and run the generator.

## Files

| File | Format | Holds |
| --- | --- | --- |
| `urdf/mycobot_280_arduino/mycobot_280_arduino.urdf.xacro` | URDF in [xacro](https://github.com/ros/xacro) | Links, meshes, joint origins and axes; the gripper (with `gripper:=true`) |
| `config/mycobot_280_arduino/joint_limits.yaml` | ros2_control `joint_limits` | Position, speed, acceleration and effort limits |
| `config/mycobot_280_arduino/servos.yaml` | Our own | Servo IDs, directions, models, gains, acceleration registers, servo model (ωn, ζ) |
| `config/mycobot_280_arduino/calibration.yaml` | Our own | This robot only: encoder corrections. Only the Julia package uses it (`ENCODER_CORRECTION`). |

The generator `tools/gen_robot.py` writes:

| Generated file | Used by |
| --- | --- |
| `mycobot_description/urdf/mycobot_280_arduino/mycobot_280_arduino.urdf` | Julia (`load_mechanism`), meshes as `package://…/*.dae` |
| `website/public/robot/mycobot_280_arduino.urdf` | Control page 3D view, meshes as `*.glb` |
| `mycobot_description/urdf/mycobot_280_arduino/mycobot_280_arduino_gripper.urdf` | Julia, the arm with the gripper (`load_mechanism(urdf=...)`) |
| `website/public/robot/mycobot_280_arduino_gripper.urdf` | Control page 3D view with the gripper (`ArmView.setGripper(true)`) |
| `firmware/atom_controller/robot_params.h` | Controller firmware (`motion_limits.h`, joint directions, gains) |
| `website/src/control/robot_params.ts` | Control page (`protocol.ts`) |
| `tools/robot_params.py` | Simulator (`tools/atom_sim.py`) |
| `src/robot_params.jl` | Julia package (`SERVO_IDS`, `JOINT_SIGN`, `GAINS`, `SERVO_WN`, …) |

Before 2026-10-06 the limits were written by hand in five places, and the Control
page had its own copy of the URDF.

## Xacro arguments

| Argument | Default | Effect |
| --- | --- | --- |
| `mesh_uri` | `package://mycobot_description/urdf/mycobot_280_arduino/` | Prefix of the mesh file names. The web copy uses `""` (meshes next to the URDF). |
| `mesh_ext` | `dae` | Mesh file type. The web copy uses `glb`. |
| `gripper` | `false` | `true` adds the adaptive gripper at the flange ([Gripper](/mycobot-280-lab/system/gripper/#model-urdf)). |

The gripper part has two properties in the xacro file:

| Property | Value | Meaning |
| --- | --- | --- |
| `gripper_mount_deg` | 0 | Mount angle about the J6 axis, from Elephant's mount ([Mount angle](/mycobot-280-lab/system/gripper/#mount-angle)) |
| `gripper_closed`, `gripper_open` | −0.7, 0.15 rad | Range of the actuated finger joint `gripper_controller` |

The gripper meshes are `gripper_*.dae` in `urdf/mycobot_280_arduino/`. After you add or
change a mesh, convert it for the Control page (only the named meshes):

```bash
uv run --with trimesh --with pycollada --with pillow --with scipy python tools/web_meshes.py gripper_base
```

## Change a value

1. Edit the xacro file or a YAML file in `mycobot_description/`.
2. Run the generator on the laptop:
   ```bash
   uv run tools/gen_robot.py
   ```
   `uv` installs xacro and PyYAML for the script (the script header lists them).
3. Commit the source and the generated files together.
4. If the firmware values changed, build and update the firmware
   ([Build, flash and update](/mycobot-280-lab/firmware/build-flash/)).

The Julia tests run `uv run tools/gen_robot.py --check`. It fails if a generated
file is not up to date. The check is skipped where `uv` is not installed (the Pi).

## Why these formats

URDF holds the kinematic tree, the meshes, the inertias and, per joint,
`<limit>` (`lower`, `upper`, `velocity`, `effort`), `<safety_controller>`,
`<dynamics>` (`damping`, `friction`), `<calibration>` (homing switch positions
only) and `<mimic>`. It has no acceleration limits, no sensors and no actuator
model.

| Need | Format | Why |
| --- | --- | --- |
| Acceleration, deceleration and jerk limits | ros2_control `joint_limits` YAML | The ROS standard for limits that URDF does not have. Keys: `max_position`, `max_velocity`, `max_acceleration`, `max_deceleration`, `max_jerk`, `max_effort`, each with `has_*_limits`. |
| YAML files next to the URDF | xacro with `xacro.load_yaml` | Universal Robots (`ur_description`) and Franka (`franka_description`) do the same. |
| Per-robot calibration (later) | A YAML file per robot | Universal Robots keeps the calibrated kinematics of each robot in `default_kinematics.yaml`. |
| Servo data | Our own `servos.yaml` | No robot description format has bus servos, registers or a position-loop model. MuJoCo's MJCF has `kp` and damping, but not the acceleration limit of our servos. |
| Named poses, groups | SRDF (MoveIt) | Not used yet. Add it if a MoveIt planner is used. |

The limit values are in SI units (rad, rad/s, rad/s²), so ROS tools can read the
file. The generator converts them to degrees for the firmware and the Control page.

## Values and where they come from

| Value | Source |
| --- | --- |
| Lengths, joint axes, meshes | Elephant's `mycobot_280_arduino.urdf` (mycobot_ros), with the flange turned 45° about its axis (2025-02-16). |
| Position limits (±165, ±140, ±150, ±150, ±160°; J6 −225° to +135°) | Inside the end stops measured by hand on 2026-10-04; J6: one turn inside the gripper cable stops (2026-10-06) ([Robot](/mycobot-280-lab/system/robot/#joint-limits)). |
| Speed limit 90 °/s | Our firmware (MOVE_TO, TRACK). |
| Acceleration limits 400 / 2000 °/s² | Our firmware, below the servo limits (439 / 2197 °/s², [Servo dynamics](/mycobot-280-lab/results/servo-dynamics/)). |
| Effort | Not known: the servo models are not identified. `has_effort_limits: false`, `effort="0"` in the URDF. |
| Gains, ωn, ζ | [Servo dynamics](/mycobot-280-lab/results/servo-dynamics/) (2026-10-04). |
| Gripper meshes and finger joints | Elephant's `mycobot_280m5_with_gripper_parallel.urdf` (mycobot_ros), with the finger joints on the body moved 7.5 mm ([Gripper](/mycobot-280-lab/system/gripper/#model-urdf)). |
| Gripper mount angle | The user's report of the mount on this arm (2026-10-06). |
| Base plate (`g_base`) | Elephant's mesh, turned 90° about z (2026-10-06): on this robot the round base is mounted on the plate 90° from Elephant's model. The 150 mm side is along x, and the plate is centred on the J1 axis. On 2026-10-08 the outline of the round base, fitted to the camera image with the camera registered on the plate screws, was within 2 mm of the plate centre. The 2026-10-06 model had the plate 10 mm toward −y, from a less accurate camera model; that offset moved every camera measurement 10 mm. Caliper check (2026-10-09): plate 149–150 × 109 × 32 mm (CAD 150 × 110 × 32); gaps from the plate edges to the round base 7 mm left and right (model 7), 26.7–26.8 mm at the front (model 27); so J1 is within 0.3 mm of the plate centre. |
| Connector face of the round base | **Differs from the mesh by about 2.5 mm.** The flat face with the connectors is about 62 mm from the J1 axis (13 mm from the back edge of the plate, caliper, 2026-10-09); the `joint1` mesh has it at 59.5 mm (cover outside at 67.3 mm). Only the drawing is affected. |
| Masses and inertias | **Not in the description yet** (arm and gripper). |

The generated URDF gives the same forward kinematics as the URDF before 2026-10-06
(difference 0 over 200 random poses). Two small changes: the collision origin of
`g_base` now equals its visual origin (it was 1 cm off), and the fixed joint
`g_base_to_joint1` has no `<limit>` element.

## What Elephant's repositories have (checked 2026-10-06)

`mycobot_ros` (c0d6fbe), `mycobot_ros2` (d42ff61) and `pymycobot` (v4.0.4):

- **No real dynamics data for the 280.** Every `<inertial>` is a placeholder
  (for example mass 0.1 kg and inertia 0.03 kg·m² on every link). Every MoveIt
  `joint_limits.yaml` for the 280 has no speed or acceleration limits.
- The same lengths in all 280 URDFs for Arduino and M5. Our flange is turned 45°.
  The ROS 1 Arduino URDF turns the whole robot 90° about the base axis.
- Joint limits: mostly ±168, ±140, ±150, ±150, −155 to +160, ±180°
  (`pymycobot/robot_info.py`), and other sets in other files.
- No servo models, masses, payload or repeatability in the documentation.
- The SRDF has one named pose, `init_pose` (all joints at 0).

## Next

- More in `calibration.yaml`: the IMU mounting and accelerometer calibration, and the
  play of each joint ([IMU and encoder errors](/mycobot-280-lab/results/imu-encoders/)).
- Store the calibration on each ATOM, so the firmware and the Control page can use it.
- `inertials.yaml`: masses and centres of mass from a gravity identification.
- Effort limits, when the servo models are known.
