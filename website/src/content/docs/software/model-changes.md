---
title: Changes to Elephant's model
description: Every difference between our robot description and Elephant Robotics' URDF files and meshes for the myCobot 280, with the reason and the measurement for each.
---

Our robot description starts from Elephant Robotics' model of the myCobot 280. Where this
arm is different, we changed the model. This page records each change: what Elephant's
model has, what ours has, when we changed it and how we know.

The sources of our model are the xacro file and the YAML files in `mycobot_description/`,
and `tools/fix_meshes.py` for the meshes. See [Robot description](/mycobot-280-lab/software/robot-description/)
for how to change them.

## Elephant's files

All are in [mycobot_ros](https://github.com/elephantrobotics/mycobot_ros) (branch `noetic`,
`mycobot_description/urdf/`). `mycobot_ros2` (branch `humble`) has the same files. We
checked both on 2026-10-09: no newer meshes for the 280.

| What | Elephant's file | Ours |
| --- | --- | --- |
| Arm: links, joints, meshes | `mycobot_280_arduino/mycobot_280_arduino.urdf`, `*.dae`, `*.png` | The xacro file. Elephant's meshes are in this repository (Git LFS, commit `7cea001`, 2025-02-16), the same bytes as in mycobot_ros. |
| Base plate | `mycobot_280_m5/G_base.dae` (link `g_base` of the M5 model) | Link `g_base` |
| Adaptive gripper | `mycobot_280_m5/mycobot_280m5_with_gripper_parallel.urdf`, `gripper_*.dae` | The gripper part of the xacro file (`gripper:=true`) |
| STEP model "myCobot3.3" (SolidWorks, 2021) | `mycobot_280_arduino/mycobot_step.STEP` | Not used: it has no joint frames. Its wrist sizes agree with this arm within about 1 mm; we use it as a check. |

## Kinematics

These changes move the arm in the model: forward and inverse kinematics, the 3D view and
the camera overlay.

| Item | Elephant | Ours | Since | How we know |
| --- | --- | --- | --- | --- |
| J1 frame | `joint2_to_joint1` turned 90° about z; `joint1` mesh turned −90° | Not turned, as in Elephant's M5 model; `joint1` mesh turned 180° | 2025-02-16 | The camera fit gives a J1 zero offset of +0.95° only ([Kinematic calibration](/mycobot-280-lab/results/kinematic-calibration/)) |
| Flange turned about the J6 axis | 0° | 45° (`joint6output_to_joint6` rpy −90°, 45°, 0) | 2025-02-16 | The flange holes and the gripper on this arm |
| Flange face along the J6 axis | 45.6 mm from the J5–J6 axes crossing | 43.2 mm | 2026-10-09 | The 9 palm holes of the gripper in 10 camera snapshots put the gripper body 2.4 mm nearer J6 (held out: 10.0 → 2.7 px). Caliper: the gripper's back face to the first palm hole 13 mm (CAD 13.3 mm), so the gripper body matches its CAD and the flange moves. ATOM face to flange face 73 mm. |
| Joint limits | Elephant's values (J1 ±168°, …) | Measured by hand, `config/…/joint_limits.yaml` | 2026-10-04 | [Joint limits](/mycobot-280-lab/system/robot/#joint-limits) |
| Joint zero offsets, encoder correction | None | `config/…/calibration.yaml`. Not in the URDF: the Julia package and the Control page apply them. | 2026-10-06, 2026-10-09 | [Kinematic calibration](/mycobot-280-lab/results/kinematic-calibration/) |

## Gripper

| Item | Elephant | Ours | Since | How we know |
| --- | --- | --- | --- | --- |
| Mount roll | 1.579 rad | π/2 | 2026-10-06 | The fingers along the J6 axis |
| Mount position | 34 mm along the J6 axis, centred | 34 mm, and 1.2 mm to the side (flange y) | 2026-10-09 | The palm holes (camera fit). The gripper is held by LEGO-style connectors. |
| Mount angle about the J6 axis | — | Property `gripper_mount_deg` (0 on this arm) | 2026-10-06 | [Mount angle](/mycobot-280-lab/system/gripper/#mount-angle) |
| Finger joints on the body | y 27 mm and 5 mm | 7.5 mm nearer the flange (19.5 mm and −2.5 mm) | 2026-10-06 | The gears then sit in the slot of the body and the links on their pivots, as in the meshes' own coordinates |
| Finger range | −0.78 to 0.15 rad; the following joints have narrower limits | −0.70 to 0.15 rad; the following joints have the range that the mimic gives | 2026-10-06 | Pads about 6 mm apart closed, 43 mm open. The camera fit of the closed angle gives −0.693 rad. |

## Meshes and visual origins

These changes move only the drawing: the 3D view, the camera overlay and the outline that
the camera calibration fits. `tools/fix_meshes.py` makes the changed meshes from
Elephant's; do not edit a `.dae` file by hand.

| Item | Elephant | Ours | Since | How we know |
| --- | --- | --- | --- | --- |
| Base plate (`g_base`) | 30 mm under the base frame, turned 90° | 32 mm under the base frame (2025-02-16); not turned (2026-10-06); centred on J1 (2026-10-09) | 2026-10-09 | Plate 32 mm thick (caliper). The round base fitted in the camera image and a caliper check: J1 within 0.3 mm of the plate centre. |
| Screws on top of the base (`joint1.dae`) | 8 holes, 45° apart | 6 holes, 60° apart, at 31° + k·60° in the base frame, radius 34.55 mm. Two edges of the hexagon are parallel to the front and back faces. | 2026-10-09 | The camera tracker finds them there (`experiments/camera-calib`). The cap is rebuilt: its cross-section revolved, minus 6 holes of Elephant's shape. |
| Shoulder (`joint2` visual) | 60.96 mm under the J2 axis | 59.88 mm: 1.08 mm higher | 2026-10-09 | Elephant's J2 housing on this link was 1.08 mm off the J2 axis, so it did not meet the `joint3` housing that turns against it. A camera fit that lets the J2 axis height change does not improve the error on new snapshots (palm holes: 2.8 px, 3.2 px with the height free), so the axis stays and the mesh moves. |
| J6 housing (`joint6.dae`, the link with the ATOM) | ATOM end 21.6 mm, flange-side end 35.1 mm from the J5 axis (the flange overlaps the housing) | 28.1 mm and 31.4 mm | 2026-10-09 | Caliper: ATOM face to flange face 73 mm, flange 13–14 mm. A side photo and the STEP model (27.4 mm and 32.5 mm) agree within about 1 mm. The decals (USB-C port, connector, pin labels) keep their size. |
| Control page meshes (`website/public/robot/*.glb`) | One normal per face: each facet of a curved surface shows | Smooth normals (crease angle 30°), not simplified | 2026-10-09 | `tools/web_meshes.py` |

## Differences not in the model

| Part | This arm | Model | Note |
| --- | --- | --- | --- |
| Closed fingertips | 115 mm from the gripper's back face to the end of the plastic finger (middle of the inner edge) | 113.0 mm | Not resolved. Open: 95 mm measured, 95.2 mm in the model. Fully closed, the foam pads do not touch, as in the model. The camera fit agrees with the model's closed angle (−0.693 rad), and the palm holes put the gripper body where its CAD puts it, so the 2 mm is in the fingers. To check: the gap between the closed plastic ends (6 mm in the model), the length of one finger, and which point and frame `fingers_urdf.py` used for its 114.7 mm (2026-10-08). |
| Connector face of the base | About 62 mm from the J1 axis | 59.5 mm | Caliper, 2026-10-09: 13 mm from the plate edge to the flat connector face (not its cover), 15.5 mm in the model. Only the drawing: the camera registration and the kinematics do not use this face. |
