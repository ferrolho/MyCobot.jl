---
title: Kinematic calibration with the camera
description: Joint zero offsets of the lab arm and the gripper position on the flange, measured with the calibrated lab camera on 2026-10-09.
---

The robot model puts each joint's 0° where Elephant's CAD puts it. On the real arm the zeros
are a little different. A small error at a joint near the base moves the fingertips by several
millimetres. This page measures the zero offsets of the lab arm with the lab camera.

## Method

1. Register the camera on the six base plate screws and the six screws on top of the base
   (`experiments/camera-calib/tool/track_screws.py`). The camera then knows where the base is.
   It does not know the arm.
2. Move the arm through poses in free air (random joint angles, the gripper closed, at least
   45 mm above the table, out of a keep-out box over the laptop). At each pose, take a snapshot
   and record the joint angles and the IMU. 39 poses in three batches, then 16 new poses to test.
3. Find the edges of the arm in each snapshot: subtract the background (the median of all the
   snapshots, because the arm moves and the scene does not), then Canny edges.
4. Draw the model's outline at the recorded angles and keep the points that no other part hides
   (ray test). Fit the six zero offsets so that the outline points lie on the edges (distance map,
   clipped at 15 px, robust loss). Repeat with the new outline until it does not change.
5. Find the 9 holes in the gripper's palm where the palm faces the camera (10 of the 55
   snapshots): dark discs of the expected size, then the whole hole pattern matched at once, so
   that a hole cannot be confused with its neighbour (the holes are about 8 px apart). Fit the
   offsets to the outline and the holes together.

The tools are in `experiments/kinematic-calib` on the plush-pick branch: `collect.jl` (the poses),
`kc_model.py` (the model, its outline and the camera), `kc_holes.py` (the palm holes) and
`kc_fit.py` (the fit; `--holes` adds the palm holes).

## Result (2026-10-09)

| Joint | J1 | J2 | J3 | J4 | J5 | J6 |
| --- | --- | --- | --- | --- | --- | --- |
| Zero offset (°) | +0.95 | −1.54 | +0.67 | −0.43 | +0.25 | +1.63 |

The gripper body is not where Elephant's model puts it. The camera puts it 2.4 mm nearer J6
along the J6 axis and 1.2 mm to the side. The palm holes and the screws on the back face of the
body agree.

- **Along the axis, it is the flange:** the body's back face sits on the flange face. A body
  2.4 mm nearer J6 means a flange face 2.4 mm nearer J6: the URDF joint `joint6output_to_joint6`
  is now 43.2 mm (Elephant: 45.6 mm). For the gripper and the fingertips this is the same as
  moving the body.
- **To the side, it is the mount:** `joint6output_to_gripper_base` is now at x −0.2, y 1.2 mm
  (z 34 mm as before). The gripper is held by LEGO-style connectors: it came off the flange on
  2026-10-07 and was put back.

- **Why the offsets alone did not fit:** with the zero offsets only, the hole pattern had the
  right shape in each snapshot (1–3 px), but the whole pattern was 4–8 px off, and in a
  different direction for each pose. A shift of the gripper body removes this. A lens
  distortion term, a gravity sag of J2 and J3, or a tilt of the gripper do not.
- **J1 is the clear result.** All fits give +0.9 to +1.3°.
- **J2 to J6 are weaker.** A fit on three batches and a fit on all four differ by up to 0.4°.
  The arm is white on a white wall and a light laptop, so many outline points have no edge to
  match. Only the palm holes are sharp point features.

Test on the 16 poses of the fourth batch, with the fit on the first three:

| | Palm holes (3 poses, 35 holes): error | Outline: mean distance to an edge |
| --- | --- | --- |
| No offsets, Elephant’s mount | — | 7.6 px |
| Outline fit only (the first result, below) | 10.0 px | 6.8 px |
| Outline + holes, gripper shifted | 2.7 px (about 1.3 mm) | 6.7 px |

The outline score does not separate the models: about 6.7 px is the limit of the edge images.
The palm holes do.

First result (outline fit only, the base plate screws only): +1.08, −0.20, −1.57, +0.34, −0.13,
+0.64°. It moved the fingertips by 5 mm on average.

## Where the offsets apply

- `mycobot_description/config/mycobot_280_arduino/calibration.yaml` (`zero_offset`) holds them.
- The Julia package applies them to every angle (`encoder_error`, `JOINT_ZERO_OFFSET`). The ATOM
  still gets its own angles, so the arm goes where the calibrated model says.
- The Control page draws the calibrated pose in the 3D view and the camera overlay when the lab
  service serves `/lab/calibration.json`. The numbers on the page and the goals it sends are the
  ATOM's angles.
- The firmware does not apply them.

## Not done yet

- **The closed fingertips are 2 mm short in the model.** From the body's back face (the face on
  the flange) to the end of the plastic finger (the middle of the inner edge), closed: 115 mm
  measured (2026-10-08 and 2026-10-09), 113.0 mm in the model. Open: 95 mm measured, 95.2 mm in
  the model. Fully closed, the foam pads do not touch, as in the model (−0.70 rad: 6 mm between
  the plastic pad faces); the camera fit of the closed finger angle agrees (−0.693 rad).
- **The J6 housing mesh** was 6.5 mm short at the ATOM end and 3.7 mm long at the flange end.
  It is now corrected from caliper measurements (see
  [Changes to Elephant's model](/mycobot-280-lab/software/model-changes/)). The
  outline fits in this page used the old mesh. A fit that lets the J6 axis move from J5 does
  not improve the error on new snapshots (palm holes: 2.8 px, 3.2 px with the J6 axis free), so
  the kinematics stay.
- A gravity sag term for J2 and J3: the fits did not agree, so it is not in the model.
- The finger model (the J7 to finger angle map) and the pads that the user added.
- A camera fit and an arm fit together. The camera is fixed by the screws here.
- More snapshots with the palm toward the camera: only 10 of the 55 have it.
