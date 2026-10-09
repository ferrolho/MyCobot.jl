---
title: Kinematic calibration with the camera
description: Joint zero offsets of the lab arm, measured with the calibrated lab camera on 2026-10-09.
---

The robot model puts each joint's 0° where Elephant's CAD puts it. On the real arm the zeros
are a little different. A small error at a joint near the base moves the fingertips by several
millimetres. This page measures the zero offsets of the lab arm with the lab camera.

## Method

1. Register the camera on the six base plate screws (1.1 px rms). The camera then knows where
   the base is. It does not know the arm.
2. Move the arm through poses in free air (random joint angles, the gripper closed, at least
   45 mm above the table, out of a keep-out box over the laptop). At each pose, take a snapshot
   and record the joint angles and the IMU. 39 poses in three batches, then 16 new poses to test.
3. Find the edges of the arm in each snapshot: subtract the background (the median of all the
   snapshots, because the arm moves and the scene does not), then Canny edges.
4. Draw the model's outline at the recorded angles and keep the points that no other part hides
   (ray test). Fit the six zero offsets so that the outline points lie on the edges (distance map,
   clipped at 15 px, robust loss). Repeat with the new outline until it does not change.

The tools are in `experiments/kinematic-calib` on the plush-pick branch: `collect.jl` (the poses),
`kc_model.py` (the model, its outline and the camera) and `kc_fit.py` (the fit).

## Result (2026-10-09)

| Joint | J1 | J2 | J3 | J4 | J5 | J6 |
| --- | --- | --- | --- | --- | --- | --- |
| Zero offset (°) | +1.08 | −0.20 | −1.57 | +0.34 | −0.13 | +0.64 |

- The fingertips move by 5 mm on average with these offsets, 9 mm at most.
- **J1 is the clear result.** Fits on each batch alone give +0.9 to +1.3°.
- **J2 to J6 are weaker.** Fits on each batch alone differ by up to about ±1.5°. The arm is white
  on a white wall and a light laptop, so many outline points have no edge to match.
- **The IMU agrees on the wrist tilt.** The ATOM's accelerometer (on the J5–J6 link) gives
  J2 + J3 + J4 = −1.0°; the camera fit gives −1.4°.

Test on 16 poses that the fit did not use:

| | Mean distance from the outline to an edge | Outline points within 3 px of an edge |
| --- | --- | --- |
| Without the offsets | 7.6 px | 32 % |
| With the offsets | 6.8 px | 39 % |

14 of the 16 poses improve.

## Where the offsets apply

- `mycobot_description/config/mycobot_280_arduino/calibration.yaml` (`zero_offset`) holds them.
- The Julia package applies them to every angle (`encoder_error`, `JOINT_ZERO_OFFSET`). The ATOM
  still gets its own angles, so the arm goes where the calibrated model says.
- The Control page draws the calibrated pose in the 3D view and the camera overlay when the lab
  service serves `/lab/calibration.json`. The numbers on the page and the goals it sends are the
  ATOM's angles.
- The firmware does not apply them.

## Not done yet

- A gravity sag term for J2 and J3: the fits did not agree, so it is not in the model.
- The finger model (the J7 to finger angle map) and the pads that the user added.
- A camera fit and an arm fit together. The camera is fixed by the screws here.
