---
title: Pick and place (plush toy)
description: The arm picked a plush cow off the table and put it in a tissue box (2026-10-06), with the lab camera, a table touch with the IMU and the gripper at full torque. Calibration results, what failed and the grasp that worked.
---

On 2026-10-06 the arm picked a plush cow (about 110 × 70 × 45 mm, belly up) from the table
and put it in an open tissue box. The 13th attempt worked. There was no marker and no depth
camera: only the lab camera (monocular), the joint encoders, the IMU and the gripper state.

## Camera calibration

The camera (Logitech C505, 1280×960) has no fixed mount. Its pose was found from the robot:

1. The gripper (closed, pointing down) went to 16 known points at two heights (z = 96–141 mm).
2. A script found the fingertip in each snapshot: the difference to a frame without the arm,
   white pixels only, then the lowest pixel of the largest blob. One detection was wrong and
   was removed.
3. PnP with the focal length from the data sheet: 60° diagonal at 1280×720 gives
   f = 734 / tan 30° = **1272 px**. With f free or the image centre free, the fit is not stable.

| Result | Value |
| --- | --- |
| Reprojection error (15 points) | 3.7 px rms, 6.0 px max (about 2 mm at the toy) |
| Camera centre in the base frame | (−278, −353, 449) mm |
| Check: the J1 axis | Projects through the middle of the base cylinder |
| Check: the base plate | Fits only when turned 90° from Elephant's model (fixed, see [Robot description](/mycobot-280-lab/software/robot-description/)) |

**Depth is the weak direction.** A pixel gives a ray; its point on the table depends on the
height that you assume. 10 mm of height error moves the point about 7 mm along the line of
sight. The first grasps missed by 20–25 mm for this reason.

## Table height (touch with the IMU)

The closed fingertip went down in 3 mm steps at 6 °/s, next to the toy.

| Signal | Free air | At the table |
| --- | --- | --- |
| Peak acceleration during a step (IMU) | 0.12–0.19 g | **0.40 g, then 0.71 g** |
| Tilt at rest (IMU) | ±0.007 g | 0.027 g |
| Static load J2 | jumps between −1 and −23 | not usable (gear play) |

Table: **z = −30 mm** (fingertip model frame). The model has the base bottom at −32 mm.

Later the user saw the fingers touch the table at a modelled fingertip height of −22 mm. Thus
the real closed fingertip is about 8 mm lower than the model, or the table is higher there.
Not measured yet.

## Where the toy is: poke it

A poke from behind the toy, along the line of sight, measured its depth. The closed gripper
moved toward the camera in 4 mm steps, 12 mm above the table. After each step a script compared
the image of the head with a reference (mean absolute difference of a region; noise 3.4, limit 13).

| Step | Region change | |
| --- | --- | --- |
| 1–9 | 3.4–3.6 | No contact |
| 10 | 5.3 | First contact |
| 11 | 14.9 | The toy moves: stop |

The contact point and the pixel of the toy's far edge agree if the edge is at **z ≈ +15 mm**.
The earlier guesses (−18, then +5) were too low. A silhouette edge is the tangent point,
higher on the toy than the point that the fingers touch.

## The grasp that worked

The gripper opens to about 43 mm (rated clamp width 20–45 mm). The body (70 mm) does not fit;
the head (about 34 mm) does.

1. Target: the head pixel, back-projected at z = +15 mm, then 18 mm toward the camera. Finger
   axis across the head.
2. Go down with the fingers open to about 7 mm above the table.
3. **Close in steps and lift the arm by the fingertip drop of each step.** The fingertips move on
   an arc, 18 mm down from open to closed. Without this, the pads push the soft part down and out.
4. Lift in 10 mm steps. Read the opening at each step: it falls when the object slides out.
5. Carry at the lift height, open over the box.

Each step keeps the **commanded** target and changes only z. A first version took the measured
TCP as the next target: the gravity sag (3–6 mm) added up, 13 mm in one close.

## Gripper torque

Elephant's settings on the gripper servo (ID 7) limit it to 30 %. For the last grasps the limits
were raised in the servo's RAM (a power cycle restores them) and put back at the end.

| Register | Stock | Used | Note |
| --- | --- | --- | --- |
| 16 Max torque | 140 | 1000 | EEPROM area; the lock is on, so the value lasts to the next power cycle |
| 28 Protection current | 300 | 1000 | **This one caps register 48**: writes above 300 were ignored until it was raised |
| 48 Torque limit | 300 | 1000 | J1–J6 use 1000 |

At 100 % the gripper load read −1000 (full) while it held. Its temperature stayed at 45–51 °C.

## The attempts

| # | Target | Result |
| --- | --- | --- |
| 1 | Front leg, back-projected at z = −18 | Miss: the fingers went down 20–25 mm behind the leg |
| 3 | Other front leg, finger axis along the line of sight, z = +5 | Held (opening 490); slid out at 25 mm lift |
| 4 | Same leg, 10 mm lower | Held (440), squeezed out while closing (fingertip arc) |
| 5–6 | Head | Fingers on the head, then on the table: no close; J5 at 66 °C from pressing (lifted at once) |
| 7 | Head, fingertips 14 mm higher, 30 % torque | Held (766); slid out at 35 mm |
| 8–9 | Head, 60 % torque | Slid out at 20 mm; then a miss (the sag drift, fixed after this) |
| 10 | Poke from behind | Depth calibrated (above) |
| 11 | Head, poke-calibrated, 60 % | Held (924); slid out at 30 mm |
| 12 | Head, 100 %, fingertips lower | Held (429); slid out at 70 mm, nearly the whole toy in the air |
| 13 | Head (the toy had turned), 100 % | **Held (579 → 303) through a 120 mm lift and the carry. In the box.** |

The opening fell during every lift: the head slid slowly in the pads. A grasp lower on the
neck, nearer the centre of mass, should hold better (the user's suggestion; not tried).

## Tools

| Tool | Use |
| --- | --- |
| `tools/pi/record_session.py` | Records camera frames (5 fps, 1280×960) and the robot state (10 Hz) on the Pi |
| `tools/pi/make_timelapse.py` | Keeps only the parts with motion: 104 min → 22.7 min in 64 parts → 213 s at 30 fps |
| Lab scene (`/lab/scene.json`) | The box and the toy in the Control page's 3D view (see [Control page](/mycobot-280-lab/software/control-page/#lab-scene)) |

The helpers for this session (IK with the gripper pointing down, workspace checks, the poke and
the grasp) are in a scratch file on the Pi (`~/scratch/plush/plush.jl`), not in the repository yet.
