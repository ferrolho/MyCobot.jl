---
title: Pick and place (plush toy, corks)
description: The arm put a plush cow and then four wine corks in a tissue box (2026-10-06), with the lab camera, a table touch with the IMU and the gripper. Calibration results, what failed and the grasps that worked.
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

## Second task: four wine corks (2026-10-06 evening)

The box moved next to the wall, left of the robot. Four corks lay on the table. All four
went in the box. Corks C, D and B worked with the method above (B on the third try). Cork A,
next to the box wall, took seven tries.

### Checks with known sizes

| Object | Data | Camera estimate |
| --- | --- | --- |
| Wine cork | 24 mm diameter; 38, 44.5 or 49 mm long (standard sizes) | 39–49 mm long (back-projected at z = −10, about the top of a cork) |
| Tissue box (Kleenex Ultra Soft Extra Large) | 15.5 × 16 × 3.5 cm (user, with a ruler) | 16 × 15 × 3.5–3.9 cm |

### The camera moved: calibrate again, at the work height

During the task the camera moved about 50 mm (a lamp was added and the camera
reconnected on USB). A few pixels of shift on a fixed object did not show this: the camera
had also turned. Fingertips predicted with the old model were 50 px (20–25 mm) off.

A new calibration used 9 points at fingertip heights −13 to 80 mm, near the corks:

| Item | Value |
| --- | --- |
| Error | **2.0 px rms**, 3.6 px max |
| Focal length, fitted | 1276 px (data sheet: 1272 px) |
| Camera centre | (−232, −376, 462) mm |
| Rejected detections | 3 of 12 (white tissues under the gripper; light wood under the new lamp) |

The first calibration had points only at z = 96–141 mm. It extrapolated down to the table
with an error of 20 mm or more. This explains the depth errors of the first task (the
"+15 mm" from the poke and the 18 mm shift). **Calibrate at the height where you grasp.**

### What made cork A fail, and the fix

| Problem | Effect | Fix |
| --- | --- | --- |
| The sag correction moved the fingers sideways at cork height | The fingers pushed the cork away | Measure the sag 15 mm above, then go straight down with that offset |
| A finger came down on the cork | Fingers pushed open (opening 974 for goal 700) | Better position (new calibration) |
| Fingers across the cork at 45° | The cork spans 48 mm along the fingers: more than 43 mm | Keep the fingers perpendicular to the cork |
| Pads above the middle of the cork | Closed at 188–257 and slid out | Fingertips about 3 mm lower |

The grasps that held stalled at an opening of 266–325 (16–18 mm between the pads for a
24 mm cork) and kept it through the lift.

### Other findings

- The joint-space path from the box (near the wall) to a far cork goes near the wall:
  carry through a waypoint near the robot, (−130, −110).
- The ATOM did not join WiFi again after a router restart (fixed in firmware 4.6.1).
- After a power cycle the arm sags into a new pose. Check the pose before the first move.

## Third run: the corks again (2026-10-06, 19:00–19:50)

The user put the corks back on the table, in a group, and the box further left. Claude
worked alone. All four corks went in the box. Two findings made it work.

### The fingertip is not where the model says

The first calibrations used one gripper yaw only. Then an offset of the fingertip in the
flange frame is a constant world offset, and the camera fit absorbs it. At other yaws the
offset turns with the gripper and comes back as an error of up to 2× its size. This explained
why the errors changed from cork to cork (and the fingers that landed on a cork end).

A calibration with four yaws (24°, 114°, 204°, 294°; 11 points, fingertip 28–74 mm above the
table) fitted the camera and the fingertip offset together:

| Fit | Error |
| --- | --- |
| Camera only | 12.4 px rms |
| Camera + fingertip offset | **1.3 px rms**, 2.1 px max |
| Fingertip centre in the flange frame | x **+6.7 mm** (along the finger axis), y 10.6 mm (model: 0, 8) |

**Calibrate with several yaws.** The offset is now part of the TCP.

Also: the end-face centres of a cork lie on its axis (z = −18), not on its top. Back-projected
at z = −18, the four corks measured 41–44 mm long (standard: 44.5 mm).

### A smooth grasp (the user's idea)

The stepped close (close a little, lift the arm, repeat) let the fingertips touch the table
or the cork between the steps. The user proposed one smooth close while the arm moves up by
the fingertip extension that the gripper's measured position gives:

1. Compute the joint positions for TCP lifts of 0–24 mm (1 mm steps) before the close.
2. Send one GRIPPER command (goal 0) and SUBSCRIBE to the STREAM at 50 Hz.
3. At each STREAM message: fingertip extension = `tip_z(opening) − tip_z(start)`; send the
   interpolated joint goal with TRACK (the firmware takes GRIPPER during TRACK, not during PLAY).
4. Stop when the opening has not changed for 0.3 s (holding or closed).

These runs used firmware 4.6 and its GRIPPER command. With firmware 5.0 the gripper is joint
[J7](/mycobot-280-lab/comms/websocket-api/#joints-j1j6-and-j7-50): `grasp_smooth!` sends J7's
goal in the same TRACK messages as the arm, and `gripper!` moves J7 with MOVE_TO. The helpers
keep the opening scale of these runs (0 closed to 1000 open, servo steps 1477–2033). Not tested
on the robot yet.

A close takes 1.1–1.4 s and lifts the arm 12–13 mm. Results:

| Cork | Tries | Note |
| --- | --- | --- |
| 1 | 1 | Held at 284 |
| 4 | 2 | First try: a finger on the cork (stall at 949). Then held at 339. At r = 263 mm the lift stops at the reach limit (about 78 mm). |
| 3 | 2 | First try: the back finger on the cork (the finger axis is along the line of sight). Then held at 296. |
| 2 | 1 | Held at 309 |

An immediate stall above about 850 means that a finger is on the object: lift, move along
the finger axis, try again.

## Fourth task: a Jellycat espresso cup (2026-10-06, 20:20–20:35)

A plush espresso cup (10 × 5 × 5 cm, 7 cm high: data sheet) and the box at a new place. The
cup body (about 50 mm) does not fit in the gripper (43 mm). The handle does: a soft loop
about 30 mm long, out of the side of the cup. The arm took the cup by the handle on the
fourth try (the smooth grasp, fingers across the loop), lifted it 70 mm and put it in the box.

| Try | Result |
| --- | --- |
| 1 | Closed to 34: the fingers were at the hole of the loop |
| 2 | Aim at the outer part of the loop: closed to 0 (empty) |
| 3 | Back-projected at z = 0 (the handle is lower than assumed): held at 59, then the check opened the gripper |
| 4 | Same place, gripper torque 100 %, lower limits: held at 62 (load −1000) through the lift and the carry. **In the box.** |

**The opening is not a grasp check for thin, soft parts.** A squeezed handle reads almost
"closed" (30–60). The user saw try 3 hold the handle while the check let go. Better (the
user's suggestion): lift to a fixed height first, then judge the grasp (camera, load).
The camera did not move during this task (< 0.5 px), so the calibration of run 3 stayed valid.

