# Camera calibration from the robot base and a tape measure (2026-10-08)

No arm motion and no printed target. Working copy: `~/myCobot/media/camera-calib/` (the camera model in use is
`current_cam.npy` there; `experiments/pick-and-place/laptop/cam.py` loads it).

## Result

| Source | Points | rms |
| --- | --- | --- |
| Brass screws on the base plate, at the hole positions of the CAD model (`G_base.dae`, base frame = mesh + (0, 0, −32) mm since 2026-10-08 late; before, (0, −10, −32), and the camera files here were shifted 10 mm in y to the new frame) | 6 | 2.1 px |
| Tape measure on the table (cm digits, 10 mm apart; table z = −32 mm) | 241 in 7 placements | 1.9 px |
| Fingertip points of the 2026-10-06 calibration (arm FK) | 9 of 15 | 3.2 px |

- f = 1457 ± 9 px (the C505 data sheet value, 1272 px, is wrong for 1280×960). The camera is about 50 mm further
  away than the old model (`ref_cam.npy`, from fingertips only) said. Old vs new: 8–11 mm on the table, 11–18 mm at
  65–100 mm height. The old model was 15 px off at the screws.
- The tapes alone fix the table plane but not f against the camera distance; the fingertip heights (80 and 123 mm)
  do.
- The fingertip detector of 2026-10-06 marked the centre of the left pad, not the point where the closed pads meet:
  the fitted offset (−9.9, 10.4) mm is the left pad centre (CAD −10.2, 8.2). The meeting point is (−0.6, 8.2) mm
  (CAD, and a measurement on the arm: pads 13 × 21 × 25 mm, the pad face on the servo side flush with the J6 axis).

## Re-register after the camera moved (one snapshot, a few seconds)

```sh
cd ~/myCobot/media/camera-calib
python3 tool/rereg_screws.py          # snapshot from the Pi, report only
python3 tool/rereg_screws.py --save   # also write current_cam.npy (backup kept); refused above 2.5 px rms
```

It finds the plate (coarse shift), then each screw with a template, solves the screw-only pose and applies the
camera motion since the reference image to the accurate joint-fit pose. Tests: unmoved images 0.1–0.8 mm; simulated
camera turns of 1.5–3° recovered within 1.1 mm everywhere in the workspace. Needs 4 of the 6 screws in view (the arm
can hide some).

Redo the tape fit only if the lens changes (zoom, focus, resolution).

## Continuous tracking (2026-10-09)

`tool/track_screws.py` runs on the Pi and keeps the camera model up to date while someone watches the camera:
it rewrites `~/myCobot/lab-camera.json` (the lab service serves it as `/lab/camera.json`; the Control page's
overlay reads it every 0.5 s) with the screws it found as `marks`.

    ~/venvs/mycobot/bin/python experiments/camera-calib/tool/track_screws.py        # on the Pi (needs opencv-python-headless)
    python3 tool/track_screws.py --replay IMG ...                                  # offline: the filter over images

- Model: camera = M · T_accurate (the calibrated pose of the reference image), M = the camera's motion seen
  through the screws (as `rereg_screws.relative`). A Kalman filter on M (rotation, translation): random walk;
  each screw found near its predicted place is a measurement; a gate (chi-square) drops screws hidden by the
  arm or matched to the wrong hole; an update needs 2+ screws that agree within 3 px.
- A move: if fewer than 2 screws match, the filter searches again at once with a larger uncertainty and keeps
  the result if 3+ screws agree; after 2 such frames, the full-image search (`rereg_screws.register`) finds the
  plate again.
- Tests (2026-10-09): 55 snapshots with the arm moving and hiding screws: a point in the gripper's workspace
  (200, 100, 150) mm stays within 0.1–0.2 px (sd; range 1.3 px). Simulated camera turns (exact image warps of
  one snapshot): a 1° pan followed within 2.3 px, 2° and 4° bumps followed in the same frame within 0.8–3.3 px.
  About 10 ms per frame on the laptop.
- Top screws: the six plate screws are in one plane, so they leave the camera's tilt and sideways shift poorly
  determined. The tracker also uses the six screws on top of the base (J1 housing): light dots found by a
  difference of Gaussians. They are not where the URDF mesh puts them: the real base has **6 screws 60° apart**
  (two hexagon edges parallel to the front and back faces), the mesh has 8 screws 45° apart. Measured
  (consistency scan over the snapshots, 1.17 px rms): radius 34.5 mm, height 73 mm, first screw at 31°.
- The update drops the worst screw while its error is above 3 px and keeps the rest.
