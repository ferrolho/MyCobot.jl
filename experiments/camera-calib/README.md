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
