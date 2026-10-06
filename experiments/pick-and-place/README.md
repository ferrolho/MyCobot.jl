# Pick-and-place experiments (2026-10-06)

Experiment code from the session "plush-toy-experiment". This code is for these runs only and stays on the
branch `plush-pick`. The reusable parts (session recorder, timelapse tool, lab scene, firmware 4.6.1 and 4.7.0)
are on `main`.

The scripts are copies of the working files. They contain the paths of the original folders (the Pi's
`~/scratch/...` and a temporary folder on the laptop). Change the paths before you run them again.

## Tasks and data

| Task | Result | Recording on the Pi (`~/recordings/2026-10-06-pick-and-place/`) | Time (BST) | Videos on the laptop (`~/myCobot/media/2026-10-06-pick-and-place/`) |
|---|---|---|---|---|
| Plush cow into the tissue box | Done (attempt 13) | `plush/rec` (29 364 frames) | 10-06 15:33–17:17 | `plush-cow-timelapse-*.mp4`, `plush-cow-timelapse-annotated-*.mp4` |
| 4 wine corks into the box, run 1 | Done | `corks/rec` (16 245 frames) | 10-06 17:47–18:52 | `corks-timelapse-*.mp4`, `corks-timelapse-annotated-*.mp4` |
| Corks, runs 2 and 3 | Done | `corks2/rec` (15 006 frames) | 10-06 18:56–19:49 | `corks-run2-timelapse-*.mp4`, `corks-run3-annotated-*.mp4` |
| Jellycat espresso cup into the box | Done (try 4) | `cup/rec` (6 391 frames) | 10-06 20:16–20:38 | `espresso-cup-timelapse-*.mp4` |
| Fork and spoon into the mug | Spoon done; fork not done | `cutlery/rec` (36 726 frames) | 10-06 22:23 – 10-07 00:38 | No video yet |

Each `rec` folder has `frames/<unix ms>.jpg` (camera, about 4.7 frames/s) and `robot.csv` (STREAM at 50 Hz,
Pi clock). The task folders also have the snapshots of each attempt (`*.jpg`) and the calibration points
(`cal*.csv`).

## Make the videos again

1. `tools/pi/make_timelapse.py <rec> --list kept.txt` writes the frames where the robot moves and the motion parts.
   The lists of the published videos are in `annotation/` (`kept_*.txt`, and `kept.txt` in each folder).
2. `annotation/<video>/annotate.py` draws the labels (ASD-STE100 style, lower left on the wall, speed label)
   and encodes at 60 frames/s with ffmpeg (libx264). Edit `STEPS` to change the text and the times.

| Folder | Video | Recording |
|---|---|---|
| `annotation/plush-cow` | Plush cow, annotated | `plush` |
| `annotation/corks-run1` | Corks run 1, annotated | `corks` |
| `annotation/corks-run3` | Corks run 3, annotated (60 frames/s, faster) | `corks2` |

## Code

| Folder | Contents |
|---|---|
| `pi/plush.jl` | Julia helpers on the Pi (included by `laptop/pj.sh`): collision zones, IK with a tool pose, moves with sag correction, smooth grasp (one close, the arm follows the measured opening), utensil routines (hang, tilt, aligned descent), contact probe |
| `laptop/` | Camera model (`cam.py`: project, ray, back-projection on a plane), PnP and calibration fits (`fit*.py`, `camfit.py`, `pnp.py`), tip detection (`detect*.py`), contact sheets (`watch.py`), grid overlay (`grid.py`), hang angle from the camera (`hangfit.py`), finger geometry from the gripper URDF (`fingers_urdf.py`) |
| `calibration/` | Camera fits (`cam*.npy`; `cam10_free.npy` is the last one, after the base turned 180°) and calibration points (`*_cal*.csv`) |

## Open problems

- **Fingertip model.** The helpers model the closed fingertip as a point 105–115 mm along the flange z axis
  (fitted with the gripper pointing down). The gripper URDF puts the fingertips at about 61 mm. With the gripper
  pointing down, the camera fit cannot see a tool length error. With a tilted gripper, the error is large.
  Measure it before tilted moves near the table.
- **Contact.** Do not find contact by position lag: the gripper mount bends first. On 2026-10-07 a probe
  pressed the gripper into the table and the mount came apart. Stop at the first J2/J3 load change or IMU
  spike, with a small limit past the expected contact.
- **Fork.** The fork is not in the mug. The hang method that put the spoon in (grip, let it swing, fit the angle
  with the camera, tilt about the finger axis until it hangs vertical, lower into the mug) should work.
