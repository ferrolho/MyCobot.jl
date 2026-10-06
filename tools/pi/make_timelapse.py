#!/usr/bin/env python3
"""
Make a timelapse from a recording of tools/pi/record_session.py: keep the frames where the robot
moves (a plan plays, a joint moves or the gripper opening changes), with a margin, and cut the rest.

    python3 tools/pi/make_timelapse.py REC_DIR OUT.mp4 [--fps 30] [--pad 1.0] [--merge 3.0] [--width 960] [--list FILE]

A frame is kept if it is within --pad s of motion. Pauses shorter than --merge s stay in.
Needs ffmpeg. Prints how much was kept.
"""
import argparse
import bisect
import csv
import os
import subprocess
import tempfile

ap = argparse.ArgumentParser()
ap.add_argument("rec")
ap.add_argument("out")
ap.add_argument("--fps", type=float, default=30.0)
ap.add_argument("--pad", type=float, default=1.0)
ap.add_argument("--merge", type=float, default=3.0)
ap.add_argument("--width", type=int, default=960)
ap.add_argument("--list", help="also write the kept frames (one file name per line) and the parts (# start end, Unix s)")
args = ap.parse_args()

PLAYING = 3
moving = []
with open(os.path.join(args.rec, "robot.csv")) as f:
    prev = None
    for row in csv.DictReader(f):
        t = float(row["t"])
        pos = [int(row[f"pos{j}"]) for j in range(1, 7)]
        op = int(row["opening"])
        if prev is not None:
            dpos = max(abs(a - b) for a, b in zip(pos, prev[0]))
            if int(row["state"]) == PLAYING or dpos > 3 or abs(op - prev[1]) > 3:
                moving.append(t)
        prev = (pos, op)

# Intervals of motion, padded and merged.
spans = []
for t in moving:
    a, b = t - args.pad, t + args.pad
    if spans and a <= spans[-1][1] + args.merge:
        spans[-1][1] = max(spans[-1][1], b)
    else:
        spans.append([a, b])
starts = [s[0] for s in spans]

frames = sorted(os.listdir(os.path.join(args.rec, "frames")))
times = [int(f[:-4]) / 1000 for f in frames]


def kept(t):
    i = bisect.bisect_right(starts, t) - 1
    return i >= 0 and t <= spans[i][1]


keep = [f for f, t in zip(frames, times) if kept(t)]
total = times[-1] - times[0]
active = sum(min(b, times[-1]) - max(a, times[0]) for a, b in spans)
if args.list:
    with open(args.list, "w") as f:
        for a, b in spans:
            f.write(f"# {a:.3f} {b:.3f}\n")
        f.writelines(k + "\n" for k in keep)
print(f"{len(frames)} frames over {total / 60:.1f} min; kept {len(keep)} frames ({active / 60:.1f} min of motion, {len(spans)} parts)")

with tempfile.TemporaryDirectory() as tmp:
    for i, f in enumerate(keep):
        os.symlink(os.path.join(os.path.abspath(args.rec), "frames", f), os.path.join(tmp, f"{i:06d}.jpg"))
    subprocess.run(["nice", "-n", "19", "ffmpeg", "-loglevel", "error", "-y", "-framerate", str(args.fps),
                    "-i", os.path.join(tmp, "%06d.jpg"), "-vf", f"scale={args.width}:-2", "-c:v", "libx264",
                    "-preset", "veryfast", "-crf", "26", "-pix_fmt", "yuv420p", "-threads", "2", args.out], check=True)
print("wrote", args.out, f"({len(keep) / args.fps:.0f} s at {args.fps:g} fps)")
