"""
Plot flange paths computed by scripts/trace_recording.jl: planned path vs the path
traced by the robot, plus Cartesian error over time.

Usage: python plot_circle.py recordings/<a>_path.csv [recordings/<b>_path.csv ...]
       (writes <last>.png)
"""

import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

paths = sys.argv[1:]
fig, axes = plt.subplots(1, 2, figsize=(14, 6.5))
for k, path in enumerate(paths):
    d = np.genfromtxt(path, delimiter=",", names=True)
    plan = np.column_stack([d["plan_x"], d["plan_y"], d["plan_z"]])
    meas = np.column_stack([d["meas_x"], d["meas_y"], d["meas_z"]])
    err = np.linalg.norm(meas - plan, axis=1)
    stamp, *_, tag = path.removesuffix("_path.csv").rsplit("/", 1)[-1].split("_")
    label = f"{tag}, {stamp[-6:-4]}:{stamp[-4:-2]}"
    if k == 0:
        axes[0].plot(plan[:, 1], plan[:, 2], "k--", lw=1.2, label="planned")
    axes[0].plot(meas[:, 1], meas[:, 2], lw=1.2, label=f"traced ({label})")
    axes[1].plot(d["t"], err, lw=1, label=label)

axes[0].set_aspect("equal")
axes[0].set_xlabel("y (mm)")
axes[0].set_ylabel("z (mm)")
axes[0].set_title("flange path seen from the front")
axes[0].legend(fontsize=9)
axes[1].set_xlabel("t (s)")
axes[1].set_ylabel("|traced − planned| (mm)")
axes[1].set_title("Cartesian tracking error")
axes[1].legend(fontsize=9)
fig.tight_layout()
out = paths[-1].removesuffix(".csv") + ".png"
fig.savefig(out, dpi=110)
print("saved", out)
