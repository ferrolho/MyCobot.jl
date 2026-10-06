# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib"]
# ///
"""
Plot the encoder error against the encoder angle, from the CSV files that
scripts/fit_encoder_sweep.jl writes with --plot=DIR (one J<j>_encoder_error.csv per joint).

    uv run tools/python/plot_encoder_sweep.py DIR OUT.png
    uv run tools/python/plot_encoder_sweep.py --compare J BEFORE.csv AFTER.csv OUT.png

--compare plots one joint from two runs (for example before and after a correction).
"""
import csv
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

def load(path):
    rows = [list(map(float, r)) for r in csv.reader(open(path))]
    return [r[1] for r in rows], [r[2] for r in rows], [r[3] for r in rows]


if sys.argv[1] == "--compare":
    j, before, after, out = sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5]
    fig, ax = plt.subplots(figsize=(7, 3.6))
    for path, label in ((before, "before the correction"), (after, "with the correction")):
        q, e, _ = load(path)
        mid = (max(e) + min(e)) / 2
        ax.plot(q, [x - mid for x in e], ".", ms=1, label=f"{label}: {max(e) - min(e):.2f}° peak to peak")
    ax.set_title(f"J{j}: encoder angle minus gyro angle")
    ax.set_xlabel("joint angle (°)")
    ax.set_ylabel("error (°)")
    ax.grid(alpha=0.3)
    ax.legend(markerscale=8, fontsize=8)
    plt.tight_layout()
    plt.savefig(out, dpi=110)
    print(f"wrote {out}")
    sys.exit(0)

d, out = sys.argv[1], sys.argv[2]
joints = [j for j in range(1, 6) if os.path.exists(os.path.join(d, f"J{j}_encoder_error.csv"))]
fig, axs = plt.subplots(1, len(joints), figsize=(4 * len(joints), 3.4))
for ax, j in zip(axs, joints):
    rows = [list(map(float, r)) for r in csv.reader(open(os.path.join(d, f"J{j}_encoder_error.csv")))]
    q = [r[1] for r in rows]
    e = [r[2] for r in rows]
    fit = [r[3] for r in rows]
    up = [i for i in range(1, len(q)) if q[i] - q[i - 1] > 0.005]
    down = [i for i in range(1, len(q)) if q[i] - q[i - 1] < -0.005]
    ax.plot([q[i] for i in up], [e[i] - e[0] for i in up], ".", ms=1, label="angle increases")
    ax.plot([q[i] for i in down], [e[i] - e[0] for i in down], ".", ms=1, label="angle decreases")
    if max(q) - min(q) >= 150:
        ax.plot(q, [f - e[0] for f in fit], "k-", lw=0.8, label="fit (1× and 2× per turn)")
    ax.set_title(f"J{j}")
    ax.set_xlabel("encoder angle (°)")
    ax.grid(alpha=0.3)
axs[0].set_ylabel("encoder − gyro (°)")
axs[0].legend(markerscale=8, fontsize=7)
plt.tight_layout()
plt.savefig(out, dpi=110)
print(f"wrote {out}")
