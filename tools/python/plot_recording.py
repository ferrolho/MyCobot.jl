"""
Plot a recording from smooth_motion_demo.py and print tracking statistics.

Usage: python plot_recording.py recordings/<file>.csv   (writes <file>.png)
"""

import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

path = sys.argv[1]
d = np.genfromtxt(path, delimiter=",", names=True)
t = d["t"]
ok = ~np.isnan(d["q_1"])
dt = np.diff(t)

fig, axes = plt.subplots(6, 3, figsize=(15, 14), sharex=True)
print(f"{'joint':5s} {'RMS err':>8s} {'max err':>8s} {'lag':>7s} {'max |dq|':>9s} {'max |load|':>10s}")
for j in range(1, 7):
    qt, q, dq, load = d[f"q_target_{j}"][ok], d[f"q_{j}"][ok], d[f"dq_{j}"][ok], d[f"load_{j}"][ok]
    tt = t[ok]
    err = q - qt
    # Lag: shift (in samples) that best aligns the measured position with the target
    shifts = range(0, 120)
    lag_k = min(shifts, key=lambda k: np.mean((q[k:] - qt[:len(qt) - k]) ** 2))
    lag_ms = lag_k * np.mean(dt) * 1000
    dqt = np.gradient(qt, tt)
    print(f"J{j:<4d} {np.sqrt(np.mean(err ** 2)):7.2f}° {np.max(np.abs(err)):7.2f}° {lag_ms:5.0f} ms "
          f"{np.max(np.abs(dq)):7.1f}°/s {np.max(np.abs(load)):9.1f}%")

    ax = axes[j - 1]
    ax[0].plot(tt, qt, label="target", lw=1.5)
    ax[0].plot(tt, q, label="measured", lw=1)
    ax[0].set_ylabel(f"J{j} (°)")
    ax[1].plot(tt, dqt, label="target", lw=1.5)
    ax[1].plot(tt, dq, label="measured", lw=0.8)
    ax[1].set_ylabel("°/s")
    ax[2].plot(tt, load, lw=0.8, color="C2")
    ax[2].set_ylabel("load %")
    if j == 1:
        ax[0].set_title(f"position (lag ≈ {lag_ms:.0f} ms on J1)")
        ax[1].set_title("velocity")
        ax[2].set_title("load")
        ax[0].legend(loc="upper right", fontsize=8)
for ax in axes[-1]:
    ax.set_xlabel("t (s)")
fig.suptitle(f"{path} — {len(t)} cycles, {1 / np.mean(dt):.0f} Hz "
             f"(period {np.mean(dt) * 1000:.2f} ± {np.std(dt) * 1000:.2f} ms, max {np.max(dt) * 1000:.1f} ms)")
fig.tight_layout()
out = path.rsplit(".", 1)[0] + ".png"
fig.savefig(out, dpi=110)
print("saved", out)
