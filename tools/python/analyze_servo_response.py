"""
Offline analysis of recordings (no robot needed):

1. Response model per joint: measured position vs commanded position, fitted as a
   pure delay, and as delay + first-order lag (T dq/dt = q_cmd(t - tau) - q).
   Fitted on the first smooth-motion run, validated on the second and on the
   lag-compensated circle run.
2. Friction: Coulomb friction from the jump in load at each velocity reversal
   (gravity is continuous across a reversal, friction flips sign), and a
   Coulomb + viscous fit on J1 (vertical axis, no gravity load).
3. Velocity estimation: the servo's speed register and causal filtered finite
   differences, against an offline (non-causal) reference.

Usage: python analyze_servo_response.py   (writes recordings/servo_analysis.png)
"""

import glob

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

DT = 0.002   # s, uniform grid for the analysis
LSB = 360 / 4096


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    ok = ~np.isnan(d["q_1"])
    names = d.dtype.names
    cmd = "q_cmd" if "q_cmd_1" in names else "q_target"
    ref = "q_plan" if "q_plan_1" in names else "q_target"
    t = d["t"][ok]
    grid = np.arange(t[0], t[-1], DT)
    get = lambda prefix: np.column_stack([np.interp(grid, t, d[f"{prefix}_{j}"][ok]) for j in range(1, 7)])
    return dict(t=grid, cmd=get(cmd), ref=get(ref), q=get("q"), dq=get("dq"), load=get("load"),
                t_raw=t, q_raw=np.column_stack([d[f"q_{j}"][ok] for j in range(1, 7)]))


def simulate(cmd, tau, T):
    k = int(round(tau / DT))
    u = np.concatenate([np.full(k, cmd[0]), cmd[:len(cmd) - k]]) if k else cmd
    if T <= 0:
        return u
    y = np.empty_like(u)
    y[0] = u[0]
    a = DT / (T + DT)
    for i in range(1, len(u)):
        y[i] = y[i - 1] + a * (u[i] - y[i - 1])
    return y


def fit_response(cmd, q):
    best = (np.inf, 0, 0)
    for tau in np.arange(0, 0.2001, 0.004):
        for T in np.arange(0, 0.1501, 0.005):
            e = np.sqrt(np.mean((simulate(cmd, tau, T) - q) ** 2))
            if e < best[0]:
                best = (e, tau, T)
    delay_only = min((np.sqrt(np.mean((simulate(cmd, tau, 0) - q) ** 2)), tau) for tau in np.arange(0, 0.3001, 0.002))
    return best, delay_only


def centered_slope(x, half):
    """Non-causal least-squares slope over ±half samples (reference velocity)."""
    k = np.arange(-half, half + 1)
    w = k / np.sum(k ** 2) / DT
    return np.convolve(x, w[::-1], mode="same")


def causal_fd_lowpass(x, fc):
    v = np.gradient(x, DT)
    v[1:] = np.diff(x) / DT
    a = DT / (DT + 1 / (2 * np.pi * fc))
    y = np.empty_like(v)
    y[0] = 0
    for i in range(1, len(v)):
        y[i] = y[i - 1] + a * (v[i] - y[i - 1])
    return y


def causal_slope(x, n):
    """Causal least-squares slope over the last n samples."""
    k = np.arange(n) - (n - 1) / 2
    w = k / np.sum(k ** 2) / DT
    return np.concatenate([np.zeros(n - 1), np.convolve(x, w[::-1], mode="valid")])


def best_lag(est, ref):
    errs = [(np.sqrt(np.mean((est[s:] - ref[:len(ref) - s]) ** 2)), s) for s in range(0, 100)]
    return min(errs)


if __name__ == "__main__":
    smooth = sorted(glob.glob("recordings/*_smooth_motion.csv"))
    circle = sorted(glob.glob("recordings/*_circle_lagcomp.csv"))[-1]
    fit_run, val_runs = load(smooth[0]), [load(smooth[1]), load(circle)]

    print("1. Response model (fit on", smooth[0].split("/")[-1], ")")
    print(f"{'joint':5s} {'delay-only':>18s} {'delay + 1st order':>30s}   validation RMS (smooth 2, circle)")
    params = []
    for j in range(6):
        (e, tau, T), (e0, tau0) = fit_response(fit_run["cmd"][:, j], fit_run["q"][:, j])
        val = [np.sqrt(np.mean((simulate(r["cmd"][:, j], tau, T) - r["q"][:, j]) ** 2)) for r in val_runs]
        params.append((tau, T))
        print(f"J{j + 1:<4d} τ={tau0 * 1000:4.0f} ms RMS {e0:.2f}°   τ={tau * 1000:4.0f} ms T={T * 1000:4.0f} ms RMS {e:.2f}°   "
              f"{val[0]:.2f}°, {val[1]:.2f}°")

    print("\n2. Friction (load in % of max torque)")
    for j in range(6):
        jumps = []
        for r in [fit_run] + val_runs:
            v = centered_slope(r["q"][:, j], 15)
            moving = np.abs(v) > 3
            sgn = np.sign(v) * moving
            idx = np.flatnonzero(sgn != 0)
            for a, b in zip(idx, idx[1:]):
                if sgn[a] != sgn[b] and (b - a) * DT < 0.6:
                    before = r["load"][max(a - 25, 0):a, j].mean()
                    after = r["load"][b:b + 25, j].mean()
                    jumps.append((after - before) * sgn[b])
        coulomb = np.median(jumps) / 2 if jumps else np.nan
        print(f"  J{j + 1}: Coulomb ≈ {coulomb:4.1f}% from {len(jumps)} reversals")
    r = fit_run
    v = centered_slope(r["q"][:, 0], 15)
    m = np.abs(v) > 3
    A = np.column_stack([np.sign(v[m]), v[m], np.ones(m.sum())])
    (c, b, off), *_ = np.linalg.lstsq(A, r["load"][m, 0], rcond=None)
    print(f"  J1 fit: load = {c:.1f}%·sign(v) + {b:.3f}%/(°/s)·v + {off:.1f}%  "
          f"(viscous part at 50°/s: {b * 50:.1f}%)")

    print("\n3. Velocity estimation vs offline reference (smooth run 2, all joints)")
    r = val_runs[0]
    rows = []
    for name, f in [("speed register", lambda x, j: r["dq"][:, j]),
                    ("finite diff + low-pass 10 Hz", lambda x, j: causal_fd_lowpass(x, 10)),
                    ("finite diff + low-pass 20 Hz", lambda x, j: causal_fd_lowpass(x, 20)),
                    ("causal LS slope, 10 samples (20 ms)", lambda x, j: causal_slope(x, 10)),
                    ("causal LS slope, 20 samples (40 ms)", lambda x, j: causal_slope(x, 20))]:
        errs, lags = [], []
        for j in range(6):
            x = np.interp(r["t"], r["t_raw"], r["q_raw"][:, j])   # raw (quantised) positions
            ref = centered_slope(x, 25)
            est = f(x, j)
            errs.append(np.sqrt(np.mean((est[50:-50] - ref[50:-50]) ** 2)))
            lags.append(best_lag(est[50:-50], ref[50:-50])[1] * DT * 1000)
        rows.append((name, np.mean(errs), np.mean(lags)))
        print(f"  {name:36s} RMS error {np.mean(errs):5.2f}°/s, lag ≈ {np.mean(lags):3.0f} ms")

    # Plot: J2 response model on the circle, and velocity estimators on J1
    fig, ax = plt.subplots(2, 1, figsize=(12, 8))
    c = val_runs[1]
    tau, T = params[1]
    ax[0].plot(c["t"], c["ref"][:, 1], "k--", lw=1, label="plan")
    ax[0].plot(c["t"], c["cmd"][:, 1], lw=1, label="commanded (lag-compensated)")
    ax[0].plot(c["t"], c["q"][:, 1], lw=1.5, label="measured")
    ax[0].plot(c["t"], simulate(c["cmd"][:, 1], tau, T), lw=1, label=f"model τ={tau * 1000:.0f} ms, T={T * 1000:.0f} ms")
    ax[0].set_title("J2 on the circle: the model misses the sticking after reversals (~3.9, 7.9, 12 s)")
    ax[0].set_ylabel("°")
    ax[0].legend(fontsize=8)
    x = np.interp(r["t"], r["t_raw"], r["q_raw"][:, 0])
    ax[1].plot(r["t"], r["dq"][:, 0], lw=0.8, label="speed register")
    ax[1].plot(r["t"], causal_slope(x, 20), lw=1, label="causal LS slope (40 ms)")
    ax[1].plot(r["t"], centered_slope(x, 25), "k", lw=1, label="offline reference")
    ax[1].set_title("J1 velocity estimates (smooth motion run 2)")
    ax[1].set_xlabel("t (s)")
    ax[1].set_ylabel("°/s")
    ax[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig("recordings/servo_analysis.png", dpi=110)
    print("\nsaved recordings/servo_analysis.png")
