"""
Frequency response and model fit of one joint from a sysid recording (scripts/sysid.jl).

    python3 analyze_sysid.py RECORDING.csv [--plot OUT.png]

Input x = commanded position q_cmd_j, output y = measured position q_j. Estimates the
frequency response H(f) = Y/X (Welch cross-spectra) with its coherence, then fits

    G(s) = exp(-s*tau) * wn^2 / (s^2 + 2*zeta*wn*s + wn^2)

over 0.2-5 Hz, and checks the fit in the time domain (simulated vs measured position).
"""
import argparse
import re

import numpy as np
from scipy import optimize, signal


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    j = int(re.search(r"_J(\d)_", path).group(1))
    t, x, y = d["t"], d[f"q_cmd_{j}"], d[f"q_{j}"]
    ok = ~np.isnan(y)
    fs = 1 / np.median(np.diff(t))
    tu = np.arange(t[0], t[-1], 1 / fs)
    return j, fs, tu, np.interp(tu, t[ok], x[ok]), np.interp(tu, t[ok], y[ok])


def model_freq(f, tau, wn, zeta):
    s = 2j * np.pi * f
    return np.exp(-s * tau) * wn**2 / (s**2 + 2 * zeta * wn * s + wn**2)


def simulate(t, x, tau, wn, zeta):
    sys = signal.TransferFunction([wn**2], [1, 2 * zeta * wn, wn**2])
    xd = np.interp(t - tau, t, x, left=x[0])
    _, y, _ = signal.lsim(sys, xd, t, X0=None)
    return y


def analyse(path, plot=None):
    j, fs, t, x, y = load(path)
    nper = int(4 * fs)
    f, pxy = signal.csd(x, y, fs=fs, nperseg=nper)
    _, pxx = signal.welch(x, fs=fs, nperseg=nper)
    _, coh = signal.coherence(x, y, fs=fs, nperseg=nper)
    H = pxy / pxx
    band = (f >= 0.2) & (f <= 5.0) & (coh > 0.8)

    def resid(p):
        e = (model_freq(f[band], *p) - H[band]) * np.sqrt(coh[band])
        return np.concatenate([e.real, e.imag])

    fit = optimize.least_squares(resid, [0.05, 30.0, 0.7], bounds=([0, 1, 0.05], [0.5, 300, 3]))
    tau, wn, zeta = fit.x
    y_sim = simulate(t, x, tau, wn, zeta)
    rms = lambda e: float(np.sqrt(np.mean(e**2)))
    lag_best = min(np.arange(0, 0.3, 0.002), key=lambda L: rms(np.interp(t - L, t, x, left=x[0]) - y))

    print(f"J{j}  {path.split('/')[-1]}")
    print(f"  rate {fs:.0f} Hz, cmd RMS {rms(x):.2f}°, tracking RMS {rms(y - x):.2f}°")
    for fq in (0.5, 1, 2, 3, 4, 5):
        k = np.argmin(abs(f - fq))
        ph = np.angle(H[k], deg=True)
        print(f"  {fq:>4} Hz: gain {abs(H[k]):.3f}, phase {ph:7.1f}°, delay {-ph / 360 / fq * 1000:6.1f} ms, coherence {coh[k]:.2f}")
    print(f"  fit: tau {tau * 1000:.1f} ms, wn {wn:.1f} rad/s ({wn / 2 / np.pi:.2f} Hz), zeta {zeta:.2f}")
    print(f"  time domain: model RMS {rms(y_sim - y):.3f}°, best pure delay {lag_best * 1000:.0f} ms RMS {rms(np.interp(t - lag_best, t, x, left=x[0]) - y):.3f}°")

    if plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, figsize=(7, 8))
        fb = (f >= 0.15) & (f <= 6)
        ax[0].semilogx(f[fb], 20 * np.log10(abs(H[fb])), ".", label="measured")
        ax[0].semilogx(f[fb], 20 * np.log10(abs(model_freq(f[fb], *fit.x))), "-", label="fit")
        ax[0].set_ylabel("gain (dB)"); ax[0].legend(); ax[0].grid(True, which="both")
        ax[1].semilogx(f[fb], np.unwrap(np.angle(H[fb])) * 180 / np.pi, ".")
        ax[1].semilogx(f[fb], np.unwrap(np.angle(model_freq(f[fb], *fit.x))) * 180 / np.pi, "-")
        ax[1].set_ylabel("phase (°)"); ax[1].set_xlabel("frequency (Hz)"); ax[1].grid(True, which="both")
        ax[2].plot(t, x, lw=0.8, label="command"); ax[2].plot(t, y, lw=0.8, label="measured")
        ax[2].plot(t, y_sim, lw=0.8, ls="--", label="model")
        ax[2].set_xlabel("t (s)"); ax[2].set_ylabel(f"J{j} (°)"); ax[2].legend(); ax[2].grid(True)
        fig.tight_layout(); fig.savefig(plot, dpi=110)
    return dict(joint=j, tau=tau, wn=wn, zeta=zeta)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("recording")
    ap.add_argument("--plot")
    a = ap.parse_args()
    analyse(a.recording, a.plot)
