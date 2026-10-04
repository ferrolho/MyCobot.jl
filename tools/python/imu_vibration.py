"""
End-effector vibration from the ATOM's IMU in an onboard recording (scripts/play_plan.jl --atom=IP).

    python3 imu_vibration.py RECORDING.csv [...] [--from=4 --to=12]

Vibration = the signal minus its 0.2 s centred moving average (content above ~2.5 Hz), as on
the "Onboard control and vibration" page. Prints the RMS and peak of the acceleration vector
(mg) and of the rotation-rate vector (°/s) between --from and --to seconds (default: the circle
part of the circle plan, 4-12 s).
"""
import sys

import numpy as np


def vibration(path, t0=4.0, t1=12.0):
    d = np.genfromtxt(path, delimiter=",", names=True)
    t = d["t"]
    fs = 1 / np.median(np.diff(t))
    n = max(1, int(round(0.2 * fs)))
    k = np.ones(n) / n
    hp = lambda x: x - np.convolve(np.nan_to_num(x), k, mode="same")
    m = (t >= t0) & (t <= t1)
    acc = np.sqrt(sum(hp(d[f"acc_{a}"]) ** 2 for a in "xyz"))[m] * 1000
    gyr = np.sqrt(sum(hp(d[f"gyro_{a}"]) ** 2 for a in "xyz"))[m]
    rms = lambda x: float(np.sqrt(np.mean(x**2)))
    return rms(acc), float(np.max(acc)), rms(gyr), float(np.max(gyr))


if __name__ == "__main__":
    opts = {a.split("=")[0][2:]: float(a.split("=")[1]) for a in sys.argv[1:] if a.startswith("--")}
    for p in [a for a in sys.argv[1:] if not a.startswith("--")]:
        a, ap, g, gp = vibration(p, opts.get("from", 4.0), opts.get("to", 12.0))
        print(f"{p.split('/')[-1]}: acceleration RMS {a:.0f} mg peak {ap:.0f} mg | rotation rate RMS {g:.1f} peak {gp:.0f} °/s")
