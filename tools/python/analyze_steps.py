"""
Step-response metrics of one joint from sysid step recordings (scripts/sysid.jl JOINT steps).

    python3 analyze_steps.py RECORDING.csv [...]

For each step: steady-state error (mean of the last 0.4 s of the hold), overshoot past the
target, and the time to reach 90 % of the step after the commanded ramp starts.
"""
import re
import sys

import numpy as np


def analyse(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    j = int(re.search(r"_J(\d)_", path).group(1))
    t, r, y = d["t"], d[f"q_plan_{j}"], d[f"q_{j}"]
    v = np.gradient(r, t)
    moving = np.abs(v) > 1e-3
    # hold windows = runs of constant plan after a step ramp
    edges = np.flatnonzero(np.diff(moving.astype(int)) == -1) + 1      # ramp ends
    sse, ovs, rise = [], [], []
    for k, e in enumerate(edges):
        nxt = np.flatnonzero(moving[e:])
        end = e + (nxt[0] if len(nxt) else len(t) - e)
        if t[end - 1] - t[e] < 0.8:
            continue
        start = np.flatnonzero(moving[:e])[-1]
        r0 = r[max(start - 1, 0)]
        while start > 0 and moving[start - 1]:
            start -= 1
        r0, r1 = r[start - 1], r[e]
        step = r1 - r0
        if abs(step) < 1.0:
            continue
        w = (t >= t[end - 1] - 0.4) & (t < t[end - 1])
        sse.append(np.nanmean(y[w] - r1))
        seg = slice(start, end)
        ovs.append(max(0.0, np.nanmax((y[seg] - r1) * np.sign(step))))
        reach = np.flatnonzero((y[seg] - r0) * np.sign(step) >= 0.9 * abs(step))
        rise.append(t[seg][reach[0]] - t[start] if len(reach) else np.nan)
    name = path.split("/")[-1]
    pid = re.search(r"_pid([\d-]+)", name)
    print(f"J{j} PID {pid.group(1) if pid else 'default':>8}: steady error mean |e| {np.mean(np.abs(sse)):.2f}° "
          f"(max {np.max(np.abs(sse)):.2f}°), overshoot mean {np.mean(ovs):.2f}° max {np.max(ovs):.2f}°, "
          f"rise-90% {np.nanmean(rise) * 1000:.0f} ms   [{len(sse)} steps; errors {np.round(sse, 2)}]")


for p in sys.argv[1:]:
    analyse(p)
