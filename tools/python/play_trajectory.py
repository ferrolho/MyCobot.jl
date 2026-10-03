"""
MOVES THE ROBOT. Stream a joint trajectory (CSV from a Julia planner, e.g.
scripts/plan_circle.jl) directly over the Feetech bus at ~300 Hz and record it.

The plan file has columns t,q_1..q_6 (seconds, degrees, ATOM angle convention)
and must start and end at the zero pose. Each joint's target is sent early by
its measured lag (LAG_S) unless --no-lag-comp is given.

Usage:
    python play_trajectory.py plans/circle.csv [--no-lag-comp]

Output: recordings/<timestamp>_<plan name>_<lagcomp|nolag>.csv
Run `ftdi_latency.py 1` first. Don't send ATOM commands while this runs.
"""

import argparse
import csv
import math
import os
import statistics
import time
from datetime import datetime

import numpy as np

from mycobot_bus import (REG_ACCELERATION, REG_GOAL_POSITION, REG_GOAL_SPEED,
                         REG_PRESENT_POSITION, SERVO_IDS, ft_sync_read, ft_sync_write,
                         open_port)
from smooth_motion_demo import SIGN, STEPS_PER_DEG, angle_to_pos, decode_load, decode_speed, le16, pos_to_angle

LAG_S = np.array([0.120, 0.113, 0.120, 0.054, 0.038, 0.028])   # measured in smooth_motion_demo
MAX_JOINT_SPEED = 90.0      # deg/s, sanity check on the plan
MAX_TRACKING_ERROR = 20.0   # deg, abort and hold beyond this
SPEED_CAP = 2000            # steps/s


def load_plan(path):
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    t, q = d[:, 0], d[:, 1:7]
    assert np.all(np.abs(q[0]) < 1) and np.all(np.abs(q[-1]) < 1), "plan must start and end at the zero pose"
    speed = np.abs(np.diff(q, axis=0) / np.diff(t)[:, None]).max()
    assert speed < MAX_JOINT_SPEED, f"plan joint speed {speed:.0f} °/s too high"
    return t, q


def sample(t_plan, q_plan, t, lag):
    return np.array([np.interp(t + lag[j], t_plan, q_plan[:, j]) for j in range(6)])


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("plan")
    ap.add_argument("--no-lag-comp", action="store_true")
    args = ap.parse_args()

    t_plan, q_plan = load_plan(args.plan)
    lag = np.zeros(6) if args.no_lag_comp else LAG_S
    duration = t_plan[-1] + 1.0

    sp = open_port()
    state = ft_sync_read(sp, REG_PRESENT_POSITION, 2)
    start = [pos_to_angle(j, state[j + 1][0] | state[j + 1][1] << 8) for j in range(6)]
    assert all(abs(a) < 3 for a in start), "start from the zero pose (send_angles zeros first)"

    q_cmd = sample(t_plan, q_plan, 0.0, lag)
    ft_sync_write(sp, REG_GOAL_POSITION, {j + 1: le16(angle_to_pos(j, q_cmd[j])) for j in range(6)})
    ft_sync_write(sp, REG_ACCELERATION, {sid: [0] for sid in SERVO_IDS})
    ft_sync_write(sp, REG_GOAL_SPEED, {sid: le16(SPEED_CAP) for sid in SERVO_IDS})

    rows, aborted = [], None
    t0 = time.perf_counter()
    try:
        while (t := time.perf_counter() - t0) < duration:
            q_ref = sample(t_plan, q_plan, t, np.zeros(6))
            q_cmd = sample(t_plan, q_plan, t, lag)
            ft_sync_write(sp, REG_GOAL_POSITION, {j + 1: le16(angle_to_pos(j, q_cmd[j])) for j in range(6)})
            state = ft_sync_read(sp, REG_PRESENT_POSITION, 6)
            if len(state) != 6:
                rows.append([t] + list(q_ref) + list(q_cmd) + [math.nan] * 18)
                continue
            q, dq, load = [], [], []
            for j in range(6):
                s = state[j + 1]
                q.append(pos_to_angle(j, s[0] | s[1] << 8))
                dq.append(SIGN[j] * decode_speed(s[2] | s[3] << 8) / STEPS_PER_DEG)
                load.append(SIGN[j] * decode_load(s[4] | s[5] << 8))
            rows.append([t] + list(q_ref) + list(q_cmd) + q + dq + load)
            err = np.abs(np.array(q) - q_ref)
            if err.max() > MAX_TRACKING_ERROR:
                aborted = f"J{err.argmax() + 1} tracking error {err.max():.1f}°"
                break
    finally:
        if aborted:
            here = ft_sync_read(sp, REG_PRESENT_POSITION, 2)
            ft_sync_write(sp, REG_GOAL_POSITION, {sid: list(here[sid]) for sid in here})
            print("!! ABORTED and holding:", aborted)
        time.sleep(1.0)
        ft_sync_write(sp, REG_GOAL_SPEED, {sid: [0, 0] for sid in SERVO_IDS})
    sp.close()

    os.makedirs("recordings", exist_ok=True)
    name = os.path.splitext(os.path.basename(args.plan))[0]
    tag = "nolag" if args.no_lag_comp else "lagcomp"
    path = f"recordings/{datetime.now():%Y%m%d-%H%M%S}_{name}_{tag}.csv"
    header = (["t"] + [f"q_plan_{j}" for j in range(1, 7)] + [f"q_cmd_{j}" for j in range(1, 7)]
              + [f"q_{j}" for j in range(1, 7)] + [f"dq_{j}" for j in range(1, 7)] + [f"load_{j}" for j in range(1, 7)])
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    dts = [b[0] - a[0] for a, b in zip(rows, rows[1:])]
    failed = sum(1 for r in rows if math.isnan(r[13]))
    print(f"{len(rows)} cycles, {failed} failed reads, {1 / statistics.mean(dts):.0f} Hz "
          f"(max period {max(dts) * 1000:.1f} ms)")
    print("saved", path)
