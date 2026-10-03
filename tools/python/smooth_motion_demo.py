"""
MOVES THE ROBOT (all joints). Streams a smooth multi-joint trajectory directly
over the Feetech bus and records target and measured state every cycle.

Starts and ends at the zero pose (arm upright). Each joint follows
A_j · env(t) · sin(2π f t + φ_j), where env ramps 0 → 1 → 0 with a quintic
smoothstep so the motion starts and ends at rest. Each cycle sends goals to
all servos in one SYNC WRITE and reads position, speed and load in one SYNC READ.

Output: recordings/<timestamp>_smooth_motion.csv (see plot_recording.py).
Run `ftdi_latency.py 1` first. Don't send ATOM commands while this runs.
"""

import csv
import math
import os
import statistics
import time
from datetime import datetime

from mycobot_bus import (REG_ACCELERATION, REG_GOAL_POSITION, REG_GOAL_SPEED,
                         REG_PRESENT_POSITION, SERVO_IDS, ft_sync_read, ft_sync_write,
                         open_port)

AMPLITUDE_DEG = [30, 20, 25, 20, 30, 45]
PHASE = [0, 0, math.pi, math.pi / 2, math.pi / 2, 0]   # J3 opposite to J2
FREQ_HZ = 0.25
DURATION = 10.0
RAMP = 1.5                 # s, fade in and out
SPEED_CAP = 2000           # steps/s (~176°/s)
MAX_TRACKING_ERROR = 20.0  # degrees, abort and hold beyond this

SIGN = [-1, -1, +1, -1, -1, -1]          # joint angle direction vs servo position
STEPS_PER_DEG = 4096 / 360


def angle_to_pos(j, deg):
    return int(round(2048 + SIGN[j] * deg * STEPS_PER_DEG))


def pos_to_angle(j, pos):
    return SIGN[j] * (pos - 2048) / STEPS_PER_DEG


def le16(v):
    return [v & 0xFF, v >> 8]


def smoothstep(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * x * (x * (6 * x - 15) + 10)


def envelope(t):
    return smoothstep(t / RAMP) * smoothstep((DURATION - t) / RAMP)


def target(t):
    e = envelope(t)
    return [AMPLITUDE_DEG[j] * e * math.sin(2 * math.pi * FREQ_HZ * t + PHASE[j]) for j in range(6)]


def decode_speed(v):
    return -(v & 0x7FFF) if v & 0x8000 else v


def decode_load(v):
    return -(v & 0x3FF) / 10 if v & 0x400 else (v & 0x3FF) / 10   # percent


if __name__ == "__main__":
    sp = open_port()
    state = ft_sync_read(sp, REG_PRESENT_POSITION, 2)
    start = [pos_to_angle(j, state[j + 1][0] | state[j + 1][1] << 8) for j in range(6)]
    print("start angles:", [round(a, 2) for a in start])
    assert all(abs(a) < 3 for a in start), "start from the zero pose (send_angles zeros first)"

    first = target(0.0)
    ft_sync_write(sp, REG_GOAL_POSITION, {j + 1: le16(angle_to_pos(j, first[j])) for j in range(6)})
    ft_sync_write(sp, REG_ACCELERATION, {sid: [0] for sid in SERVO_IDS})
    ft_sync_write(sp, REG_GOAL_SPEED, {sid: le16(SPEED_CAP) for sid in SERVO_IDS})

    rows = []
    aborted = None
    t0 = time.perf_counter()
    try:
        while (t := time.perf_counter() - t0) < DURATION:
            q_target = target(t)
            ft_sync_write(sp, REG_GOAL_POSITION, {j + 1: le16(angle_to_pos(j, q_target[j])) for j in range(6)})
            state = ft_sync_read(sp, REG_PRESENT_POSITION, 6)
            t_read = time.perf_counter() - t0
            if len(state) != 6:
                rows.append([t, t_read] + q_target + [math.nan] * 18)
                continue
            q, dq, load = [], [], []
            for j in range(6):
                s = state[j + 1]
                q.append(pos_to_angle(j, s[0] | s[1] << 8))
                dq.append(SIGN[j] * decode_speed(s[2] | s[3] << 8) / STEPS_PER_DEG)
                load.append(SIGN[j] * decode_load(s[4] | s[5] << 8))
            rows.append([t, t_read] + q_target + q + dq + load)
            worst = max(range(6), key=lambda j: abs(q[j] - q_target[j]))
            if abs(q[worst] - q_target[worst]) > MAX_TRACKING_ERROR:
                aborted = f"J{worst + 1} tracking error {q[worst] - q_target[worst]:+.1f}°"
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
    path = f"recordings/{datetime.now():%Y%m%d-%H%M%S}_smooth_motion.csv"
    header = (["t", "t_read"] + [f"q_target_{j}" for j in range(1, 7)] + [f"q_{j}" for j in range(1, 7)]
              + [f"dq_{j}" for j in range(1, 7)] + [f"load_{j}" for j in range(1, 7)])
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)

    dts = [b[0] - a[0] for a, b in zip(rows, rows[1:])]
    failed = sum(1 for r in rows if math.isnan(r[8]))
    print(f"{len(rows)} cycles in {rows[-1][0]:.2f} s, {failed} failed reads, "
          f"period {statistics.mean(dts) * 1000:.2f} ± {statistics.stdev(dts) * 1000:.2f} ms "
          f"(max {max(dts) * 1000:.1f}) -> {1 / statistics.mean(dts):.0f} Hz")
    print("saved", path)
