"""
MOVES THE ROBOT. Closed loop entirely from the laptop over the Feetech bus:
each cycle SYNC WRITEs goal positions for all six servos and SYNC READs their
position/speed/load. J1 follows a small sine; J2-J6 hold where they are.

Aborts (and sends every joint back to its hold goal) if any joint strays.
Don't send ATOM commands while this runs.
"""

import math
import statistics
import time

from mycobot_bus import (REG_GOAL_POSITION, REG_GOAL_SPEED, REG_PRESENT_POSITION, SERVO_IDS,
                         decode_signed15, ft_read, ft_sync_read, ft_sync_write, open_port)

JOINT = 1
AMPLITUDE_DEG = 5.0
FREQ_HZ = 0.5
DURATION = 4.0
MAX_HOLD_DEV = 30        # steps (~2.6°) for the joints that should hold still
MAX_SINE_DEV = 10.0      # degrees from centre for the moving joint
SPEED_CAP = 1000         # steps/s (~88°/s); goal speed 0 means the servo won't move at all

STEPS_PER_DEG = 4096 / 360


def le16(value):
    value = int(round(value))
    return [value & 0xFF, value >> 8]


if __name__ == "__main__":
    sp = open_port()
    # Goal position uses the same units as present position (position mode)
    start = {j: int.from_bytes(ft_read(sp, j, REG_PRESENT_POSITION, 2), "little") for j in SERVO_IDS}
    hold_packet = {j: le16(start[j]) for j in SERVO_IDS}
    print("start:", start)
    ft_sync_write(sp, REG_GOAL_POSITION, hold_packet)          # goals = here, before enabling motion
    ft_sync_write(sp, REG_GOAL_SPEED, {j: le16(SPEED_CAP) for j in SERVO_IDS})

    log = []
    aborted = None
    t0 = time.perf_counter()
    try:
        while (t := time.perf_counter() - t0) < DURATION:
            target = start[JOINT] + AMPLITUDE_DEG * STEPS_PER_DEG * math.sin(2 * math.pi * FREQ_HZ * t)
            goals = dict(hold_packet)
            goals[JOINT] = le16(target)
            ft_sync_write(sp, REG_GOAL_POSITION, goals)

            state = ft_sync_read(sp, REG_PRESENT_POSITION, 6)
            if len(state) != len(SERVO_IDS):
                log.append((t, target, None, None))
                continue
            pos = {j: state[j][0] | state[j][1] << 8 for j in SERVO_IDS}
            speed = decode_signed15(state[JOINT][2] | state[JOINT][3] << 8)
            log.append((t, target, pos[JOINT], speed))

            for j in SERVO_IDS:
                if j != JOINT and abs(pos[j] - start[j]) > MAX_HOLD_DEV:
                    aborted = f"J{j} strayed to {pos[j]} (start {start[j]})"
            if abs(pos[JOINT] - start[JOINT]) > MAX_SINE_DEV * STEPS_PER_DEG:
                aborted = f"J{JOINT} beyond {MAX_SINE_DEV}°"
            if aborted:
                break
    finally:
        ft_sync_write(sp, REG_GOAL_POSITION, hold_packet)   # everyone back to hold
        time.sleep(0.5)
        ft_sync_write(sp, REG_GOAL_SPEED, {j: [0, 0] for j in SERVO_IDS})   # back to "don't move"
    if aborted:
        print("!! ABORTED:", aborted)

    ok = [s for s in log if s[2] is not None]
    dts = [b[0] - a[0] for a, b in zip(log, log[1:])]
    print(f"\ncycles {len(log)} in {log[-1][0]:.2f} s, failed reads {len(log) - len(ok)}")
    print(f"loop period {statistics.mean(dts) * 1000:.2f} ± {statistics.stdev(dts) * 1000:.2f} ms "
          f"(min {min(dts) * 1000:.2f}, max {max(dts) * 1000:.2f}) -> {1 / statistics.mean(dts):.0f} Hz")

    err = [(p - tg) / STEPS_PER_DEG for _, tg, p, _ in ok]
    print(f"J{JOINT} tracking error: RMS {math.sqrt(sum(e * e for e in err) / len(err)):.2f}°, max {max(map(abs, err)):.2f}°")

    # Delay estimate: shift that best aligns actual with target
    best = min(range(0, 200), key=lambda k: sum((ok[i + k][2] - ok[i][1]) ** 2 for i in range(len(ok) - 200)))
    print(f"best-fit delay ≈ {best} cycles ≈ {best * statistics.mean(dts) * 1000:.0f} ms")

    for t, tg, p, v in ok[::max(1, len(ok) // 16)]:
        print(f"  t={t:5.2f}s  target {(tg - start[JOINT]) / STEPS_PER_DEG:+6.2f}°  "
              f"actual {(p - start[JOINT]) / STEPS_PER_DEG:+6.2f}°  speed {v:5d}")
    end = {j: int.from_bytes(ft_read(sp, j, REG_PRESENT_POSITION, 2), "little") for j in SERVO_IDS}
    print("end (reported):", end)
    sp.close()
