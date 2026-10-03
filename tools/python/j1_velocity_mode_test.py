"""
MOVES THE ROBOT. Switch J1 (base yaw) to velocity mode, spin at 100 steps/s
(~8.8°/s) for 1 s, stop, then return to position mode without a jump.

All reads and writes go directly to the servo over the Feetech bus (no ATOM).
Aborts if J1 moves more than ~25°. Keep a hand near the power switch.

First run 2026-10-03 (through the ATOM): tracked 103 steps/s for 100
commanded, with ~0.25 s of dead time before motion started. See
docs/fast-communication.md.
"""

import time

from mycobot_bus import (FT_READ, REG_GOAL_POSITION, REG_GOAL_SPEED, REG_MODE,
                         REG_PRESENT_POSITION, REG_TORQUE_ENABLE, decode_signed15, ft_packet,
                         ft_read, ft_txrx, ft_write, open_port, parse_status_packets)

J = 1
SPEED = 100      # steps/s; 4096 steps per turn
DURATION = 1.0   # s
MAX_DEV = 285    # steps (~25°)


def deg(steps):
    return steps * 360 / 4096


if __name__ == "__main__":
    sp = open_port()
    rd = lambda a: ft_read(sp, J, a, 1)[0]
    wr = lambda a, *data: ft_write(sp, J, a, list(data))
    pkt = ft_packet(J, FT_READ, [REG_PRESENT_POSITION, 4])

    def pos_speed():
        pkts = parse_status_packets(ft_txrx(sp, pkt, 10))
        if not pkts:
            raise IOError("no reply from J1")
        p = pkts[0][2]
        return p[0] | p[1] << 8, decode_signed15(p[2] | p[3] << 8)

    assert rd(REG_MODE) == 0, "J1 is not in position mode"

    wr(REG_GOAL_SPEED, 0, 0)          # speed 0 first: no motion in either mode
    wr(REG_MODE, 1)
    assert rd(REG_MODE) == 1, "mode change did not take"
    time.sleep(0.2)
    # In velocity mode the reported position is raw (calibration offset not applied)
    p0, _ = pos_speed()

    log = []
    t0 = time.perf_counter()
    try:
        wr(REG_GOAL_SPEED, SPEED & 0xFF, SPEED >> 8)
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < DURATION:
            p, v = pos_speed()
            log.append((time.perf_counter() - t0, p, v))
            if abs(p - p0) > MAX_DEV:
                print("!! deviation limit hit")
                break
    finally:
        wr(REG_GOAL_SPEED, 0, 0)
        print("stopped")

    t_stop = time.perf_counter()
    while time.perf_counter() - t_stop < 0.3:
        p, v = pos_speed()
        log.append((time.perf_counter() - t0, p, v))
    p1, _ = pos_speed()

    # Back to position mode. The mode switch turns torque off; goal speed is 0, so
    # nothing moves. Then set goal = present (same units as present in position mode),
    # which turns torque back on and holds the joint where it is.
    wr(REG_MODE, 0)
    time.sleep(0.1)
    here = int.from_bytes(ft_read(sp, J, REG_PRESENT_POSITION, 2), "little")
    wr(REG_GOAL_POSITION, here & 0xFF, here >> 8)
    print(f"back in position mode: {rd(REG_MODE) == 0}, torque {rd(REG_TORQUE_ENABLE)}, holding at {here}")

    print(f"\n{len(log)} samples, {len(log) / log[-1][0]:.0f} Hz logging")
    for t, p, v in log[::max(1, len(log) // 15)]:
        print(f"  t={t:5.3f}s  {deg(p - p0):+6.2f}°  reported speed {v:4d} steps/s")
    steady = [s for s in log if 0.3 < s[0] < DURATION]
    if len(steady) > 2:
        (ta, pa, _), (tb, pb, _) = steady[0], steady[-1]
        print(f"steady speed {(pb - pa) / (tb - ta):.0f} steps/s (commanded {SPEED}), "
              f"moved {deg(p1 - p0):+.2f}°")
    sp.close()
