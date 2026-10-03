"""
Benchmark ATOM commands against direct Feetech servo-bus reads. Read-only.

Run `python ftdi_latency.py 1` first, otherwise every reply waits ~16 ms.
"""

import statistics
import time

from mycobot_bus import (BROADCAST_ID, FT_READ, FT_SYNC_READ, REG_PRESENT_POSITION, SERVO_IDS,
                         atom_xfer, ft_packet, ft_txrx, open_port, read_state)


def bench(label, fn, n=300):
    times, failures = [], 0
    for _ in range(n):
        t0 = time.perf_counter()
        ok = fn()
        dt = (time.perf_counter() - t0) * 1000
        if ok:
            times.append(dt)
        else:
            failures += 1
    mean = statistics.mean(times)
    print(f"{label:40s} {mean:6.2f} ± {statistics.stdev(times):.2f} ms  "
          f"(min {min(times):.2f}, max {max(times):.2f}, failures {failures}/{n}, ~{1000 / mean:.0f} Hz)")


if __name__ == "__main__":
    sp = open_port()

    for sid, s in read_state(sp).items():
        print(f"  ID{sid} pos {s['position']:5d}  speed {s['speed']:5d}  load {s['load']:5d}  "
              f"{s['voltage']:.1f} V  {s['temperature']} °C")
    print()

    one = ft_packet(1, FT_READ, [REG_PRESENT_POSITION, 2])
    pos6 = ft_packet(BROADCAST_ID, FT_SYNC_READ, [REG_PRESENT_POSITION, 2, *SERVO_IDS])
    full6 = ft_packet(BROADCAST_ID, FT_SYNC_READ, [REG_PRESENT_POSITION, 15, *SERVO_IDS])
    bench("ATOM get_angles (0x20)", lambda: atom_xfer(sp, 0x20) is not None, n=100)
    bench("ATOM IS_POWER_ON (0x12)", lambda: atom_xfer(sp, 0x12) is not None, n=100)
    bench("direct read, J1 position", lambda: len(ft_txrx(sp, one, 8)) == 8)
    bench("direct sync read, 6 positions", lambda: len(ft_txrx(sp, pos6, 6 * 8)) == 6 * 8)
    bench("direct sync read, 6 x 15-byte state", lambda: len(ft_txrx(sp, full6, 6 * 21)) == 6 * 21)
    sp.close()
