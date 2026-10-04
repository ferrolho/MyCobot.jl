---
title: Latency and loop rate
description: From the 20 ms get_angles of issue #53 to a 1.26 ms bus read and a 500 Hz loop.
---

## The starting point

In April 2025, [elephantrobotics/myCobot#53](https://github.com/elephantrobotics/myCobot/issues/53)
reported that `get_angles()` takes at least 20 ms, so a read-only loop is limited
to about 50 Hz. ATOM firmware v6.5 gave 23.5 ms, v7.2 gave 20.6 ms. The issue
has no replies.

## Cause 1: the FT232R latency timer

| Command (100 samples each) | Latency timer 16 ms | Latency timer 1 ms |
| --- | --- | --- |
| Stock `get_angles` | 20.5 ms | 8.3–9.0 ms |
| Stock `is_power_on` | 16.0 ms | 2.0 ms |
| One servo register through the stock ATOM | 16.0 ms | 2.8 ms |

The flat 16.0 ± 0.1 ms for every short reply showed the cause. See
[Laptop link](/mycobot-280-lab/comms/laptop-link/).

## Cause 2: the stock firmware's extra work

The stock `get_angles` pings servo 7 (no reply without a gripper) and then does
one SYNC READ. The remaining ~6 ms after the latency fix is that wait and the
conversion.

## Direct bus access

| Transaction (6 servos) | Laptop (FT232R, 1 ms timer) | ATOM (native UART) |
| --- | --- | --- |
| SYNC READ of positions (2 bytes each) | 1.76–2.01 ms | — |
| SYNC READ of state (6 bytes each) | ~2.0 ms | **1.26 ms** (max 1.28, 0 failures in 2000) |
| SYNC READ of full state (15 bytes each) | 2.75–3.0 ms | — |
| SYNC WRITE + SYNC READ cycle | 3.2–3.3 ms with a 1 ms gap | **1.54 ms** with no gap |

## The rate sweep and the lost writes

The first Julia player runs at full rate (308–318 Hz) tracked worse than the
Python player (7–8 mm against 5 mm RMS). A sweep of capped rates was not monotonic:

| Julia rate | Circle error (RMS) |
| --- | --- |
| 150 Hz | 5.1 mm |
| 200 Hz | 13.9 mm (stopped by the tracking check) |
| 240 Hz | 5.2 mm |
| 308–318 Hz | 7.3–8.0 mm |

Recordings (2026-10-04): `tools/python/recordings/20261004-0914*`, `-0916*`
and `-0917*` (`circle_lagcomp_*_jl.csv`).

The joints froze for 100–500 ms at random moments, while all reads succeeded. A
test with goal speed 0 (no motion) found the cause: on the laptop path, a
SYNC WRITE followed by another request within ~0.3 ms is lost.

| Gap after SYNC WRITE | Lost writes |
| --- | --- |
| 0 | 24 % |
| 0.3 ms | 0.17 % |
| ≥ 0.5 ms | 0 |
| drain (Python `flush()`) | 0 |

With a 1 ms gap, the Julia player gives 5.0 mm RMS at 284 Hz. The gap costs only
about 0.2 ms per cycle, because it overlaps the USB frame timing.

## Onboard loop

On the ATOM, the player runs at a fixed 500 Hz: period 1.985–2.016 ms, 0 late
cycles in every run so far. See [Onboard control](/mycobot-280-lab/results/onboard/).
