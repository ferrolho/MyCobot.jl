---
title: Laptop link (FT232)
description: The FT232R adapter, its latency timer, and the laptop-side servo loop.
---

The laptop reaches the servo bus through an **FT232R** USB-serial adapter
(USB ID `0403:6001`, serial `B00033ZX`) on the base pins 13/14/GND. On macOS
it is `/dev/tty.usbserial-B00033ZX`, with Apple's built-in AppleUSBFTDI driver.

## The latency timer

The FT232R keeps received bytes until it has 62 of them, or until its **latency
timer** expires. The default is **16 ms**. Every reply from the robot is short,
so every reply waited 16 ms. This was most of the "20 ms `get_angles`" problem.

| Command | Latency timer 16 ms | Latency timer 1 ms |
| --- | --- | --- |
| Stock `get_angles` | 20.5 ms | 8.3–9.0 ms |
| Stock `is_power_on` | 16.0 ms | 2.0 ms |
| Direct SYNC READ of 6 servos | — | ~2.0 ms |

Apple's driver has no setting for the timer. The chip accepts the FTDI vendor
request on endpoint 0 while the port is open:

```julia
import MyCobot
MyCobot.set_latency_timer(1)     # selects the FT232R by serial number
MyCobot.get_latency_timer()      # 1
```

or `python tools/python/ftdi_latency.py 1`.

:::caution
The timer **resets to 16 ms** when the adapter is unplugged or the robot is
power-cycled. Set it again each time you connect. The scripts do this.
:::

On Linux, write `1` to `/sys/bus/usb-serial/devices/ttyUSB0/latency_timer`.

## The laptop loop

`MyCobot.play_trajectory` (Julia) and `tools/python/play_trajectory.py` do this
in each cycle:

1. Interpolate the commanded joint angles at the current time.
2. SYNC WRITE the goal positions of all six servos.
3. Wait 1 ms (Julia) or drain the port (Python). See [rules for writes](/mycobot-280-lab/comms/servo-bus/#rules-for-writes).
4. SYNC READ position, speed and load of all six servos.
5. Stop and hold if a joint is more than 20° from the plan.

| Player | Loop rate | Notes |
| --- | --- | --- |
| Julia | ~285–300 Hz | Occasional 30–80 ms stalls. The loop allocates 7.3 KB per cycle, so the garbage collector runs. |
| Python | ~236–250 Hz | `flush()` drains the port, which costs ~1 ms per cycle. |

The Julia player can cap its rate (`rate=` keyword, `--rate=` in `play_plan.jl`).
