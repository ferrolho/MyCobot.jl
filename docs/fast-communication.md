# Fast communication with the myCobot 280 (for Arduino)

Findings from 2026-10-03, measured on this robot: ATOM firmware v7.2, FT232R USB-serial adapter wired to the base board, macOS.

## Summary

- The **~20 ms `get_angles()`** reported in [elephantrobotics/myCobot#53][issue-53] was mostly **not** the ATOM firmware. About 16 ms of it came from the **FT232R's USB latency timer**. Setting the timer to 1 ms brings `get_angles()` down to **~8.5 ms**.
- The **Feetech servo bus is reachable from the laptop** through the existing serial port. The base board passes the laptop's bytes onto the bus, so the laptop can send Feetech packets and the servos answer.
- Reading the full state of all six servos (position, speed, load, voltage, temperature) in **one sync read takes ~2.75–3 ms (~330–360 Hz)**. Positions only take **~1.8–2 ms (~500–570 Hz)**. No hardware changes are needed.
- The servos are **Feetech STS** (standard STS register map, 1 Mbaud, IDs 1–6). Their operating mode can be changed. **Velocity mode was tested on J1** and works.

| Read                                                  | Default (16 ms latency timer) | Latency timer 1 ms |
| ----------------------------------------------------- | ----------------------------: | -----------------: |
| ATOM `get_angles` (0x20)                              |                      20.5 ms  |   8.3–9.0 ms (~115 Hz) |
| ATOM `IS_POWER_ON` (0x12)                             |                      16.0 ms  |     2.0 ms |
| ATOM servo register read (0x53)                       |                      16.0 ms  |     2.8 ms |
| Direct Feetech read, 1 servo position                 |                            –  | 1.3–1.6 ms (~600–790 Hz) |
| Direct Feetech sync read, 6 positions                 |                            –  | 1.8–2.0 ms (~500–570 Hz) |
| Direct Feetech sync read, 6 × 15-byte state           |                            –  | 2.75–3.0 ms (~330–360 Hz) |

Ranges are the means from separate runs of 100–300 samples each. No reads failed.

## System architecture (as observed)

```
 Laptop ──USB── FT232R ──UART 1 Mbaud── Base board (Arduino Mega, "transponder" firmware)
                                              │
                                              │  passes bytes through in both directions
                                              ▼
             ┌──────────── Feetech servo bus (half-duplex, 1 Mbaud) ────────────┐
             │                                                                  │
        ATOM Matrix (ESP32)    J1   J2   J3   J4   J5   J6   (ID 7 = gripper, if fitted)
```

What was observed:
- After an ATOM command, the laptop receives the ATOM's Feetech packets to the servos, the servos' replies, and then the ATOM's `FE FE` reply.
- Feetech packets sent by the laptop reach the servos, and the servos' replies come back to the laptop.
- The laptop's own packets are **not echoed back**. So the laptop isn't wired straight onto the half-duplex bus; something (presumably the base board) sits in between and passes bytes through.
- With no commands being sent, the bus is idle. The ATOM doesn't poll the servos in the background.

Not yet verified: exactly how the base board connects to the bus, and whether it forwards bytes one at a time or whole packets. A multimeter or logic analyzer would settle this, but it isn't needed to use the bus.

> The Nov 2025 plan in `pymycobot/notebooks/find_servo_bus.md` (probe base pins 0/1 to find the servo bus) is superseded. The bus is already reachable on the existing connection.

## Finding 1: the FT232R latency timer

FTDI chips hold received bytes until either 62 bytes are buffered or the **latency timer** expires (default **16 ms**). Every reply from the robot is short, so every reply waited ~16 ms. Short ATOM replies measured as a tight **16.0 ± 0.1 ms** whatever the command. That flat time is the giveaway.

The adapter is an FT232R (VID `0403`, PID `6001`, serial `B00033ZX`). It uses macOS's built-in **AppleUSBFTDI** DriverKit driver, which has no latency setting. But the chip accepts the FTDI vendor control request on endpoint 0, which works while the serial port is open:

```python
dev.ctrl_transfer(0x40, 0x09, 1, 1, None)   # SIO_SET_LATENCY_TIMER = 1 ms, interface A
dev.ctrl_transfer(0xC0, 0x0A, 0, 1, 1)      # SIO_GET_LATENCY_TIMER -> [1]
```

Use [`tools/python/ftdi_latency.py`](../tools/python/ftdi_latency.py) for this. The setting survives closing and reopening the serial port. It **resets to 16 ms when the adapter is unplugged**, so set it every time you connect.

The same issue affects any FTDI adapter on any OS. On Linux, `setserial /dev/ttyUSB0 low_latency` or `/sys/bus/usb-serial/devices/ttyUSB0/latency_timer` does the same job.

## Finding 2: direct access to the servo bus

### What the ATOM does for `get_angles`

[`tools/python/sniff_atom_command.py`](../tools/python/sniff_atom_command.py) captures this:

```
FEETECH  ff ff 07 02 01 f5                                 PING ID 7 (gripper) – no reply here
FEETECH  ff ff fe 0a 82 38 0f 01 02 03 04 05 06 19          SYNC READ 15 bytes from reg 56, IDs 1–6
FEETECH  ff ff 01 11 00 80 08 00 00 00 00 4c 19 ... 78     reply from ID 1
  ... one reply each from IDs 2–6 ...
ATOM     fe fe 0e 20 <6 × int16 BE angle·100> fa            ATOM's reply to the laptop
```

So the ATOM already reads all servos efficiently in one request. Its extra ~6 ms on top of the bus time is probably the wait for the missing gripper (ID 7) plus processing.

### Talking to the servos from the laptop

Feetech STS packets go out on the same serial port, at the same 1 Mbaud:

```
Request:  FF FF <ID> <LEN = n_params + 2> <INSTR> <params...> <CHK>
Reply:    FF FF <ID> <LEN> <ERROR> <data...> <CHK>
CHK = ~(ID + LEN + INSTR/ERROR + params/data) & 0xFF
```

Instructions: `0x01` PING, `0x02` READ, `0x03` WRITE, `0x82` SYNC READ, `0x83` SYNC WRITE. Multi-byte values are **little-endian**.

Examples that were verified:
```
ff ff 01 04 02 38 02 be                  read 2 bytes at reg 56 (position) from ID 1
ff ff 01 04 00 80 08 72                  reply: position 0x0880 = 2176
ff ff fe 0a 82 38 0f 01 02 03 04 05 06 19  sync read 15 bytes from reg 56, IDs 1–6 -> 6 × 21-byte replies
```

The ATOM only reacts to `FE FE` frames and ignores Feetech packets, and the servos ignore `FE FE` frames. The two protocols can share the line, as long as **only one side talks at a time**. Don't send ATOM commands while running a direct-bus control loop, or the packets will collide on the bus.

Direct **writes** (WRITE / SYNC WRITE from the laptop) have **not been tested yet**. All register writes so far went through the ATOM's `0x52` command.

## What is the ATOM for, then?

It's the convenience layer that makes the arm usable from myStudio, myBlockly, pymycobot and ROS without knowing anything about servos:

- **Protocol and units:** turns `FE FE` commands into Feetech traffic and converts raw steps to degrees.
- **Kinematics:** `get_coords` (0x23) does the same sync read, then returns a Cartesian pose. So forward kinematics (and presumably IK for `send_coords`) runs on the ATOM.
- **Motion commands:** `send_angles` to the current pose made the ATOM read the state twice and then write nothing. So it checks the current state before moving. How it shapes an actual move (one goal write vs. interpolated stream) **hasn't been captured yet**.
- **Gripper (ID 7)** on the same bus, plus the **IO pins at the end effector** for tools and pumps.
- **The 5×5 LED matrix:** `set_color` produced no bus traffic, so it's local to the ATOM.
- **Power, free mode, calibration** (calibration writes the offset registers) and joint limits.

None of that is needed for high-rate control from a laptop. While idle the ATOM stays off the bus, so it can stay fitted and still be used for setup (power on, calibration, LED) between control sessions.

## Servo modes

Register 33 sets the mode. STS servos support:

| Mode | Command register | Notes |
| ---: | --- | --- |
| 0 position (default) | goal position (42–43) | Servo's own PID (registers 21–23). |
| 1 velocity | goal speed (46–47), steps/s, bit 15 = direction | **Tested on J1, works.** No joint limits, so the controller must enforce them. |
| 2 PWM / open loop | goal time / PWM (44–45) | Closest thing to torque control. **Not tested.** |
| 3 step | relative position | Not useful here. |

### J1 velocity-mode test (2026-10-03)

[`tools/python/j1_velocity_mode_test.py`](../tools/python/j1_velocity_mode_test.py) sets the mode and speed through the ATOM and logs J1 with direct reads (638 Hz):
- Commanded 100 steps/s (~8.8°/s), measured **103 steps/s** in steady state, and it stopped cleanly.
- **~0.25 s of dead time** before motion started. This may be the speed loop's integral term building up enough to overcome static friction at this low speed. It needs characterising at other speeds.
- Reported speed is coarsely quantised (steps of 50).
- Switching back to position mode caused no motion, once the goal position had been set to the current position first (see the gotchas).

## Gotchas (read before writing to servos)

1. **Stale goal position.** Goal position (42–43) read `0` on all servos after power-up, and keeps whatever was last written. A servo whose goal is stale jumps to it when it next drives in position mode, for example when switching back from velocity mode.
2. **Writing a goal position switches torque on.** With torque off (register 40 = 0), writing registers 42–43 set register 40 back to 1. Only write a goal that equals where the joint is.
3. **The goal is in raw units: reported position + offset.** Present position (56–57) in position mode has the calibration offset (31–32) subtracted. In velocity mode it's reported **raw**, without the offset. That makes it look like the joint jumps by the offset when you switch modes; it doesn't move. The goal register uses raw units. To hold position in mode 0, write `goal = present + offset`. This was verified on J1 (offset +146). Other joints have offsets with bit 11 set, which on STS servos means negative (sign-magnitude). Verify on each joint before relying on it.
4. **Don't parse ATOM replies by searching for the first `FE FE`.** Feetech packets share the stream, and their checksum byte can be `0xFE`, which once made a parser lock onto a fake header. Check every candidate header for length, footer and command (see `find_atom_frame` in `mycobot_bus.py`).
5. **Two-byte register reads through the ATOM (`0x53` with a mode byte) gave wrong values.** Read 1 byte at a time through the ATOM, or use direct Feetech reads.
6. **PWM mode has no safety net.** A servo keeps the last command. If the host stalls, the arm sags or spins. Use a watchdog and position limits, ideally on a microcontroller close to the bus.
7. **Register 55 (EEPROM lock) = 1.** As far as I know, on STS servos this means writes to the EEPROM area (including the mode, register 33) are not saved and revert at power-off. Not verified with a power cycle.
8. The latency timer **resets on unplug** (see Finding 1).

## Servo identification

See [servo-registers.md](servo-registers.md) for the full register dump. The model numbers are `0x0809` (J1–J3), `0x0709` (J4) and `0x0209` (J5–J6), all on firmware 3.9. The common STS3215 reports `0x0309` (777). These three numbers aren't in any public table found so far, so they may be custom versions made for Elephant Robotics. To identify them: check the label on the back of a servo, use Feetech's FD software (Windows, needs direct bus access), or ask Feetech.

## Open questions and next steps

- [ ] Direct **SYNC WRITE** of goal positions from the laptop. Then measure a full read + write control loop rate.
- [ ] Capture what the ATOM sends on the bus for a **real** `send_angles` move (one goal write, or interpolation?).
- [ ] Characterise each mode: delay, bandwidth, the ~0.25 s velocity-mode start-up lag, and position-mode tracking at different PID gains.
- [ ] Try PWM mode (mode 2) carefully on J1 (no gravity load), with a watchdog.
- [ ] Check the goal = present + offset rule on joints with negative offsets.
- [ ] Add a direct-bus layer to MyCobot.jl:
  - set the latency timer on connect,
  - `read_state` (sync read),
  - `write_goals` (sync write),
  - joint limits and a watchdog.
- [ ] Redo `scripts/example_sinewave.jl` at ~300 Hz.
- [ ] Post these findings on [issue #53][issue-53], which still has no replies.

## History

- **Feb 2025:** MyCobot.jl started. A from-scratch implementation of the ATOM protocol showed pymycobot wasn't the bottleneck.
- **Apr 2025:** opened [issue #53][issue-53], "Slow ATOM firmware — 50 Hz is the maximum rate for reading joint angles". ATOM v6.5 measured 23.5 ms and v7.2 measured 20.6 ms. No replies.
- **Apr–Nov 2025:** work on a byte-by-byte frame reader and retries (uncommitted WIP in `src/serial/`).
- **Nov 2025:**
  - Tried transponder mode; it's not supported on this firmware (`get_transponder_mode` returns -1).
  - Planned to find the servo bus on the base pins and wrote probing scripts (`pymycobot/notebooks/`). No results were recorded.
- **2026-10-03:**
  - Found the latency timer was the cause.
  - Identified the servos as Feetech STS.
  - Found direct bus access from the laptop.
  - Ran the J1 velocity-mode test.

[issue-53]: https://github.com/elephantrobotics/myCobot/issues/53
