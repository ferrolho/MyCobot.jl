# Fast communication with the myCobot 280 (for Arduino)

Findings from 2026-10-03, measured on this robot: ATOM firmware v7.2, FT232R USB-serial adapter wired to the base board, macOS.

## Summary

- The **~20 ms `get_angles()`** reported in [elephantrobotics/myCobot#53][issue-53] was mostly **not** the ATOM firmware. About 16 ms of it came from the **FT232R's USB latency timer**. Setting the timer to 1 ms brings `get_angles()` down to **~8.5 ms**.
- The **Feetech servo bus is reachable from the laptop** through the existing serial port. The base board passes the laptop's bytes onto the bus, so the laptop can send Feetech packets and the servos answer.
- Reading the full state of all six servos (position, speed, load, voltage, temperature) in **one sync read takes ~2.75–3 ms (~330–360 Hz)**. Positions only take **~1.8–2 ms (~500–570 Hz)**. No hardware changes are needed.
- A **closed loop driven entirely from the laptop** ran at **300 Hz**: each cycle sent goals to all six servos in one packet and read their states in one request. J1 followed a ±5° sine with 1.4° RMS error and a **~130 ms lag**. The servos' own response is now the limit, not communication.
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

Use [`tools/python/ftdi_latency.py`](../tools/python/ftdi_latency.py) for this. The setting survives closing and reopening the serial port. It **resets to 16 ms when the adapter is unplugged or the robot is power-cycled**, so set it every time you connect.

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

Direct **writes** work too:
- **WRITE** (`0x03`) gets a status reply with error byte 0.
- **SYNC WRITE** (`0x83`) gets no reply, and the servos act on it.
- With these, the ATOM isn't needed for anything, including mode changes.

### Closed loop from the laptop (2026-10-03)

[`tools/python/sync_write_sine_test.py`](../tools/python/sync_write_sine_test.py) runs one loop per cycle: a SYNC WRITE of goal positions for all six servos, then a SYNC READ of 6 bytes (position, speed, load) from all six. J1 followed a ±5°, 0.5 Hz sine for 4 s while J2–J6 held their positions:

| | |
| --- | --- |
| Loop period | **3.34 ± 0.50 ms (300 Hz)**, min 2.83, max 8.03 |
| Failed reads | 0 of 1,199 cycles |
| J1 tracking error | 1.4° RMS, 2.1° max |
| J1 lag behind target | **~130 ms** (best-fit delay) |
| Other joints | held still |

The ~130 ms lag and the flat spots at each reversal come from the servo itself (its position loop, the 3-step dead zone, friction), not from communication. For MPC/RL, model this delay or tune the servo's PID gains (registers 21–23).

## What is the ATOM for, then?

It's the convenience layer that makes the arm usable from myStudio, myBlockly, pymycobot and ROS without knowing anything about servos:

- **Protocol and units:** turns `FE FE` commands into Feetech traffic and converts raw steps to degrees.
- **Kinematics:** `get_coords` (0x23) does the same sync read, then returns a Cartesian pose. So forward kinematics (and presumably IK for `send_coords`) runs on the ATOM.
- **Motion commands:** `send_angles` / `send_angle` made the ATOM read the state twice and then write nothing. This happened both when targeting the current pose and for a +3° move of J6, which never moved. So after our experiments the ATOM was refusing to move, for reasons not yet known. How it shapes a real move **hasn't been captured yet**.
- **It can freeze:** after a series of status queries, starting at `GET_ROBOT_STATUS` (0x19), which this firmware doesn't seem to support, the ATOM stopped answering every command. The servos were unaffected.
  - The ATOM's reset button can't be reached while it's mounted in the arm. Power-cycling the robot fixes it, but the servos lose power, so **support the arm first**.
  - After the reboot the ATOM was blue. Pressing it turned it green, the normal state: per the FAQ in `assets/`, "the robotic arm will self-lock and the Atom will light up in green after powering on". It then answered normally.
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

[`tools/python/j1_velocity_mode_test.py`](../tools/python/j1_velocity_mode_test.py) was first run with mode and speed set through the ATOM, then re-run with everything direct:
- Commanded 100 steps/s (~8.8°/s), measured **102–103 steps/s** in steady state, and it stopped cleanly. J1 was logged at ~650 Hz with direct reads.
- **~0.25 s of dead time** before motion started, in both runs. This may be the speed loop's integral term building up enough to overcome static friction at this low speed. It needs characterising at other speeds.
- Reported speed is coarsely quantised (steps of 50).
- Returning to position mode without a jump: keep goal speed at 0, switch the mode (torque turns off), then write goal = present (torque turns back on and the joint holds). See the gotchas.

## Gotchas (read before writing to servos)

1. **Goal position uses the same units as present position** (position mode). To hold a joint, write `goal = present`. Don't add the calibration offset. An earlier version of this doc said the opposite; that was wrong. It looked right only because the servos weren't allowed to move at the time (see gotchas 2 and 4).
2. **Goal speed 0 means "don't move" in position mode.** Goal speed (46–47) reads 0 after power-up. With speed 0, a new goal is accepted but the servo stays put. Set a nonzero speed, which also works as a speed cap (e.g. 1000 steps/s). Setting the speed back to 0 afterwards makes stale goals harmless.
3. **Stale goal position.** Goal position (42–43) reads `0` after power-up and keeps whatever was last written. Before setting a nonzero goal speed or enabling torque, set every goal to the joint's present position.
4. **Writing a goal position switches torque on, and switching mode switches it off.** With torque off (register 40 = 0), writing registers 42–43 set it back to 1. Writing the mode (register 33) left torque at 0. A joint you've just switched modes on is limp until you write a goal.
5. **Velocity mode reports raw position.** In velocity mode, present position comes back **without** the calibration offset (31–32), so it looks like the joint jumps by the offset when you switch modes; it doesn't move. The offset is sign-magnitude: bit 11 set means negative. For example, J6 `0x0D4C` = −1356, and its velocity-mode reading was 1355 below its position-mode reading.
6. **Don't parse ATOM replies by searching for the first `FE FE`.** Feetech packets share the stream, and their checksum byte can be `0xFE`, which once made a parser lock onto a fake header. Check every candidate header for length, footer and command (see `find_atom_frame` in `mycobot_bus.py`).
7. **Two-byte register reads through the ATOM (`0x53` with a mode byte) gave wrong values.** Read 1 byte at a time through the ATOM, or use direct Feetech reads.
8. **PWM mode has no safety net.** A servo keeps the last command. If the host stalls, the arm sags or spins. Use a watchdog and position limits, ideally on a microcontroller close to the bus.
9. **Register 55 (EEPROM lock) = 1.** As far as I know, on STS servos this means writes to the EEPROM area (including the mode, register 33) are not saved and revert at power-off. Not verified with a power cycle.
10. The latency timer **resets on unplug and on robot power-cycle** (see Finding 1).
11. **A power cycle resets every servo's goal position and goal speed to 0** (mode back to 0, torque on after the ATOM is pressed to green). The arm can droop while the power is off; on the first reboot, J4–J6 changed by up to 50°.
12. **Unsupported ATOM status queries can freeze the ATOM** (see "What is the ATOM for"). Stick to the commands you know work.

## Servo identification

See [servo-registers.md](servo-registers.md) for the full register dump. The model numbers are `0x0809` (J1–J3), `0x0709` (J4) and `0x0209` (J5–J6), all on firmware 3.9. The common STS3215 reports `0x0309` (777). These three numbers aren't in any public table found so far, so they may be custom versions made for Elephant Robotics. To identify them: check the label on the back of a servo, use Feetech's FD software (Windows, needs direct bus access), or ask Feetech.

## Open questions and next steps

- [x] Direct **SYNC WRITE** of goal positions from the laptop, and a full read + write loop rate: **300 Hz**.
- [ ] Find out why the ATOM refused to move before it froze (it answers again after a reboot). Then capture what it sends on the bus for a **real** `send_angles` move (one goal write, or interpolation?).
- [ ] Measure the ~130 ms position-mode lag against PID gains and goal speed/acceleration settings.
- [ ] Characterise each mode: delay, bandwidth, the ~0.25 s velocity-mode start-up lag, and position-mode tracking at different PID gains.
- [ ] Try PWM mode (mode 2) carefully on J1 (no gravity load), with a watchdog.
- [ ] Add a direct-bus layer to MyCobot.jl:
  - set the latency timer on connect,
  - `read_state` (sync read),
  - `write_goals` (sync write),
  - joint limits and a watchdog.
- [ ] Post these findings on [issue #53][issue-53], which still has no replies.

## History

- **Feb 2025:** MyCobot.jl started. A from-scratch implementation of the ATOM protocol showed pymycobot wasn't the bottleneck.
- **Apr 2025:** opened [issue #53][issue-53], "Slow ATOM firmware — 50 Hz is the maximum rate for reading joint angles". ATOM v6.5 measured 23.5 ms and v7.2 measured 20.6 ms. No replies.
- **Apr–Nov 2025:** work on a byte-by-byte frame reader, retries and `scripts/example_sinewave.jl`. These were workarounds for the latency problem. They were never committed and were set aside in a git stash on 2026-10-03.
- **Nov 2025:**
  - Tried transponder mode; it's not supported on this firmware (`get_transponder_mode` returns -1).
  - Planned to find the servo bus on the base pins and wrote probing scripts (`pymycobot/notebooks/`). No results were recorded.
- **2026-10-03:**
  - Found the latency timer was the cause.
  - Identified the servos as Feetech STS.
  - Found direct bus access from the laptop.
  - Ran the J1 velocity-mode test.
  - Got direct WRITE / SYNC WRITE working.
  - Ran the 300 Hz closed loop from the laptop.
  - Corrected the goal-units rule (goal = present, not present + offset).

[issue-53]: https://github.com/elephantrobotics/myCobot/issues/53
