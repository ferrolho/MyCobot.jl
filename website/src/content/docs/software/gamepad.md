---
title: Gamepad teleoperation
description: Move the gripper in Cartesian space with an Xbox controller (Bluetooth) from the Control page. The mappings, the frames, and the end-effector controller on the ATOM (firmware 5.1) with its limits.
---

The [Control page](/mycobot-280-lab/software/control-page/) can move the **TCP** (the tool
point: between the finger pads, or the flange without the gripper) with a gamepad. The sticks
set the speed of the TCP in Cartesian space (a **twist**), not the joint speeds. The page sends
the twist to the ATOM every 20 ms ([end-effector JOG](/mycobot-280-lab/comms/websocket-api/#end-effector-jog-51)),
and the ATOM turns it into joint goals at 500 Hz.

:::caution[Gamepad not tested on firmware 5.1 yet]
Firmware 5.1 is on the arm since 2026-10-10, and `twist_ws_test.py` passed on it (below). The
gamepad has not moved the arm with 5.1 yet. On 2026-10-09 an earlier version, which ran the same
controller in the browser and sent TRACK, moved the real arm with an Xbox controller. For the
first test with the gamepad, use the lowest speed step and keep the power switch in reach.
:::

## What you need

- The laptop with Chrome (the browser that shows the Control page).
- An Xbox controller, paired with **the laptop** over Bluetooth (or connected with a USB
  cable). Do not pair it with the Raspberry Pi: the page reads the gamepad in the browser.
  Any gamepad that Chrome gives the `standard` mapping also works.
- Controller firmware **5.1** or later. Update it on the [Setup](/mycobot-280-lab/setup/) page.
  With the gripper, A and B also move J7.

## Start

1. Pair the controller with the laptop (macOS: System Settings → Bluetooth; hold the
   pair button on the controller until the Xbox button flashes fast).
2. Open the Control page, connect to the robot and click **Take control**.
3. Press any button on the controller. Chrome shows a gamepad to a page only after a
   button press. The line under **Gamepad** shows the controller name.
4. If the arm is in the zero pose (straight up), hold **X** until the status line shows
   **at the ready pose**. See [Singular poses](#singular-poses).
5. Press **Menu** (≡), or tick **Gamepad** on the **Gamepad** tab. The page shows the
   **Gamepad** tab, and switches Live mode off if it is on.
6. Move the sticks. Press **Menu** again, or **Esc**, to stop.

The page locks the faders and the typed goals while the gamepad moves the arm. The faders
(on the **Joints** tab) and the see-through arm show the goals. While the gamepad moves the
arm, the **Gamepad** tab has an amber dot.

## The Gamepad tab

The joints card has two tabs: **Joints** (the faders) and **Gamepad**. The **Gamepad** tab has:

- the **Gamepad** switch, the mapping, and the name of the controller;
- **Frame** (Base or Tool), **Speed** (the four speed steps) and **Ahead**. The buttons on the
  gamepad (View, D-pad) change the same settings;
- the TCP position (mm, base frame), the joint speed limit (the **Speed** setting on the
  **Joints** tab), and the reason when the arm slows or stops;
- a diagram of the controller: each control has a line to its function in the selected
  mapping. A control and its label light up while you use it, also while the gamepad is off.
  Use it to try a mapping before you move the arm. A table with the same text is there for
  screen readers.

The controller drawing is `controller_xboxseries.svg` from Kenney's
[Input Prompts](https://kenney.nl/assets/input-prompts) (version 1.5, CC0). Microsoft publishes
no controller drawing for this use; Kenney's is a free, simplified drawing made for button
prompts.

## Mappings

Select the mapping on the **Gamepad** tab. The page remembers it. Values are in the base frame:
**x** ahead, **y** to the left, **z** up (see [Frames](#frames)).

### Twin stick (default)

Translation on the left stick and the triggers, rotation on the right stick and the bumpers.

| Control | Motion of the TCP |
| --- | --- |
| Left stick up / down | Ahead / back (x) |
| Left stick left / right | Left / right (y) |
| RT / LT (analog) | Up / down (z) |
| Right stick up / down | Tilt: with the tool down, the tip moves ahead / back (rotation about y) |
| Right stick left / right | Turn about the vertical axis through the TCP (rotation about z) |
| LB / RB | Tilt: with the tool down, the tip moves left / right (rotation about x) |

### MoveIt Servo

The mapping of the MoveIt Servo joystick example (`joystick_servo_example.cpp`), for users
who know it. MoveIt also jogs single joints with the D-pad and A, B, X, Y. This page does
not: those buttons have the functions in [All mappings](#all-mappings).

| Control | Motion of the TCP |
| --- | --- |
| Right stick left / right | Left / right (y) |
| Right stick up / down | Up / down (z) |
| RT / LT | Ahead / back (x) |
| Left stick left / right | Rotation about x |
| Left stick up / down | Rotation about y |
| RB / LB | Rotation about z |

### Drone (mode 2)

As a camera drone: height and heading on the left stick, the horizontal motion on the right.

| Control | Motion of the TCP |
| --- | --- |
| Left stick up / down | Up / down (z) |
| Left stick left / right | Turn about the vertical axis (rotation about z) |
| Right stick | Ahead, back, left, right (x, y) |
| RT / LT | Tilt: the tip moves ahead / back |
| LB / RB | Tilt: the tip moves left / right |

### All mappings

| Control | Function |
| --- | --- |
| **Menu** (≡) | Start or stop. |
| **View** (⧉) | Base frame or tool frame. |
| **A** (hold) | Close the gripper (30 °/s on J7). |
| **B** (hold) | Open the gripper. |
| **X** (hold) | Go to the ready pose, in joint space (TRACK), at half the **Speed** setting. |
| **Y** (hold) | Turn the tool to point straight down, the shortest way. |
| **D-pad up / down** | Speed step up / down. |

Speed steps (TCP): 10 mm/s and 10 °/s, **25 mm/s and 20 °/s** (the start step), 50 mm/s and
40 °/s, 100 mm/s and 60 °/s. The stick value sets a fraction of the step: a dead zone of
12 %, then 30 % linear and 70 % cubic, for fine control near the centre.

Other choices that are common, not implemented: a deadman button (the arm moves only
while LB is held, as in ROS `teleop_twist_joy`), and joint jogging on the D-pad (as in
MoveIt Servo). Tell us if you want one of them.

## Frames

- **Base frame** (the default): the frame of the URDF. Set **Ahead** to the base axis
  that points away from you (+x, +y, −x or −y). Then the left stick moves the TCP as you see
  it. The rotations are about the base axes, through the TCP.
- **Tool frame** (press **View**): the flange axes. z is the approach axis of the gripper:
  in the twin-stick mapping, RT moves the TCP along the fingers, into the object.

The TCP with the gripper is at (−0.6, 8.2, 100) mm in the flange frame: between the finger
pads, 15 mm inside the closed fingertips (115 mm out) and 3 mm beyond the open fingertips
(97 mm). Rotations turn the gripper about this point. Without the gripper, the TCP is the
flange.

## How it works

```text
gamepad ─▶ page: twist (mm/s, °/s) ─JOG frame 1 or 2, every 20 ms─▶ ATOM: twist.h at 500 Hz ─▶ joint goals ─▶ servos
```

The page (`website/src/control/gamepad.ts`) reads the gamepad, applies the mapping, the speed
step and **Ahead**, and sends the twist. In the tool frame it sends frame 2, and the ATOM turns
the twist with the tool. The page computes nothing else for the motion; it uses the robot model
(`kinematics.ts`) only for the TCP position on the tab and for **Y**.

The ATOM (`firmware/atom_controller/twist.h`) does these steps every 2 ms:

1. Limit the change of the twist: 300 mm/s² and 4 rad/s² (230 °/s²).
2. Calculate the error from the pose of the joint goals to a **target pose**. The target
   moves with the twist. The error removes the drift of the linear step.
3. Calculate the joint speeds with damped least squares (the Jacobian of the TCP). The damping
   starts near a singular pose (smallest singular value below 0.05, with the linear rows in
   units of 100 mm).
4. Scale **all** joint speeds by one factor, so that the TCP keeps its direction:
   - every joint stays below the joint speed cap (the **Speed** setting);
   - every joint stays inside 80 % of its acceleration limit (400 °/s² on J1–J3, 2000 °/s² on J4–J6);
   - every joint can stop 2.5° inside its limit with that acceleration.
5. Add the speeds to the joint goals, and write the goals to the servos.

The kinematic chain in the firmware comes from the URDF (`robot_params.h`, generated by
`tools/gen_robot.py`). The test `tools/firmware-tests/test_twist_check.cpp` checks it against
`src/kinematics.jl`: 3e-5 mm on five poses in single precision. Neither the ATOM nor the page
applies the calibration of this arm (zero offsets, the J5 encoder correction): they change the
TCP pose by a few millimetres, which does not matter when a person closes the loop.

On the laptop, one controller step takes 0.8 µs. On the ATOM it takes about 450 µs (`twist_us`
in the status log, 2026-10-10): the ESP32 has no hardware float division or square root, and
each one is a library call. The smallest singular value (Jacobi sweeps) was about 80 % of these
calls, so the controller computes it only every 4th step (8 ms). `twist_us` shows the longest
step, which is one with this computation.

The loop integrates the measured time since the last goals (0.5–6 ms), not a fixed 2 ms. With a
fixed 2 ms, the first test on the arm (2026-10-10, before the change) moved 11 % short: 35.7 mm
for 40 mm. A cycle that runs late now does not change the distance.

### Results on the arm (2026-10-10, `twist_ws_test.py`)

| Test | Wanted | Measured (goals, firmware FK) |
| --- | --- | --- |
| Base +x, 20 mm/s for 2 s | 40 mm | 40.15 mm; 0.22 mm off the line; 0.09° turn |
| Tool +z, 10 mm/s for 1 s | 10 mm | 10.25 mm |
| Turn about the vertical, 10 °/s for 1 s | 10°, TCP fixed | 10.11°; TCP 0.06 mm from the start |
| Back at the start | 0 | 0.25 mm, 0.13° |

TRACK during the JOG and back, the refused frame of the wrong length, and the deadman also
passed. These numbers come from the goals. The servos follow them about 0.11 s later.

### Singular poses

The arm is singular (it cannot move the TCP in some direction) at:

| Pose | Joint angle | What happens |
| --- | --- | --- |
| Arm straight | J3 = 0° | The TCP cannot move along the arm. |
| Wrist | J5 = ±90° | J4 and J6 are parallel: one rotation is lost. |
| Wrist over the base | The TCP on the J1 axis | J1 cannot move the TCP. |

The page treats them as follows:

- **J3 and J5:** when the joint is more than 10° from the singular angle, the angle ± 10° is
  a joint limit on its side. The arm stops before it is singular, and it can always move
  back. The tab shows the reason, and the gamepad rumbles.
- **If the joints cannot give the wanted twist** (more than 20 % off), the arm brakes and
  stays. It does not move in a direction that you did not ask for.
- **The zero pose is singular** (J3 = 0°). From it, the TCP can move ahead or back, but not
  down. Hold **X** to go to the ready pose first: J3 = −90°, the tool down, the TCP 165 mm
  ahead of J1 and about 95 mm above the base frame.

### Safety functions

| Event | Result |
| --- | --- |
| **Menu**, **Esc**, **Stop**, **Hold**, the **Gamepad** switch | The page sends HOLD: the TCP brakes along its path at once, then the ATOM holds. |
| The gamepad disconnects | The page stops and sends HOLD. |
| The page is hidden or loses focus, control is lost, an ATOM error, a refused JOG | The page stops and sends HOLD. |
| The page stops sending (frozen tab, WiFi drop) | The ATOM keeps the last twist for 200 ms, then brakes the TCP along its path and holds. |
| A WiFi stall shorter than 200 ms | The ATOM keeps the last twist. The path does not change: the ATOM integrates it. |
| A joint is blocked (a finger, an object) | The ATOM's following-error check stops the arm (20° plus a speed term). |

:::danger
The gamepad and **Esc** stop the robot through the software. They are not an emergency
stop. Keep the power switch or a real emergency stop in reach. See
[Safety](/mycobot-280-lab/start/safety/).
:::

## Tests (2026-10-09)

**The controller** (`tools/firmware-tests/test_twist_check.cpp`, on the laptop at 500 Hz):

| Test | Result |
| --- | --- |
| FK against `src/kinematics.jl`, 5 poses | 3.1e-5 mm, 1.8e-7 |
| +x at 30 mm/s for 2 s, tool down | 60.10 mm, 0.023 mm off the line, the tool turns 0.0006° |
| Turn at 0.5 rad/s for 2 s about the TCP | The TCP moves 0.001 mm; 57.3° |
| Tool frame +z at 30 mm/s for 1 s, tool down | −30.00 mm in z |
| Down until J2 reaches its limit; out to the reach limit | Inside the speed, acceleration and joint limits; J3 stops at −10.00° |
| A full reversal at 100 mm/s and 1 rad/s | Inside the limits |
| From the zero pose: ahead, then down | Ahead 49.2 mm in 2 s; down: blocked, the TCP stays (0.00 mm) |
| 10 min of random twists, base and tool frame | Never outside the limits |
| 5 min of random translations, often at the limits | The tool turns 4.5° at most (the anchor accepts 3° per limit stop) |
| A hand-over from TRACK with the arm moving, then a zero twist | It brakes to rest at 53 % of the acceleration limit at most |

**The protocol** (`tools/firmware-tests/twist_ws_test.py`, against `tools/atom_sim.py`, which runs
`twist.h` through `tools/twist_lib.cpp`): base +x 20 mm/s for 2 s moved the TCP 40.36 mm with the
tool turning 0.000°; TRACK switched the run to tracking and JOG back; the tool frame +z 10 mm/s for
1 s moved 9.75 mm along the approach axis; a turn of 10 °/s for 1 s gave 10.20° with the TCP
0.12 mm from its start; a frame-1 JOG of the wrong length was refused; the deadman stopped the run;
the arm came back to its start within 0.22 mm and 0.02°.

**The page** (Chrome, headless, a simulated Xbox controller, the simulated ATOM): the stick moved the
TCP 50 mm at 25 mm/s, LT 25 mm down, a turn of 2 s moved J6 by 39.5° with the TCP in place, **X**
reached the ready pose (the run switched to TRACK and back), **Y** turned the tool down, the tool
frame moved the TCP 25 mm along the fingers, **A** closed J7, and pushing out stopped at J3 = −10°
with the reason on the tab.

To run the tests:

```bash
c++ -std=c++17 -O2 -I firmware/atom_controller tools/firmware-tests/test_twist_check.cpp -o /tmp/t && /tmp/t
python3 tools/atom_sim.py &                                   # builds build/twist_lib.so on first use
python3 tools/firmware-tests/twist_ws_test.py                 # MOVES THE ROBOT with --atom mycobot.local
cd website && node scripts/test-kinematics.mjs                # the page's FK against the same poses
julia --project=. tools/firmware-tests/fk_reference.jl        # after a change of the URDF: new reference poses
```

## The hobby servos

The servos are position-controlled. The page cannot command joint speed or torque, so it
integrates the twist into joint **goals**. The lab's measurements limit what the twist
control can do:

- **Lag.** The servos follow a moving goal about 0.11 s late (J1–J3 up to ~0.12 s; see
  [TRACK](/mycobot-280-lab/comms/websocket-api/#track-live-mode) and the
  [servo dynamics](/mycobot-280-lab/results/servo-dynamics/)). The TCP follows the stick
  about 0.1 s late. When joints with different lags move together, the TCP leaves the
  straight line during a change of speed. The acceleration limit of the twist (300 mm/s²)
  keeps this small. Lag compensation per joint (as for plans) is a possible next step.
- **Resolution.** One servo step is 0.088°: about 0.3 mm at the TCP at 200 mm. Slow motion
  (10 mm/s) moves the goal by about one step per 20 ms period.
- **Sag and play.** J4 and J5 settle up to 1–2.4° from their goals under gravity. The controller
  starts from the goals that the ATOM holds (firmware 5.0), not from the measured angles, so the
  sag does not add up.
- **Acceleration limits.** The ATOM limits J1–J3 to 400 °/s². The controller stays at 80 % of
  it, so a hand-over to TRACK or JOG never needs more.
- **Following error.** A blocked joint stops the arm after about 0.5 s (20° plus a speed term).

## Possible next steps

- Test the gamepad on the arm with firmware 5.1: straight lines, turns about the TCP, the stops
  before J3 = 0° and J5 = ±90°, a WiFi stall.
- Lag compensation of the goals, as for plans.
- The calibration of this arm in the kinematics (the firmware would need it in NVS; see the roadmap).
- A deadman button on the gamepad, if the tests on the arm show that it is needed.
