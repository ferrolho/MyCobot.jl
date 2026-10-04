---
title: Safety
description: What protects the arm, what does not, and the power and USB conditions to avoid.
---

:::danger[There is no emergency stop]
The ATOM button is **not** an emergency stop: the end effector can move it out
of reach. To stop the arm, disconnect the 12 V supply. Add a real emergency stop
on the 12 V line before you try fast or unattended motion.
:::

## Software checks

The players (laptop and ATOM) do these checks:

| Check | Effect |
| --- | --- |
| The plan starts and ends at the zero pose | The player refuses other plans. |
| Each joint stays 10° inside its URDF limit | The player refuses the plan. |
| No joint goes faster than 90°/s | The player refuses the plan. |
| The arm is within 3° of the plan's first pose | The player refuses to start. |
| Each joint stays within 20° of the plan | If not, every joint holds where it is and the run stops. |
| Servo goal speed cap (2000 steps/s ≈ 176°/s) | Limits the speed of a wrong command. |
| Goal speed 0 when idle | Servos accept new goals but do not move. |

## Power-up behaviour

- **With the controller firmware on the ATOM**, the ATOM holds the pose at
  power-up: torque on, goal = present position, goal speed 0. The arm does not move.
- **Without the ATOM, or with firmware that does not do this**, the servos start
  with torque off. The arm is limp. Park or support the arm before you connect power.
- A power cycle resets the servo goal positions and goal speeds to 0. The arm
  can sag while the power is off.

## USB conditions to avoid

:::caution[USB back-feed]
Do not connect the ATOM's USB cable while the ATOM is in the arm and the 12 V
supply is off. The laptop's USB port then supplies the arm through the ATOM: the
servos run at 4.4–5.4 V and hold torque. Flash the ATOM out of the arm, then use
OTA updates.
:::

- While the 12 V supply is on, the ATOM's USB does not connect at all.
- The FT232R and the ATOM's USB chip both report the USB ID `0403:6001`. Tools
  that change the FT232R must select it by its serial number (`B00033ZX`).

## Who drives the bus

Only one device can drive the servo bus at a time. Do not run a laptop loop
while the ATOM plays a plan. The ATOM is silent on the bus when it is idle.
