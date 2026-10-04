---
title: Quick start
description: Plan the circle, play it from the laptop or on the ATOM, and examine the result.
---

These procedures play the 100 mm circle demo. Read [Safety](/mycobot-280-lab/start/safety/) first.

## Before you start

- Clone the repository and install the Julia dependencies:
  ```bash
  julia --project=. -e 'import Pkg; Pkg.instantiate()'
  ```
- For the plots, make a Python environment in `tools/python/` with
  `pip install -r requirements.txt matplotlib numpy`.
- Make sure that the space around the arm is clear.

## Plan the circle

1. Run the planner:
   ```bash
   julia --project=. scripts/plan_circle.jl
   ```
2. Make sure that the output shows `plan: 15.0 s` and no assertion errors. The
   plan goes to `tools/python/plans/circle.csv`.

The plan starts and ends at the zero pose (arm upright).

## Play the circle on the ATOM (WiFi)

The ATOM must run the [controller firmware](/mycobot-280-lab/firmware/controller/).

1. Connect the 12 V supply. The LED matrix shows blue, then green. Green means
   that the arm holds its pose.
2. Find the ATOM's address in its status log (UDP broadcast, port 5005), or use
   the last known address `192.168.1.107`.
3. Move the arm to the zero pose:
   ```julia
   import MyCobot
   link = MyCobot.AtomLink("192.168.1.107")
   MyCobot.atom_move_to(link, zeros(6); duration=6.0)
   close(link)
   ```
4. Play the circle:
   ```bash
   julia --project=. scripts/play_plan.jl tools/python/plans/circle.csv --atom=192.168.1.107
   ```
5. Make sure that the output shows `result = "done"` and `late_cycles = 0`.

## Play the circle from the laptop (FT232)

The ATOM must be idle (the controller firmware is idle when it does not play a plan).

1. Connect the FT232R to the base: GND, and the base pins 13 and 14. See
   [Robot and wiring](/mycobot-280-lab/system/robot/).
2. Connect the 12 V supply.
3. Play the circle. The script sets the FT232R latency timer to 1 ms:
   ```bash
   julia --project=. scripts/play_plan.jl tools/python/plans/circle.csv
   ```

To move the arm to zero from the laptop, use `MyCobot.move_to(sp, zeros(6))` on an
open serial port.

## Examine the result

1. Calculate the flange path from the recording:
   ```bash
   julia --project=. scripts/trace_recording.jl tools/python/recordings/<recording>.csv
   ```
2. Plot it:
   ```bash
   cd tools/python && python plot_circle.py recordings/<recording>_path.csv
   ```

## Improve the result with learning control

1. Make the next plan from the last plan and its recording:
   ```bash
   julia --project=. scripts/ilc_step.jl tools/python/plans/circle.csv tools/python/recordings/<recording>.csv
   ```
2. Play the new plan (`circle_ilc1.csv`) as above.
3. Do the two steps again with the new plan and its recording. Three iterations
   reduce the error from 5 mm to about 1 mm RMS.
