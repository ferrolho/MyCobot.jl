---
title: Control page
description: The browser page that watches and moves the arm — how to use it, the lab service on the Raspberry Pi, and how to develop it without the robot.
---

The [Control page](/mycobot-280-lab/control/) shows the state of the arm and moves it.
The browser connects straight to the ATOM over WiFi with the
[WebSocket API](/mycobot-280-lab/comms/websocket-api/). You need only the arm and a
browser.

The page needs controller firmware **4.2.0** or later on the ATOM. On 2026-10-05 the
page on GitHub Pages, opened in Chrome on a laptop, connected to `mycobot.local`,
controlled the real robot and showed the laptop's webcam. Without the robot, use the
[simulated ATOM](#develop-without-the-robot).

## Where to open it

| Place | URL | Camera |
| --- | --- | --- |
| GitHub Pages (any user) | `https://ferrolho.github.io/mycobot-280-lab/control/` | A camera on your own computer |
| The Raspberry Pi in the lab (Tailscale only) | `http://raspberrypi5:8280/mycobot-280-lab/control/` | The webcam next to the robot |

Add `?atom=<address>` to the URL to connect at once, for example
`?atom=192.168.1.107`.

## Use it

On a screen of 1440 × 900 pixels or more, the page fits in one screen:

| Area | Content |
| --- | --- |
| Top bar | Robot address and **Connect**; connection, real robot or simulator, robot state; the control lease; **Hold** and **Stop**. A green edge: connected to the real robot. |
| Camera (left) | The Pi camera or a camera on this computer (see [Camera](#camera)) |
| 3D view (middle) | The measured pose (solid) and the goal pose (see-through blue). **Reset view** restores the camera. |
| Joints (right) | One strip per joint: the angle, a vertical fader, the typed goal, jog buttons, the temperature and the voltage. Below: **Live**, the **Speed** setting, **Use current pose**, **Go to zero**, **Move**. |
| Plots | The last 20 s: angle, speed, load, temperature (one line per joint), and the IMU acceleration and angular rate (x, y, z) |

On a narrower screen, the areas are stacked and the page scrolls.

1. Type the address of the ATOM: an IP address (`192.168.1.107`) or `mycobot.local`.
2. Click **Connect**. If Chrome asks for access to devices on the local network,
   allow it. The page remembers the addresses that you used.
3. Watch the state. Each joint has a colour: the same colour marks the joint in its
   strip and in the plots. Each plot shows the values as text above it: the latest
   values, or the values under the pointer. One pointer line goes through all plots.
   A pause in the data shows as a gap.
4. Click **Take control** to move the robot (in the top bar, or in the banner above
   the joints). Only one client has control. The others can only watch: the joint
   controls are dimmed, and the banner shows **Watching only**. The angles and the
   measured lines stay up to date.
5. Move the robot:
   - **▼ / ▲** under a joint: press and hold to jog that joint at the **Speed** setting (at most 30 °/s).
   - **Fader**: the track runs from the lower limit (bottom) to the upper limit (top).
     The white (dark in the light theme) line is the measured angle; the coloured bar
     goes from 0° to it. Drag the blue marker to set the goal of that joint. The
     see-through blue arm in the 3D view shows the goal pose. Then click **Move**.
   - **Typed goal** (the field under the fader): type an angle in degrees (0.1°
     steps) and press **Enter**. The page keeps the goal 2° inside the joint limits.
     The field is blue while the goal differs from the angle.
   - **Use current pose**: set all goals to the measured pose.
   - **Go to zero**: move all joints to 0°.
   - **Speed** (5–90 °/s): the top speed of **Move**, **Go to zero** and Live mode.
     A move is smooth (minimum jerk): the joint that moves farthest reaches this
     speed in the middle of the move. If the acceleration limits do not allow it,
     the move takes the shortest time that they allow. The jog buttons use at most
     30 °/s.
6. Click **Release control** when you stop.

### Live mode

With **Live** on, the arm follows the goals at once: drag a fader, or type a goal and
press **Enter**. There is no **Move**. Live mode needs controller firmware **4.4** or
later.

- The page sends the goal pose to the ATOM (TRACK): at once when a goal changes, and
  every 50 ms. The ATOM moves each joint to its goal at up to the **Speed** setting
  (5–90 °/s, as fast as a Move) and the joint's acceleration limit, and brakes to stop
  exactly on it. See [TRACK](/mycobot-280-lab/comms/websocket-api/#track-live-mode).
- Live mode never sends MOVE_TO. **Move**, **Go to zero** and the jog buttons are off.
- When you switch Live on, the goals become the measured pose: the arm does not move
  to an old goal.
- Live switches off when you release control, when you lose control, when the
  connection closes, when you click **Stop** or **Hold** or press **Esc**, and when the
  page is hidden or loses focus. Then the page sends HOLD: the arm brakes at once.
- If the page stops sending (a frozen tab, a WiFi drop), the ATOM brakes and holds
  0.2 s after the last TRACK.
- The joints panel has an amber frame while Live is on.
- The jog buttons (▼/▲) use the **Speed** setting, at most 30 °/s.

:::danger
**Stop** and the **Esc** key stop the robot through the software. They are not an
emergency stop. Keep the power switch or a real emergency stop in reach. See
[Safety](/mycobot-280-lab/start/safety/).
:::

- The jog (and Live mode) stops **0.2 s** after you release the button, close the
  page, or lose the connection (the deadman in the firmware).
- The API has no authentication. Every device on the home network can take control.

## Camera

The camera panel shows one of two sources:

- **Pi camera**: when the Raspberry Pi serves the page. It starts by itself.
- **A camera on this computer**: on GitHub Pages. The camera must be connected to
  the computer that shows the page (for example the laptop), pointed at the robot.
  The browser asks for permission first. A web page cannot reach a camera that is
  connected to another computer.

## The lab service (Raspberry Pi)

`tools/pi/lab_service.py` serves the built site and the webcam on the Pi's Tailscale
address. It is the only program that opens `/dev/video0`.

| Path | Content |
| --- | --- |
| `/mycobot-280-lab/` | The site (`website/dist`). Build it first with `npm run build`. |
| `/camera.json` | The page shows the Pi camera if this path exists. |
| `/camera.mjpg` | MJPEG stream, 1280×960 at 30 fps. |
| `/snapshot.jpg` | One recent frame. From cold it takes about 3.6 s, because the first 10 frames are skipped while the exposure settles. |
| `/log.json` | The page sends its session log if this path exists. |
| `POST /log?session=ID` | Session log events (JSONL). The service appends them to `~/myCobot/lab-logs/ID.jsonl`. |

The service opens the camera only while a client streams, and closes it 10 s after
the last request. `tools/pi/camera.sh snapshot` asks the service first and uses the
camera directly only if the service does not answer.

It runs as a systemd user service from a separate checkout of main,
`~/myCobot/lab-services`, so that edits in the working copy do not change the served
site. It starts at boot (linger is on) and restarts if it stops.

| Task | Command (on the Pi) |
| --- | --- |
| Status, start, stop, restart | `systemctl --user status lab-service` (or `start`, `stop`, `restart`) |
| Log | `journalctl --user -u lab-service -f` |
| Update the site | `cd ~/myCobot/lab-services && git pull --ff-only && cd website && npm ci && npm run build` (the service serves `website/dist`; no restart needed) |
| After a change to `lab_service.py` | `systemctl --user restart lab-service` |
| Install the unit | see the comments in `tools/pi/lab-service.service` |

### Session log

When the lab service serves the Control page, the page records the session on the Pi,
in `~/myCobot/lab-logs/<start time>-<id>.jsonl` (one JSON object per line). Use it to
find out what happened without copying values from the screen. The page on GitHub
Pages has no lab service, so it records nothing.

| `type` | Content |
| --- | --- |
| `session`, `page` | The start (URL, browser, screen); the page hidden or shown |
| `click`, `input`, `key` | The user's clicks, input changes (with the value) and Enter/Esc |
| `tx`, `rx` | Commands sent (JOG only when its speeds change; not the lease renewals) and replies (ACK, DONE, PONG) |
| `link`, `atom` | The connection status; the ATOM's status lines |
| `state` | The stream at 10 Hz: ATOM time, state, control, angles, speeds, loads, temperatures |
| `gap` | Every pause in the stream over 100 ms, with the ATOM's own time step: a large `atomGapMs` means that the ATOM did not send; a normal one (20 ms) means that the network held the packets |
| `stats`, `error` | Once a second, over the last 5 s: packets per second, the largest gap and the arrival jitter (95th percentile minus the smallest arrival delay); page errors |

Each line has `t` (Unix time, ms) and `pt` (the page's clock, ms).

The stream, measured with the session log on 2026-10-05 (laptop on WiFi, real robot):

| Firmware | During MOVE_TO | Largest gap | CONTROL reply |
| --- | --- | --- | --- |
| 4.3.0 | 17–30 packets/s, 60 pauses of 100–525 ms in 7 moves | 525 ms | up to 0.7 s |
| 4.3.1 | 49.5–50.4 packets/s, 1 pause of 117 ms in 6 moves | 117 ms | 22 ms |

The pauses were the ATOM's own (its `t_ms` jumped as much as the arrival time): the
MOVE_TO telemetry over the WebSocket blocked its network task (see the
[changelog](/mycobot-280-lab/firmware/changelog/), 4.3.1). With 4.3.1 the page draws
the newest packet, and the motion is smooth.

Node is installed for the user in `~/.local/opt/node`. Put `~/.local/opt/node/bin` on
`PATH` before `npm`.

## Develop without the robot

Run these on the Pi, on its Tailscale address. Never use the home network address.

1. Start the simulated ATOM. It speaks the same WebSocket API:
   ```bash
   ~/venvs/control/bin/python tools/atom_sim.py --host 100.69.15.110 --port 8281 \
       --recording tools/python/recordings/<recording>.csv
   ```
2. Start the dev server. It reloads the page when a file changes:
   ```bash
   cd website
   npx astro dev --host 100.69.15.110 --port 4322 --allowed-hosts raspberrypi5
   ```
3. Open `http://raspberrypi5:4322/mycobot-280-lab/control/?atom=raspberrypi5:8281`.

The code is in `website/src/pages/control.astro` and `website/src/control/`:

| File | Content |
| --- | --- |
| `protocol.ts` | Message codes, encoders, decoders and units |
| `connection.ts` | The WebSocket: reconnect, SUBSCRIBE and CONTROL renewals, jog timer |
| `app.ts` | The page: joint strips and faders, the six plots (uPlot), controls |
| `viewer3d.ts` | The 3D view (three.js, urdf-loader) |
| `camera.ts` | The camera panel |

### 3D model

The URDF meshes (`mycobot_description/`, 25 MB of COLLADA) are too large for a web
page. `tools/web_meshes.py` converts them to compressed GLB files (2 MB in total) and
writes a copy of the URDF to `website/public/robot/`. Run it again when the URDF or
the meshes change:

```bash
~/venvs/control/bin/python tools/web_meshes.py              # needs trimesh, pycollada, pillow and Node
~/venvs/control/bin/python tools/web_meshes.py --urdf-only  # only the URDF, when only the URDF changed
```

The script removes extra spaces from the `xyz` and `rpy` number lists of the web URDF.
urdf-loader does not trim `<axis xyz="...">`: before 2026-10-05, the axes of J3 and J4
(`xyz=" 0 0 1"`) became NaN, and the links after them disappeared from the 3D view
when J3 or J4 moved from 0°. The page now refuses a URDF with an invalid joint axis
("The 3D model did not load").

The URDF uses the same joint angles as the ATOM (degrees, 0° = the zero pose), so the
page sets the joints directly.
