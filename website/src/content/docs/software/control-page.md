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

### At home and away from home

The page from the Pi asks the lab service for the ATOM's address (`/atom.json`) and
tries it for 1.5 s:

- **At home** the browser reaches the ATOM, and the page uses its address
  (`192.168.1.107`). This path does not share the Pi's link with the camera stream.
- **Away from home** the page uses `raspberrypi5:8280/atom/ws`, the
  [relay](#the-lab-service-raspberry-pi) in the lab service. It works on any network
  that has Tailscale, for example a phone hotspot. The browser cannot reach
  `192.168.1.107` or `mycobot.local` from there.

Both addresses are in the address list.

Round trip of a command (a GRIPPER that the ATOM refuses at once), laptop at home,
2026-10-06:

| Path | No camera stream | Camera stream before the fix (1280×960, 30 fps) | Preview stream (640×480, 30 fps) |
| --- | --- | --- | --- |
| Control page with its camera on | 18–53 ms | **0.8–2 s** | 12–61 ms (direct) |
| Relay, another client | 19–64 ms | 250–550 ms | 18–38 ms |
| Straight to the ATOM | 12–23 ms | 17–42 ms | 11–19 ms |

The full stream (3.2 MB/s) filled the buffers on the way from the Pi to the laptop,
and every message through the Pi waited behind it. The Live mode and the arm readings
were late by the same time. Now the page shows a small preview (0.64 MB/s), the stream
socket has a small send buffer (a slow viewer gets fewer frames, not old ones), and
the page connects straight to the ATOM at home. With the full stream and the small
send buffer, the relay round trip is 36–48 ms.

#### The camera is behind the 3D view

The 3D view follows the arm's state stream (50 Hz; the gripper opening 10 times a
second). The camera image is later. Measured in Chrome on the Control page, from a
GRIPPER command to the first visible change, 2026-10-06:

| Preview | 3D view data changes | Camera image changes |
| --- | --- | --- |
| 10 fps, boundary at the start of the next frame | 117–165 ms | 600–730 ms |
| 30 fps, boundary straight after each frame (now) | 140–214 ms | 482–502 ms |

A browser shows an MJPEG frame only when the next boundary comes, so the service now
sends it straight after each frame. The rest is the webcam itself (exposure, its MJPEG
encoder, USB; about 0.1–0.2 s for a USB webcam) and the larger change that the image
needs before it shows. Expect the camera about 0.3 s behind the 3D view.

The stream through the relay, measured on 2026-10-06: laptop on a phone hotspot with
Tailscale (round trip to the Pi 16–84 ms), watch only, 5 runs of 20 s for each path,
in turn:

| Path | Packets/s | Gap p50 | Gap p99 (median of 5) | Largest gap (median of 5) |
| --- | --- | --- | --- | --- |
| Relay (`raspberrypi5:8280/atom/ws`) | 50.0–50.1 | 19–20 ms | 43 ms (42–116) | 77 ms (67–159) |
| SSH tunnel to the ATOM, for comparison | 50.0–50.1 | 18–20 ms | 42 ms (40–141) | 91 ms (79–211) |

The two paths are equal. The worst runs (p99 over 100 ms) were the same minute for
both: the hotspot, not the path.

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

### Gripper (J7)

With firmware 4.6 or later, a **J7** strip appears next to J6 when the ATOM finds the
[gripper](/mycobot-280-lab/system/gripper/#control). It disappears when the gripper is
disconnected. The 3D view then shows the arm with the gripper.

J7 works as the other joints, with the opening in % (0 closed, 100 open):

- The fader and the typed value set the goal. The ghost shows it. **Move** sends the
  joints and J7 together. With **Live** on, J7 follows the goal at once.
- Hold **▼** to close and **▲** to open. When you let go, the gripper stops where it
  is. As the jog buttons, they are off in Live mode.
- **Use current pose** and switching Live on set the J7 goal to the measured opening.
- The strip shows the measured opening and the servo load (%).
- These controls need control, as the joints do.

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

`tools/pi/lab_service.py` serves the built site, the webcam and a relay to the ATOM on
the Pi's Tailscale address. It is the only program that opens `/dev/video0`.

| Path | Content |
| --- | --- |
| `/mycobot-280-lab/` | The site (`website/dist`). Build it first with `npm run build`. |
| `/camera.json` | The page shows the Pi camera if this path exists. |
| `/camera.mjpg` | MJPEG stream: a 640×480 preview at 30 fps (about 0.64 MB/s), or with `?full=1` 1280×960 at 30 fps (3.2 MB/s). It ends when the camera stops. If the camera gives no frame in 10 s (for example, another program uses it), the answer is 503 with the camera error. |
| `/snapshot.jpg` | One recent frame. From cold it takes about 3.6 s, because the first 10 frames are skipped while the exposure settles. |
| `/log.json` | The page sends its session log if this path exists. |
| `POST /log?session=ID` | Session log events (JSONL). The service appends them to `~/myCobot/lab-logs/ID.jsonl`. |
| `/atom.json` | The ATOM's address and the relay path. The page tries the ATOM's address first, then the relay. |
| `/atom/ws` | The relay: the ATOM's WebSocket (`ws://192.168.1.107/ws`, set with `--atom`), byte for byte. |
| `/lab/scene.json` | Objects near the robot (see [Lab scene](#lab-scene)), from `~/myCobot/lab-scene.json`. 404 if the file does not exist. |

The relay is for a browser away from home (see [At home and away from home](#at-home-and-away-from-home)).
Only the Pi talks to the ATOM, on the home network. Thus a slow link does not fill the
ATOM's small send buffers. Both connections send each packet at once (`TCP_NODELAY`).
Without it, the 50 Hz stream arrives in clumps, once per round trip. If a browser is
gone (for example, the hotspot drops), its motion stops after 0.2 s (the deadman) and
its control ends after 2 s (the firmware). The relay closes its connection after about
30 s, so that it does not keep one of the ATOM's WebSocket slots.

The service opens the camera only while a client streams, and closes it 10 s after
the last request. `camera.sh stdout FD` copies the full frames (for `/snapshot.jpg`)
and makes the preview at the same time (ffmpeg: about 45 % of one core on the Pi 5). `tools/pi/camera.sh snapshot` asks the service first and uses the
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

### Lab scene

The 3D view shows objects near the robot when the lab service has a scene file
(`~/myCobot/lab-scene.json` on the Pi). The page reads `/lab/scene.json` every 5 s, so a
script can move the objects while the page is open. The page on GitHub Pages gets 404
and shows only the arm.

```json
{
  "table_z": -30,
  "objects": [
    {"name": "tissue box", "shape": "box", "center": [-10, -128, -13.5], "size": [165, 130, 33], "yaw": -17, "color": "#2b2b2e"},
    {"name": "plush body", "shape": "ellipsoid", "center": [-172, -100, -8], "size": [70, 115, 44], "color": "#a67c45"}
  ]
}
```

- Base frame, millimetres, z up. `center` and `size` are the centre and the full
  size (box edges, or ellipsoid diameters). `yaw` turns the object about z, in degrees.
- `shape` is `box` or `ellipsoid`. `table_z` puts the grid at the table height (0 if absent).
- The file is data from the lab, not part of the repository. Other keys are ignored
  (for example `source`: how the values were measured).

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
page. `tools/web_meshes.py` converts them to compressed GLB files (2 MB in total, the
gripper 0.2 MB) in `website/public/robot/`. `tools/gen_robot.py` writes the URDF copies
there ([Robot description](/mycobot-280-lab/software/robot-description/)). Run
`web_meshes.py` again when a mesh changes:

```bash
~/venvs/control/bin/python tools/web_meshes.py               # needs trimesh, pycollada, pillow and Node
~/venvs/control/bin/python tools/web_meshes.py gripper_base  # only the named meshes
```

`ArmView.setGripper(true)` swaps the solid arm and the ghost to
`mycobot_280_arduino_gripper.urdf`. It loads that URDF at the first call and keeps both
models, so later swaps are immediate. See [Gripper](/mycobot-280-lab/system/gripper/#model-urdf).

The script removes extra spaces from the `xyz` and `rpy` number lists of the web URDF.
urdf-loader does not trim `<axis xyz="...">`: before 2026-10-05, the axes of J3 and J4
(`xyz=" 0 0 1"`) became NaN, and the links after them disappeared from the 3D view
when J3 or J4 moved from 0°. The page now refuses a URDF with an invalid joint axis
("The 3D model did not load").

The URDF uses the same joint angles as the ATOM (degrees, 0° = the zero pose), so the
page sets the joints directly.
