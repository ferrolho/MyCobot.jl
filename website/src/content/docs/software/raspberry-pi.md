---
title: Raspberry Pi 5
description: The Raspberry Pi 5 next to the robot — FT232R link, camera, repository sync and setup.
---

A Raspberry Pi 5 sits next to the robot. The FT232R (servo bus) and a webcam are
connected to it, so the arm can be controlled and watched remotely.

| Item | Value |
| --- | --- |
| SSH | `ssh raspberrypi5` (Tailscale; on the home network also `raspberrypi5.local`) |
| System | Raspberry Pi 5, 16 GB, Debian 12 (bookworm), aarch64 |
| FT232R | `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B00033ZX-if00-port0` (`/dev/ttyUSB0`) |
| Webcam | Logitech C505, `/dev/video0` |
| Repository | `~/myCobot/MyCobot.jl` (working copy) and `~/git/MyCobot.jl.git` (hub) |
| Julia | 1.11 with juliaup: `~/.juliaup/bin/julia` (only login shells have it on `PATH`) |

## Repository sync

The Pi has a bare hub repository. The laptop has it as remote `pi`, and the Pi's
working copy clones it. Nothing goes to GitHub this way.

```text
laptop  ──push/pull──▶  raspberrypi5:git/MyCobot.jl.git  ◀──push/pull──  Pi working copy
```

Use the script on the laptop. It also copies the Git LFS files (meshes, images),
because an SSH remote has no LFS server:

```bash
tools/sync-pi.sh push        # laptop → hub (current branch)
tools/sync-pi.sh pull        # hub → laptop (fetch; then merge yourself)
```

On the Pi, use plain `git pull` and `git push` in `~/myCobot/MyCobot.jl`. LFS works
there, because the hub is a local path.

## FT232R latency timer

On Linux the `ftdi_sio` driver keeps the latency timer in sysfs. A udev rule sets it
to 1 ms each time the adapter is connected:

```text
/etc/udev/rules.d/99-ftdi-latency.rules
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

`MyCobot.set_latency_timer` reads sysfs on Linux, and `MyCobot.default_port()`
gives the correct port on macOS and Linux. The scripts use both.

## Camera

`tools/pi/camera.sh` uses **1280×960 MJPEG at 30 fps**. This is the full 4:3 field
of view of the sensor; 1280×720 crops the top and bottom. It sets the mains
frequency to 50 Hz against flicker.

Watch it from the laptop (over SSH, no open port; needs `ffplay` from
`brew install ffmpeg`):

```bash
ssh raspberrypi5 '~/myCobot/MyCobot.jl/tools/pi/camera.sh stdout' | ffplay -loglevel error -fflags nobuffer -f mjpeg -i -
```

Take one picture on the Pi: `tools/pi/camera.sh snapshot /tmp/arm.jpg`.
