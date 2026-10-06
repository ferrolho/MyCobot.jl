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
| Repository | `~/myCobot/mycobot-280-lab` (working copy) and `~/git/mycobot-280-lab.git` (hub) |
| Julia | 1.11 with juliaup: `~/.juliaup/bin/julia` (only login shells have it on `PATH`) |
| Kernel | `kernel8.img`, **4 KB memory pages** (since 2026-10-04) |
| Tools | Python `~/venvs/mycobot`, `~/bin/arduino-cli` (ESP32 core 3.3.10), Node `~/.local/opt/node` |

## Kernel: 4 KB pages

The Pi 5's default kernel (`kernel_2712.img`) uses 16 KB memory pages. Some prebuilt
programs assume 4 KB pages and crash: Pagefind, the docs search indexer, failed with
"memory allocation of 16 bytes failed". `/boot/firmware/config.txt` now selects the
4 KB-page kernel:

```text
[all]
kernel=kernel8.img
```

The original file is `/boot/firmware/config.txt.bak-20261004`. To undo it, remove the
line and reboot. Check with `getconf PAGESIZE` (4096 now).

## Repository sync

The Pi has a bare hub repository. The laptop has it as remote `pi`, and the Pi's
working copy clones it. Nothing goes to GitHub this way.

```text
laptop  ──push/pull──▶  raspberrypi5:git/mycobot-280-lab.git  ◀──push/pull──  Pi working copy
```

Use the script on the laptop. It also copies the Git LFS files (meshes, images),
because an SSH remote has no LFS server:

```bash
tools/sync-pi.sh push        # laptop → hub (current branch)
tools/sync-pi.sh pull        # hub → laptop (fetch; then merge yourself)
```

On the Pi, use plain `git pull` and `git push` in `~/myCobot/mycobot-280-lab`. LFS works
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

`tools/pi/camera.sh` uses **1280×960 MJPEG at 30 fps** from the Logitech C505. This
is the full 4:3 field of view of the sensor; 1280×720 crops the top and bottom. Each
time it starts the camera, it sets the image settings below. The camera forgets them
when it loses power.

### Image settings

With the default settings the picture is flat: the camera's auto exposure makes the
white wall mid-grey, and the black base is grey. The settings below stretch the tones.
Auto exposure and auto white balance stay on, so the picture adapts when the light
changes.

| Control | Default | Set to | Why |
| --- | --- | --- | --- |
| `power_line_frequency` | 60 Hz | 50 Hz | No flicker from the mains lights |
| `exposure_dynamic_framerate` | 0 | 0 | Always 30 fps. With 1, the camera can lower the frame rate in dim light. |
| `contrast` | 32 | 64 | Black is black, white is white; more detail on the arm |
| `brightness` | 128 | 144 | Lifts the picture after the contrast stretch. It is an offset, not an exposure bias. |
| `saturation` | 32 | 40 | Natural colours (cables, boards) |

Measured on 2026-10-06 in daylight. Each line is the last frame of 4 s of stream.
Brightness (luma, 0–255) and detail on the arm (the variance of the Laplacian):

| Settings | Darkest 1% | Median | Brightest 1% | Clipped | Detail on the arm |
| --- | --- | --- | --- | --- | --- |
| Defaults | 18 | 130 | 155 | 0% | 32 |
| `brightness` 192 only | 87 | 204 | 231 | 0% | 36 |
| `contrast` 64 only | 2 | 156 | 211 | 0% | 101 |
| `contrast` 64, `brightness` 156 | 2 | 206 | 255 | 3.3% | 123 |
| **Set: `contrast` 64, `brightness` 144, `saturation` 40** | **2** | **183** | **236** | **0%** | **116** |

Settings that do not help:

- `gain`: auto exposure sets it. A value that you write has no effect.
- `backlight_compensation` 1: the picture is flatter (darkest 1%: 40).
- `sharpness` 40: detail 127 instead of 116. This is in the spread between runs (121 and 116 for
  two runs that differ only in saturation), so it stays at the default 24.
- `brightness` alone: it lifts the black too (darkest 1%: 87). This is the "milky" look.

The camera gave 29.8 fps with every setting in daylight. Not measured yet: dim light
(evening). A white arm in front of a white wall has little contrast at any setting:
a darker background or light from the side would help more than camera settings.

Watch it in a browser: the [lab service](/mycobot-280-lab/software/control-page/#the-lab-service-raspberry-pi)
streams it at `http://raspberrypi5:8280/camera.mjpg` (Tailscale only), and the Control
page shows it. While the service streams, it owns the camera: other programs get
"device busy".

Or watch it from the laptop over SSH (no open port; needs `ffplay` from
`brew install ffmpeg`, and the lab service must not be streaming):

```bash
ssh raspberrypi5 '~/myCobot/mycobot-280-lab/tools/pi/camera.sh stdout' | ffplay -loglevel error -fflags nobuffer -f mjpeg -i -
```

Take one picture on the Pi: `tools/pi/camera.sh snapshot /tmp/arm.jpg`. It asks the lab
service first (`/snapshot.jpg`) and opens the camera itself only if the service does
not answer.
