---
title: Build, flash and update
description: Build the ATOM firmware with arduino-cli, flash it over USB once, then update it over WiFi. Public and lab builds.
---

To install the controller firmware without tools, use the
[Setup page](/mycobot-280-lab/start/setup/) in Chrome or Edge. This page is for
development builds.

## Public and lab builds

| | Public build (4.3+) | Lab build |
| --- | --- | --- |
| Made by | `tools/build-public-firmware.sh` (GitHub Actions for the site) | `arduino-cli compile` with `-I$HOME/.config/mycobot` (below) |
| WiFi | Improv over USB only (Setup page) | The network saved over Improv, else the one in `wifi_secrets.h` |
| OTA updates | No | Yes, with `OTA_PASSWORD` |
| Status log | `atom_controller v4.3.0 (<git>, public)` | `… (<git>, lab)` |

- The public build never includes `wifi_secrets.h` (`-DPUBLIC_BUILD`). The script
  also stops if the binary contains a value from that file.
- If you install the public build on the lab ATOM, OTA stops working. Flash a lab
  build over USB (below) to get OTA back. The saved WiFi network stays.
- The script writes `website/public/installer/` (not committed). Run it before you
  test the Setup page on a dev server: `tools/build-public-firmware.sh`
  (on the Pi: `ARDUINO_CLI=~/bin/arduino-cli`). The installer needs `localhost` or https.

## Tools

- `arduino-cli` (Homebrew) with the ESP32 core 3.3.x (`esp32:esp32`).
- The libraries `Adafruit NeoPixel` (1.15.5) and `WebSockets` (Markus Sattler, 2.7.2).
- `espota.py` from the ESP32 core, for OTA updates.

## WiFi credentials (lab builds)

The credentials are **not** in the repository. Without this file, a lab build
waits for WiFi setup over Improv, like the public build. Make the file
`~/.config/mycobot/wifi_secrets.h` with mode 600:

```c
#pragma once
#define WIFI_SSID "..."
#define WIFI_PASSWORD "..."
#define OTA_PASSWORD "..."   // optional: without it, anyone on the network can flash the ATOM
```

## Build

```bash
FQBN="esp32:esp32:m5stack_atom:PartitionScheme=min_spiffs,UploadSpeed=115200"
GIT=$(git describe --always --dirty)     # shown in the status log
arduino-cli compile --fqbn "$FQBN" \
    --build-property "compiler.cpp.extra_flags=-I$HOME/.config/mycobot -DFW_GIT=\"$GIT\"" \
    --output-dir build/atom_controller firmware/atom_controller
```

- `min_spiffs` gives two 1.9 MB app slots. OTA needs two slots. The board's
  default (`huge_app`) has only one.
- The controller is about 1.0 MB.

## First flash (USB)

1. Turn off the arm (the 12 V supply). Do this before you connect the USB cable.
2. Connect the ATOM to the computer with a USB-C data cable. The ATOM can stay on
   the arm. If the upload fails, remove the ATOM from the arm (see the table below).
3. Flash:
   ```bash
   arduino-cli upload -p /dev/cu.usbserial-A952DE0075 --fqbn "$FQBN" firmware/atom_controller
   ```
4. Make sure that the output shows `Hash of data verified`.
5. Disconnect the USB cable. If you removed the ATOM, put it back on the arm.

Use 115 200 baud. Higher speeds corrupt data on the ATOM's USB chip.

| ATOM | USB | Result (checked on 2026-10-05) |
| --- | --- | --- |
| Out of the arm | Connected | Works. Only the ATOM gets power. Use it if the upload fails in the arm. |
| In the arm, 12 V off | Connected | Works (the usual way). The USB port also supplies the servos (4.4–5.4 V). |
| In the arm, USB connected first, then 12 V on | Stays connected | Works. |
| In the arm, 12 V on, then USB connected | No USB device | The computer does not see the ATOM. |

An upload writes the bootloader, the partition table, `boot_app0` and the
application. It does not erase NVS, so the WiFi network saved over Improv stays.

### Flash a built binary with esptool

To flash the files from `--output-dir` (for example a lab build after the public
build), write the same four parts:

```bash
OUT=build/atom_controller
BOOT_APP0=$(ls ~/Library/Arduino15/packages/esp32/hardware/esp32/*/tools/partitions/boot_app0.bin | tail -1)
esptool --chip esp32 --port /dev/cu.usbserial-A952DE0075 --baud 115200 write-flash \
    0x1000 $OUT/atom_controller.ino.bootloader.bin \
    0x8000 $OUT/atom_controller.ino.partitions.bin \
    0xe000 "$BOOT_APP0" \
    0x10000 $OUT/atom_controller.ino.bin
```

Do not write `atom_controller.ino.merged.bin` (4 MB): it also overwrites NVS and
removes the saved WiFi network.

## Update over WiFi (OTA)

The ATOM can stay in the arm with the 12 V supply on.

```bash
python3 ~/Library/Arduino15/packages/esp32/hardware/esp32/*/tools/espota.py \
    -i 192.168.1.107 -p 3232 --auth=<OTA_PASSWORD> \
    -f build/atom_controller/atom_controller.ino.bin
```

An update takes about 15 s. The LED matrix shows magenta. The ATOM restarts and
holds the pose. Check the version in the status log on UDP port 5005.

For a new version: update `FW_MAJOR`/`FW_MINOR`/`FW_PATCH` and the
[changelog](/mycobot-280-lab/firmware/changelog/), commit, build, flash, and tag the
commit `atom-controller-vX.Y.Z`.

## Read the status log

```bash
python3 -c "import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(('',5005)); print(s.recv(512).decode())"
```
