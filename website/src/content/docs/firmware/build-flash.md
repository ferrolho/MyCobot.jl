---
title: Build, flash and update
description: Build the ATOM firmware with arduino-cli, flash it over USB once, then update it over WiFi.
---

## Tools

- `arduino-cli` (Homebrew) with the ESP32 core 3.3.x (`esp32:esp32`).
- The library `Adafruit NeoPixel`.
- `espota.py` from the ESP32 core, for OTA updates.

## WiFi credentials

The credentials are **not** in the repository. Make the file
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

1. Remove the ATOM from the arm.
2. Connect the ATOM to the laptop with USB-C.
3. Flash:
   ```bash
   arduino-cli upload -p /dev/cu.usbserial-A952DE0075 --fqbn "$FQBN" firmware/atom_controller
   ```
4. Make sure that the output shows `Hash of data verified`.
5. Disconnect the USB cable and put the ATOM back in the arm.

Use 115 200 baud. Higher speeds corrupt data on the ATOM's USB chip.

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
