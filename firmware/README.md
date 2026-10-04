# ATOM firmware

Custom firmware for the ATOM on the myCobot 280 (for Arduino): an M5Stack ATOM Matrix, **ESP32-PICO-D4** (Wi-Fi + Bluetooth, 2 × 240 MHz, 4 MB flash, MAC e8:6b:ea:31:4c:f4).

The stock Elephant firmware (v7.2) is backed up, outside this repository, in `~/myCobot/firmware-backups/` together with its restore commands.

## Hardware facts (verified 2026-10-04)

| What | Where |
| --- | --- |
| Servo bus (Feetech STS, half-duplex, 1 Mbaud) | **G19 = RX, G22 = TX**. Verified with `atom_probe`: the ATOM sees all bus traffic (0 checksum errors in 382 packets), and its replies reach the base. It does **not** hear its own transmissions, so no echo filtering is needed. |
| 5×5 LED matrix (SK6812) | G27 |
| Button (under the matrix, active low) | G39 |
| IMU MPU6886 (I²C 0x68) | SDA G25, SCL G21 |
| IR LED | G12 |

- With the stock firmware, the ATOM switches servo torque on at power-up. **With custom firmware, nothing does, so the arm is limp at power-up** until the firmware writes goal = present position (which turns torque on).
- The ATOM's **USB only works when the robot is powered off** (or the ATOM is out of the arm). With the robot off and the ATOM in the arm, USB back-feeds the arm: the servos run on 4.4–5.4 V from the laptop's USB port. Don't leave it like that; flash with the ATOM out of the arm, then use OTA.
- The ATOM's USB chip reports as FTDI 0403:6001 (serial A952DE0075), like the robot's FT232 (B00033ZX). Tools that pick "the FTDI device" must select by serial.
- Its USB serial corrupted data above 115 200 baud when reading flash; upload at 115 200.

## Building and flashing

WiFi credentials live outside the repository in `~/.config/mycobot/wifi_secrets.h` (mode 600):

```c
#pragma once
#define WIFI_SSID "..."
#define WIFI_PASSWORD "..."
#define OTA_PASSWORD "..."   // optional
```

```bash
FQBN="esp32:esp32:m5stack_atom:PartitionScheme=min_spiffs,UploadSpeed=115200"
arduino-cli compile --fqbn "$FQBN" --build-property "compiler.cpp.extra_flags=-I$HOME/.config/mycobot" \
    --output-dir build/atom_probe firmware/atom_probe

# First time, over USB (ATOM out of the arm):
arduino-cli upload -p /dev/cu.usbserial-A952DE0075 --fqbn "$FQBN" firmware/atom_probe

# After that, over WiFi (ATOM in the arm, robot on):
python3 ~/Library/Arduino15/packages/esp32/hardware/esp32/*/tools/espota.py \
    -i <atom-ip> -p 3232 --auth=<OTA_PASSWORD> -f build/atom_probe/atom_probe.ino.bin
```

`min_spiffs` gives two 1.9 MB app slots, which OTA needs (the board's default `huge_app` has no OTA).

## Sketches

- **`atom_probe/`**: passive bus probe. Counts every Feetech packet on G19, answers PING and READ (counters) as servo ID 7 on G22, never commands servos 1–6. Broadcasts a status line every second on UDP port 5005; LED rows show WiFi, bus bytes, valid packets, ID 7 replies and IMU; the button lights the matrix blue.
