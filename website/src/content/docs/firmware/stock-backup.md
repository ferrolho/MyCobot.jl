---
title: Stock firmware backup
description: Where the backup of Elephant's ATOM firmware is, and how to restore it.
---

The full flash of the ATOM (4 MB) was read on 2026-10-04, before any custom
firmware was installed. The file is **outside the repository**, because it is
Elephant's proprietary firmware:

```text
~/myCobot/firmware-backups/atom-stock-elephant-v7.2-esp32picod4-e86bea314cf4-20261004.bin
SHA-256 8cfe61a43ca1aadfaf643638c1c8738cc2d1ab949356938ee86880060bf2e66d
```

The chip verified the file (`verify-flash`, MD5 digest matched). The stock
firmware was built with PlatformIO and Arduino-ESP32 2.0.x.

## Partitions

| Partition | Offset | Size |
| --- | --- | --- |
| nvs | 0x9000 | 20 KB |
| otadata | 0xE000 | 8 KB |
| app0 | 0x10000 | 1280 KB |
| app1 | 0x150000 | 1280 KB |
| spiffs | 0x290000 | 1472 KB |

## Restore

1. Remove the ATOM from the arm and connect it with USB-C.
2. Write the backup:
   ```bash
   esptool --port /dev/cu.usbserial-A952DE0075 write-flash 0 atom-stock-elephant-v7.2-esp32picod4-e86bea314cf4-20261004.bin
   ```
3. Verify it:
   ```bash
   esptool --port /dev/cu.usbserial-A952DE0075 verify-flash 0 atom-stock-elephant-v7.2-esp32picod4-e86bea314cf4-20261004.bin
   ```

## Official firmware from Elephant

If the backup is lost, install the official ATOM firmware with Elephant's
**myStudio**. Download it from
[github.com/elephantrobotics/myStudio/releases](https://github.com/elephantrobotics/myStudio/releases)
(v3.6.6 on 2026-07-20). myStudio downloads the firmware and flashes it over USB.

- Select myCobot 280 for Arduino, the ATOM board, and the ATOM main firmware
  ("AtomMain"). The stock firmware on this arm was v7.2.
- Remove the ATOM from the arm first. Over USB in the arm, the ATOM powers the
  servos when the 12 V supply is off, and does not connect when it is on.
- myStudio may also offer firmware for the base ("Transponder"). This project
  does not need it. Do not flash it unless you know that the base needs it.

### Where myStudio keeps the firmware

myStudio stores the firmware files inside the app:

```text
/Applications/myStudio.app/Contents/Resources/static/res/mycobot/
```

An app update can replace these files, so copies are in
`~/myCobot/firmware-backups/elephant-official/` (with `SHA256SUMS`), outside the
repository:

| File | What it is |
| --- | --- |
| `MyCobot280_atom_v7.3_beta_20251105.bin` | AtomMain v7.3 **beta** (2025-11-05). Downloaded with myStudio 3.6.6 on 2026-10-04. |
| `MyCobot280_atom_v6.5_20231228.bin`, `v6.4`, `v6.2`, `v5.11` | Older AtomMain versions, included with myStudio. |
| `CheckPID-2.0.ino.bin` | Elephant's PID check tool. It contains the presets "P = 10, I = 1, D = 0", "P = 5, I = 0, D = 15" and "P = 8, I = 0, D = 24". |
| `cobot_atom_test_servo.bin` | Elephant's servo test firmware. |

These are app images (they start with `0xE9`), not full-flash images. Do not write
them at offset 0 with esptool. Let myStudio flash them.

### AtomMain v7.3 (beta)

Elephant's release notes in myStudio:

1. The soft limit of J6 did not work. v7.3 adds it.
2. J5 and J6 of the 280 Pi locked first at power-up (bug 7.3.2). Fixed.
3. The J5 soft limit is now −155° to +160°, because the structure cannot reach ±165°.

Elephant's product page gives the joint limits for v7.3 (see
[joint limits](/mycobot-280-lab/system/robot/#joint-limits)). This arm does not
need v7.3: the custom controller firmware does not use the stock limits.

:::caution[Keep a second copy]
The backup file is only on the laptop. Keep a copy on a second device. Do not put
it in a public repository.
:::

The ESP32 has a ROM bootloader that cannot be erased, so a bad flash cannot
damage the ATOM permanently.
