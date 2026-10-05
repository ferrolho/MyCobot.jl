---
title: Set up the robot
description: Install the controller firmware from the browser, connect the ATOM to WiFi, and find the robot. No tools on the computer.
---

The [Setup page](/mycobot-280-lab/setup/) installs our controller firmware on the
ATOM, connects the ATOM to WiFi and finds the robot on the network. It needs only a
browser. Built on 2026-10-05 with firmware 4.3.0. **Not yet tested on the real ATOM.**

## What you need

- The myCobot 280 (for Arduino) and its 12 V supply.
- A USB-C **data** cable. Some cables can only charge.
- A computer with **Chrome or Edge** (Windows 10 or later, macOS, Linux or
  ChromeOS). Phones and tablets cannot install firmware: they have no Web Serial.
- A **2.4 GHz** WiFi network. The ATOM cannot use 5 GHz networks. The computer
  and the robot must be on the same network.

## Procedure

1. Turn off the arm (the 12 V supply). Do this before you connect the USB cable.
2. Connect the ATOM to the computer with the USB-C cable. The ATOM stays on the
   arm. If the installation fails, remove the ATOM from the arm and connect it
   again.
3. On the [Setup page](/mycobot-280-lab/setup/), click **Connect and install**.
   Select the port of the ATOM (FTDI `0403:6001`, for example "FT232R USB UART").
4. Click **Install**. For the first installation, select **Erase**. Wait about
   2 minutes.
5. When the installer asks, select the WiFi network and type the password.
6. Disconnect the cable. If you removed the ATOM, put it back on the arm. Turn on
   the 12 V supply.
   The LED matrix shows blue, then green.
7. On the Setup page, click **Search**. Then click **Open Control**.

Do not connect the ATOM's USB port while the ATOM is in the arm. With the 12 V
supply on, the port does not connect. With it off, the port supplies power to the
servos. See [ATOM controller board](/mycobot-280-lab/system/atom/#usb).

## How it works

| Step | Link | What happens |
| --- | --- | --- |
| Install | USB, 115 200 baud | [ESP Web Tools](https://esphome.github.io/esp-web-tools/) writes four parts: bootloader (`0x1000`), partition table (`0x8000`), `boot_app0` (`0xE000`) and the firmware (`0x10000`). |
| WiFi | USB | [Improv WiFi](https://www.improv-wifi.com/serial/) sends the network and the password. The ATOM tries to connect. It saves them in its NVS only when the connection works. |
| Find | WiFi | The page opens `ws://<address>/ws` and sends PING. PONG gives the firmware version and the state. |

- The installer writes separate parts, not one full flash image. An update thus
  keeps the saved WiFi network (NVS at `0x9000`). An installation with **Erase**
  removes it.
- The password goes over USB to the ATOM only. The website does not keep or send it.
- After the WiFi setup, the installer's **Visit Device** opens the Control page
  with the robot's IP address.

## The firmware on the Setup page

The Setup page installs the **public build** of the
[controller firmware](/mycobot-280-lab/firmware/controller/):

- No WiFi network and no password inside. WiFi is set up with Improv.
- No updates over WiFi (OTA). Without a password, anyone on the network could
  install firmware. To update, use the Setup page again: **Update** keeps the
  WiFi network.
- GitHub Actions builds it from the repository at each deployment of the site
  (`tools/build-public-firmware.sh`, in `.github/workflows/docs.yml`).

Lab builds are different. See
[Build, flash and update](/mycobot-280-lab/firmware/build-flash/#public-and-lab-builds).

## LED signals during the setup

| LED matrix | Meaning |
| --- | --- |
| Blinks white | No WiFi network is saved. Do steps 3 to 5. |
| Red (ATOM out of the arm) | No servos answer. This is normal out of the arm. |
| Blue, then green (ATOM in the arm) | The firmware starts, then holds the pose. |

All signals: [LED matrix signals](/mycobot-280-lab/firmware/led-signals/).

## Troubleshooting

| Problem | Possible cause | Remedy |
| --- | --- | --- |
| The port is not in the list. | A charge-only cable. The arm was on when you connected the cable. | Use a data cable. Disconnect the cable, turn off the arm, and connect the cable again. If the port is still not in the list, remove the ATOM from the arm. |
| "Failed to initialize". | Another program uses the port. | Close myStudio, the Arduino IDE and serial monitors. Disconnect the cable, connect it again and try again. |
| "Unable to connect" to WiFi. | Wrong password. A 5 GHz network. | Use a 2.4 GHz network. Type the password again. |
| Search does not find `mycobot.local`. | Some Windows and Android versions and some networks do not resolve `.local` names. | Use the IP address from **Visit Device** or from the router. |
| Search does not find the IP address. | The computer is on another network. Chrome blocked access to the local network. | Use the same network. Allow the access to devices on the local network. |
| The firmware is old. | The robot has an older version. | Search shows it. Click **Connect and install** again and select **Update**. |

## Go back to the stock firmware

Elephant Robotics' firmware is not open source. This site does not provide it.
See [Stock firmware backup](/mycobot-280-lab/firmware/stock-backup/).
