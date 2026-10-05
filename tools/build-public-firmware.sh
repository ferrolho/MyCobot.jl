#!/usr/bin/env bash
# Build the PUBLIC controller firmware for the Setup page's browser installer (ESP Web Tools):
#   website/public/installer/manifest.json and website/public/installer/<version>/*.bin
# Public = no WiFi credentials and no OTA (-DPUBLIC_BUILD); users set up WiFi with Improv over USB.
# GitHub Actions runs this before the site build (.github/workflows/docs.yml). On the Pi:
#   ARDUINO_CLI=~/bin/arduino-cli tools/build-public-firmware.sh
# The output is not committed (.gitignore).
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

CLI="${ARDUINO_CLI:-arduino-cli}"
FQBN="esp32:esp32:m5stack_atom:PartitionScheme=min_spiffs"
SKETCH=firmware/atom_controller
ver() { sed -n "s/^#define FW_$1 \([0-9]*\)$/\1/p" "$SKETCH/atom_controller.ino"; }
VERSION="$(ver MAJOR).$(ver MINOR).$(ver PATCH)"
GIT="$(git describe --always --dirty)"
BUILD="$(mktemp -d)"
trap 'rm -rf "$BUILD"' EXIT

# No -I to ~/.config/mycobot: the public build never sees wifi_secrets.h (and ignores it with PUBLIC_BUILD).
"$CLI" compile --fqbn "$FQBN" \
    --build-property "compiler.cpp.extra_flags=-DPUBLIC_BUILD -DFW_GIT=\"$GIT\"" \
    --output-dir "$BUILD" "$SKETCH"

# Safety net on a lab machine: the binary must not contain the lab WiFi or OTA strings.
SECRETS="$HOME/.config/mycobot/wifi_secrets.h"
if [ -f "$SECRETS" ]; then
    while IFS= read -r value; do
        [ -n "$value" ] || continue
        if grep -qaF -- "$value" "$BUILD/atom_controller.ino.bin"; then
            echo "error: the public binary contains a value from $SECRETS" >&2
            exit 1
        fi
    done < <(sed -n 's/^#define [A-Z_]* "\(.*\)".*$/\1/p' "$SECRETS")
fi

# Separate parts, not the merged image: the merged image covers the NVS partition (0x9000) and an
# update would erase the saved WiFi network. boot_app0 (0xE000) makes app0 boot after an OTA image.
BOOT_APP0="$(ls "$("$CLI" config get directories.data)"/packages/esp32/hardware/esp32/*/tools/partitions/boot_app0.bin | tail -1)"
OUT=website/public/installer
rm -rf "$OUT"
mkdir -p "$OUT/$VERSION"
cp "$BUILD/atom_controller.ino.bootloader.bin" "$OUT/$VERSION/bootloader.bin"
cp "$BUILD/atom_controller.ino.partitions.bin" "$OUT/$VERSION/partitions.bin"
cp "$BOOT_APP0" "$OUT/$VERSION/boot_app0.bin"
cp "$BUILD/atom_controller.ino.bin" "$OUT/$VERSION/atom_controller.bin"

# "name" must equal the firmware name in the Improv device info (improv_handle, GET_INFO): then the
# installer offers an update that keeps the WiFi settings.
cat > "$OUT/manifest.json" <<EOF
{
  "name": "myCobot 280 controller",
  "version": "$VERSION",
  "git": "$GIT",
  "new_install_prompt_erase": true,
  "new_install_improv_wait_time": 20,
  "builds": [
    {
      "chipFamily": "ESP32",
      "improv": true,
      "parts": [
        { "path": "$VERSION/bootloader.bin", "offset": 4096 },
        { "path": "$VERSION/partitions.bin", "offset": 32768 },
        { "path": "$VERSION/boot_app0.bin", "offset": 57344 },
        { "path": "$VERSION/atom_controller.bin", "offset": 65536 }
      ]
    }
  ]
}
EOF
echo "public firmware $VERSION ($GIT) in $OUT"
