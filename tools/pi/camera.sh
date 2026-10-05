#!/usr/bin/env bash
# Logitech C505 webcam on the Raspberry Pi 5, pointed at the arm.
#
#   tools/pi/camera.sh snapshot [FILE]   # on the Pi: one 1280x960 JPEG (default /tmp/arm.jpg), from the
#                                        # lab service if it runs, else from the camera
#   tools/pi/camera.sh stdout            # on the Pi: MJPEG stream to stdout
#
# Watch it from the laptop (over SSH, no open port; needs ffplay from `brew install ffmpeg`):
#   ssh raspberrypi5 '~/myCobot/mycobot-280-lab/tools/pi/camera.sh stdout' | ffplay -loglevel error -fflags nobuffer -f mjpeg -i -
#
# 1280x960 (4:3) is the sensor's full field of view; 1280x720 crops the top and bottom.
set -euo pipefail
DEV=/dev/video0
v4l2-ctl -d "$DEV" -c power_line_frequency=1 >/dev/null 2>&1 || true   # 50 Hz mains
IN=(-hide_banner -loglevel error -f v4l2 -input_format mjpeg -video_size 1280x960 -framerate 30 -i "$DEV")
case "${1:-}" in
  snapshot)
    # The lab service (tools/pi/lab_service.py) owns the camera while it runs: ask it first.
    if curl -fsS -m 6 -o "${2:-/tmp/arm.jpg}" "${LAB_SERVICE:-http://100.69.15.110:8280}/snapshot.jpg" 2>/dev/null; then :
    else ffmpeg "${IN[@]}" -frames:v 10 -update 1 -y "${2:-/tmp/arm.jpg}" 2>/dev/null; fi
    echo "${2:-/tmp/arm.jpg}" ;;
  stdout)   exec ffmpeg "${IN[@]}" -c:v copy -f mjpeg - ;;
  *)        sed -n '2,10p' "$0"; exit 1 ;;
esac
