#!/usr/bin/env bash
# Logitech C505 webcam on the Raspberry Pi 5, pointed at the arm.
#
#   tools/pi/camera.sh snapshot [FILE]   # on the Pi: one 1280x960 JPEG (default /tmp/arm.jpg), from the
#                                        # lab service if it runs, else from the camera
#   tools/pi/camera.sh stdout [FD]       # on the Pi: MJPEG stream to stdout; with FD, also a 640x480
#                                        # preview to that file descriptor (the lab service)
#
# Watch it from the laptop (over SSH, no open port; needs ffplay from `brew install ffmpeg`):
#   ssh raspberrypi5 '~/myCobot/mycobot-280-lab/tools/pi/camera.sh stdout' | ffplay -loglevel error -fflags nobuffer -f mjpeg -i -
#
# 1280x960 (4:3) is the sensor's full field of view; 1280x720 crops the top and bottom.
set -euo pipefail
DEV=/dev/video0
# Image settings (measured 2026-10-06, docs: software/raspberry-pi.md, "Image settings"). Auto exposure
# and auto white balance stay on. Contrast stretches the tones: with the defaults, auto exposure makes the
# white wall mid-grey and the black base grey. The camera keeps them only until it loses power.
v4l2-ctl -d "$DEV" -c power_line_frequency=1,exposure_dynamic_framerate=0,contrast=64,brightness=144,saturation=40 \
  >/dev/null 2>&1 || true   # 50 Hz mains; always 30 fps
IN=(-hide_banner -loglevel error -f v4l2 -input_format mjpeg -video_size 1280x960 -framerate 30 -i "$DEV")
case "${1:-}" in
  snapshot)
    # The lab service (tools/pi/lab_service.py) owns the camera while it runs: ask it first.
    if curl -fsS -m 6 -o "${2:-/tmp/arm.jpg}" "${LAB_SERVICE:-http://100.69.15.110:8280}/snapshot.jpg" 2>/dev/null; then :
    else ffmpeg "${IN[@]}" -frames:v 10 -update 1 -y "${2:-/tmp/arm.jpg}" 2>/dev/null; fi
    echo "${2:-/tmp/arm.jpg}" ;;
  stdout)
    # The full frames are copied (no decoding). The preview is decoded, scaled and encoded again, at
    # 30 fps (as the camera; fewer frames add delay): about 0.5 MB/s instead of 3.2 MB/s (2026-10-06:
    # the full stream delayed the robot's WebSocket by 1-2 s on the laptop).
    if [[ -n "${2:-}" ]]; then
      # Low delay: one thread (frame threads hold frames back), and each preview frame written at once.
      exec ffmpeg -fflags nobuffer -flags low_delay -threads 1 "${IN[@]}" -map 0:v -c:v copy -f mjpeg pipe:1 \
        -map 0:v -vf scale=640:480 -c:v mjpeg -q:v 7 -threads 1 -flush_packets 1 -f mjpeg "pipe:$2"
    fi
    exec ffmpeg "${IN[@]}" -c:v copy -f mjpeg - ;;
  *)        sed -n '2,10p' "$0"; exit 1 ;;
esac
