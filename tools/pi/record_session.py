#!/usr/bin/env python3
"""
Record a robot session for a timelapse: camera frames (from the lab service) and the robot state
(STREAM from the ATOM), each with the Pi's clock. tools/pi/make_timelapse.py keeps only the parts
where the robot moves.

    python3 tools/pi/record_session.py OUT_DIR [--fps 5] [--atom 192.168.1.107]

OUT_DIR/frames/<unix ms>.jpg (1280x960) and OUT_DIR/robot.csv. Stop with Ctrl-C or SIGTERM.
"""
import argparse
import os
import signal
import socket
import struct
import threading
import time
import urllib.request

ap = argparse.ArgumentParser()
ap.add_argument("out")
ap.add_argument("--fps", type=float, default=5.0)
ap.add_argument("--atom", default="192.168.1.107")
ap.add_argument("--camera", default="http://127.0.0.1:8280/camera.mjpg?full=1")
args = ap.parse_args()
frames = os.path.join(args.out, "frames")
os.makedirs(frames, exist_ok=True)
stop = threading.Event()
signal.signal(signal.SIGTERM, lambda *_: stop.set())


def camera():
    """Keep one frame every 1/fps s from the MJPEG stream; reconnect on errors."""
    while not stop.is_set():
        try:
            with urllib.request.urlopen(args.camera, timeout=10) as r:
                buf, last = b"", 0.0
                while not stop.is_set():
                    chunk = r.read(65536)
                    if not chunk:
                        break
                    buf += chunk
                    while True:
                        a = buf.find(b"\xff\xd8")
                        b = buf.find(b"\xff\xd9", a + 2)
                        if a < 0 or b < 0:
                            buf = buf[a:] if a >= 0 else b""
                            break
                        jpg, buf = buf[a:b + 2], buf[b + 2:]
                        now = time.time()
                        if now - last >= 1 / args.fps:
                            last = now
                            with open(os.path.join(frames, f"{int(now * 1000)}.jpg"), "wb") as f:
                                f.write(jpg)
        except OSError as e:
            print("camera:", e, flush=True)
            stop.wait(2)


def robot():
    """SUBSCRIBE at 10 Hz (renewed every 0.5 s); one CSV row per STREAM message."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0.2)
    with open(os.path.join(args.out, "robot.csv"), "a", buffering=1) as f:
        if f.tell() == 0:
            f.write("t,state," + ",".join(f"pos{j}" for j in range(1, 7)) + ",gripper,opening,gload\n")
        renew = 0.0
        while not stop.is_set():
            if time.time() >= renew:
                s.sendto(b"\x0c" + struct.pack("<H", 10), (args.atom, 5006))
                renew = time.time() + 0.5
            try:
                m = s.recv(256)
            except OSError:
                continue
            if len(m) >= 79 and m[0] == 0x88:
                pos = struct.unpack_from("<6H", m, 7)
                g, op, gl = m[74], *struct.unpack_from("<2h", m, 75)
                f.write(f"{time.time():.3f},{m[5]}," + ",".join(map(str, pos)) + f",{g},{op},{gl}\n")
        s.sendto(b"\x0c" + struct.pack("<H", 0), (args.atom, 5006))


threads = [threading.Thread(target=camera), threading.Thread(target=robot)]
for t in threads:
    t.start()
try:
    while not stop.is_set():
        stop.wait(1)
except KeyboardInterrupt:
    stop.set()
for t in threads:
    t.join()
