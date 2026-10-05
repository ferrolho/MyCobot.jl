"""
The lab web service on the Raspberry Pi: serves the built documentation site (with the Control
page) and the webcam next to the robot. It is the only process that opens the camera.

    python3 tools/pi/lab_service.py [--host 100.69.15.110] [--port 8280]

    /mycobot-280-lab/...   the site (website/dist; build it first with `npm run build`)
    /camera.json           {"camera": true, "running": ..., "clients": ...}: the page shows the Pi camera if this exists
    /camera.mjpg           MJPEG stream (multipart/x-mixed-replace), 1280x960 at 30 fps
    /snapshot.jpg          one recent frame
    /log.json              {"log": true}: the Control page sends its session log if this exists
    POST /log?session=ID   JSONL events from the Control page, appended to LOG_DIR/ID.jsonl

The camera runs (tools/pi/camera.sh stdout) only while a client streams, and for IDLE_S after the
last frame request, so other tools can open /dev/video0 when nobody watches.
Listen on the Tailscale address only, never on the home network. Standard library only.
"""
import argparse
import json
import os
import re
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SITE = os.path.join(ROOT, "website", "dist")
BASE = "/mycobot-280-lab/"
CAMERA_CMD = [os.path.join(ROOT, "tools", "pi", "camera.sh"), "stdout"]
IDLE_S = 10.0          # stop the camera this long after the last client
BOUNDARY = "frame"
LOG_DIR = os.path.expanduser("~/myCobot/lab-logs")   # Control page session logs, one JSONL file per session
LOG_MAX_BODY = 1 << 20


class Camera:
    """One capture process; frames shared with every client."""

    def __init__(self):
        self.lock = threading.Condition()
        self.frame = None          # latest JPEG
        self.seq = 0
        self.proc = None
        self.clients = 0           # open streams
        self.last_use = 0.0
        self.error = ""
        self.start_seq = 0         # seq when the capture started

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def touch(self):
        with self.lock:
            self.last_use = time.monotonic()
            if not self.running():
                self.start()

    def start(self):
        self.frame, self.error, self.start_seq = None, "", self.seq
        try:
            self.proc = subprocess.Popen(CAMERA_CMD, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        except OSError as e:
            self.error = str(e)
            return
        threading.Thread(target=self.reader, args=(self.proc,), daemon=True).start()

    def reader(self, proc):
        buf = b""
        while True:
            chunk = proc.stdout.read(65536)
            if not chunk:
                break
            buf += chunk
            # The stream is concatenated JPEGs: SOI ff d8 ... EOI ff d9.
            while True:
                start = buf.find(b"\xff\xd8")
                end = buf.find(b"\xff\xd9", start + 2) if start >= 0 else -1
                if end < 0:
                    buf = buf[start:] if start > 0 else buf
                    break
                with self.lock:
                    self.frame = buf[start:end + 2]
                    self.seq += 1
                    self.lock.notify_all()
                buf = buf[end + 2:]
        err = proc.stderr.read().decode(errors="replace").strip()
        with self.lock:
            self.error = err or f"camera process ended ({proc.poll()})"
            self.lock.notify_all()

    def snapshot(self, warmup=10, timeout=5.0):
        """A recent frame. After a cold start, skip the first frames (the exposure settles), as camera.sh does."""
        self.touch()
        with self.lock:
            self.lock.wait_for(lambda: self.seq >= self.start_seq + warmup or not self.running(), timeout)
            return self.frame

    def wait_frame(self, after_seq, timeout=3.0):
        with self.lock:
            self.lock.wait_for(lambda: self.seq > after_seq or not self.running(), timeout)
            return self.seq, self.frame

    def reaper(self):
        while True:
            time.sleep(1.0)
            with self.lock:
                idle = self.clients == 0 and time.monotonic() - self.last_use > IDLE_S
                if idle and self.running():
                    self.proc.terminate()
                    self.proc = None


camera = Camera()


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=SITE, **kw)

    def log_message(self, fmt, *args):
        pass

    def end_headers(self):
        live = self.path.startswith(("/camera", "/snapshot"))
        self.send_header("Cache-Control", "no-store" if live else "no-cache")
        super().end_headers()

    def do_GET(self):
        route = urlsplit(self.path).path   # without the query string
        if route in ("/", ""):
            self.send_response(HTTPStatus.FOUND)
            self.send_header("Location", BASE)
            return self.end_headers()
        if route == "/camera.json":
            return self.json({"camera": os.path.exists("/dev/video0"), "running": camera.running(),
                              "clients": camera.clients, "error": camera.error})
        if route == "/snapshot.jpg":
            frame = camera.snapshot()
            if not frame:
                return self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, camera.error or "no frame from the camera")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            return self.wfile.write(frame)
        if route == "/camera.mjpg":
            return self.stream()
        if route == "/log.json":
            return self.json({"log": True})
        if self.path.startswith(BASE):
            self.path = "/" + self.path[len(BASE):]
            return super().do_GET()
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self):
        u = urlsplit(self.path)
        if u.path != "/log":
            return self.send_error(HTTPStatus.NOT_FOUND)
        session = (parse_qs(u.query).get("session") or [""])[0]
        n = int(self.headers.get("Content-Length") or 0)
        if not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", session) or not 0 < n <= LOG_MAX_BODY:
            return self.send_error(HTTPStatus.BAD_REQUEST)
        body = self.rfile.read(n)
        os.makedirs(LOG_DIR, exist_ok=True)
        with open(os.path.join(LOG_DIR, session + ".jsonl"), "ab") as f:
            f.write(body if body.endswith(b"\n") else body + b"\n")
        self.send_response(HTTPStatus.NO_CONTENT)
        self.end_headers()

    def json(self, obj):
        body = json.dumps(obj).encode()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def stream(self):
        with camera.lock:
            camera.clients += 1
        try:
            camera.touch()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
            self.end_headers()
            seq = 0
            while True:
                new_seq, frame = camera.wait_frame(seq)
                camera.touch()
                if frame is None or new_seq == seq:   # no new frame (the camera starts or failed)
                    time.sleep(0.2)
                    continue
                seq = new_seq
                self.wfile.write(f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(frame)}\r\n\r\n".encode())
                self.wfile.write(frame + b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            with camera.lock:
                camera.clients -= 1
                camera.last_use = time.monotonic()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--host", default="127.0.0.1")   # or the Tailscale address; never the home network
    ap.add_argument("--port", type=int, default=8280)
    a = ap.parse_args()
    threading.Thread(target=camera.reaper, daemon=True).start()
    server = ThreadingHTTPServer((a.host, a.port), Handler)
    server.daemon_threads = True
    print(f"Lab service on http://{a.host}:{a.port}{BASE} (site: {SITE})", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
