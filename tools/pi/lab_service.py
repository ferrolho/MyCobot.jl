"""
The lab web service on the Raspberry Pi: serves the built documentation site (with the Control
page), the webcam next to the robot, and a relay to the ATOM's WebSocket. It is the only process
that opens the camera.

    python3 tools/pi/lab_service.py [--host 0.0.0.0] [--port 8280] [--atom 192.168.1.107] [--allow ADDRESS ...]

    /mycobot-280-lab/...   the site (website/dist; build it first with `npm run build`)
    /camera.json           {"camera": true, "running": ..., "clients": ...}: the page shows the Pi camera if this exists
    /camera.mjpg           MJPEG stream (multipart/x-mixed-replace): a 640x480 preview at 30 fps;
                           ?full=1 for 1280x960 at 30 fps (3.2 MB/s: too much for a slow link)
    /snapshot.jpg          one recent frame
    /log.json              {"log": true}: the Control page sends its session log if this exists
    POST /log?session=ID   JSONL events from the Control page, appended to LOG_DIR/ID.jsonl
    /atom.json             {"atom": ADDRESS, "ws": "/atom/ws"}: the page connects through the relay if this exists
    /atom/ws               the ATOM's WebSocket (ws://ATOM/ws), relayed byte for byte
    /lab/scene.json        ~/myCobot/lab-scene.json: objects near the robot, for the 3D view (404 if missing)
    /lab/camera.json       ~/myCobot/lab-camera.json: the camera model, for the camera overlay (404 if missing)

The relay lets a browser away from home reach the ATOM over Tailscale. Only the Pi talks to the
ATOM, on the home network, so a slow link (for example a phone hotspot) does not fill the ATOM's
small send buffers. Both sockets have TCP_NODELAY: without it, the 50 Hz stream arrives in clumps,
once per round trip.

The camera runs (tools/pi/camera.sh stdout) only while a client streams, and for IDLE_S after the
last frame request, so other tools can open /dev/video0 when nobody watches.
The service has no login, and the relay moves the arm. Thus it serves only loopback, Tailscale
and the clients in --allow (one computer each on the home network); it closes all other
connections at once. Standard library only.
"""
import argparse
import ipaddress
import json
import os
import re
import select
import socket
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
FIRST_FRAME_S = 10.0   # a stream waits this long for the camera's first frame
BOUNDARY = "frame"
LOG_DIR = os.path.expanduser("~/myCobot/lab-logs")   # Control page session logs, one JSONL file per session
LOG_MAX_BODY = 1 << 20
# Lab data files, served as they are (written by lab scripts; not in the repository).
LAB_FILES = {
    "/lab/scene.json": os.path.expanduser("~/myCobot/lab-scene.json"),     # objects near the robot, for the 3D view
    "/lab/camera.json": os.path.expanduser("~/myCobot/lab-camera.json"),   # the camera model, for the camera overlay
}
ATOM = "192.168.1.107"   # the ATOM's address on the home network (--atom)
ALLOW = ("127.0.0.0/8", "100.64.0.0/10")   # clients always served: loopback and Tailscale (more with --allow)


class Camera:
    """One capture process; frames shared with every client: the full frame and a small preview."""

    def __init__(self):
        self.lock = threading.Condition()
        self.frame = None          # latest JPEG, 1280x960
        self.seq = 0
        self.preview = None        # latest preview JPEG, 640x480 at 30 fps
        self.preview_seq = 0
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
        self.frame, self.preview, self.error, self.start_seq = None, None, "", self.seq
        r, w = os.pipe()   # the preview stream
        try:
            self.proc = subprocess.Popen(CAMERA_CMD + [str(w)], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                         bufsize=0, pass_fds=(w,))
        except OSError as e:
            self.error = str(e)
            os.close(r)
            return
        finally:
            os.close(w)
        threading.Thread(target=self.reader, args=(self.proc, self.proc.stdout, False), daemon=True).start()
        threading.Thread(target=self.reader, args=(self.proc, os.fdopen(r, "rb", buffering=0), True), daemon=True).start()

    def reader(self, proc, pipe, preview):
        buf = b""
        while True:
            chunk = pipe.read(65536)
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
                    if preview:
                        self.preview = buf[start:end + 2]
                        self.preview_seq += 1
                    else:
                        self.frame = buf[start:end + 2]
                        self.seq += 1
                    self.lock.notify_all()
                buf = buf[end + 2:]
        pipe.close()
        if preview:
            return
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

    def wait_frame(self, after_seq, timeout=3.0, preview=False):
        with self.lock:
            latest = (lambda: (self.preview_seq, self.preview)) if preview else (lambda: (self.seq, self.frame))
            self.lock.wait_for(lambda: latest()[0] > after_seq or not self.running(), timeout)
            return latest()

    def reaper(self):
        while True:
            time.sleep(1.0)
            with self.lock:
                idle = self.clients == 0 and time.monotonic() - self.last_use > IDLE_S
                if idle and self.running():
                    self.proc.terminate()
                    self.proc = None


camera = Camera()


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow = [ipaddress.ip_network(n) for n in ALLOW]

    def verify_request(self, request, client_address):
        return any(ipaddress.ip_address(client_address[0]) in n for n in self.allow)


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
                return self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "No frame from the camera", camera.error)
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            return self.wfile.write(frame)
        if route == "/camera.mjpg":
            return self.stream(full=parse_qs(urlsplit(self.path).query).get("full") == ["1"])
        if route in LAB_FILES:
            try:
                with open(LAB_FILES[route], "rb") as f:
                    body = f.read()
            except OSError:
                return self.send_error(HTTPStatus.NOT_FOUND, f"No {os.path.basename(LAB_FILES[route])}")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            return self.wfile.write(body)
        if route == "/log.json":
            return self.json({"log": True})
        if route == "/atom.json":
            return self.json({"atom": ATOM, "ws": "/atom/ws"})
        if route == "/atom/ws":
            return self.relay()
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

    def relay(self):
        """Pass the WebSocket upgrade to the ATOM, then copy the bytes both ways until one side closes."""
        if self.headers.get("Upgrade", "").lower() != "websocket":
            return self.send_error(HTTPStatus.BAD_REQUEST, "WebSocket only")
        try:
            atom = socket.create_connection((ATOM, 80), timeout=3.0)
        except OSError as e:
            return self.send_error(HTTPStatus.BAD_GATEWAY, "No answer from the ATOM", f"{ATOM}: {e}")
        atom.settimeout(None)
        client = self.connection
        for s in (atom, client):
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        # Close the link to a browser that is gone (a dropped hotspot) after about 30 s, so that it
        # does not keep one of the ATOM's few WebSocket slots: TCP_USER_TIMEOUT while data waits
        # for an ACK, keepalive when idle. Its moves stop after 0.2 s (the deadman) and its control
        # ends 2 s after its last message (the firmware).
        client.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        for opt, v in (("TCP_USER_TIMEOUT", 30000), ("TCP_KEEPIDLE", 10), ("TCP_KEEPINTVL", 5), ("TCP_KEEPCNT", 4)):
            if hasattr(socket, opt):
                client.setsockopt(socket.IPPROTO_TCP, getattr(socket, opt), v)
        headers = "".join(f"{k}: {v}\r\n" for k, v in self.headers.items() if k.lower() != "host")
        atom.sendall(f"GET /ws HTTP/1.1\r\nHost: {ATOM}\r\n{headers}\r\n".encode("latin-1"))

        def atom_to_client():
            try:
                while data := atom.recv(65536):
                    client.sendall(data)
            except OSError:
                pass
            finally:
                for s in (client, atom):   # wake the other direction
                    try:
                        s.shutdown(socket.SHUT_RDWR)
                    except OSError:
                        pass

        t = threading.Thread(target=atom_to_client, daemon=True)
        t.start()
        try:
            while data := self.rfile.read1(65536):   # read1: the bytes already buffered first
                atom.sendall(data)
        except OSError:
            pass
        finally:
            try:
                atom.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            t.join()
            atom.close()
            self.close_connection = True

    def client_gone(self):
        """True if the browser closed the connection. It sends nothing else while it watches a stream."""
        try:
            readable, _, _ = select.select([self.connection], [], [], 0)
            return bool(readable) and not self.connection.recv(1, socket.MSG_PEEK)
        except OSError:
            return True

    def stream(self, full=False):
        """MJPEG until the browser closes it or the camera stops. If the camera gives no first frame, 503.

        The latest frame only: a write waits while the link is slow, and the frames that came meanwhile
        are skipped. A small send buffer keeps that wait short, so that the stream does not fill the
        link's buffers and delay the other connections (the robot's WebSocket)."""
        preview = not full
        self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 64 * 1024)
        if hasattr(socket, "TCP_NOTSENT_LOWAT"):
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NOTSENT_LOWAT, 16 * 1024)
        with camera.lock:
            camera.clients += 1
        try:
            camera.touch()
            seq, frame = 0, None
            deadline = time.monotonic() + FIRST_FRAME_S
            while frame is None and camera.running() and time.monotonic() < deadline:
                seq, frame = camera.wait_frame(seq, timeout=1.0, preview=preview)
            if frame is None:   # the error text in the body: it can have several lines, the status line cannot
                return self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "No frame from the camera", camera.error)
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
            self.end_headers()
            self.wfile.write(f"--{BOUNDARY}\r\n".encode())
            while True:
                # The boundary straight after each frame: a browser shows a part only when the next
                # boundary comes (with it at the start of the next frame, every frame was one frame late).
                self.wfile.write(f"Content-Type: image/jpeg\r\nContent-Length: {len(frame)}\r\n\r\n".encode()
                                 + frame + f"\r\n--{BOUNDARY}\r\n".encode())
                # Wait for the next frame. A closed connection shows when we write; while no frames
                # come, only client_gone() sees it.
                while True:
                    new_seq, frame = camera.wait_frame(seq, preview=preview)
                    if not camera.running():
                        return   # the camera stopped: end the stream (its error is in /camera.json)
                    with camera.lock:
                        camera.last_use = time.monotonic()
                    if new_seq != seq and frame is not None:
                        seq = new_seq
                        break
                    if self.client_gone():
                        return
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            with camera.lock:
                camera.clients -= 1
                camera.last_use = time.monotonic()


def main():
    global ATOM
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--host", default="127.0.0.1", help="the address to listen on (0.0.0.0: all)")
    ap.add_argument("--port", type=int, default=8280)
    ap.add_argument("--atom", default=ATOM, help="the ATOM's address, for the relay at /atom/ws")
    ap.add_argument("--allow", action="append", default=[], metavar="ADDRESS",
                    help="also serve this client (address or network); loopback and Tailscale always")
    a = ap.parse_args()
    ATOM = a.atom
    threading.Thread(target=camera.reaper, daemon=True).start()
    server = Server((a.host, a.port), Handler)
    server.allow = server.allow + [ipaddress.ip_network(n) for n in a.allow]
    print(f"Lab service on http://{a.host}:{a.port}{BASE} (site: {SITE}; clients: {', '.join(map(str, server.allow))})", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
