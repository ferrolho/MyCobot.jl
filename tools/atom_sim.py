"""
A simulated ATOM that speaks the WebSocket API (draft for controller firmware 4.2). Use it to
develop the Control page without the robot. Docs: website/src/content/docs/comms/websocket-api.md.

    python3 tools/atom_sim.py [--host 127.0.0.1] [--port 8281] [--recording RECORDING.csv]

The arm is simulated: each joint follows its goal with a 0.12 s lag and a 90 °/s speed limit.
MOVE_TO, JOG and TRACK (with the 200 ms deadman), CONTROL, SUBSCRIBE/STREAM, HOLD, STOP, PING, STATE,
REG_READ and REG_WRITE work as specified. PLAY and PLAY_SIGNAL play the recording (if given) as
500 Hz telemetry with tools/atom_replay.py, and the simulated arm follows it.
Needs: pip install websockets numpy
"""
import argparse
import asyncio
import math
import os
import random
import struct
import sys
import time

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import atom_replay  # noqa: E402
import robot_params  # noqa: E402  (generated from mycobot_description/config)

VERSION = (4, 6, 0)
SIGN = robot_params.SIGN
STEPS_PER_DEG = robot_params.STEPS_PER_DEG
LIMIT_MIN = robot_params.LIMIT_MIN   # model joint limits (°)
LIMIT_MAX = robot_params.LIMIT_MAX
BOOTING, HOLDING, READY, PLAYING, ERROR, OTA, MOVING, JOGGING, TRACKING = range(9)
STATE_NAMES = ("booting", "holding", "ready", "playing", "error", "ota", "moving", "jogging", "tracking")

DT = 0.002            # simulation step (500 Hz, as the firmware's control loop)
TAU = 0.12            # servo lag (s)
SERVO_VMAX = 90.0     # °/s
MOVE_VMAX = robot_params.VMAX   # MOVE_TO speed limit (°/s)
MOVE_AMAX = robot_params.AMAX   # MOVE_TO acceleration limits (°/s²)
JOG_VMAX = 30.0       # °/s
JOG_AMAX = 200.0      # °/s²
JOG_MARGIN = 2.0      # stop this far inside the joint limits (°)
DEADMAN = 0.2         # s
LEASE = 2.0           # s
SUB_TIMEOUT = 2.0     # s
MAX_SUBS = 8


def pos_raw(j, deg):
    hi = 0xFFFF if robot_params.MULTI_TURN[j] else 4095   # multi-turn joints go past one turn (4.5)
    return int(min(hi, max(0, round(2048 + SIGN[j] * deg * STEPS_PER_DEG))))


def track_step(q, v, goal, vmax, stop):
    """One TRACK step, as motion::track_step in the firmware (motion.h): to the goal at up to vmax
    and MOVE_AMAX, braking to stop on it; with stop, brake to zero."""
    vmax = max(0.0, min(MOVE_VMAX, vmax))
    for j in range(6):
        a = MOVE_AMAX[j]
        dv = a * DT
        stoppable = lambda d: 0.0 if d <= 0 else dv * (math.sqrt(0.25 + 2 * d / (a * DT * DT)) - 0.5)
        lo, hi = LIMIT_MIN[j] + JOG_MARGIN, LIMIT_MAX[j] - JOG_MARGIN
        g = max(lo, min(hi, goal[j]))
        e = g - q[j]
        t = 0.0 if stop else math.copysign(min(vmax, stoppable(abs(e))), e)
        t = max(-stoppable(q[j] - lo), min(stoppable(hi - q[j]), t))
        v0 = v[j]
        v[j] += max(-dv, min(dv, t - v[j]))
        qn = q[j] + v[j] * DT
        if not stop and e != 0 and (g - qn) * e <= 0 and abs(v0) <= 1.5 * dv:
            qn, v[j] = g, 0.0
        if qn > hi or qn < lo:
            b = hi if qn > hi else lo
            v[j], qn = (b - q[j]) / DT, b
        q[j] = qn


def minjerk(x):
    x = min(1.0, max(0.0, x))
    return x ** 3 * (10 - 15 * x + 6 * x ** 2)


class Client:
    def __init__(self, ws):
        self.ws = ws
        self.last_msg = time.monotonic()
        self.sub_rate = 0
        self.sub_seen = 0.0
        self.sub_due = 0.0


class Sim:
    def __init__(self, recording):
        self.rec = atom_replay.Replay(atom_replay.load(recording)) if recording else None
        self.t0 = time.monotonic()
        self.q = [0.0] * 6
        self.dq = [0.0] * 6
        self.goal = [0.0] * 6
        self.jog_v = [0.0] * 6        # current jog velocity (ramped)
        self.jog_cmd = [0.0] * 6      # requested jog velocity
        self.jog_last = 0.0
        self.track_goal = [0.0] * 6   # TRACK (4.4): goal pose and speed cap
        self.track_vmax = 0.0
        self.gripper = 0.5            # gripper (4.6): opening 0..1, its goal, and torque on
        self.gripper_goal = 0.5
        self.gripper_on = False
        self.state = HOLDING
        self.plan_samples = 0
        self.gains = {j: [32, 4, 16] if j <= 3 else [32, 8, 0] for j in range(1, 8)}
        self.temp = [27.0, 29.0, 28.0, 26.0, 25.0, 25.0]
        self.clients = set()
        self.controller = None
        self.move = None              # (start, goal, t_start, duration, client)
        self.play_task = None
        self.play_client = None

    # --- Packets -------------------------------------------------------------------------------

    def load(self):
        # A rough gravity load on J2 and J3 (percent), plus a little noise.
        s2 = math.sin(math.radians(self.q[1]))
        s23 = math.sin(math.radians(self.q[1] + self.q[2]))
        l = [0.0, -25 * s2 - 10 * s23, -10 * s23, 0.0, 0.0, 0.0]
        return [x + random.gauss(0, 0.3) for x in l]

    def raw(self):
        p = [pos_raw(j, self.q[j]) for j in range(6)]
        s = [atom_replay.sm15(SIGN[j] * self.dq[j] * STEPS_PER_DEG) for j in range(6)]
        l = [atom_replay.load_raw(j, x) for j, x in enumerate(self.load())]
        acc = [int(round(random.gauss(0, 0.004) * 4096)), int(round(random.gauss(0, 0.004) * 4096)),
               int(round((1 + random.gauss(0, 0.004)) * 4096))]
        gyr = [int(round(random.gauss(0, 0.2) * 16.4)) for _ in range(3)]
        return p, s, l, acc, gyr

    def stream(self, client):
        p, s, l, acc, gyr = self.raw()
        volt = [76, 76, 76, 66, 64, 64]
        ctrl = 0 if self.controller is None else (1 if self.controller is client else 2)
        t_ms = int((time.monotonic() - self.t0) * 1000) & 0xFFFFFFFF
        g_load = 30 if self.gripper_on and abs(self.gripper - self.gripper_goal) < 1e-3 else 5
        return struct.pack("<BIBB6H6H6H6B6B6B3h3hBBhh", 0x88, t_ms, self.state, 1, *p, *s, *l,
                           *[int(round(t)) for t in self.temp], *volt, *([0] * 6), *acc, *gyr, ctrl,
                           1, int(round(self.gripper * 1000)), g_load if self.gripper_on else 0)

    def status_line(self):
        ctrl = "none" if self.controller is None else "taken"
        return (f"atom-sim {'.'.join(map(str, VERSION))} state={STATE_NAMES[self.state]} "
                f"clients={len(self.clients)} control={ctrl} plan={self.plan_samples}")

    # --- Simulation ----------------------------------------------------------------------------

    def step(self, now):
        if self.gripper_on:   # the full stroke in about 0.6 s
            d = self.gripper_goal - self.gripper
            self.gripper += max(-DT / 0.6, min(DT / 0.6, d))
        if self.controller is not None and now - self.controller.last_msg > LEASE:
            run = (self.state == PLAYING and self.play_client is self.controller) or \
                  (self.state == MOVING and self.move and self.move[4] is self.controller)
            if run:   # control does not expire during the holder's run; the timer restarts at its end
                self.controller.last_msg = now
            else:
                self.controller = None
        if self.state == MOVING and self.move:
            start, goal, t_start, dur, client = self.move
            x = minjerk((now - t_start) / dur)
            self.goal = [a + x * (b - a) for a, b in zip(start, goal)]
            if now - t_start >= dur:
                self.move = None
                self.state = HOLDING
                self.done(client, 0)
        if self.state == JOGGING:
            if now - self.jog_last > DEADMAN:
                self.jog_cmd = [0.0] * 6
            for j in range(6):
                dv = max(-JOG_AMAX * DT, min(JOG_AMAX * DT, self.jog_cmd[j] - self.jog_v[j]))
                self.jog_v[j] += dv
                lo, hi = LIMIT_MIN[j] + JOG_MARGIN, LIMIT_MAX[j] - JOG_MARGIN
                g = self.goal[j] + self.jog_v[j] * DT
                if g >= hi or g <= lo:
                    g = hi if g >= hi else lo
                    self.jog_v[j] = 0.0
                self.goal[j] = g
            if not any(self.jog_cmd) and not any(self.jog_v):
                self.state = HOLDING
        if self.state == TRACKING:
            stop = now - self.jog_last > DEADMAN
            track_step(self.goal, self.jog_v, self.track_goal, self.track_vmax, stop)
            if stop and not any(self.jog_v):
                self.state = HOLDING
        if self.state != PLAYING:
            for j in range(6):
                v = max(-SERVO_VMAX, min(SERVO_VMAX, (self.goal[j] - self.q[j]) / TAU))
                self.q[j] += v * DT
                self.dq[j] = v
        for j in range(6):   # temperature: slowly towards 25 °C + 10 °C at full speed
            target = 25 + 10 * min(1.0, abs(self.dq[j]) / 60)
            self.temp[j] += (target - self.temp[j]) * DT / 60

    def stop_motion(self, result):
        """Stop a move, a jog or a play, and hold the pose."""
        if self.play_task:
            self.play_task.cancel()
        if self.state == MOVING and self.move:
            self.done(self.move[4], result)
        self.move = None
        self.jog_cmd = [0.0] * 6
        self.jog_v = [0.0] * 6
        self.goal = list(self.q)
        if self.state != PLAYING:   # a cancelled play sets the state when it ends
            self.state = READY if self.plan_samples else HOLDING

    # --- Sending -------------------------------------------------------------------------------

    def send(self, client, data):
        if client in self.clients:
            asyncio.ensure_future(self._send(client, data))

    async def _send(self, client, data):
        try:
            await client.ws.send(data)
        except ConnectionClosed:
            pass

    def ack(self, client, code, status, value=0):
        self.send(client, struct.pack("<BBbI", 0x83, code, status, value))

    def done(self, client, result):
        self.send(client, struct.pack("<BBIIIIBh", 0x85, result, 0, 2000, 0, 0, 0, 0))

    # --- Commands ------------------------------------------------------------------------------

    def moving(self):
        return self.state in (PLAYING, MOVING, JOGGING, TRACKING)

    def handle(self, c, b):
        now = time.monotonic()
        c.last_msg = now
        code = b[0]
        needs_control = code in (0x04, 0x05, 0x06, 0x07, 0x0A, 0x0B, 0x0E, 0x0F, 0x10, 0x11)
        if needs_control and self.controller is None:
            self.controller = c       # nobody has control: the command takes it (as CONTROL 1)
        if needs_control and self.controller is not c:
            return self.ack(c, code, -2)
        if code == 0x01:
            self.send(c, struct.pack("<BHBIHBBBB", 0x81, VERSION[0], self.state, self.plan_samples, 500, 1, 1,
                                     VERSION[1], VERSION[2]))
        elif code == 0x02:
            if self.state != PLAYING:
                p, s, l, acc, gyr = self.raw()
                self.send(c, struct.pack("<BB6H6H6H3h3h", 0x82, 1, *p, *s, *l, *acc, *gyr))
        elif code == 0x03:      # HOLD: also clears an error
            self.stop_motion(2)
            self.ack(c, 0x03, 0)
        elif code == 0x04:
            if self.moving():
                return self.ack(c, 0x04, -1)
            self.plan_samples = struct.unpack_from("<I", b, 1)[0]
            self.ack(c, 0x04, 0)
        elif code == 0x05:
            self.ack(c, 0x05, 0, struct.unpack_from("<I", b, 1)[0])
        elif code == 0x06:
            self.state = READY
            self.ack(c, 0x06, 0)
        elif code in (0x07, 0x0B):
            if self.moving() or self.rec is None:
                return self.ack(c, code, -1)
            self.ack(c, code, 0)
            self.state = PLAYING
            self.play_client = c
            self.play_task = asyncio.ensure_future(self.play(c))
        elif code == 0x08:
            self.stop_motion(2)
            self.ack(c, 0x08, 0)
        elif code == 0x09 and len(b) >= 4:
            sid, a, n = b[1], b[2], b[3]
            if self.state == PLAYING:
                return self.send(c, bytes([0x86, sid, a, n, 0xFF]))
            regs = bytearray(128)
            regs[21:24] = bytes(self.gains.get(sid, [0, 0, 0]))
            if 1 <= sid <= 6:
                regs[62], regs[63] = (76 if sid <= 3 else 65), int(round(self.temp[sid - 1]))
            self.send(c, bytes([0x86, sid, a, n, 0]) + bytes(regs[a:a + n]))
        elif code == 0x0A and len(b) >= 5:
            sid, a, n = b[1], b[2], b[3]
            ok = a >= 9 and a + n <= 80 and not (a <= 55 < a + n) and not self.state == PLAYING
            if ok and a == 21 and n == 3:
                self.gains[sid] = list(b[4:7])
            self.send(c, bytes([0x87, sid, a, 0 if ok else 0xFB, 0]))
        elif code == 0x0C and len(b) >= 3:
            rate = struct.unpack_from("<H", b, 1)[0]
            subs = sum(1 for x in self.clients if x.sub_rate)
            if rate == 0:
                c.sub_rate = 0
            elif c.sub_rate or subs < MAX_SUBS:
                c.sub_rate, c.sub_seen = min(rate, 100), now
        elif code == 0x0D and len(b) >= 2:
            action = b[1]
            if action == 0:
                if self.controller is c:
                    self.controller = None
                self.ack(c, 0x0D, 0)
            elif action == 1:
                if self.controller in (None, c):
                    self.controller = c
                    self.ack(c, 0x0D, 0)
                else:
                    self.ack(c, 0x0D, -2)
            elif action == 2:
                if self.controller not in (None, c) and self.moving():
                    self.ack(c, 0x0D, -1)
                else:
                    self.controller = c
                    self.ack(c, 0x0D, 0)
        elif code == 0x0E and len(b) >= 15:
            vals = struct.unpack_from("<6hH", b, 1)
            goal, dur_ms = [v / 100 for v in vals[:6]], vals[6]
            if self.moving():
                return self.ack(c, 0x0E, -1)
            for j in range(6):
                if not LIMIT_MIN[j] + JOG_MARGIN <= goal[j] <= LIMIT_MAX[j] - JOG_MARGIN:   # as motion::move_validate
                    return self.ack(c, 0x0E, -10 - (j + 1))
            start = list(self.q)
            t_min = max(max(1.875 * d / MOVE_VMAX, math.sqrt(5.77 * d / a))
                        for d, a in ((abs(g - s), a) for g, s, a in zip(goal, start, MOVE_AMAX)))
            dur = dur_ms / 1000 if dur_ms else max(0.2, t_min)
            if dur < t_min - 1e-6:
                return self.ack(c, 0x0E, -1)
            self.move = (start, goal, now, dur, c)
            self.state = MOVING
            self.ack(c, 0x0E, 0)
        elif code == 0x0F and len(b) >= 14:
            frame = b[1]
            vel = [v / 10 for v in struct.unpack_from("<6h", b, 2)]
            if frame != 0 or self.state in (PLAYING, MOVING):
                return self.ack(c, 0x0F, -1)
            self.jog_cmd = [max(-JOG_VMAX, min(JOG_VMAX, v)) for v in vel]
            self.jog_last = now
            if any(self.jog_cmd) and self.state != JOGGING:
                self.goal = list(self.q)
                self.state = JOGGING
        elif code == 0x11 and len(b) >= 3:    # GRIPPER (4.6): opening (0.1 %) or 0xFFFF (torque off)
            v = struct.unpack_from("<H", b, 1)[0]
            if v > 1000 and v != 0xFFFF or self.state == PLAYING:
                return self.ack(c, 0x11, -1)
            self.gripper_on = v != 0xFFFF
            if self.gripper_on:
                self.gripper_goal = v / 1000
        elif code == 0x10 and len(b) >= 15:   # TRACK (4.4): goal pose (0.01°) and speed cap (0.1°/s)
            goal = [v / 100 for v in struct.unpack_from("<6h", b, 1)]
            vmax = struct.unpack_from("<H", b, 13)[0] / 10
            bad = [j for j in range(6) if not LIMIT_MIN[j] + JOG_MARGIN <= goal[j] <= LIMIT_MAX[j] - JOG_MARGIN]
            if bad:
                return self.ack(c, 0x10, -10 - (bad[0] + 1))
            if self.state not in (HOLDING, READY, TRACKING):
                return self.ack(c, 0x10, -1)
            self.track_goal, self.track_vmax, self.jog_last = goal, vmax, now
            if self.state != TRACKING:
                self.goal = list(self.q)
                self.jog_v = [0.0] * 6
                self.state = TRACKING

    async def play(self, c):
        rec, seq, result = self.rec, 0, 0
        t_start = time.monotonic()
        try:
            k = 0
            while k < rec.n:
                batch = []
                while k < rec.n and rec.r["t"][k] <= time.monotonic() - t_start and len(batch) < 18:
                    row = rec.r[k]
                    batch.append(rec.sample(row))
                    for j in range(6):
                        self.q[j] = float(row[f"q_{j+1}"])
                        self.dq[j] = float(row[f"dq_{j+1}"])
                    k += 1
                if batch:
                    self.send(c, struct.pack("<BIB", 0x84, seq, len(batch)) + b"".join(batch))
                    seq += len(batch)
                await asyncio.sleep(0.01)
        except asyncio.CancelledError:
            result = 2
        self.done(c, result)
        self.goal = list(self.q)
        self.state = READY if self.plan_samples else HOLDING
        self.play_task = None


async def run(sim, host, port):
    async def handler(ws):
        if ws.request.path != "/ws":
            await ws.close(1008, "use /ws")
            return
        c = Client(ws)
        sim.clients.add(c)
        try:
            async for msg in ws:
                if isinstance(msg, (bytes, bytearray)) and msg:
                    sim.handle(c, bytes(msg))
        except ConnectionClosed:
            pass
        finally:
            sim.clients.discard(c)
            if sim.controller is c:
                sim.controller = None
                if sim.state in (MOVING, JOGGING):
                    sim.stop_motion(2)

    async def ticker():
        nxt = time.monotonic()
        last_log = 0.0
        while True:
            now = time.monotonic()
            while nxt <= now:
                sim.step(nxt)
                nxt += DT
            for c in list(sim.clients):
                if c.sub_rate and now - c.sub_seen > SUB_TIMEOUT:
                    c.sub_rate = 0
                if c.sub_rate and now >= c.sub_due:
                    sim.send(c, sim.stream(c))
                    c.sub_due = max(c.sub_due + 1 / c.sub_rate, now - 0.05)
            if now - last_log >= 1.0:
                last_log = now
                line = sim.status_line()
                for c in list(sim.clients):
                    sim.send(c, line)
            await asyncio.sleep(0.004)

    async with serve(handler, host, port, max_size=2 ** 20, compression=None):
        print(f"Simulated ATOM (WebSocket API {'.'.join(map(str, VERSION))}) on ws://{host}:{port}/ws"
              + (f", recording {sim.rec.n} samples" if sim.rec else ", no recording (PLAY refused)"))
        await ticker()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--host", default="127.0.0.1")   # or the Tailscale address; never the home network
    ap.add_argument("--port", type=int, default=8281)
    ap.add_argument("--recording", help="an ATOM recording for PLAY and PLAY_SIGNAL")
    a = ap.parse_args()
    asyncio.run(run(Sim(a.recording), a.host, a.port))


if __name__ == "__main__":
    main()
