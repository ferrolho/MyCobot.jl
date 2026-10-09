"""
A stand-in for the ATOM controller (UDP protocol 4.1) that plays a saved recording instead of
moving the robot. Use it to develop clients (bridge, web control centre, scripts) without the
robot. Docs: website/src/content/docs/comms/clients.md.

    python3 tools/atom_replay.py RECORDING.csv [--host 127.0.0.1] [--port 5106]

RECORDING.csv: an ATOM recording (scripts/play_plan.jl --atom, scripts/sysid.jl), with the columns
t, q_plan_j, q_cmd_j, q_j, dq_j, load_j and acc_x..gyro_z.

Behaviour (as firmware 4.1): every reply goes to the sender; PLAY/PLAY_SIGNAL stream the
recording as 500 Hz telemetry (TELEM 0x84, 77-byte samples) to the client that started the
run, then DONE; SUBSCRIBE (0x0C) streams STREAM (0x88) packets, looping over the recording;
REG_READ/REG_WRITE work on gains (21-23), voltage (62) and temperature (63).
"""
import argparse
import asyncio
import math
import os
import struct
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_params import SIGN, STEPS_PER_DEG  # noqa: E402  (generated; J1-J7)
HOLDING, READY, PLAYING = 1, 2, 3


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    ok = ~np.isnan(np.c_[[d[f"q_{j}"] for j in range(1, 7)]].T).any(1)
    return d[ok]


def pos(j, deg):
    return int(min(4095, max(0, round(2048 + SIGN[j] * deg * STEPS_PER_DEG))))


def sm15(x):   # sign-magnitude, bit 15
    v = min(0x7FFF, int(round(abs(x))))
    return v | 0x8000 if x < 0 else v


def load_raw(j, pct):   # 0.1 % units, bit 10 = sign
    x = pct * SIGN[j]
    v = min(0x3FF, int(round(abs(x) * 10)))
    return v | 0x400 if x < 0 else v


class Replay:
    def __init__(self, rec):
        self.r = rec
        self.n = len(rec)
        self.t0 = time.monotonic()
        self.state = HOLDING
        self.plan_samples = 0
        self.gains = {j: [32, 4, 16] if j <= 3 else [32, 8, 0] for j in range(1, 8)}
        self.subs = {}      # addr -> (rate, last_seen, next_due)
        self.play_task = None

    def row(self, k):
        return self.r[k % self.n]

    def idle_row(self):
        dt = self.r["t"][-1] / max(self.n - 1, 1)
        return self.row(int((time.monotonic() - self.t0) / dt))

    def raw_state(self, row):
        p = [pos(j, row[f"q_{j+1}"]) for j in range(6)]
        s = [sm15(SIGN[j] * row[f"dq_{j+1}"] * STEPS_PER_DEG) for j in range(6)]
        l = [load_raw(j, row[f"load_{j+1}"]) for j in range(6)]
        acc = [int(round(row[f"acc_{a}"] * 4096)) for a in "xyz"]
        gyr = [int(round(row[f"gyro_{a}"] * 16.4)) for a in "xyz"]
        return p, s, l, acc, gyr

    def sample(self, row):   # TELEM sample, 77 bytes
        p, s, l, acc, gyr = self.raw_state(row)
        cmd = [pos(j, row[f"q_cmd_{j+1}"]) for j in range(6)]
        ref = [pos(j, row[f"q_plan_{j+1}"]) for j in range(6)]
        return struct.pack("<I6H6H6H6H6H3h3hB", int(row["t"] * 1e6), *cmd, *ref, *p, *s, *l, *acc, *gyr, 1)

    def stream_packet(self):   # STREAM, 73 bytes
        row = self.idle_row()
        p, s, l, acc, gyr = self.raw_state(row)
        return struct.pack("<BIBB6H6H6H6B6B6B3h3h", 0x88, int((time.monotonic() - self.t0) * 1000) & 0xFFFFFFFF,
                           self.state, 1, *p, *s, *l, *([27] * 6), *([76, 76, 76, 66, 64, 64]), *([0] * 6), *acc, *gyr)


class Protocol(asyncio.DatagramProtocol):
    def __init__(self, rp):
        self.rp = rp

    def connection_made(self, transport):
        self.tx = transport

    def send(self, data, addr):
        self.tx.sendto(data, addr)

    def ack(self, typ, status, addr, value=0):
        self.send(struct.pack("<BBbI", 0x83, typ, status, value), addr)

    def datagram_received(self, b, addr):
        rp, c = self.rp, b[0]
        if c == 0x01:     # PING: version 4.1.0
            self.send(struct.pack("<BHBIHBBBB", 0x81, 4, rp.state, rp.plan_samples, 50, 1, 1, 1, 0), addr)
        elif c == 0x02 and rp.state != PLAYING:
            p, s, l, acc, gyr = rp.raw_state(rp.idle_row())
            self.send(struct.pack("<BB6H6H6H3h3h", 0x82, 1, *p, *s, *l, *acc, *gyr), addr)
        elif c == 0x03:
            if rp.state == PLAYING:
                self.ack(0x03, -1, addr)
            else:
                rp.state = READY if rp.plan_samples else HOLDING
                self.ack(0x03, 0, addr)
        elif c == 0x04:
            rp.plan_samples = struct.unpack_from("<I", b, 1)[0]
            self.ack(0x04, 0, addr)
        elif c == 0x05:
            self.ack(0x05, 0, addr, struct.unpack_from("<I", b, 1)[0])
        elif c == 0x06:
            rp.state = READY
            self.ack(0x06, 0, addr)
        elif c in (0x07, 0x0B):
            if rp.state == PLAYING:
                self.ack(c, -1, addr)
                return
            self.ack(c, 0, addr)
            rp.state = PLAYING
            rp.play_task = asyncio.ensure_future(self.play(addr))
        elif c == 0x08:
            if rp.play_task:
                rp.play_task.cancel()
            self.ack(0x08, 0, addr)
        elif c == 0x09 and len(b) >= 4:
            sid, a, n = b[1], b[2], b[3]
            regs = bytearray(128)
            regs[21:24] = bytes(rp.gains.get(sid, [0, 0, 0]))
            regs[62], regs[63] = (76 if sid <= 3 else 65), 28
            self.send(bytes([0x86, sid, a, n, 0]) + bytes(regs[a:a + n]), addr)
        elif c == 0x0A and len(b) >= 5:
            sid, a, n = b[1], b[2], b[3]
            ok = a >= 9 and a + n <= 80 and not (a <= 55 < a + n)
            if ok and a == 21 and n == 3:
                rp.gains[sid] = list(b[4:7])
            self.send(bytes([0x87, sid, a, 0 if ok else 0xFB, 0]), addr)
        elif c == 0x0C and len(b) >= 3:
            rate = struct.unpack_from("<H", b, 1)[0]
            if rate == 0:
                rp.subs.pop(addr, None)
            elif addr in rp.subs or len(rp.subs) < 4:
                rp.subs[addr] = (min(rate, 100), time.monotonic())

    async def play(self, addr):
        rp, seq, cycles = self.rp, 0, 0
        t_start = time.monotonic()
        result = 0
        try:
            k = 0
            while k < rp.n:
                batch = []
                while k < rp.n and rp.r["t"][k] <= time.monotonic() - t_start and len(batch) < 18:
                    batch.append(rp.sample(rp.r[k]))
                    k += 1
                if batch:
                    self.send(struct.pack("<BIB", 0x84, seq, len(batch)) + b"".join(batch), addr)
                    seq += len(batch)
                    cycles += len(batch)
                await asyncio.sleep(0.01)
        except asyncio.CancelledError:
            result = 2
        self.send(struct.pack("<BBIIIIBh", 0x85, result, cycles, 2000, 0, 0, 0, 0), addr)
        rp.state = READY if rp.plan_samples else HOLDING


async def streamer(proto, rp):
    while True:
        now = time.monotonic()
        for addr, (rate, seen) in list(rp.subs.items()):
            if now - seen > 2.0:
                rp.subs.pop(addr, None)
        pkt = None
        for addr, (rate, seen) in rp.subs.items():
            due = getattr(streamer, "due", {}).get(addr, 0)
            if now >= due:
                pkt = pkt or rp.stream_packet()
                proto.send(pkt, addr)
                streamer.due = getattr(streamer, "due", {})
                streamer.due[addr] = now + 1.0 / rate
        await asyncio.sleep(0.005)


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recording")
    ap.add_argument("--host", default="127.0.0.1")   # or the Tailscale address; never the home network
    ap.add_argument("--port", type=int, default=5106)
    a = ap.parse_args()
    rp = Replay(load(a.recording))
    loop = asyncio.get_running_loop()
    transport, proto = await loop.create_datagram_endpoint(lambda: Protocol(rp), local_addr=(a.host, a.port))
    print(f"ATOM replay (protocol 4.1) on {a.host}:{a.port}: {rp.n} samples, {rp.r['t'][-1]:.1f} s")
    await streamer(proto, rp)


if __name__ == "__main__":
    asyncio.run(main())
