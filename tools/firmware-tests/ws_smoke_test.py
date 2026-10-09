"""
Smoke test of controller firmware 5.0 on the real ATOM: WebSocket transport, state stream (n joints,
the control byte), control between a UDP and a WebSocket client, JOG with the deadman, MOVE_TO, and J7
(the gripper): with it, 7-value MOVE_TO and J7's goal kept by a 6-value MOVE_TO and by HOLD; without
it, a 7-value MOVE_TO is refused (-7).
MOVES THE ROBOT: J6 jogs about +10° and back, then a MOVE_TO the current pose of J1-J5 with J6 at the
start. J7 does not move (its goals are the goal it holds).

    ~/venvs/mycobot/bin/python tools/firmware-tests/ws_smoke_test.py [--atom mycobot.local]
"""
import argparse
import asyncio
import socket
import struct
import time

import websockets

SIGN = (-1, -1, 1, -1, -1, -1, 1)
deg = lambda j, p: SIGN[j] * (p - 2048) * 360 / 4096


def joints(s):
    """STREAM (5.0): n, the angles and the goals (°)."""
    n = s[8]
    return (n, [deg(j, v) for j, v in enumerate(struct.unpack_from(f"<{n}H", s, 9))],
            [deg(j, v) for j, v in enumerate(struct.unpack_from(f"<{n}H", s, 9 + 2 * n))])


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        check.failures += 1
check.failures = 0


async def main(atom):
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("", 0)); udp.settimeout(1.0)
    addr = (socket.gethostbyname(atom), 5006)

    def udp_ack(msg, code):
        udp.sendto(msg, addr)
        t = time.time()
        while time.time() - t < 1.0:
            m = udp.recv(2000)
            if m[0] == 0x83 and m[1] == code:
                return struct.unpack("b", m[2:3])[0]
        return None

    async with websockets.connect(f"ws://{atom}/ws", max_size=None) as ws:
        frames = asyncio.Queue()

        async def reader():
            async for m in ws:
                await frames.put(m)
        rt = asyncio.ensure_future(reader())

        async def renew():                 # a subscription ends 2 s after the last SUBSCRIBE
            while True:
                await asyncio.sleep(0.5)
                await ws.send(b"\x0c" + struct.pack("<H", 20))

        async def wait_for(pred, timeout=2.0):
            end = time.time() + timeout
            while time.time() < end:
                try:
                    m = await asyncio.wait_for(frames.get(), end - time.time())
                except asyncio.TimeoutError:
                    break
                if pred(m):
                    return m
            return None

        m = None
        await ws.send(b"\x01")
        m = await wait_for(lambda m: isinstance(m, bytes) and m[0] == 0x81)
        check(m is not None and m[1:3] == b"\x05\x00", f"WS PING -> version {m[1] if m else '?'}.{m[12] if m else '?'}.{m[13] if m else '?'}")
        log = await wait_for(lambda m: isinstance(m, str), 3.0)
        check(log is not None and "v5." in log, "WS text frame: status log")

        await ws.send(b"\x0c" + struct.pack("<H", 20))
        rn = asyncio.ensure_future(renew())
        s = await wait_for(lambda m: isinstance(m, bytes) and m[0] == 0x88)
        check(s is not None and s[8] in (6, 7) and len(s) == 21 + 11 * s[8], f"STREAM over WS, {len(s) if s else 0} bytes, {s[8] if s else '?'} joints")
        check(s is not None and s[7] == 0, "control byte: nobody")

        check(udp_ack(b"\x0d\x01", 0x0D) == 0, "UDP client takes control")
        await ws.send(b"\x0c" + struct.pack("<H", 20))
        s = await wait_for(lambda m: isinstance(m, bytes) and m[0] == 0x88 and m[7] == 2)
        check(s is not None, "control byte: another client")
        await ws.send(b"\x0f\x00" + struct.pack("<6h", 0, 0, 0, 0, 0, 100))
        a = await wait_for(lambda m: isinstance(m, bytes) and m[0] == 0x83 and m[1] == 0x0F)
        check(a is not None and struct.unpack("b", a[2:3])[0] == -2, "WS JOG refused while UDP has control (-2)")
        check(udp_ack(b"\x0d\x00", 0x0D) == 0, "UDP releases control")

        await ws.send(b"\x0d\x01")
        a = await wait_for(lambda m: isinstance(m, bytes) and m[0] == 0x83 and m[1] == 0x0D)
        check(a is not None and a[2] == 0, "WS takes control")
        s = await wait_for(lambda m: isinstance(m, bytes) and m[0] == 0x88 and m[7] == 1)
        check(s is not None, "control byte: you")
        n, q0, g0 = joints(s)

        # JOG J6 at +10°/s for 1 s (a JOG every 50 ms), then stop sending: the deadman ramps down
        t = time.time()
        while time.time() - t < 1.0:
            await ws.send(b"\x0f\x00" + struct.pack("<6h", 0, 0, 0, 0, 0, 100))
            await asyncio.sleep(0.05)
        jog = await wait_for(lambda m: isinstance(m, bytes) and m[0] == 0x88 and m[5] == 7, 0.5)
        check(jog is not None, "state 7 (jogging) in the stream")
        held = await wait_for(lambda m: isinstance(m, bytes) and m[0] == 0x88 and m[5] in (1, 2), 2.0)
        check(held is not None, "deadman: back to holding after the JOGs stop")
        q1 = joints(held)[1] if held else q0
        check(6 < q1[5] - q0[5] < 14, f"J6 moved {q1[5] - q0[5]:.1f}° (expected ~10°)")
        check(max(abs(q1[j] - q0[j]) for j in range(5)) < 0.5, "J1-J5 did not move")

        # MOVE_TO: J1-J6 back to the goals they held at the start; J7 (if found) at the goal it holds
        if n == 6:
            await ws.send(b"\x0e" + struct.pack("<7hH", *[round(x * 100) for x in q0], -2000, 0))
            a = await wait_for(lambda m: isinstance(m, bytes) and m[0] == 0x83 and m[1] == 0x0E)
            check(a is not None and struct.unpack("b", a[2:3])[0] == -7, "no gripper: 7-value MOVE_TO refused (-7)")
        # the goals the robot held at the start (5.0: measured positions as goals would let the loaded joints sag)
        goal = [round(g0[j] * 100) for j in range(6)] + ([round(g0[6] * 100)] if n == 7 else [])
        await ws.send(b"\x0e" + struct.pack(f"<{n}hH", *goal, 0))
        a = await wait_for(lambda m: isinstance(m, bytes) and m[0] == 0x83 and m[1] == 0x0E)
        check(a is not None and a[2] == 0, "MOVE_TO accepted")
        d = await wait_for(lambda m: isinstance(m, bytes) and m[0] == 0x85, 6.0)
        check(d is not None and d[1] == 0, f"MOVE_TO DONE result {d[1] if d else '?'}")
        await asyncio.sleep(0.3)
        while not frames.empty():          # drop stream packets queued before the move ended
            frames.get_nowait()
        s = await wait_for(lambda m: isinstance(m, bytes) and m[0] == 0x88 and m[5] in (1, 2), 2.0)
        check(s is not None, "fresh STREAM after the move")
        q2 = joints(s)[1] if s else q1
        check(abs(q2[5] - q0[5]) < 1.0, f"J6 back at the start ({q2[5] - q0[5]:+.2f}°)")

        if n == 7:   # J7 keeps its goal through a 6-value MOVE_TO and a HOLD (a grasp stays closed)
            await ws.send(b"\x0e" + struct.pack("<6hH", *goal[:6], 0))
            d = await wait_for(lambda m: isinstance(m, bytes) and m[0] == 0x85, 6.0)
            await ws.send(b"\x03")
            await asyncio.sleep(0.5)
            while not frames.empty():
                frames.get_nowait()
            s = await wait_for(lambda m: isinstance(m, bytes) and m[0] == 0x88 and m[5] in (1, 2), 2.0)
            g3 = joints(s)[2] if s else g0
            check(d is not None and d[1] == 0 and abs(g3[6] - g0[6]) < 0.2, f"J7 goal kept: {g0[6]:.1f}° -> {g3[6]:.1f}°")

        await ws.send(b"\x0d\x00")
        await ws.send(b"\x0c\x00\x00")
        rn.cancel()
        rt.cancel()
    print("all checks passed" if not check.failures else f"{check.failures} FAILURES")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--atom", default="mycobot.local")
    asyncio.run(main(ap.parse_args().atom))
