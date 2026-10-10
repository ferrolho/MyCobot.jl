"""
Test of end-effector JOG (controller firmware 5.1, JOG frames 1 and 2) over the WebSocket API, on the
simulated ATOM or the real one. The TCP pose comes from the goals in the STREAM and the firmware's own FK
(twist.h, through tools/twist_lib.cpp, as tools/atom_sim.py builds it).
MOVES THE ROBOT (on the real ATOM): first a MOVE_TO the ready pose (J3 = −90°, J1, J2, J4-J6 0°: the tool
down; J7 keeps its goal), then the TCP moves 40 mm along base +x and back, 10 mm along the tool z axis and
back, and turns 10° about the vertical and back.

    python3 tools/firmware-tests/twist_ws_test.py [--atom 127.0.0.1:8281]   # the simulator
    ~/venvs/mycobot/bin/python tools/firmware-tests/twist_ws_test.py --atom mycobot.local
"""
import argparse
import asyncio
import math
import os
import struct
import sys
import time

import websockets

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import atom_sim  # noqa: E402  (twist_lib, floats, robot_params)

SIGN = atom_sim.SIGN
deg = lambda j, p: SIGN[j] * (p - 2048) * 360 / 4096
L = atom_sim.twist_lib()


def stream(m):
    n = m[8]
    return m[5], n, [deg(j, v) for j, v in enumerate(struct.unpack_from(f"<{n}H", m, 9 + 2 * n))]   # state, n, goals


def tcp(goals, n):
    p, R = atom_sim.floats([0.0] * 3), atom_sim.floats([0.0] * 9)
    L.twist_fk(atom_sim.floats(goals[:6]), atom_sim.floats(atom_sim.GRIPPER_TCP if n > 6 else [0.0] * 3), p, R)
    return list(p), list(R)


def rot_deg(A, B):   # angle between two rotations (row-major)
    tr = sum(A[3 * i + k] * B[3 * i + k] for i in range(3) for k in range(3))   # trace(A Bᵀ)
    return math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1) / 2))))


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    check.failures += not cond
check.failures = 0


def ee_jog(frame, lin=(0, 0, 0), ang_deg=(0, 0, 0), j7=0.0, vmax=45.0):
    return struct.pack("<BB7hH", 0x0F, frame, *[round(x * 10) for x in lin], *[round(x * 10) for x in ang_deg], round(j7 * 10), round(vmax * 10))


async def main(atom):
    url = atom if atom.startswith("ws") else f"ws://{atom}/ws"
    async with websockets.connect(url) as ws:
        last = {}
        acks = []

        async def reader():
            async for m in ws:
                if isinstance(m, bytes) and m[0] == 0x88:
                    last["s"] = stream(m)
                elif isinstance(m, bytes) and m[0] == 0x83:
                    acks.append(struct.unpack_from("<Bb", m, 1))
        async def renew():   # a subscription ends 2 s after the last SUBSCRIBE; control 2 s after the last message
            while True:
                await ws.send(bytes([0x0C, 50, 0]))   # SUBSCRIBE 50 Hz
                await ws.send(bytes([0x0D, 1]))       # CONTROL take
                await asyncio.sleep(0.5)
        task = asyncio.create_task(reader())
        renewer = asyncio.create_task(renew())
        await asyncio.sleep(0.5)

        async def send_for(msg, seconds):
            t_end = time.monotonic() + seconds
            while time.monotonic() < t_end:
                await ws.send(msg)
                await asyncio.sleep(0.02)

        await ws.send(struct.pack("<B6hH", 0x0E, 0, 0, -9000, 0, 0, 0, 0))   # MOVE_TO the ready pose (shortest time)
        await asyncio.sleep(0.3)
        while last["s"][0] == 6:   # moving
            await asyncio.sleep(0.1)
        await asyncio.sleep(0.5)
        state, n, g0 = last["s"]
        p0, R0 = tcp(g0, n)
        print(f"start: state {state}, {n} joints, TCP {[round(x, 1) for x in p0]} mm")
        await send_for(ee_jog(1, lin=(20, 0, 0)), 2.0)
        check(last["s"][0] == 7, "end-effector JOG: state jogging")
        await send_for(ee_jog(1), 0.6)
        _, _, g1 = last["s"]
        p1, R1 = tcp(g1, n)
        d = [a - b for a, b in zip(p1, p0)]
        check(abs(d[0] - 40) < 1.5 and math.hypot(d[1], d[2]) < 0.5, f"base +x 20 mm/s for 2 s: {[round(x, 2) for x in d]} mm (40, 0, 0 wanted)")
        check(rot_deg(R1, R0) < 0.2, f"the tool keeps its orientation: {rot_deg(R1, R0):.3f}°")
        await ws.send(bytes([0x10]) + struct.pack(f"<{n}hH", *[round(x * 100) for x in g1], 300))   # TRACK the same pose
        await asyncio.sleep(0.1)
        check(last["s"][0] == 8, "TRACK switches the running JOG to tracking (5.1)")
        await send_for(ee_jog(1, lin=(-20, 0, 0)), 2.0)
        check(last["s"][0] == 7, "end-effector JOG switches back to jogging")
        await send_for(ee_jog(2, lin=(0, 0, 10)), 1.0)
        await send_for(ee_jog(2), 0.5)
        _, _, g2 = last["s"]
        p2, _ = tcp(g2, n)
        z = [R1[2], R1[5], R1[8]]
        along = sum((a - b) * c for a, b, c in zip(p2, p0, z))
        check(abs(along - 10) < 1, f"tool frame +z 10 mm/s for 1 s: {along:.2f} mm along the approach axis (10 wanted)")
        await send_for(ee_jog(2, lin=(0, 0, -10)), 1.0)
        await send_for(ee_jog(1, ang_deg=(0, 0, 10)), 1.0)
        await send_for(ee_jog(1), 0.5)
        _, _, g3 = last["s"]
        p3, R3 = tcp(g3, n)
        check(abs(rot_deg(R3, R0) - 10) < 0.5 and math.dist(p3, p0) < 1, f"turn 10°/s for 1 s about the TCP: {rot_deg(R3, R0):.2f}°, TCP {math.dist(p3, p0):.2f} mm from the start")
        await send_for(ee_jog(1, ang_deg=(0, 0, -10)), 1.0)
        await send_for(ee_jog(1), 0.3)   # zero first: the deadman keeps the last twist for 200 ms
        acks.clear()
        await ws.send(struct.pack("<BB6h", 0x0F, 1, 0, 0, 0, 0, 0, 0))   # 14 bytes: frame 1 needs 18
        await asyncio.sleep(0.2)
        check((0x0F, -1) in acks, "a frame-1 JOG of the wrong length is refused (-1)")
        await asyncio.sleep(0.6)   # no more messages: the deadman stops the run
        check(last["s"][0] in (1, 2), f"deadman: the robot holds (state {last['s'][0]})")
        _, _, g4 = last["s"]
        p4, R4 = tcp(g4, n)
        check(math.dist(p4, p0) < 1.5 and rot_deg(R4, R0) < 0.3, f"back at the start: {math.dist(p4, p0):.2f} mm, {rot_deg(R4, R0):.2f}°")
        renewer.cancel()
        await ws.send(bytes([0x0D, 0]))
        task.cancel()
    print("all passed" if not check.failures else f"{check.failures} failed")
    return check.failures


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--atom", default="127.0.0.1:8281")
    sys.exit(asyncio.run(main(ap.parse_args().atom)))
