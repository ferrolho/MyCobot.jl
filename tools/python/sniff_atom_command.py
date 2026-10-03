"""
Send one ATOM command and print every byte that comes back, split into ATOM
frames and Feetech servo packets. This shows what the ATOM does on the servo bus.

Usage:
    python sniff_atom_command.py 0x20            # get_angles
    python sniff_atom_command.py 0x23            # get_coords
    python sniff_atom_command.py 0x53 1 56       # read servo 1 register 56

Only send commands that are safe to run; this script doesn't check.
"""

import sys
import time

from mycobot_bus import atom_frame, open_port


def capture(sp, cmd, payload, duration):
    sp.reset_input_buffer()
    sp.write(atom_frame(cmd, payload))
    sp.flush()
    t0 = time.perf_counter()
    buf = b""
    while time.perf_counter() - t0 < duration:
        buf += sp.read(sp.in_waiting or 1)
    return buf


def split(buf):
    i = 0
    while i < len(buf):
        if buf[i:i + 2] == b"\xff\xff" and i + 3 < len(buf):
            n = buf[i + 3]
            yield "FEETECH", buf[i:i + 4 + n]
            i += 4 + n
        elif buf[i:i + 2] == b"\xfe\xfe" and i + 2 < len(buf):
            n = buf[i + 2]
            yield "ATOM", buf[i:i + 3 + n]
            i += 3 + n
        else:
            yield "?", buf[i:i + 1]
            i += 1


if __name__ == "__main__":
    cmd = int(sys.argv[1], 0)
    payload = [int(x, 0) for x in sys.argv[2:]]
    sp = open_port()
    buf = capture(sp, cmd, payload, duration=0.1)
    print(f"{len(buf)} bytes")
    for kind, chunk in split(buf):
        print(f"  {kind:8s} {chunk.hex(' ')}")
    sp.close()
