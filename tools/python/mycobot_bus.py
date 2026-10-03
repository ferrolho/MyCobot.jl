"""
Helpers for talking to the myCobot 280 (for Arduino) over its serial port, both
through the ATOM firmware protocol (FE FE ... FA frames) and directly to the
Feetech STS servos on the shared servo bus (FF FF ... checksum packets).

See docs/fast-communication.md for the background.
"""

import time

import serial

PORT = "/dev/tty.usbserial-B00033ZX"
BAUDRATE = 1_000_000
SERVO_IDS = [1, 2, 3, 4, 5, 6]


def open_port(port=PORT, baudrate=BAUDRATE, timeout=0.005):
    sp = serial.Serial(port, baudrate, timeout=timeout)
    time.sleep(0.3)
    sp.reset_input_buffer()
    return sp


# --- ATOM protocol (FE FE LEN CMD DATA... FA) ---------------------------------

def atom_frame(cmd, payload=()):
    return bytes([0xFE, 0xFE, len(payload) + 2, cmd, *payload, 0xFA])


def find_atom_frame(buf, cmd):
    """Return the first complete ATOM frame for `cmd` in `buf`, or None.

    Every candidate header is checked, because Feetech packets from the servo bus
    are mixed into the same byte stream and their checksum byte can be 0xFE.
    """
    for i in range(len(buf) - 4):
        n = buf[i + 2]
        if (buf[i] == buf[i + 1] == 0xFE and 2 <= n <= 20 and len(buf) >= i + 3 + n
                and buf[i + 2 + n] == 0xFA and buf[i + 3] == cmd):
            return buf[i:i + 3 + n]
    return None


def atom_xfer(sp, cmd, payload=(), timeout=0.1):
    """Send an ATOM command and return its reply frame (or None on timeout)."""
    sp.reset_input_buffer()
    sp.write(atom_frame(cmd, payload))
    sp.flush()
    t0 = time.perf_counter()
    buf = b""
    while time.perf_counter() - t0 < timeout:
        buf += sp.read(sp.in_waiting or 1)
        frame = find_atom_frame(buf, cmd)
        if frame is not None:
            return frame
    return None


def atom_get_angles(sp):
    frame = atom_xfer(sp, 0x20)
    data = frame[4:-1]
    return [int.from_bytes(data[i:i + 2], "big", signed=True) / 100 for i in range(0, 12, 2)]


def atom_read_servo_register(sp, servo_id, address):
    """Read one byte of a servo register through the ATOM (GET_SERVO_DATA 0x53)."""
    frame = atom_xfer(sp, 0x53, [servo_id, address])
    if frame is None or len(frame) < 6:
        raise IOError(f"no reply reading servo {servo_id} register {address}")
    return frame[4]


def atom_write_servo_register(sp, servo_id, address, value):
    """Write one byte of a servo register through the ATOM (SET_SERVO_DATA 0x52)."""
    return atom_xfer(sp, 0x52, [servo_id, address, value])


# --- Feetech STS protocol, direct on the servo bus -----------------------------

FT_PING, FT_READ, FT_WRITE, FT_SYNC_READ, FT_SYNC_WRITE = 0x01, 0x02, 0x03, 0x82, 0x83
BROADCAST_ID = 0xFE

# Register addresses (Feetech STS memory map, see docs/servo-registers.md)
REG_OFFSET = 31          # 2 bytes, calibration offset
REG_MODE = 33            # 0 position, 1 velocity, 2 PWM, 3 step
REG_TORQUE_ENABLE = 40
REG_GOAL_POSITION = 42   # 2 bytes, same units as present position (position mode)
REG_GOAL_SPEED = 46      # 2 bytes, steps/s, bit 15 = direction; 0 = no motion in position mode
REG_LOCK = 55
REG_PRESENT_POSITION = 56


def ft_checksum(body):
    return (~sum(body)) & 0xFF


def ft_packet(servo_id, instruction, params=()):
    body = [servo_id, len(params) + 2, instruction, *params]
    return bytes([0xFF, 0xFF, *body, ft_checksum(body)])


def ft_txrx(sp, packet, n_reply_bytes, timeout=0.02):
    """Send a Feetech packet and read exactly `n_reply_bytes` (our own TX is not echoed)."""
    sp.reset_input_buffer()
    sp.write(packet)
    sp.flush()
    t0 = time.perf_counter()
    buf = b""
    while len(buf) < n_reply_bytes and time.perf_counter() - t0 < timeout:
        buf += sp.read(sp.in_waiting or 1)
    return buf


def parse_status_packets(buf):
    """Split a byte stream into Feetech status packets: list of (id, error, params)."""
    out = []
    i = 0
    while i + 5 < len(buf):
        if buf[i] == buf[i + 1] == 0xFF:
            n = buf[i + 3]
            pkt = buf[i:i + 4 + n]
            if len(pkt) == 4 + n and ft_checksum(pkt[2:-1]) == pkt[-1]:
                out.append((pkt[2], pkt[4], pkt[5:-1]))
                i += 4 + n
                continue
        i += 1
    return out


def ft_read(sp, servo_id, address, length):
    buf = ft_txrx(sp, ft_packet(servo_id, FT_READ, [address, length]), 6 + length)
    pkts = parse_status_packets(buf)
    if not pkts:
        raise IOError(f"no reply from servo {servo_id}: {buf.hex(' ')}")
    return pkts[0][2]


def ft_write(sp, servo_id, address, data):
    """WRITE bytes starting at `address`; returns the servo's error byte (0 = ok)."""
    buf = ft_txrx(sp, ft_packet(servo_id, FT_WRITE, [address, *data]), 6)
    pkts = parse_status_packets(buf)
    if not pkts:
        raise IOError(f"no ack from servo {servo_id}: {buf.hex(' ')}")
    return pkts[0][1]


def ft_sync_write(sp, address, data_by_id):
    """SYNC WRITE the same register range on several servos. No replies are sent."""
    length = len(next(iter(data_by_id.values())))
    params = [address, length]
    for sid, data in data_by_id.items():
        params += [sid, *data]
    sp.write(ft_packet(BROADCAST_ID, FT_SYNC_WRITE, params))
    sp.flush()


def ft_sync_read(sp, address, length, ids=SERVO_IDS):
    """One request, one reply per servo. Returns {id: params bytes}."""
    buf = ft_txrx(sp, ft_packet(BROADCAST_ID, FT_SYNC_READ, [address, length, *ids]),
                  len(ids) * (6 + length))
    return {sid: params for sid, _, params in parse_status_packets(buf)}


def decode_signed15(value):
    """Feetech sign-magnitude encoding (bit 15 = negative) used for speed."""
    return -(value & 0x7FFF) if value & 0x8000 else value


def read_state(sp, ids=SERVO_IDS):
    """Sync-read position, speed, load, voltage, temperature of all servos (~2.75 ms)."""
    raw = ft_sync_read(sp, REG_PRESENT_POSITION, 15, ids)
    state = {}
    for sid, p in raw.items():
        state[sid] = dict(
            position=p[0] | p[1] << 8,
            speed=decode_signed15(p[2] | p[3] << 8),
            load=p[4] | p[5] << 8,          # bit 10 = direction, 0.1 % units
            voltage=p[6] / 10,
            temperature=p[7],
        )
    return state
