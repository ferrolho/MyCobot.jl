# Direct access to the Feetech STS servo bus over the robot's serial port.
# Documentation: website/src/content/docs/comms/servo-bus.mdx and website/src/content/docs/reference/gotchas.md.
#
# Packets:  request FF FF <ID> <LEN = n_params + 2> <INSTR> <params...> <CHK>
#           reply   FF FF <ID> <LEN> <ERROR> <data...> <CHK>
#           CHK = ~(ID + LEN + INSTR/ERROR + params/data) & 0xFF, multi-byte values little-endian.
#
# All I/O goes through three transport functions (`transport_write`, `transport_read`,
# `transport_discard_input`), implemented here for `LibSerialPort.SerialPort` and by a
# simulated servo bus in the tests.

import LibSerialPort

module Feetech
    const PING = 0x01
    const READ = 0x02
    const WRITE = 0x03
    const SYNC_READ = 0x82
    const SYNC_WRITE = 0x83
    const BROADCAST_ID = 0xFE

    # Register addresses (STS memory map, see website/src/content/docs/reference/registers.md)
    const REG_OFFSET = 31            # 2 bytes, sign-magnitude (bit 11)
    const REG_MODE = 33              # 0 position, 1 velocity, 2 PWM, 3 step
    const REG_TORQUE_ENABLE = 40
    const REG_ACCELERATION = 41      # the ATOM uses 50, 0 = no ramp
    const REG_GOAL_POSITION = 42     # 2 bytes, same units as present position
    const REG_GOAL_SPEED = 46        # 2 bytes, steps/s; 0 = no motion in position mode
    const REG_PRESENT_POSITION = 56  # 2 bytes; then speed (2), load (2), voltage, temperature
end

const SERVO_IDS = UInt8.(1:6)

# A SYNC WRITE followed by another request within ~0.3 ms is lost by every servo (24 % of
# writes with no gap, 0.17 % at 0.3 ms, none at ≥ 0.5 ms; measured 2026-10-04). SYNC WRITE
# gets no reply, so nothing else would notice. Waiting for the bytes to drain also works
# but costs a USB frame (~1 ms more per cycle).
const SYNC_WRITE_GAP = 1e-3   # s
const JOINT_SIGN = (-1, -1, +1, -1, -1, -1)   # joint angle direction vs servo position
const STEPS_PER_DEG = 4096 / 360

# --- Transport -----------------------------------------------------------------

"""
    transport_write(io, bytes)

Write `bytes` to the bus and wait until they have been sent.
"""
function transport_write(sp::LibSerialPort.SerialPort, bytes::Vector{UInt8})
    LibSerialPort.sp_blocking_write(sp.ref, bytes, 100)
    return nothing
end

"""
    transport_read(io, n, timeout_ms)

Read up to `n` bytes, returning early once `n` have arrived.
"""
function transport_read(sp::LibSerialPort.SerialPort, n::Integer, timeout_ms::Integer)
    nread, bytes = LibSerialPort.sp_blocking_read(sp.ref, n, timeout_ms)
    return bytes[1:nread]
end

"""
    transport_discard_input(io)

Drop any bytes waiting in the input buffer.
"""
transport_discard_input(sp::LibSerialPort.SerialPort) = (LibSerialPort.sp_flush(sp, LibSerialPort.SP_BUF_INPUT); nothing)

# --- Packets -------------------------------------------------------------------

ft_checksum(body) = ~UInt8(sum(Int, body) & 0xFF)

"""
    ft_packet(id, instruction, params=UInt8[])

Build a Feetech request packet.
"""
function ft_packet(id::Integer, instruction::Integer, params::AbstractVector{<:Integer}=UInt8[])
    body = UInt8[id, length(params) + 2, instruction, params...]
    return UInt8[0xFF, 0xFF, body..., ft_checksum(body)]
end

"""
    parse_status_packets(buf)

Split a byte stream into Feetech status packets with valid checksums.
Returns a vector of `(id, error, data)` tuples.
"""
function parse_status_packets(buf::AbstractVector{UInt8})
    out = Tuple{UInt8,UInt8,Vector{UInt8}}[]
    i = 1
    while i + 5 <= length(buf)
        if buf[i] == 0xFF && buf[i+1] == 0xFF
            n = Int(buf[i+3])
            last = i + 3 + n
            if last <= length(buf) && ft_checksum(@view buf[i+2:last-1]) == buf[last]
                push!(out, (buf[i+2], buf[i+4], buf[i+5:last-1]))
                i = last + 1
                continue
            end
        end
        i += 1
    end
    return out
end

le16(v::Integer) = UInt8[v & 0xFF, (v >> 8) & 0xFF]
u16(lo::UInt8, hi::UInt8) = Int(lo) | Int(hi) << 8
decode_signed15(v::Integer) = (v = Int(v); v & 0x8000 != 0 ? -(v & 0x7FFF) : v)
decode_load(v::Integer) = (v = Int(v); (v & 0x400 != 0 ? -(v & 0x3FF) : v & 0x3FF) / 10)   # percent

# --- Requests ------------------------------------------------------------------

function ft_request(io, packet::Vector{UInt8}, n_reply::Integer; timeout_ms::Integer=20)
    transport_discard_input(io)
    transport_write(io, packet)
    return n_reply == 0 ? UInt8[] : transport_read(io, n_reply, timeout_ms)
end

"""
    ft_read(io, id, address, length)

READ `length` bytes from one servo. Throws if it doesn't reply.
"""
function ft_read(io, id::Integer, address::Integer, length::Integer)
    pkts = parse_status_packets(ft_request(io, ft_packet(id, Feetech.READ, [address, length]), 6 + length))
    isempty(pkts) && error("no reply from servo $id")
    return pkts[1][3]
end

"""
    ft_write(io, id, address, data)

WRITE `data` starting at `address` on one servo. Returns the servo's error byte (0 = ok).
"""
function ft_write(io, id::Integer, address::Integer, data::AbstractVector{<:Integer})
    pkts = parse_status_packets(ft_request(io, ft_packet(id, Feetech.WRITE, [address, data...]), 6))
    isempty(pkts) && error("no ack from servo $id")
    return pkts[1][2]
end

"""
    ft_sync_read(io, address, length; ids=SERVO_IDS)

SYNC READ: one request, one reply per servo. Returns `Dict(id => data)` for the
servos that replied.
"""
function ft_sync_read(io, address::Integer, length::Integer; ids=SERVO_IDS)
    packet = ft_packet(Feetech.BROADCAST_ID, Feetech.SYNC_READ, [address, length, ids...])
    buf = ft_request(io, packet, Base.length(ids) * (6 + length))
    return Dict(id => data for (id, _, data) in parse_status_packets(buf))
end

"""
    ft_sync_write(io, address, data_by_id; gap=SYNC_WRITE_GAP)

SYNC WRITE the same register range on several servos (`id => bytes` pairs). No replies.
Waits `gap` seconds afterwards, because a request sent sooner makes the servos drop the write.
"""
function ft_sync_write(io, address::Integer, data_by_id; gap::Real=SYNC_WRITE_GAP)
    pairs = collect(data_by_id)
    n = length(last(first(pairs)))
    params = UInt8[address, n]
    for (id, data) in pairs
        length(data) == n || throw(ArgumentError("all servos need the same number of bytes"))
        append!(params, UInt8(id), data)
    end
    ft_request(io, ft_packet(Feetech.BROADCAST_ID, Feetech.SYNC_WRITE, params), 0)
    t = time()
    while time() - t < gap end
    return nothing
end

# --- Joint-level helpers ---------------------------------------------------------

"""
    angle_to_position(j, deg)

Servo position (0–4095) for joint `j` at `deg` degrees (ATOM angle convention; 0° = 2048).
"""
angle_to_position(j::Integer, deg::Real) = clamp(round(Int, 2048 + JOINT_SIGN[j] * deg * STEPS_PER_DEG), 0, 4095)

"""
    position_to_angle(j, pos)

Joint angle in degrees for servo position `pos` of joint `j`.
"""
position_to_angle(j::Integer, pos::Integer) = JOINT_SIGN[j] * (pos - 2048) / STEPS_PER_DEG

"""
    read_state(io)

SYNC READ position, speed and load of all six servos (~2 ms). Returns a NamedTuple
`(q, dq, load, ok)` with angles in degrees, speeds in °/s, loads in percent; `ok` is
false if any servo didn't reply (its entries are then `NaN`).
"""
function read_state(io)
    data = ft_sync_read(io, Feetech.REG_PRESENT_POSITION, 6)
    q, dq, load = fill(NaN, 6), fill(NaN, 6), fill(NaN, 6)
    for j in 1:6
        d = get(data, SERVO_IDS[j], nothing)
        (d === nothing || length(d) != 6) && continue
        q[j] = position_to_angle(j, u16(d[1], d[2]))
        dq[j] = JOINT_SIGN[j] * decode_signed15(u16(d[3], d[4])) / STEPS_PER_DEG
        load[j] = JOINT_SIGN[j] * decode_load(u16(d[5], d[6]))
    end
    return (q=q, dq=dq, load=load, ok=!any(isnan, q))
end

"""
    write_goals(io, q_deg)

SYNC WRITE goal positions for all six joints (degrees). Servos only move if their
goal speed is nonzero (see `enable_motion`).
"""
write_goals(io, q_deg::AbstractVector{<:Real}) =
    ft_sync_write(io, Feetech.REG_GOAL_POSITION, (SERVO_IDS[j] => le16(angle_to_position(j, q_deg[j])) for j in 1:6))

"""
    hold_position(io)

Set every goal to the servo's present position, so nothing moves even with a nonzero speed.
"""
function hold_position(io)
    data = ft_sync_read(io, Feetech.REG_PRESENT_POSITION, 2)
    isempty(data) && error("no servo replied")
    ft_sync_write(io, Feetech.REG_GOAL_POSITION, data)
    return nothing
end

"""
    enable_motion(io; speed_cap=2000, acceleration=0)

Hold every joint where it is, then set acceleration and a nonzero goal-speed cap
(steps/s) so that new goals are followed.
"""
function enable_motion(io; speed_cap::Integer=2000, acceleration::Integer=0)
    hold_position(io)
    ft_sync_write(io, Feetech.REG_ACCELERATION, (id => UInt8[acceleration] for id in SERVO_IDS))
    ft_sync_write(io, Feetech.REG_GOAL_SPEED, (id => le16(speed_cap) for id in SERVO_IDS))
    return nothing
end

"""
    disable_motion(io)

Set goal speed back to 0 ("don't move" in position mode). Torque stays on.
"""
disable_motion(io) = ft_sync_write(io, Feetech.REG_GOAL_SPEED, (id => UInt8[0, 0] for id in SERVO_IDS))
