# Allocation-free bus I/O for control loops.
#
# `read_state` and `write_goals` (feetech.jl) build new packets, Dicts and arrays on every call:
# ~7 KB per cycle, which made the garbage collector stall the loop for 30–80 ms at times.
# `BusBuffers` holds every packet and result array, so `read_state!` and `write_goals!` allocate
# nothing on a `LibSerialPort.SerialPort` (checked by the tests and on the robot).

import LibSerialPort

"""
    open_bus(port=default_port(); baudrate=1_000_000) -> SerialPort

Open the servo bus port with raw settings: 8N1 and **no flow control**. On Linux,
`LibSerialPort.open` leaves XON/XOFF output flow control on, so a 0x13 byte in a servo
reply paused every write until its 100 ms timeout (2026-10-04 on the Raspberry Pi).
"""
function open_bus(port::AbstractString=default_port(); baudrate::Integer=1_000_000)
    sp = LibSerialPort.open(port, baudrate)
    LibSerialPort.set_flow_control(sp)          # all flow control off (XON/XOFF, RTS/CTS, DTR/DSR)
    return sp
end

const N_JOINTS = 6
const STATE_BYTES = 6                                # present position, speed, load (2 bytes each)
const STATE_REPLY = 6 + STATE_BYTES                  # FF FF ID LEN ERR data... CHK
const GOAL_PACKET = 7 + 3 * N_JOINTS + 1             # FF FF FE LEN 83 addr n [id lo hi]x6 CHK

"""
    BusBuffers()

Preallocated packets and results for `read_state!` and `write_goals!`. After `read_state!`,
the fields `q` (°), `dq` (°/s), `load` (%) and `ok` hold the state of the six joints.
"""
mutable struct BusBuffers
    read_req::Vector{UInt8}
    rx::Vector{UInt8}
    tx::Vector{UInt8}
    q::Vector{Float64}
    dq::Vector{Float64}
    load::Vector{Float64}
    seen::Vector{Bool}
    ok::Bool
end

function BusBuffers()
    read_req = ft_packet(Feetech.BROADCAST_ID, Feetech.SYNC_READ,
                         [Feetech.REG_PRESENT_POSITION, STATE_BYTES, SERVO_IDS...])
    tx = zeros(UInt8, GOAL_PACKET)
    tx[1:7] = [0xFF, 0xFF, Feetech.BROADCAST_ID, GOAL_PACKET - 4, Feetech.SYNC_WRITE, Feetech.REG_GOAL_POSITION, 2]
    for j in 1:N_JOINTS
        tx[8 + 3(j - 1)] = SERVO_IDS[j]
    end
    return BusBuffers(read_req, zeros(UInt8, N_JOINTS * STATE_REPLY), tx,
                      fill(NaN, N_JOINTS), fill(NaN, N_JOINTS), fill(NaN, N_JOINTS), falses(N_JOINTS), false)
end

"""
    transport_read!(io, buf, n, timeout_ms) -> nread

Read up to `n` bytes into `buf`. Allocation-free on a serial port; other transports
(the simulated bus) fall back to `transport_read`.
"""
function transport_read!(sp::LibSerialPort.SerialPort, buf::Vector{UInt8}, n::Integer, timeout_ms::Integer)
    # Direct ccall: the LibSerialPort wrapper boxes its return value (16 bytes per call).
    ret = ccall((:sp_blocking_read, LibSerialPort.Lib.libserialport), Cint,
                (LibSerialPort.Lib.Port, Ptr{UInt8}, Csize_t, Cuint), sp.ref, buf, min(n, length(buf)), timeout_ms)
    ret < 0 && error("sp_blocking_read failed ($ret)")
    return Int(ret)
end

function transport_read!(io, buf::Vector{UInt8}, n::Integer, timeout_ms::Integer)
    bytes = transport_read(io, n, timeout_ms)
    copyto!(buf, bytes)
    return length(bytes)
end

@inline function sum_bytes(buf, from, to)
    s = 0
    @inbounds for k in from:to
        s += buf[k]
    end
    return s
end

"""
    read_state!(b::BusBuffers, io) -> ok

SYNC READ position, speed and load of the six servos into `b.q`, `b.dq` and `b.load`.
Joints that did not reply get `NaN`. Returns `b.ok` (true if all six replied).
"""
function read_state!(b::BusBuffers, io)
    transport_discard_input(io)
    transport_write(io, b.read_req)
    n = transport_read!(io, b.rx, length(b.rx), 20)
    fill!(b.seen, false)
    rx = b.rx
    i = 1
    @inbounds while i + STATE_REPLY - 1 <= n
        if rx[i] == 0xFF && rx[i+1] == 0xFF && rx[i+3] == STATE_BYTES + 2
            last = i + STATE_REPLY - 1
            j = Int(rx[i+2])
            if 1 <= j <= N_JOINTS && ~UInt8(sum_bytes(rx, i + 2, last - 1) & 0xFF) == rx[last]
                d = i + 5
                b.q[j] = position_to_angle(j, u16(rx[d], rx[d+1]))
                b.dq[j] = JOINT_SIGN[j] * decode_signed15(u16(rx[d+2], rx[d+3])) / STEPS_PER_DEG
                b.load[j] = JOINT_SIGN[j] * decode_load(u16(rx[d+4], rx[d+5]))
                b.seen[j] = true
                i = last + 1
                continue
            end
        end
        i += 1
    end
    @inbounds for j in 1:N_JOINTS
        if !b.seen[j]
            b.q[j] = NaN; b.dq[j] = NaN; b.load[j] = NaN
        end
    end
    b.ok = all(b.seen)
    return b.ok
end

"""
    write_goals!(b::BusBuffers, io, q_deg; gap=SYNC_WRITE_GAP)

SYNC WRITE goal positions (degrees) for the six joints, then wait `gap` seconds
(see `SYNC_WRITE_GAP`). Allocation-free version of `write_goals`.
"""
function write_goals!(b::BusBuffers, io, q_deg::AbstractVector{<:Real}; gap::Real=SYNC_WRITE_GAP)
    tx = b.tx
    @inbounds for j in 1:N_JOINTS
        p = angle_to_position(j, q_deg[j])
        k = 8 + 3(j - 1)
        tx[k+1] = p & 0xFF
        tx[k+2] = (p >> 8) & 0xFF
    end
    @inbounds tx[end] = ~UInt8(sum_bytes(tx, 3, length(tx) - 1) & 0xFF)
    transport_write(io, tx)
    t = time()
    while time() - t < gap end
    return nothing
end

"""
    sample_trajectory!(out, t_plan, q_plan, t; lag=nothing)

In-place `sample_trajectory`: write the plan at time `t` (plus `lag[j]` per joint, if given)
into `out`.
"""
function sample_trajectory!(out::AbstractVector, t_plan::AbstractVector, q_plan::AbstractMatrix, t::Real;
                            lag::Union{Nothing,AbstractVector}=nothing)
    @inbounds for j in eachindex(out)
        tj = lag === nothing ? t : t + lag[j]
        if tj <= t_plan[1]
            out[j] = q_plan[1, j]
        elseif tj >= t_plan[end]
            out[j] = q_plan[end, j]
        else
            i = searchsortedlast(t_plan, tj)
            s = (tj - t_plan[i]) / (t_plan[i+1] - t_plan[i])
            out[j] = (1 - s) * q_plan[i, j] + s * q_plan[i+1, j]
        end
    end
    return out
end
