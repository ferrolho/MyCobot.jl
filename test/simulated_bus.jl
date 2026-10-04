# A simulated Feetech STS servo bus for testing without the robot. It implements the
# MyCobot transport functions, parses request packets, keeps a register map per servo,
# replies like the real servos, and moves each servo toward its goal position at its
# goal speed (goal speed 0 = no motion, as observed on the robot).

import MyCobot

const SIM_IDS = 1:6

mutable struct SimulatedBus
    regs::Dict{Int,Vector{UInt8}}   # register memory per servo (index = address + 1)
    outbox::Vector{UInt8}           # bytes waiting to be read by the host
    received::Vector{Vector{UInt8}} # every packet the host sent
    stuck::Set{Int}                 # servo ids that never move
    position::Dict{Int,Float64}     # exact simulated position (steps)
    last_update::Float64
end

function SimulatedBus(; positions=Dict(id => 2048 for id in SIM_IDS), stuck=Set{Int}())
    regs = Dict{Int,Vector{UInt8}}()
    for id in SIM_IDS
        r = zeros(UInt8, 128)
        r[5+1] = id                  # ID
        r[40+1] = 1                  # torque enabled
        r[55+1] = 1                  # EEPROM lock
        regs[id] = r
    end
    bus = SimulatedBus(regs, UInt8[], Vector{UInt8}[], stuck, Dict(id => Float64(positions[id]) for id in SIM_IDS), time())
    for id in SIM_IDS
        set16!(bus, id, 56, positions[id])
    end
    return bus
end

get16(bus, id, addr) = Int(bus.regs[id][addr+1]) | Int(bus.regs[id][addr+2]) << 8
set16!(bus, id, addr, v) = (bus.regs[id][addr+1] = UInt8(v & 0xFF); bus.regs[id][addr+2] = UInt8((v >> 8) & 0xFF))

function advance!(bus::SimulatedBus)
    now = time()
    dt = now - bus.last_update
    bus.last_update = now
    for id in keys(bus.regs)
        speed = get16(bus, id, 46) & 0x7FFF
        moving = !(id in bus.stuck) && speed > 0 && bus.regs[id][40+1] == 1 && bus.regs[id][33+1] == 0
        p = bus.position[id]
        if moving
            goal = get16(bus, id, 42)
            step = clamp(goal - p, -speed * dt, speed * dt)
            p += step
            v = round(Int, step / max(dt, 1e-6))
        else
            v = 0
        end
        bus.position[id] = p
        set16!(bus, id, 56, round(Int, p))
        set16!(bus, id, 58, v < 0 ? (-v) | 0x8000 : v)
    end
end

function status_packet(id, data)
    body = UInt8[id, length(data) + 2, 0x00, data...]
    return UInt8[0xFF, 0xFF, body..., MyCobot.ft_checksum(body)]
end

function write_registers!(bus, id, addr, data)
    bus.regs[id][addr+1:addr+length(data)] .= data
    addr <= 42 < addr + length(data) && (bus.regs[id][40+1] = 1)   # writing a goal turns torque on
    addr == 33 && (bus.regs[id][40+1] = 0)                          # writing the mode turns torque off
end

function MyCobot.transport_write(bus::SimulatedBus, bytes::Vector{UInt8})
    advance!(bus)
    push!(bus.received, copy(bytes))
    length(bytes) >= 6 && bytes[1] == 0xFF && bytes[2] == 0xFF || return nothing
    id, n, instr = Int(bytes[3]), Int(bytes[4]), bytes[5]
    params = bytes[6:4+n-1]
    MyCobot.ft_checksum(bytes[3:end-1]) == bytes[end] || return nothing
    if instr == MyCobot.Feetech.READ && haskey(bus.regs, id)
        addr, len = params
        append!(bus.outbox, status_packet(id, bus.regs[id][addr+1:addr+len]))
    elseif instr == MyCobot.Feetech.WRITE && haskey(bus.regs, id)
        write_registers!(bus, id, params[1], params[2:end])
        append!(bus.outbox, status_packet(id, UInt8[]))
    elseif instr == MyCobot.Feetech.SYNC_READ
        addr, len = params[1], params[2]
        for sid in params[3:end]
            haskey(bus.regs, sid) && append!(bus.outbox, status_packet(sid, bus.regs[sid][addr+1:addr+len]))
        end
    elseif instr == MyCobot.Feetech.SYNC_WRITE
        addr, len = params[1], params[2]
        for k in 3:(len+1):length(params)
            sid = params[k]
            haskey(bus.regs, sid) && write_registers!(bus, sid, addr, params[k+1:k+len])
        end
    end
    return nothing
end

function MyCobot.transport_read(bus::SimulatedBus, n::Integer, timeout_ms::Integer)
    advance!(bus)
    k = min(n, length(bus.outbox))
    out = bus.outbox[1:k]
    deleteat!(bus.outbox, 1:k)
    return out
end

MyCobot.transport_discard_input(bus::SimulatedBus) = (empty!(bus.outbox); nothing)

"A transport that only records what is written (to compare packets byte for byte)."
struct RecordingIO
    written::Vector{Vector{UInt8}}
end
RecordingIO() = RecordingIO(Vector{UInt8}[])
MyCobot.transport_write(io::RecordingIO, bytes::Vector{UInt8}) = (push!(io.written, copy(bytes)); nothing)
MyCobot.transport_read(::RecordingIO, n::Integer, timeout_ms::Integer) = UInt8[]
MyCobot.transport_discard_input(::RecordingIO) = nothing
