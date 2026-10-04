# WiFi link to the ATOM running firmware/atom_controller: upload a plan, play it onboard at a
# fixed rate (default 500 Hz), and collect telemetry (joint state + end-effector IMU).
# Protocol: see the header of firmware/atom_controller/atom_controller.ino.

import CRC32c
import Sockets

const ATOM_CMD_PORT = 5006
const ATOM_REPLY_PORT = 5007
const ATOM_STATES = ("booting", "holding", "ready", "playing", "error", "ota")
const PLAY_RESULTS = ("done", "tracking error", "stopped", "not at the start pose", "bus error")
const IMU_HEADER = ["acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"]   # g, °/s

"""
    AtomLink(ip)

UDP connection to the ATOM. A background task collects every reply into a channel.
Close with `close(link)`.
"""
struct AtomLink
    ip::Sockets.IPAddr
    sock::Sockets.UDPSocket
    inbox::Channel{Vector{UInt8}}
    receiver::Task
end

function AtomLink(ip::AbstractString)
    sock = Sockets.UDPSocket()
    Sockets.bind(sock, Sockets.ip"0.0.0.0", ATOM_REPLY_PORT; reuseaddr=true) || error("can't bind UDP port $ATOM_REPLY_PORT")
    inbox = Channel{Vector{UInt8}}(4096)
    receiver = @async while isopen(sock)
        try
            put!(inbox, Sockets.recv(sock))
        catch e
            isopen(sock) && @warn "ATOM receiver stopped" exception = e
            break
        end
    end
    return AtomLink(Sockets.getaddrinfo(ip), sock, inbox, receiver)
end

Base.close(link::AtomLink) = close(link.sock)

send(link::AtomLink, msg::Vector{UInt8}) = Sockets.send(link.sock, link.ip, ATOM_CMD_PORT, msg)

function drain!(link::AtomLink)
    while isready(link.inbox)
        take!(link.inbox)
    end
end

"Wait for the next message of type `type` (other messages are dropped). Returns `nothing` on timeout."
function receive(link::AtomLink, type::UInt8; timeout::Real=0.5)
    deadline = time() + timeout
    while time() < deadline
        if isready(link.inbox)
            m = take!(link.inbox)
            !isempty(m) && m[1] == type && return m
        else
            sleep(0.0005)
        end
    end
    return nothing
end

le(x) = reinterpret(UInt8, [htol(x)])
rd(::Type{T}, b::AbstractVector{UInt8}, i::Integer) where {T} = ltoh(reinterpret(T, b[i:i+sizeof(T)-1])[1])

function request_ack(link::AtomLink, msg::Vector{UInt8}; timeout=0.5, retries=3)
    for _ in 1:retries
        drain!(link)
        send(link, msg)
        m = receive(link, 0x83; timeout=timeout)
        m !== nothing && m[2] == msg[1] && return reinterpret(Int8, m[3])
    end
    error("no ACK from the ATOM for message $(string(msg[1], base=16))")
end

"""
    atom_ping(link)

Returns `(version, state, plan_samples, plan_rate, imu_ok, gains_ok)`. `version` is a
`VersionNumber` (minor and patch are reported from firmware 3.1). `gains_ok` (firmware ≥ 3):
the servo gains were written at power-up.
"""
function atom_ping(link::AtomLink; timeout=1.0)
    drain!(link)
    send(link, UInt8[0x01])
    m = receive(link, 0x81; timeout=timeout)
    m === nothing && error("the ATOM didn't answer at $(link.ip)")
    major = Int(rd(UInt16, m, 2))
    minor, patch = length(m) >= 14 ? (Int(m[13]), Int(m[14])) : (0, 0)
    return (version=VersionNumber(major, minor, patch), state=ATOM_STATES[m[4]+1], plan_samples=Int(rd(UInt32, m, 5)),
            plan_rate=Int(rd(UInt16, m, 9)), imu_ok=m[11] == 1, gains_ok=length(m) >= 12 && m[12] == 1)
end

decode_imu(acc, gyro) = vcat(acc ./ 4096, gyro ./ 16.4)

"""
    atom_state(link)

Joint state read by the ATOM, plus the IMU: `(ok, q, dq, load, imu)` in degrees, °/s, %, and g / °/s.
"""
function atom_state(link::AtomLink)
    drain!(link)
    send(link, UInt8[0x02])
    m = receive(link, 0x82; timeout=0.5)
    m === nothing && error("no STATE reply")
    raw(k, j) = rd(UInt16, m, 3 + 12k + 2(j - 1))
    q = [position_to_angle(j, raw(0, j)) for j in 1:6]
    dq = [JOINT_SIGN[j] * decode_signed15(raw(1, j)) / STEPS_PER_DEG for j in 1:6]
    load = [JOINT_SIGN[j] * decode_load(raw(2, j)) for j in 1:6]
    imu = decode_imu([rd(Int16, m, 39 + 2k) for k in 0:2], [rd(Int16, m, 45 + 2k) for k in 0:2])
    return (ok=m[2] == 1, q=q, dq=dq, load=load, imu=imu)
end

const REG_STATUS = Dict(0 => "ok", -1 => "busy or bad request", -4 => "no reply from the servo",
                       -5 => "write not allowed (registers 0-8, 55, 80+)", -6 => "read-back differs")

"""
    atom_read_reg(link, id, addr, len) -> Vector{UInt8}

Read `len` (1–32) bytes at `addr` from servo `id` (1–7) through the ATOM (firmware ≥ 3).
Not while a plan plays.
"""
function atom_read_reg(link::AtomLink, id::Integer, addr::Integer, len::Integer; retries=3)
    for _ in 1:retries
        drain!(link)
        send(link, UInt8[0x09, id, addr, len])
        m = receive(link, 0x86; timeout=0.5)
        m === nothing && continue
        status = reinterpret(Int8, m[5])
        status == 0 && return m[6:5+len]
        error("REG_READ J$id @$addr: ", get(REG_STATUS, status, string(status)))
    end
    error("no REG_READ reply from the ATOM")
end

"""
    atom_write_reg(link, id, addr, data)

Write `data` (1–32 bytes) at `addr` on servo `id` through the ATOM, which reads it back
(firmware ≥ 3). Registers 0–8, 55 and 80+ are refused. Not while a plan plays.
"""
function atom_write_reg(link::AtomLink, id::Integer, addr::Integer, data::AbstractVector{<:Integer}; retries=3)
    for _ in 1:retries
        drain!(link)
        send(link, UInt8[0x0A, id, addr, length(data), data...])
        m = receive(link, 0x87; timeout=0.5)
        m === nothing && continue
        status = reinterpret(Int8, m[4])
        status == 0 && return nothing
        error("REG_WRITE J$id @$addr: ", get(REG_STATUS, status, string(status)))
    end
    error("no REG_WRITE reply from the ATOM")
end

"Read the position-loop gains (P, D, I) of the six servos through the ATOM."
atom_gains(link::AtomLink) = [Tuple(Int.(atom_read_reg(link, j, 21, 3))) for j in 1:6]

"Write gains `[(P, D, I) per joint]` through the ATOM (until the next power cycle) and read them back."
function atom_set_gains!(link::AtomLink, gains=GAINS)
    for (j, g) in enumerate(gains)
        atom_write_reg(link, j, 21, collect(g))
    end
    return atom_gains(link)
end

"Hold the current pose (goals = present, goal speed 0)."
atom_hold(link::AtomLink) = request_ack(link, UInt8[0x03]) == 0 || error("HOLD failed")

"Stop a running plan: the ATOM holds where it is."
atom_stop(link::AtomLink) = send(link, UInt8[0x08])

"""
    atom_upload_plan(link, t, q_ref, q_cmd; rate=50, cubic=true)

Resample the plan (degrees, rows = samples at times `t`) to `rate` Hz, convert to servo steps,
and upload it with per-chunk ACKs and a CRC-32C check. The ATOM interpolates between samples:
Catmull-Rom with `cubic` (firmware 3.1+; older firmware interpolates linearly). Cubic at 25–50 Hz
reproduces our plans to within half a servo step (0.05°), with 5–10× less memory than 250 Hz.
"""
function atom_upload_plan(link::AtomLink, t::AbstractVector, q_ref::AbstractMatrix, q_cmd::AbstractMatrix;
                          rate::Integer=50, cubic::Bool=true)
    ts = collect(t[1]:1/rate:t[end])
    n = length(ts)
    data = Vector{UInt8}(undef, 24n)
    for (i, ti) in enumerate(ts)
        c = sample_trajectory(t, q_cmd, ti)
        r = sample_trajectory(t, q_ref, ti)
        steps = vcat([angle_to_position(j, c[j]) for j in 1:6], [angle_to_position(j, r[j]) for j in 1:6])
        data[24(i-1)+1:24i] = reinterpret(UInt8, htol.(UInt16.(steps)))
    end
    st = request_ack(link, vcat(0x04, le(UInt32(n)), le(UInt16(rate)), UInt8(cubic)))
    st == 0 || error("PLAN_BEGIN refused ($st$(st == -3 ? ": not enough memory" : ""))")
    chunk = 50
    for off in 0:chunk:n-1
        cnt = min(chunk, n - off)
        msg = vcat(0x05, le(UInt32(off)), le(UInt16(cnt)), data[24off+1:24(off+cnt)])
        request_ack(link, msg; timeout=0.3, retries=5) == 0 || error("PLAN_DATA refused at sample $off")
    end
    request_ack(link, vcat(0x06, le(UInt32(CRC32c.crc32c(data))))) == 0 || error("plan CRC mismatch on the ATOM")
    return n
end

"""
    atom_play(link; rate=500, speed_cap=2000, max_tracking_error=20.0, start_tolerance=3.0)

MOVES THE ROBOT. Play the uploaded plan on the ATOM at `rate` Hz and collect telemetry until it
finishes. The arm must be within `start_tolerance` degrees of the plan's first reference pose;
the ATOM holds and stops if any joint falls more than `max_tracking_error` degrees behind.
Returns `(samples, done)`: raw telemetry samples and the result summary.
"""
function atom_play(link::AtomLink; rate::Integer=500, speed_cap::Integer=2000,
                   max_tracking_error::Real=20.0, start_tolerance::Real=3.0, timeout::Real=120.0,
                   signal::Union{Nothing,SignalParams}=nothing)
    drain!(link)
    kind = signal === nothing ? 0x07 : 0x0B
    msg = vcat(kind, le(UInt16(rate)), le(UInt16(speed_cap)),
               le(UInt16(round(Int, max_tracking_error * STEPS_PER_DEG))), le(UInt16(round(Int, start_tolerance * STEPS_PER_DEG))))
    signal === nothing || append!(msg, pack_signal(signal))
    send(link, msg)
    samples = Dict{UInt32,Vector{UInt8}}()
    acked = false
    deadline = time() + timeout
    while time() < deadline
        isready(link.inbox) || (sleep(0.0005); continue)
        m = take!(link.inbox)
        isempty(m) && continue
        if m[1] == 0x83 && m[2] == kind
            st = reinterpret(Int8, m[3])
            st == 0 || st == -3 || st == -4 || error((kind == 0x07 ? "PLAY" : "PLAY_SIGNAL") * " refused ($st)")
            acked = true
        elseif m[1] == 0x84
            seq, cnt = rd(UInt32, m, 2), m[6]
            for k in 0:cnt-1
                samples[seq+k] = m[7+53k:7+53k+52]
            end
        elseif m[1] == 0x85
            done = (result=PLAY_RESULTS[m[2]+1], cycles=Int(rd(UInt32, m, 3)), max_period_ms=rd(UInt32, m, 7) / 1000,
                    late_cycles=Int(rd(UInt32, m, 11)), telemetry_dropped=Int(rd(UInt32, m, 15)),
                    joint=Int(m[19]), error_deg=rd(Int16, m, 20) / STEPS_PER_DEG)
            return [samples[k] for k in sort!(collect(keys(samples)))], done
        end
    end
    error(acked ? "no DONE from the ATOM within $(timeout) s" : "the ATOM didn't acknowledge PLAY")
end

"""
    atom_play_trajectory(link, t_plan, q_plan; lag=DEFAULT_LAG, q_cmd=nothing, rate=500, plan_rate=250, kwargs...)

MOVES THE ROBOT. Same contract as `play_trajectory`, but the loop runs on the ATOM: check the plan,
compute lag-shifted commands (unless `q_cmd` is given, e.g. from `ilc_update`), upload, play, and
return `(recording, done)`. `recording` has the columns `RECORDING_HEADER` plus `IMU_HEADER`.
"""
function atom_play_trajectory(link::AtomLink, t_plan::AbstractVector, q_plan::AbstractMatrix;
                              lag::AbstractVector=DEFAULT_LAG, q_cmd::Union{Nothing,AbstractMatrix}=nothing,
                              rate::Integer=500, plan_rate::Integer=50, cubic::Bool=true, mechanism=load_mechanism(),
                              max_joint_speed::Real=90.0, kwargs...)
    check_plan(t_plan, q_plan, mechanism; max_joint_speed=max_joint_speed)
    if q_cmd === nothing
        q_cmd = reduce(vcat, permutedims([sample_trajectory(t_plan, q_plan, ti + lag[j])[j] for j in 1:6]) for ti in t_plan)
    end
    check_plan(t_plan, q_cmd, mechanism; max_joint_speed=max_joint_speed)
    atom_upload_plan(link, t_plan, q_plan, q_cmd; rate=plan_rate, cubic=cubic)
    samples, done = atom_play(link; rate=rate, kwargs...)
    return decode_telemetry(samples, t -> sample_trajectory(t_plan, q_plan, t), t -> sample_trajectory(t_plan, q_cmd, t)), done
end

"Telemetry samples to recording rows (`RECORDING_HEADER` + `IMU_HEADER`), with the reference and command from `ref(t)` and `cmd(t)`."
function decode_telemetry(samples, ref, cmd)
    rows = Vector{Vector{Float64}}()
    for b in samples
        t = rd(UInt32, b, 1) / 1e6
        raw(k, j) = rd(UInt16, b, 5 + 12k + 2(j - 1))
        ok = b[53] == 1
        q = ok ? [position_to_angle(j, raw(0, j)) for j in 1:6] : fill(NaN, 6)
        dq = ok ? [JOINT_SIGN[j] * decode_signed15(raw(1, j)) / STEPS_PER_DEG for j in 1:6] : fill(NaN, 6)
        load = ok ? [JOINT_SIGN[j] * decode_load(raw(2, j)) for j in 1:6] : fill(NaN, 6)
        imu = decode_imu([rd(Int16, b, 41 + 2k) for k in 0:2], [rd(Int16, b, 47 + 2k) for k in 0:2])
        push!(rows, vcat(t, ref(t), cmd(t), q, dq, load, imu))
    end
    return isempty(rows) ? zeros(0, length(RECORDING_HEADER) + 6) : reduce(vcat, permutedims.(rows))
end

"The firmware's acceleration limits for test signals (°/s², `AMAX_DPS2` in test_signal.h)."
const SIGNAL_AMAX = [400.0, 400, 400, 2000, 2000, 2000]

"`p` with the acceleration limit the firmware uses (`amax` capped at `SIGNAL_AMAX`)."
firmware_signal(p::SignalParams) = SignalParams(p.joint, p.kind; amp=p.amp, f0=p.f0, f1=p.f1, duration=p.duration,
                                                vmax=p.vmax, amax=min(p.amax, SIGNAL_AMAX[p.joint]), base=p.base)

function pack_signal(p::SignalParams)
    buf = IOBuffer()
    write(buf, UInt8(p.joint), UInt8(SIGNAL_KINDS[p.kind]))
    for x in (p.amp, p.f0, p.f1, p.duration, p.vmax, min(p.amax, SIGNAL_AMAX[p.joint]))
        write(buf, htol(Float32(x)))
    end
    for b in p.base
        write(buf, htol(Int16(round(Int, 100b))))
    end
    return take!(buf)
end

"""
    atom_play_signal(link, p::SignalParams; rate=500, max_tracking_error=20, kwargs...) -> (recording, done)

MOVES THE ROBOT. The ATOM computes the test signal `p` onboard (firmware 3.1+): it moves from
the current pose to `p.base`, runs the signal on `p.joint`, and moves back. Nothing is uploaded,
so there is no limit on the duration from the ATOM's memory. The recording has the same columns
as `atom_play_trajectory`; the reference and command columns are computed here from `p` and
the start pose.
"""
function atom_play_signal(link::AtomLink, p::SignalParams; mechanism=load_mechanism(), kwargs...)
    v = atom_ping(link).version
    v >= v"3.1.0" || error("PLAY_SIGNAL needs controller firmware 3.1 or later (the ATOM has $v)")
    p = firmware_signal(p)
    start = atom_state(link).q
    t, q = signal_plan(p, start)
    lo, hi = joint_limits_deg(mechanism)
    all(lo' .+ 10 .< q .< hi' .- 10) || throw(ArgumentError("the test comes within 10° of a joint limit"))
    samples, done = atom_play(link; signal=p, kwargs...)
    ref(ti) = signal_pose(p, start, ti)
    return decode_telemetry(samples, ref, ref), done
end

"Write an ATOM recording (`RECORDING_HEADER` + `IMU_HEADER` columns)."
function write_atom_recording_csv(path::AbstractString, recording::AbstractMatrix)
    mkpath(dirname(path))
    open(path, "w") do io
        println(io, join(vcat(RECORDING_HEADER, IMU_HEADER), ','))
        DelimitedFiles.writedlm(io, recording, ',')
    end
    return path
end

"""
    atom_move_to(link, q_goal; duration=4.0, rate=500)

MOVES THE ROBOT. Move smoothly (minimum-jerk in joint space) from the current pose to `q_goal`
(degrees) over `duration` seconds, played on the ATOM. Returns the result summary.
"""
function atom_move_to(link::AtomLink, q_goal::AbstractVector; duration::Real=4.0, rate::Integer=500,
                      mechanism=load_mechanism(), kwargs...)
    s = atom_state(link)
    s.ok || error("not every servo replied")
    lo, hi = joint_limits_deg(mechanism)
    all(lo .< q_goal .< hi) || throw(ArgumentError("goal outside the joint limits"))
    minjerk(x) = (x = clamp(x, 0, 1); x^3 * (10 - 15x + 6x^2))
    t = collect(0:0.004:duration)
    q = reduce(vcat, permutedims(s.q .+ minjerk(ti / duration) .* (q_goal .- s.q)) for ti in t)
    speed = maximum(abs, diff(q; dims=1) ./ diff(t))
    speed < 90 || throw(ArgumentError("move too fast ($(round(speed, digits=1)) °/s); use a longer duration"))
    atom_upload_plan(link, t, q, q)
    _, done = atom_play(link; rate=rate, kwargs...)
    return done
end
