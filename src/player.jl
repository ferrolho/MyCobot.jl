# Stream a joint trajectory to the robot over the Feetech bus and record what happens.
# Julia port of tools/python/play_trajectory.py; recordings use the same CSV columns,
# so scripts/trace_recording.jl and the Python plotting scripts work on both.

import DelimitedFiles

"Measured per-joint lag of the servos in position mode (s), from the smooth-motion recordings."
const DEFAULT_LAG = [0.120, 0.113, 0.120, 0.054, 0.038, 0.028]

const RECORDING_HEADER = vcat("t", ["q_plan_$j" for j in 1:6], ["q_cmd_$j" for j in 1:6],
                              ["q_$j" for j in 1:6], ["dq_$j" for j in 1:6], ["load_$j" for j in 1:6])

"""
    sample_trajectory(t_plan, q_plan, t)

Linearly interpolate the plan (rows = samples, columns = joints) at time `t`,
holding the first/last sample outside the plan.
"""
function sample_trajectory(t_plan::AbstractVector, q_plan::AbstractMatrix, t::Real)
    t <= t_plan[1] && return q_plan[1, :]
    t >= t_plan[end] && return q_plan[end, :]
    i = searchsortedlast(t_plan, t)
    s = (t - t_plan[i]) / (t_plan[i+1] - t_plan[i])
    return (1 - s) .* q_plan[i, :] .+ s .* q_plan[i+1, :]
end

"""
    check_plan(t_plan, q_plan, mechanism; max_joint_speed=90, limit_margin=10)

Throw if the plan doesn't start and end at the zero pose, goes within `limit_margin`
degrees of a joint limit, or exceeds `max_joint_speed` (°/s).
"""
function check_plan(t_plan, q_plan, mechanism; max_joint_speed=90.0, limit_margin=10.0)
    size(q_plan, 2) == 6 || throw(ArgumentError("plan needs 6 joint columns"))
    issorted(t_plan) || throw(ArgumentError("plan times must increase"))
    all(abs.(q_plan[1, :]) .< 1) && all(abs.(q_plan[end, :]) .< 1) ||
        throw(ArgumentError("plan must start and end at the zero pose"))
    lo, hi = joint_limits_deg(mechanism)
    all(lo' .+ limit_margin .< q_plan .< hi' .- limit_margin) ||
        throw(ArgumentError("plan comes within $(limit_margin)° of a joint limit"))
    speed = maximum(abs, diff(q_plan; dims=1) ./ diff(t_plan))
    speed < max_joint_speed || throw(ArgumentError("plan joint speed $(round(speed, digits=1)) °/s is too high"))
    return nothing
end

"""
    play_trajectory(io, t_plan, q_plan; lag=DEFAULT_LAG, kwargs...)

MOVES THE ROBOT. Stream `q_plan` (degrees; rows = samples at times `t_plan`) as fast
as the bus allows (~300 Hz), sending each joint's target `lag[j]` seconds early,
and record every cycle. The robot must be at the zero pose and the plan must pass
`check_plan`. If any joint falls more than `max_tracking_error` degrees behind,
every joint holds where it is and the run stops. Goal speed is set back to 0 at the end.

Keyword arguments: `lag`, `speed_cap` (steps/s), `max_tracking_error` (°),
`max_start_error` (°), `tail` (s recorded after the plan ends), `mechanism`, and
`q_cmd`: explicit commands on the same grid (e.g. from `ilc_update`). With `q_cmd`,
`q_plan` is only the reference (for recording and the tracking-error check) and no
lag shift is applied. `rate` caps the loop rate in Hz (default: as fast as the bus allows).

Returns `(recording, aborted)`, where `recording` is a matrix with columns
`RECORDING_HEADER` and `aborted` is `nothing` or a reason string.
"""
function play_trajectory(io, t_plan::AbstractVector, q_plan::AbstractMatrix;
                         lag::AbstractVector=DEFAULT_LAG, speed_cap::Integer=2000,
                         max_tracking_error::Real=20.0, max_start_error::Real=3.0,
                         tail::Real=1.0, mechanism=load_mechanism(),
                         q_cmd::Union{Nothing,AbstractMatrix}=nothing, rate::Real=Inf)
    check_plan(t_plan, q_plan, mechanism)
    q_cmd === nothing || (size(q_cmd) == size(q_plan) || throw(ArgumentError("q_cmd must match q_plan")))
    q_cmd === nothing || check_plan(t_plan, q_cmd, mechanism)
    start = read_state(io)
    start.ok || error("not every servo replied")
    maximum(abs, start.q) < max_start_error || error("start from the zero pose (max joint angle $(maximum(abs, start.q))°)")

    # The loop below allocates nothing (BusBuffers, in-place sampling, preallocated recording),
    # so the garbage collector cannot stall it.
    bus = BusBuffers()
    q_ref = zeros(6); q_now = zeros(6)
    command!(t) = q_cmd === nothing ? sample_trajectory!(q_now, t_plan, q_plan, t; lag=lag) :
                                      sample_trajectory!(q_now, t_plan, q_cmd, t)
    enable_motion(io; speed_cap=speed_cap)   # holds every joint first
    write_goals!(bus, io, command!(0.0))

    duration = t_plan[end] + tail
    rec = Matrix{Float64}(undef, ceil(Int, duration * 1500) + 16, length(RECORDING_HEADER))
    n = 0
    aborted = nothing
    t0 = time()
    try
        while (t = time() - t0) < duration && n < size(rec, 1)
            t_cycle = time()
            sample_trajectory!(q_ref, t_plan, q_plan, t)
            write_goals!(bus, io, command!(t))
            read_state!(bus, io)
            n += 1
            @inbounds begin
                rec[n, 1] = t
                for j in 1:6
                    rec[n, 1+j] = q_ref[j]; rec[n, 7+j] = q_now[j]; rec[n, 13+j] = bus.q[j]
                    rec[n, 19+j] = bus.dq[j]; rec[n, 25+j] = bus.load[j]
                end
            end
            while time() - t_cycle < 1 / rate end   # optional rate cap (busy wait)
            bus.ok || continue
            worst, jw = 0.0, 0
            @inbounds for j in 1:6
                e = abs(bus.q[j] - q_ref[j])
                e > worst && ((worst, jw) = (e, j))
            end
            if worst > max_tracking_error
                aborted = "J$jw tracking error $(round(worst, digits=1))°"
                break
            end
        end
    finally
        aborted === nothing || hold_position(io)
        sleep(0.5)
        disable_motion(io)
    end
    return rec[1:n, :], aborted
end

"""
    read_plan_csv(path)

Read a plan written by e.g. `scripts/plan_circle.jl` (columns t,q_1..q_6). Returns
`(t, q, q_cmd)`; `q_cmd` holds explicit commands (columns cmd_1..cmd_6, written by
`scripts/ilc_step.jl`) or `nothing`.
"""
function read_plan_csv(path::AbstractString)
    data, header = DelimitedFiles.readdlm(path, ',', Float64; header=true)
    col(name) = findfirst(==(name), vec(header))
    q = data[:, [col("q_$j") for j in 1:6]]
    q_cmd = col("cmd_1") === nothing ? nothing : data[:, [col("cmd_$j") for j in 1:6]]
    return data[:, col("t")], q, q_cmd
end

"""
    write_plan_csv(path, t, q; q_cmd=nothing)

Write a plan (and optionally explicit commands) for `read_plan_csv`.
"""
function write_plan_csv(path::AbstractString, t::AbstractVector, q::AbstractMatrix; q_cmd=nothing)
    mkpath(dirname(path))
    header = vcat("t", ["q_$j" for j in 1:6], q_cmd === nothing ? String[] : ["cmd_$j" for j in 1:6])
    open(path, "w") do io
        println(io, join(header, ','))
        DelimitedFiles.writedlm(io, q_cmd === nothing ? hcat(t, q) : hcat(t, q, q_cmd), ',')
    end
    return path
end

"""
    write_recording_csv(path, recording)

Write a recording from `play_trajectory` with the same columns as the Python player.
"""
function write_recording_csv(path::AbstractString, recording::AbstractMatrix)
    mkpath(dirname(path))
    open(path, "w") do io
        println(io, join(RECORDING_HEADER, ','))
        DelimitedFiles.writedlm(io, recording, ',')
    end
    return path
end

"""
    move_to(io, q_goal; duration=4.0, rate=150, speed_cap=1000, max_tracking_error=20.0)

MOVES THE ROBOT. Move smoothly (minimum-jerk in joint space) from wherever the arm is to
`q_goal` (degrees) over `duration` seconds, streaming goals at `rate` Hz. Holds and throws
if any joint falls more than `max_tracking_error` degrees behind. Goal speed is set back
to 0 at the end.
"""
function move_to(io, q_goal::AbstractVector; duration::Real=4.0, rate::Real=150, speed_cap::Integer=1000,
                 max_tracking_error::Real=20.0)
    s = read_state(io)
    s.ok || error("not every servo replied")
    q0 = s.q
    lo, hi = joint_limits_deg(load_mechanism())
    all(lo .< q_goal .< hi) || throw(ArgumentError("goal outside the joint limits"))
    minjerk(x) = (x = clamp(x, 0, 1); x^3 * (10 - 15x + 6x^2))
    enable_motion(io; speed_cap=speed_cap)
    t0 = time()
    try
        while (t = time() - t0) < duration + 0.5
            t_cycle = time()
            q_ref = q0 .+ minjerk(t / duration) .* (q_goal .- q0)
            write_goals(io, q_ref)
            st = read_state(io)
            if st.ok && maximum(abs.(st.q .- q_ref)) > max_tracking_error
                hold_position(io)
                error("move_to aborted: tracking error $(round(maximum(abs.(st.q .- q_ref)), digits=1))°")
            end
            while time() - t_cycle < 1 / rate end
        end
    finally
        sleep(0.2)
        disable_motion(io)
    end
    return read_state(io).q
end
