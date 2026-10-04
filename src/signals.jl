# Test signals for system identification. The same signals as firmware/atom_controller/test_signal.h
# (PLAY_SIGNAL, controller firmware 3.1+), which tools/firmware-tests/ checks against this file.

const SIGNAL_KINDS = Dict("chirp" => 1, "steps" => 2)
const SIGNAL_MOVE_S = 2.0
const SIGNAL_HOLD_S = 1.0

"""
    SignalParams(joint, kind; amp=..., f0=0.2, f1=5.0, duration=..., vmax=60, amax=Inf, base=zeros(6))

A test signal on one joint, around the pose `base` (°). `kind` is "chirp" (logarithmic sweep
f0 → f1, amplitude limited by `vmax` and `amax`) or "steps" (±amp, ramps at `vmax`).
"""
Base.@kwdef struct SignalParams
    joint::Int
    kind::String
    amp::Float64 = kind == "chirp" ? 5.0 : 3.0
    f0::Float64 = 0.2
    f1::Float64 = 5.0
    duration::Float64 = kind == "chirp" ? 24.0 : 12.0
    vmax::Float64 = 60.0
    amax::Float64 = Inf
    base::Vector{Float64} = zeros(6)
end
SignalParams(joint::Integer, kind::AbstractString; kw...) = SignalParams(; joint=joint, kind=String(kind), kw...)

"Total time of the test: move to the base pose, hold, signal, hold, move back."
signal_total(p::SignalParams) = 2SIGNAL_MOVE_S + 2SIGNAL_HOLD_S + p.duration

minjerk01(x) = (x = clamp(x, 0, 1); x^3 * (10 - 15x + 6x^2))

"""
    test_signal(p, t)

The signal on joint `p.joint` at time `t` ∈ [0, p.duration] (°, relative to the base pose).
"""
function test_signal(p::SignalParams, t::Real)
    T = p.duration
    if p.kind == "chirp"
        Tc, τ = T - 2.0, t - 1.0
        (0 <= τ <= Tc) || return 0.0
        k = log(p.f1 / p.f0) / Tc
        f = p.f0 * exp(k * τ)
        phase = 2π * p.f0 * (exp(k * τ) - 1) / k
        a = min(p.amp, p.vmax / (2π * f), p.amax / (2π * f)^2)
        fade = min(1.0, τ / 0.5, (Tc - τ) / 0.5)
        return fade * a * sin(phase)
    elseif p.kind == "steps"
        levels = (0.0, p.amp, 0.0, -p.amp, 0.0, p.amp, -p.amp, 0.0)
        hold, ramp = T / 8, 2p.amp / p.vmax
        k = clamp(floor(Int, t / hold), 0, 7)
        prev = k == 0 ? 0.0 : levels[k]
        s = clamp((t - k * hold) / ramp, 0, 1)
        return prev + s * (levels[k+1] - prev)
    end
    error("signal kind must be chirp or steps")
end

"""
    signal_pose(p, start, t) -> 6-vector

Reference pose (°) at time `t` of the whole test, starting from the pose `start`.
"""
function signal_pose(p::SignalParams, start::AbstractVector, t::Real)
    t1 = SIGNAL_MOVE_S + SIGNAL_HOLD_S
    t2 = t1 + p.duration
    t3 = t2 + SIGNAL_HOLD_S
    q = map(1:6) do k
        b = p.base[k]
        t < SIGNAL_MOVE_S ? start[k] + minjerk01(t / SIGNAL_MOVE_S) * (b - start[k]) :
        t < t3 ? b : b + minjerk01((t - t3) / SIGNAL_MOVE_S) * (start[k] - b)
    end
    t1 <= t < t2 && (q[p.joint] += test_signal(p, t - t1))
    return q
end

"""
    signal_plan(p, start=zeros(6); dt=0.002) -> (t, q)

The whole test as a plan (rows = samples), e.g. for `check_plan` or `play_trajectory`.
"""
function signal_plan(p::SignalParams, start::AbstractVector=zeros(6); dt::Real=0.002)
    t = collect(0:dt:signal_total(p))
    return t, reduce(vcat, permutedims(signal_pose(p, start, ti)) for ti in t)
end
