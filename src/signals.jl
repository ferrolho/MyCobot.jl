# Test signals for system identification, computed onboard the ATOM (PLAY_SIGNAL, controller
# firmware 4.0+; firmware/atom_controller/test_signal.h is the only implementation). The firmware
# checks the parameters and the start pose, and reports the reference it used in the telemetry.

const SIGNAL_KINDS = Dict("chirp" => 1, "steps" => 2)

"Firmware acceleration limits for test signals (°/s², `AMAX_DPS2` in test_signal.h)."
const SIGNAL_AMAX = [400.0, 400, 400, 2000, 2000, 2000]

"""
    SignalParams(joint, kind; amp, f0=0.2, f1=5.0, duration, vmax=60, amax=Inf, base=zeros(6))

A test signal on one joint, around the pose `base` (°). `kind` is "chirp" (logarithmic sweep
f0 → f1; amplitude limited by `vmax` and `amax`) or "steps" (±amp, ramps at `vmax`). The ATOM
moves from the current pose to `base` (2 s), holds 1 s, runs the signal for `duration`, holds
1 s and moves back (2 s). See test_signal.h for the exact definitions.
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

"Total time of the test (s): moves and holds (6 s) plus the signal."
signal_total(p::SignalParams) = 6.0 + p.duration

"The 38-byte `sig::Params` struct for PLAY_SIGNAL. `amax` is capped at the firmware limit."
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
