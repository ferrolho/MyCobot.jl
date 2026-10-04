# A model of the joint servos in position mode, fitted on 2026-10-04 (scripts/sysid.jl,
# tools/python/analyze_sysid.py; docs: results/servo-dynamics).
#
# Inside its acceleration limit, each servo follows its goal like a second-order system
#     q(s) / goal(s) = ωn² / (s² + 2ζ ωn s + ωn²)
# Above the limit (factory register 85 × 100 steps/s²) the servo saturates: a 10° sine on J1
# stops following at ~1.2 Hz. Register 41 (acceleration) is clamped to register 85.

"Natural frequency ωn (rad/s) of each joint servo, from acceleration-limited chirps."
const SERVO_WN = [14.8, 14.7, 14.2, 34.4, 52.4, 54.9]

"Damping ratio ζ of each joint servo."
const SERVO_ZETA = [0.82, 0.75, 0.78, 0.65, 0.77, 0.83]

"Acceleration limit of each servo (°/s²): factory register 85 (50 or 250) × 100 steps/s²."
const SERVO_AMAX = [50, 50, 50, 250, 250, 250] .* 100 ./ STEPS_PER_DEG

"""
    plan_derivatives(t, q)

Velocity and acceleration of a plan (rows = samples) by central differences.
"""
function plan_derivatives(t::AbstractVector, q::AbstractMatrix)
    dq = similar(q); ddq = similar(q)
    n = length(t)
    for j in axes(q, 2), i in 1:n
        a, b = max(i - 1, 1), min(i + 1, n)
        dq[i, j] = (q[b, j] - q[a, j]) / (t[b] - t[a])
    end
    for j in axes(q, 2), i in 1:n
        a, b = max(i - 1, 1), min(i + 1, n)
        ddq[i, j] = (dq[b, j] - dq[a, j]) / (t[b] - t[a])
    end
    return dq, ddq
end

"""
    model_feedforward(t, q; wn=SERVO_WN, zeta=SERVO_ZETA, smooth=0.06)

Commands that make the second-order servo model follow the plan `q`:
`u = q + (2ζ/ωn) q̇ + q̈/ωn²`. The velocity and acceleration are smoothed with a zero-phase
moving average of `smooth` seconds, because a plan made of segments has acceleration jumps
that would otherwise become speed spikes in `u`. The plan should stay inside `SERVO_AMAX`
(see `check_acceleration`).
"""
function model_feedforward(t::AbstractVector, q::AbstractMatrix; wn=SERVO_WN, zeta=SERVO_ZETA, smooth::Real=0.06)
    dq, ddq = plan_derivatives(t, q)
    n = max(1, round(Int, smooth / ((t[end] - t[1]) / (length(t) - 1))))
    u = similar(q)
    for j in axes(q, 2)
        v = zero_phase_smooth(dq[:, j], n)
        a = zero_phase_smooth(ddq[:, j], n)
        @views u[:, j] .= q[:, j] .+ (2zeta[j] / wn[j]) .* v .+ a ./ wn[j]^2
    end
    return u
end

"""
    check_acceleration(t, q; amax=SERVO_AMAX, margin=0.8)

Peak acceleration of each joint (°/s²), and whether it stays below `margin × amax`.
"""
function check_acceleration(t::AbstractVector, q::AbstractMatrix; amax=SERVO_AMAX, margin=0.8)
    _, ddq = plan_derivatives(t, q)
    peak = vec(maximum(abs, ddq; dims=1))
    return peak, all(peak .<= margin .* amax)
end

"""
Servo position-loop gains (registers 21/22/23 = P, D, I) for each joint.

`DEFAULT_GAINS` is what the servos store and use with the custom ATOM firmware (the stock
firmware wrote 10/0/1 to J3–J6 at power-up). `TUNED_GAINS` adds integral action on J1–J3:
on the circle it halves the flange error (5.3 → 2.6 mm) with almost the same end-effector
vibration (155 → 165 mg RMS). Higher P (≥ 48) raised the vibration up to 4× (2026-10-04).
"""
const DEFAULT_GAINS = [(32, 8, 0) for _ in 1:6]
const TUNED_GAINS = [(32, 4, 16), (32, 4, 16), (32, 4, 16), (32, 8, 0), (32, 8, 0), (32, 8, 0)]

"""
    set_servo_gains!(io, gains=TUNED_GAINS) -> gains read back

Write P, D, I (registers 21–23) to every servo and read them back. With the EEPROM lock
(register 55) at 1 the change lasts until the next power cycle.
"""
function set_servo_gains!(io, gains=TUNED_GAINS)
    for (j, (p, d, i)) in enumerate(gains)
        ft_write(io, SERVO_IDS[j], 21, UInt8[p, d, i])
    end
    return [Tuple(Int.(ft_read(io, SERVO_IDS[j], 21, 3))) for j in 1:6]
end
