# Iterative learning control (ILC) for repeated trajectories. The servos' tracking error
# repeats almost exactly from run to run (lag, sticking after reversals), so each run's
# error can correct the next run's commands:
#
#   cmd[k+1](t) = cmd[k](t) + gain · w(t) · Q[ref − measured](t + lead_j)
#
# Q is a zero-phase low-pass filter (forward-backward moving average), lead_j is the
# joint's lag (the command at t shows up in the measurement around t + lead_j), and w(t)
# tapers the correction to zero at the start and end so the plan still starts and ends
# at rest.

"""
    zero_phase_smooth(x, n)

Forward-backward moving average over `n` samples (no phase shift), with the ends held.
"""
function zero_phase_smooth(x::AbstractVector, n::Integer)
    n <= 1 && return collect(float.(x))
    pass(v) = [sum(v[max(i - n + 1, 1):i]) / (i - max(i - n + 1, 1) + 1) for i in eachindex(v)]
    return reverse(pass(reverse(pass(x))))
end

"""
    taper_window(t, ramp)

1 in the interior, smoothly 0 within `ramp` seconds of either end.
"""
function taper_window(t::AbstractVector, ramp::Real)
    s(x) = (x = clamp(x, 0, 1); x^3 * (10 - 15x + 6x^2))
    return [s((ti - t[1]) / ramp) * s((t[end] - ti) / ramp) for ti in t]
end

"""
    ilc_update(t, q_ref, q_cmd, t_meas, q_meas; gain=0.5, lead=DEFAULT_LAG, smooth=0.08, ramp=0.3)

One ILC iteration. `t`, `q_ref`, `q_cmd` are the plan grid, the reference and the
commands used in the last run (rows = samples, columns = joints); `t_meas`, `q_meas`
are what was measured. `smooth` is the moving-average width in seconds. Returns the
new commands on the plan grid.
"""
function ilc_update(t::AbstractVector, q_ref::AbstractMatrix, q_cmd::AbstractMatrix,
                    t_meas::AbstractVector, q_meas::AbstractMatrix;
                    gain::Real=0.5, lead::AbstractVector=DEFAULT_LAG, smooth::Real=0.08, ramp::Real=0.3)
    dt = (t[end] - t[1]) / (length(t) - 1)
    n = max(round(Int, smooth / dt), 1)
    w = taper_window(t, ramp)
    q_new = copy(q_cmd)
    for j in 1:6
        meas = [sample_trajectory(t_meas, q_meas[:, j:j], ti)[1] for ti in t]
        e = zero_phase_smooth(q_ref[:, j] .- meas, n)
        e_lead = [sample_trajectory(t, reshape(e, :, 1), ti + lead[j])[1] for ti in t]
        q_new[:, j] .+= gain .* w .* e_lead
    end
    return q_new
end
