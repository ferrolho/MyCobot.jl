# System identification of one joint: MOVES THE ROBOT.
#
#   julia --project=. scripts/sysid.jl JOINT KIND [--acc=N] [--amp=DEG] [--vmax=DEG/S] [--amax=DEG/S2] [--T=S] [--pid=P,D,I] [--base=q1,..,q6] [--tag=NAME]
#
# KIND: chirp  logarithmic sweep 0.2 → 5 Hz, amplitude --amp (default 5°), speed ≤ --vmax (default 60°/s)
#       steps  ±--amp (default 3°) steps, ramps at --vmax, 1.5 s each
# The other joints hold zero. Starts and ends at the zero pose; aborts at max(15°, 2·amp) tracking error.
# --acc sets register 41 (acceleration) on every servo for the run (default 0). The servos clamp it
#   to their factory maximum (register 85): 50 on J1–J3, 250 on J4–J6 (×100 steps/s²).
# --amax limits the chirp acceleration (°/s²), to stay inside the servo acceleration limit.
# --pid sets registers 21/22/23 (P, D, I) on JOINT for the run and restores them after.
# --base=q1,...,q6 runs the test around that pose (minimum-jerk move there from zero and back).
# Writes tools/python/recordings/sysid/<time>_J<j>_<kind>_acc<N>[_<tag>].csv (player columns).

import Dates
import LibSerialPort
import MyCobot

opt(name, default) = (a = findfirst(startswith("--$name="), ARGS); a === nothing ? default : split(ARGS[a], "=")[2])
pos = filter(a -> !startswith(a, "--"), ARGS)
j = parse(Int, pos[1]); kind = pos[2]
acc = parse(Int, opt("acc", "0"))
T = parse(Float64, opt("T", kind == "chirp" ? "24" : "12"))
amp = parse(Float64, opt("amp", kind == "chirp" ? "5" : "3"))
pid = opt("pid", "")
tag = opt("tag", "")
vmax = parse(Float64, opt("vmax", "60"))
amax = parse(Float64, opt("amax", "Inf"))
base = parse.(Float64, split(opt("base", "0,0,0,0,0,0"), ","))
length(base) == 6 || error("--base needs 6 angles")
vmax <= 150 || error("--vmax above 150°/s (Elephant's joint speed limit)")

function plan(kind, j, amp, T; dt=0.002, vmax=60.0, amax=Inf)
    t = collect(0:dt:T)
    x = zeros(length(t))
    if kind == "chirp"
        f0, f1, Tc = 0.2, 5.0, T - 2.0                 # 1 s of rest at each end
        for (i, ti) in enumerate(t)
            τ = ti - 1.0
            (0 <= τ <= Tc) || continue
            k = log(f1 / f0) / Tc
            f = f0 * exp(k * τ)                         # instantaneous frequency
            phase = 2π * f0 * (exp(k * τ) - 1) / k
            a = min(amp, vmax / (2π * f), amax / (2π * f)^2)   # keep speed ≤ vmax, acceleration ≤ amax
            w = min(1.0, τ / 0.5, (Tc - τ) / 0.5)       # 0.5 s fade in/out
            x[i] = w * a * sin(phase)
        end
    elseif kind == "steps"
        levels = [0, amp, 0, -amp, 0, amp, -amp, 0]
        hold, ramp = T / length(levels), 2amp / vmax
        for (i, ti) in enumerate(t)
            k = clamp(floor(Int, ti / hold) + 1, 1, length(levels))
            prev = k == 1 ? 0.0 : levels[k-1]
            s = clamp((ti - (k - 1) * hold) / ramp, 0, 1)
            x[i] = prev + s * (levels[k] - prev)
        end
    else
        error("KIND must be chirp or steps")
    end
    q = zeros(length(t), 6)
    q[:, j] = x
    return t, q
end

"Wrap a test plan with a minimum-jerk move from zero to `base` and back (2 s each, 1 s holds)."
function at_base(t, q, base; dt=0.002, move=2.0, hold=1.0)
    any(!iszero, base) || return t, q
    s(x) = (x = clamp(x, 0, 1); x^3 * (10 - 15x + 6x^2))
    tm = collect(0:dt:move+hold)
    go = [s(ti / move) * base[j] for ti in tm, j in 1:6]
    back = [s(1 - (ti - hold) / move) * base[j] for ti in tm, j in 1:6]
    T1 = tm[end] + dt
    t2 = vcat(tm, T1 .+ t, T1 + t[end] + dt .+ tm)
    q2 = vcat(go, q .+ base', back)
    return t2, q2
end

t_plan, q_plan = at_base(plan(kind, j, amp, T; vmax=vmax, amax=amax)..., base)
println("FT232R latency timer: ", MyCobot.set_latency_timer(1), " ms")
sp = MyCobot.open_bus()
const pid_saved = Ref{Any}(nothing)   # Ref: avoids the soft-scope warning in try blocks
rec, aborted = try
    for id in 1:6
        v, temp = MyCobot.ft_read(sp, id, 62, 2)
        temp < 55 || error("servo $id is at $(temp) °C; let it cool")
        v >= 60 || error("servo $id supply is $(v / 10) V")
    end
    if !isempty(pid)
        pid_saved[] = MyCobot.ft_read(sp, j, 21, 3)
        P, D, I = parse.(Int, split(pid, ","))
        MyCobot.ft_write(sp, j, 21, UInt8[P, D, I])
        println("J$j PID (21/22/23) ", Int.(pid_saved[]), " → ", Int.(MyCobot.ft_read(sp, j, 21, 3)))
    end
    MyCobot.play_trajectory(sp, t_plan, q_plan; q_cmd=q_plan, acceleration=acc, max_tracking_error=max(15.0, 2amp), tail=1.0, max_joint_speed=vmax + 1)
finally
    pid_saved[] === nothing || (MyCobot.ft_write(sp, j, 21, pid_saved[]); println("J$j PID restored to ", Int.(MyCobot.ft_read(sp, j, 21, 3))))
    LibSerialPort.close(sp)
end

name = Dates.format(Dates.now(), "yyyymmdd-HHMMSS") * "_J$(j)_$(kind)_acc$(acc)" * (isfinite(amax) ? "_amax$(round(Int, amax))" : "") * "_amp$(round(Int, amp))" * (any(!iszero, base) ? "_base" * join(round.(Int, base), "_") : "") * (isempty(pid) ? "" : "_pid" * replace(pid, "," => "-")) * (isempty(tag) ? "" : "_$tag")
path = MyCobot.write_recording_csv(joinpath(@__DIR__, "..", "tools", "python", "recordings", "sysid", name * ".csv"), rec)
e = rec[:, 13+j] .- rec[:, 1+j]
println(aborted === nothing ? "done" : "ABORTED: $aborted", ": $(size(rec, 1)) cycles in $(round(rec[end, 1], digits=1)) s ",
        "($(round(size(rec, 1) / rec[end, 1], digits=0)) Hz), J$j RMS error $(round(sqrt(sum(abs2, filter(!isnan, e)) / length(e)), digits=2))°")
println("other joints max |q|: ", round(maximum(abs, filter(!isnan, rec[:, [13 + k for k in 1:6 if k != j]])), digits=2), "°")
println(relpath(path))
