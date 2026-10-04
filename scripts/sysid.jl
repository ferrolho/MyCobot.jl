# System identification of one joint: MOVES THE ROBOT.
#
#   julia --project=. scripts/sysid.jl JOINT KIND [--atom=IP] [--amp=DEG] [--vmax=DEG/S] [--amax=DEG/S2] [--T=S] [--pid=P,D,I] [--base=q1,..,q6] [--tag=NAME]
#
# KIND: chirp  logarithmic sweep 0.2 → 5 Hz, amplitude --amp (default 5°), speed ≤ --vmax (default 60°/s)
#       steps  ±--amp (default 3°) steps, ramps at --vmax, --T/8 each
# The ATOM computes the signal (firmware/atom_controller/test_signal.h). The other joints hold the
# base pose. Starts and ends at the current pose; aborts at max(15°, 2·amp) tracking error.
# The servos' acceleration register is not changed: the servos clamp it to their factory maximum
#   (register 85): 50 on J1–J3, 250 on J4–J6 (×100 steps/s²).
# --amax limits the chirp acceleration (°/s²), to stay inside the servo acceleration limit.
# --pid sets registers 21/22/23 (P, D, I) on JOINT for the run and restores them after.
# --base=q1,...,q6 runs the test around that pose (minimum-jerk move there and back).
# The ATOM (--atom=IP, default 192.168.1.107, firmware 4.0+) computes the signal onboard and plays
# it at 500 Hz; the recording has the reference it used and the IMU.
# Writes tools/python/recordings/sysid/<time>_J<j>_<kind>_acc<N>[_<tag>].csv (player columns).

import Dates
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

p = MyCobot.SignalParams(j, kind; amp=amp, duration=T, vmax=vmax, amax=amax, base=base)
atom_ip = opt("atom", "192.168.1.107")
acc == 0 || error("--acc is not supported: the ATOM player writes 0 (the servos clamp it to register 85)")
link = MyCobot.AtomLink(atom_ip)
const pid_saved = Ref{Any}(nothing)   # Ref: avoids the soft-scope warning in try blocks
rec, aborted = try
    for id in 1:6
        v, temp = MyCobot.atom_read_reg(link, id, 62, 2)
        temp < 55 || error("servo $id is at $(temp) °C; let it cool")
        v >= 60 || error("servo $id supply is $(v / 10) V")
    end
    if !isempty(pid)
        pid_saved[] = MyCobot.atom_read_reg(link, j, 21, 3)
        MyCobot.atom_write_reg(link, j, 21, parse.(Int, split(pid, ",")))
        println("J$j PID (21/22/23) ", Int.(pid_saved[]), " → ", Int.(MyCobot.atom_read_reg(link, j, 21, 3)))
    end
    r, done = MyCobot.atom_play_signal(link, p; max_tracking_error=max(15.0, 2amp))
    r, done.result == "done" ? nothing : string(done)
finally
    pid_saved[] === nothing || (MyCobot.atom_write_reg(link, j, 21, pid_saved[]); println("J$j PID restored to ", Int.(MyCobot.atom_read_reg(link, j, 21, 3))))
    close(link)
end

name = Dates.format(Dates.now(), "yyyymmdd-HHMMSS") * "_J$(j)_$(kind)_acc$(acc)" * (isfinite(amax) ? "_amax$(round(Int, amax))" : "") * "_amp$(round(Int, amp))" * (any(!iszero, base) ? "_base" * join(round.(Int, base), "_") : "") * (isempty(pid) ? "" : "_pid" * replace(pid, "," => "-")) * (isempty(tag) ? "" : "_$tag")
out = joinpath(@__DIR__, "..", "tools", "python", "recordings", "sysid", name * ".csv")
path = MyCobot.write_atom_recording_csv(out, rec)
e = rec[:, 13+j] .- rec[:, 1+j]
println(aborted === nothing ? "done" : "ABORTED: $aborted", ": $(size(rec, 1)) cycles in $(round(rec[end, 1], digits=1)) s ",
        "($(round(size(rec, 1) / rec[end, 1], digits=0)) Hz), J$j RMS error $(round(sqrt(sum(abs2, filter(!isnan, e)) / length(e)), digits=2))°")
println("other joints max |q|: ", round(maximum(abs, filter(!isnan, rec[:, [13 + k for k in 1:6 if k != j]])), digits=2), "°")
println(relpath(path))
