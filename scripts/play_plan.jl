# MOVES THE ROBOT. Play a joint-trajectory plan (e.g. from scripts/plan_circle.jl) over the
# Feetech bus at full rate and record it. Julia counterpart of tools/python/play_trajectory.py,
# with the same recording format.
#
#   julia --project=. scripts/play_plan.jl tools/python/plans/circle.csv [--no-lag-comp] [--rate=240] [--atom=192.168.1.107]
#
# With --atom=IP the plan is uploaded over WiFi and played onboard the ATOM (firmware/atom_controller)
# at --rate (default 500 Hz); the recording then also has IMU columns.
#
# Plans with explicit commands (cmd_1..cmd_6 columns, from scripts/ilc_step.jl) are played
# as given, without a lag shift.
#
# The robot must be at the zero pose. Keep a hand near the power switch.

import Dates
import LibSerialPort
import MyCobot

const PORT = MyCobot.default_port()
const BAUDRATE = 1_000_000

plan_path = ARGS[1]
lag = "--no-lag-comp" in ARGS ? zeros(6) : MyCobot.DEFAULT_LAG
rate_arg = findfirst(a -> startswith(a, "--rate="), ARGS)
rate = rate_arg === nothing ? Inf : parse(Float64, split(ARGS[rate_arg], "=")[2])

atom_arg = findfirst(a -> startswith(a, "--atom="), ARGS)
t_plan, q_plan, q_cmd = MyCobot.read_plan_csv(plan_path)

if atom_arg !== nothing
    link = MyCobot.AtomLink(split(ARGS[atom_arg], "=")[2])
    recording, done = try
        MyCobot.atom_play_trajectory(link, t_plan, q_plan; lag=lag, q_cmd=q_cmd, rate=isfinite(rate) ? round(Int, rate) : 500)
    finally
        close(link)
    end
    println("ATOM: ", done)
    done.result == "done" || println("!! ", done.result, done.joint > 0 ? " (J$(done.joint), $(round(done.error_deg, digits=1))°)" : "")
else
    println("FT232R latency timer: ", MyCobot.set_latency_timer(1), " ms")
    sp = LibSerialPort.open(PORT, BAUDRATE)
    recording, aborted = try
        MyCobot.play_trajectory(sp, t_plan, q_plan; lag=lag, q_cmd=q_cmd, rate=rate)
    finally
        LibSerialPort.close(sp)
    end
    aborted === nothing || println("!! ABORTED and holding: ", aborted)
end

t = recording[:, 1]
dt = diff(t)
failed = count(isnan, recording[:, findfirst(==("q_1"), MyCobot.RECORDING_HEADER)])
println("$(size(recording, 1)) cycles, $failed failed reads, $(round(Int, 1 / (sum(dt) / length(dt)))) Hz ",
        "(max period $(round(1000maximum(dt), digits=1)) ms)")

name = splitext(basename(plan_path))[1]
tag = q_cmd !== nothing ? "ilc" : all(iszero, lag) ? "nolag" : "lagcomp"
stamp = Dates.format(Dates.now(), "yyyymmdd-HHMMSS")
tag *= isfinite(rate) ? "_$(round(Int, rate))hz" : ""
tag *= atom_arg !== nothing ? "_atom" : "_jl"
out = joinpath(@__DIR__, "..", "tools", "python", "recordings", "$(stamp)_$(name)_$(tag).csv")
write_csv = atom_arg !== nothing ? MyCobot.write_atom_recording_csv : MyCobot.write_recording_csv
println("saved ", normpath(write_csv(out, recording)))
