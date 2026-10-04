# MOVES THE ROBOT. Play a joint-trajectory plan (e.g. from scripts/plan_circle.jl) over the
# Feetech bus at full rate and record it. Julia counterpart of tools/python/play_trajectory.py,
# with the same recording format.
#
#   julia --project=. scripts/play_plan.jl tools/python/plans/circle.csv [--no-lag-comp] [--rate=240]
#
# Plans with explicit commands (cmd_1..cmd_6 columns, from scripts/ilc_step.jl) are played
# as given, without a lag shift.
#
# The robot must be at the zero pose. Keep a hand near the power switch.

import Dates
import LibSerialPort
import MyCobot

const PORT = "/dev/tty.usbserial-B00033ZX"
const BAUDRATE = 1_000_000

plan_path = ARGS[1]
lag = "--no-lag-comp" in ARGS ? zeros(6) : MyCobot.DEFAULT_LAG
rate_arg = findfirst(a -> startswith(a, "--rate="), ARGS)
rate = rate_arg === nothing ? Inf : parse(Float64, split(ARGS[rate_arg], "=")[2])

t_plan, q_plan, q_cmd = MyCobot.read_plan_csv(plan_path)
println("FT232R latency timer: ", MyCobot.set_latency_timer(1), " ms")

sp = LibSerialPort.open(PORT, BAUDRATE)
recording, aborted = try
    MyCobot.play_trajectory(sp, t_plan, q_plan; lag=lag, q_cmd=q_cmd, rate=rate)
finally
    LibSerialPort.close(sp)
end
aborted === nothing || println("!! ABORTED and holding: ", aborted)

t = recording[:, 1]
dt = diff(t)
failed = count(isnan, recording[:, findfirst(==("q_1"), MyCobot.RECORDING_HEADER)])
println("$(size(recording, 1)) cycles, $failed failed reads, $(round(Int, 1 / (sum(dt) / length(dt)))) Hz ",
        "(max period $(round(1000maximum(dt), digits=1)) ms)")

name = splitext(basename(plan_path))[1]
tag = q_cmd !== nothing ? "ilc" : all(iszero, lag) ? "nolag" : "lagcomp"
stamp = Dates.format(Dates.now(), "yyyymmdd-HHMMSS")
tag *= isfinite(rate) ? "_$(round(Int, rate))hz" : ""
out = joinpath(@__DIR__, "..", "tools", "python", "recordings", "$(stamp)_$(name)_$(tag)_jl.csv")
println("saved ", normpath(MyCobot.write_recording_csv(out, recording)))
