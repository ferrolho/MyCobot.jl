# One iterative-learning-control step: from a plan and the recording of playing it, write
# the next plan with corrected commands (cmd_1..cmd_6 columns), ready for play_plan.jl.
#
#   julia --project=. scripts/ilc_step.jl tools/python/plans/circle.csv tools/python/recordings/<rec>.csv [--gain=0.5] [--smooth=0.08]
#
# If the plan has no explicit commands yet, the commands of the last run are taken to be
# the lag-shifted reference (what play_trajectory does with DEFAULT_LAG).

import DelimitedFiles
import MyCobot

opt(name, default) = (a = findfirst(startswith("--$name="), ARGS); a === nothing ? default : parse(Float64, split(ARGS[a], "=")[2]))
plan_path, rec_path = filter(a -> !startswith(a, "--"), ARGS)
gain, smooth = opt("gain", 0.5), opt("smooth", 0.08)
t, q_ref, q_cmd = MyCobot.read_plan_csv(plan_path)
if q_cmd === nothing
    q_cmd = reduce(vcat, permutedims([MyCobot.sample_trajectory(t, q_ref, ti + MyCobot.DEFAULT_LAG[j])[j] for j in 1:6]) for ti in t)
end

data, header = DelimitedFiles.readdlm(rec_path, ',', Float64; header=true)
col(name) = findfirst(==(name), vec(header))
rows = filter(i -> !isnan(data[i, col("q_1")]), 1:size(data, 1))
t_meas = data[rows, col("t")]
q_meas = data[rows, [col("q_$j") for j in 1:6]]

q_new = MyCobot.ilc_update(t, q_ref, q_cmd, t_meas, q_meas; gain=gain, smooth=smooth)
MyCobot.check_plan(t, q_new, MyCobot.load_mechanism())

e = [MyCobot.sample_trajectory(t_meas, q_meas, ti) for ti in t]
err = reduce(vcat, permutedims.(e)) .- q_ref
rms(x) = sqrt(sum(abs2, x) / length(x))
println("last run joint error RMS (°): ", round.([rms(err[:, j]) for j in 1:6], digits=2))
println("correction max |Δcmd| (°):     ", round.(vec(maximum(abs, q_new .- q_cmd; dims=1)), digits=2))

m = match(r"_ilc(\d+)$", splitext(basename(plan_path))[1])
k = m === nothing ? 1 : parse(Int, m[1]) + 1
base = replace(splitext(basename(plan_path))[1], r"_ilc\d+$" => "")
out = joinpath(dirname(plan_path), "$(base)_ilc$(k).csv")
println("saved ", MyCobot.write_plan_csv(out, t, q_ref; q_cmd=q_new))
