# Which joints cause the Cartesian tracking error in a play_trajectory.py recording?
# For each joint j, the flange is moved with only joint j at its measured angle (the
# others at their planned angles); the distance from the planned flange position is
# joint j's contribution. Kinematics: RigidBodyDynamics.jl via MyCobot.flange_position_mm.
#
#   julia --project=. scripts/attribute_error.jl tools/python/recordings/<file>.csv

import DelimitedFiles
import LinearAlgebra
import MyCobot
import RigidBodyDynamics as RBD

path = only(ARGS)
data, header = DelimitedFiles.readdlm(path, ',', Float64; header=true)
col(name) = findfirst(==(name), vec(header))
t = data[:, col("t")]
q_plan = data[:, [col("q_plan_$j") for j in 1:6]]
q_meas = data[:, [col("q_$j") for j in 1:6]]
rows = filter(i -> !any(isnan, q_meas[i, :]), 1:size(data, 1))

state = RBD.MechanismState(MyCobot.load_mechanism())
total = zeros(length(rows))
per_joint = zeros(length(rows), 6)
for (k, i) in enumerate(rows)
    p_plan = MyCobot.flange_position_mm(state, q_plan[i, :])
    total[k] = LinearAlgebra.norm(MyCobot.flange_position_mm(state, q_meas[i, :]) - p_plan)
    for j in 1:6
        q = copy(q_plan[i, :])
        q[j] = q_meas[i, j]
        per_joint[k, j] = LinearAlgebra.norm(MyCobot.flange_position_mm(state, q) - p_plan)
    end
end

rms(x) = sqrt(sum(abs2, x) / length(x))
println("RMS Cartesian error over the whole run: $(round(rms(total), digits=1)) mm")
println("RMS contribution per joint (mm): ", [string("J$j ", round(rms(per_joint[:, j]), digits=1)) for j in 1:6])

# The largest error peaks, at least 0.5 s apart
order = sortperm(total; rev=true)
peaks = Int[]
for k in order
    all(abs(t[rows[k]] - t[rows[p]]) > 0.5 for p in peaks) && push!(peaks, k)
    length(peaks) == 4 && break
end
println("\nLargest peaks:")
for k in sort(peaks; by=k -> t[rows[k]])
    i = rows[k]
    println("  t=$(round(t[i], digits=2)) s  total $(round(total[k], digits=1)) mm  <- ",
            join(["J$j $(round(per_joint[k, j], digits=1))" for j in sortperm(per_joint[k, :]; rev=true)[1:3]], ", "),
            " mm  (joint errors °: ", round.(q_meas[i, :] .- q_plan[i, :], digits=2), ")")
end
