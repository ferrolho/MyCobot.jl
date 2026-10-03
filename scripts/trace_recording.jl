# Flange path of a recording from tools/python/play_trajectory.py: forward kinematics
# (RigidBodyDynamics.jl) of the planned and the measured joint angles.
#
#   julia --project=. scripts/trace_recording.jl tools/python/recordings/<file>.csv
#
# Writes <file>_path.csv with columns t, plan_x/y/z, meas_x/y/z (mm), which
# tools/python/plot_circle.py plots.

import DelimitedFiles
import MyCobot
import RigidBodyDynamics as RBD

for path in ARGS
    data, header = DelimitedFiles.readdlm(path, ',', Float64; header=true)
    col(name) = findfirst(==(name), vec(header))
    plan_cols = [col("q_plan_$j") for j in 1:6]
    meas_cols = [col("q_$j") for j in 1:6]
    rows = filter(i -> !any(isnan, data[i, meas_cols]), 1:size(data, 1))

    state = RBD.MechanismState(MyCobot.load_mechanism())
    out = zeros(length(rows), 7)
    for (k, i) in enumerate(rows)
        out[k, 1] = data[i, col("t")]
        out[k, 2:4] = MyCobot.flange_position_mm(state, data[i, plan_cols])
        out[k, 5:7] = MyCobot.flange_position_mm(state, data[i, meas_cols])
    end

    err = sqrt.(sum(abs2, out[:, 5:7] .- out[:, 2:4]; dims=2))
    x_circle = sort(out[:, 2])[cld(size(out, 1), 2)]       # median planned x: the circle plane
    on_circle = abs.(out[:, 2] .- x_circle) .< 0.5
    rms = sqrt(sum(abs2, err[on_circle]) / count(on_circle))
    println("$(basename(path)): Cartesian error on the circle RMS $(round(rms, digits=1)) mm, ",
            "max $(round(maximum(err[on_circle]), digits=1)) mm; whole run max $(round(maximum(err), digits=1)) mm")

    out_path = replace(path, r"\.csv$" => "_path.csv")
    open(out_path, "w") do io
        println(io, "t,plan_x,plan_y,plan_z,meas_x,meas_y,meas_z")
        DelimitedFiles.writedlm(io, out, ',')
    end
    println("saved ", out_path)
end
