# Fit the IMU gravity model (src/imu_calibration.jl) to static-pose recordings.
#
#   julia --project=. scripts/fit_imu_gravity.jl FILE_static_samples.csv [MORE.csv ...] [--t-min=2] [--max-std=0.01]
#
# Uses the mean of each hold (samples at t ≥ --t-min s). Leaves out holds where the accelerometer
# moved (standard deviation above --max-std g on any axis). Prints the parameters with their
# standard deviations, a hold-out check (fit on half the poses, predict the other half), and the
# gyro bias.

import Printf
import MyCobot

opt(name, default) = (a = findfirst(startswith("--$name="), ARGS); a === nothing ? default : split(ARGS[a], "=")[2])
files = filter(a -> !startswith(a, "--"), ARGS)
t_min = parse(Float64, opt("t-min", "2"))
max_std = parse(Float64, opt("max-std", "0.01"))

parts = [MyCobot.static_pose_means(f; t_min=t_min) for f in files]
Q = reduce(vcat, [p.q for p in parts]); acc = reduce(vcat, [p.acc for p in parts])
gyro = reduce(vcat, [p.gyro for p in parts]); acc_std = reduce(vcat, [p.acc_std for p in parts])
quiet = vec(maximum(acc_std; dims=2)) .<= max_std
println("$(size(Q, 1)) holds, $(count(quiet)) quiet (accelerometer std ≤ $max_std g)")
Q, acc, gyro = Q[quiet, :], acc[quiet, :], gyro[quiet, :]

mechanism = MyCobot.load_mechanism()
fit = MyCobot.fit_imu_gravity(Q, acc; mechanism=mechanism)
unit(k) = k in (1, 2, 3, 4, 5, 6) ? "°" : (k >= 13 ? " mg" : " %")
scale(k) = k <= 6 ? 180 / π : (k >= 13 ? 1000 : 100)
println("\nParameters (± 1 standard deviation)")
for (k, name) in enumerate(MyCobot.IMU_PARAM_NAMES)
    Printf.@printf("  %-12s %9.3f ± %.3f%s\n", name, fit.p[k] * scale(k), fit.σ[k] * scale(k), unit(k))
end
tilt = rad2deg(hypot(fit.p[1], fit.p[2]))
Printf.@printf("\nBase tilt %.3f° (towards %.0f° azimuth in the base frame)\n", tilt,
               mod(atand(-fit.p[1], fit.p[2]), 360))
Printf.@printf("Residual RMS %.2f mg, angle RMS %.3f°, max %.3f°\n", 1000 * fit.rms_g,
               sqrt(sum(abs2, fit.angle_deg) / length(fit.angle_deg)), maximum(fit.angle_deg))

# Hold-out: fit on the odd holds, predict the even ones.
odd, even = 1:2:size(Q, 1), 2:2:size(Q, 1)
f_odd = MyCobot.fit_imu_gravity(Q[odd, :], acc[odd, :]; mechanism=mechanism)
state = MyCobot.RBD.MechanismState(mechanism)
M = MyCobot.imu_gravity_model(state, Q[even, :], f_odd.p, f_odd.R0)
A = Matrix(acc[even, :]')
ang = [acosd(clamp(MyCobot.LA.dot(M[:, i], A[:, i]) / (MyCobot.LA.norm(M[:, i]) * MyCobot.LA.norm(A[:, i])), -1, 1)) for i in 1:size(A, 2)]
Printf.@printf("Hold-out (fit odd, predict even): angle RMS %.3f°, max %.3f°; delta_pitch %.3f° (all: %.3f°)\n",
               sqrt(sum(abs2, ang) / length(ang)), maximum(ang), rad2deg(f_odd.p[6]), rad2deg(fit.p[6]))
Printf.@printf("Gyro bias (mean of the holds): %.3f, %.3f, %.3f °/s\n", (sum(gyro; dims=1) ./ size(gyro, 1))...)
