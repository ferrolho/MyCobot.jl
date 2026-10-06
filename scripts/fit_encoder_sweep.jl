# Encoder error from a gyro sweep (scripts/encoder_sweep.jl).
#
#   julia --project=. scripts/fit_encoder_sweep.jl FILE_J<j>_sweep.csv [...] [--plot=DIR]
#
# Only joint j moves, so the IMU link turns about one fixed axis (in IMU coordinates). The axis is
# the main direction of the gyro signal. On the main pass (−A to +A) the gyro rate about it, minus
# the bias (a straight line in time between the holds at −A and +A), is integrated and compared
# with the encoder angle. The difference is
# fitted as gyro scale + once- and twice-per-turn encoder error:
#   q_encoder − q_start − θ_gyro = k·Δq + a1 sin q + b1 cos q + a2 sin 2q + b2 cos 2q + c
# Over less than about 250° of range, sin q looks like a straight line and k cannot be told from
# the encoder error. So the sweeps with a long range (≥ 250°) give the gyro scale of their IMU
# axis (the mean if there are several), and every sweep about that axis uses it. --plot writes the error against the angle (CSV).
# Prints the correction terms for mycobot_description/config/mycobot_280_arduino/calibration.yaml.
# Only the samples where the other joints hold the base pose count (encoder_sweep.jl --base).

import DelimitedFiles, Printf, Statistics as S
import MyCobot
const LA = MyCobot.LA

opt(name, default) = (a = findfirst(startswith("--$name="), ARGS); a === nothing ? default : split(ARGS[a], "=")[2])
files = filter(a -> !startswith(a, "--"), ARGS)
plotdir = opt("plot", "")

function load_sweep(f)
    data, header = DelimitedFiles.readdlm(f, ',', Float64; header=true)
    col(n) = findfirst(==(n), vec(header))
    j = parse(Int, match(r"_J(\d)_sweep", f)[1])
    t = data[:, col("t")]; q = data[:, col("q_$j")]; plan = data[:, col("q_plan_$j")]
    ω = data[:, [col("gyro_x"), col("gyro_y"), col("gyro_z")]]
    # The sweep: the samples where every other joint holds the base pose (its plan value at the
    # middle of the run), and the robot is not on its way to or from it.
    others = data[:, [col("q_plan_$k") for k in 1:6 if k != j]]
    mid = others[argmax(abs.(plan .- plan[1])), :]
    ok = .!isnan.(q) .& vec(all(abs.(others .- mid') .< 1e-6; dims=2))
    t, q, plan, ω = t[ok], q[ok], plan[ok], ω[ok, :]
    # The main pass: from the hold at −A to the hold at +A. The gyro bias goes in a straight line
    # in time from the mean of the first hold to the mean of the second. (The passes to and from
    # the base pose are left out: over a whole run the gyro drifts by 0.5–1.5°.)
    still = vcat(false, abs.(diff(plan)) .< 1e-9)
    lo_hold = findall(still .& (abs.(plan .- minimum(plan)) .< 1e-6))
    hi_hold = findall(still .& (abs.(plan .- maximum(plan)) .< 1e-6))
    pass = lo_hold[end]:hi_hold[1]
    b_lo, b_hi = vec(S.mean(ω[lo_hold, :]; dims=1)), vec(S.mean(ω[hi_hold, :]; dims=1))
    t_lo, t_hi = S.mean(t[lo_hold]), S.mean(t[hi_hold])
    t, q = t[pass], q[pass]
    w = ω[pass, :] .- (b_lo' .+ (t .- t_lo) ./ (t_hi - t_lo) .* (b_hi .- b_lo)')
    axis = LA.svd(w).V[:, 1]                                     # main rotation axis (IMU frame)
    rate = w * axis
    LA.dot(rate[2:end], diff(q)) < 0 && (rate .*= -1; axis .*= -1)
    θ = vcat(0.0, cumsum((rate[1:end-1] .+ rate[2:end]) ./ 2 .* diff(t)))   # trapezoid
    e = (q .- q[1]) .- θ
    return (j=j, t=t, q=q, e=e, axis=axis, gyro_axis=argmax(abs.(axis)), range=maximum(q) - minimum(q), bias=b_lo)
end

harmonics(q) = hcat(sind.(q), cosd.(q), sind.(2q), cosd.(2q), ones(length(q)))
sweeps = [load_sweep(f) for f in files]
ks = Dict{Int,Vector{Float64}}()                                  # k per IMU axis, from long sweeps
for s in sweeps
    s.range >= 250 || continue
    c = hcat(s.q .- s.q[1], harmonics(s.q)) \ s.e
    push!(get!(ks, s.gyro_axis, Float64[]), c[1])
end
scale = Dict(a => sum(v) / length(v) for (a, v) in ks)            # the mean of the long sweeps
coefs = Dict{Int,Vector{Vector{Float64}}}()                        # encoder error terms per joint, per run
for s in sort(sweeps; by=s -> s.j)
    haskey(scale, s.gyro_axis) || (println("J$(s.j): no long sweep about IMU axis $(s.gyro_axis): skipped"); continue)
    k = scale[s.gyro_axis]
    e = s.e .- k .* (s.q .- s.q[1])
    c = harmonics(s.q) \ e
    res = e .- harmonics(s.q) * c
    err = harmonics(s.q)[:, 1:4] * c[1:4]
    Printf.@printf("J%d: %.0f…%.0f°, gyro axis %s (scale %+.2f %%): encoder error 1×/turn %.2f°, 2×/turn %.2f°, peak-to-peak %.2f° over the sweep; residual RMS %.3f°\n",
                   s.j, minimum(s.q), maximum(s.q), "xyz"[s.gyro_axis], -100k, hypot(c[1], c[2]), hypot(c[3], c[4]), maximum(err) - minimum(err), sqrt(S.mean(res .^ 2)))
    push!(get!(coefs, s.j, Vector{Float64}[]), c[1:4])
    if !isempty(plotdir)
        mkpath(plotdir)
        DelimitedFiles.writedlm(joinpath(plotdir, "J$(s.j)_encoder_error.csv"), hcat(s.t, s.q, e .- c[5], err), ',')
    end
end

# The correction for calibration.yaml: the mean over the runs of each joint (with e(0) = 0, the
# constant terms do not matter: true angle = encoder angle − Σ a_k sin(k q) − Σ b_k (cos(k q) − 1)).
for (j, cs) in sort(collect(coefs))
    m = sum(cs) / length(cs)
    spread = length(cs) > 1 ? maximum(maximum(abs.(c .- m)) for c in cs) : NaN
    Printf.@printf("J%d correction (%d run%s): sin: [%.3f, %.3f]  cos: [%.3f, %.3f]  (largest difference from the mean %.3f°)\n",
                   j, length(cs), length(cs) == 1 ? "" : "s", m[1], m[3], m[2], m[4], spread)
end
