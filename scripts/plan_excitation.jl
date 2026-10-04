# Plan an excitation trajectory for learning the servo errors: every joint moves at once,
# each as a sum of sines (5 harmonics of 0.1 Hz, random phases), faded in and out so that it
# starts and ends at the zero pose.
#
#   julia --project=. scripts/plan_excitation.jl [SEED] [--T=40] [--scale=1.0]
#
# The plan is scaled down until it passes all checks: joint limits (10° margin), speed
# ≤ 90°/s, acceleration ≤ 60 % of the servo limit (SERVO_AMAX), and every link origin and the
# flange at least 80 mm above the table. Writes tools/python/plans/excitation_<SEED>.csv.

import Random
import RigidBodyDynamics as RBD
import MyCobot

opt(name, default) = (a = findfirst(startswith("--$name="), ARGS); a === nothing ? default : split(ARGS[a], "=")[2])
pos = filter(a -> !startswith(a, "--"), ARGS)
seed = isempty(pos) ? 1 : parse(Int, pos[1])
T = parse(Float64, opt("T", "40"))
scale0 = parse(Float64, opt("scale", "1.0"))

const DT = 0.002
const AMP = [60.0, 35.0, 60.0, 70.0, 70.0, 90.0]   # target peak per joint (°), before scaling
const F0 = 0.1
const FADE = 4.0
const MIN_Z_MM = 80.0

mechanism = MyCobot.load_mechanism()
state = RBD.MechanismState(mechanism)
lo, hi = MyCobot.joint_limits_deg(mechanism)
bodies = [b for b in RBD.bodies(mechanism) if occursin(r"joint[3-6]|flange", string(b))]

function shape(seed, T)
    rng = Random.MersenneTwister(seed)
    t = collect(0:DT:T)
    env = @. clamp(min(t / FADE, (T - t) / FADE), 0, 1)
    env = @. env^3 * (10 - 15env + 6env^2)                       # smooth fade (minimum jerk)
    q = zeros(length(t), 6)
    for j in 1:6
        c = randn(rng, 5) ./ (1:5); φ = 2π .* rand(rng, 5)
        x = [sum(c[k] * sin(2π * k * F0 * ti + φ[k]) for k in 1:5) for ti in t]
        q[:, j] = AMP[j] .* env .* x ./ maximum(abs, x)
    end
    return t, q
end

function lowest_point(q)
    zmin = Inf
    for i in 1:25:size(q, 1)
        RBD.set_configuration!(state, deg2rad.(q[i, :]))
        for b in bodies
            zmin = min(zmin, 1000 * RBD.translation(RBD.transform_to_root(state, b))[3])
        end
        zmin = min(zmin, MyCobot.flange_position_mm(state, q[i, :])[3])
    end
    return zmin
end

t, q0 = shape(seed, T)
# Scale each joint to its own speed and acceleration limits first, then all joints together
# for the joint limits and the table.
dq0, ddq0 = MyCobot.plan_derivatives(t, q0)
for j in 1:6
    sj = min(1.0, 85 / maximum(abs, dq0[:, j]), 0.6 * MyCobot.SERVO_AMAX[j] / maximum(abs, ddq0[:, j]))
    q0[:, j] .*= sj
end
scale = scale0
local q
while true
    global q = scale .* q0
    dq, ddq = MyCobot.plan_derivatives(t, q)
    ok_lim = all(lo' .+ 10 .< q .< hi' .- 10)
    ok_v = maximum(abs, dq) <= 90
    ok_a = all(vec(maximum(abs, ddq; dims=1)) .<= 0.6 .* MyCobot.SERVO_AMAX)
    zmin = lowest_point(q)
    ok_lim && ok_v && ok_a && zmin >= MIN_Z_MM && break
    global scale *= 0.95
    scale > 0.1 || error("cannot find a safe scale")
end
dq, ddq = MyCobot.plan_derivatives(t, q)
MyCobot.check_plan(t, q, mechanism)
println("seed $seed, scale $(round(scale, digits=2)), lowest point $(round(lowest_point(q), digits=0)) mm")
println("  range (°):  ", [string(round(Int, minimum(q[:, j])), "..", round(Int, maximum(q[:, j]))) for j in 1:6])
println("  max speed (°/s): ", round.(vec(maximum(abs, dq; dims=1)), digits=0))
println("  max accel (°/s²): ", round.(vec(maximum(abs, ddq; dims=1)), digits=0))
path = MyCobot.write_plan_csv(joinpath(@__DIR__, "..", "tools", "python", "plans", "excitation_$seed.csv"), t, q)
println("saved ", path)
