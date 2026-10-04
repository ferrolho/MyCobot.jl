# Plan a vertical circle in front of the robot with the flange orientation held fixed,
# using RigidBodyDynamics.jl for kinematics. Writes a joint trajectory (degrees) that
# tools/python/play_trajectory.py streams to the robot at ~300 Hz.
#
#   julia --project=. scripts/plan_circle.jl
#
# Plan: zero pose -> circle start (minimum-jerk in joint space) -> LAPS laps with the
# speed eased in and out -> back to zero. Checks joint limits, joint speeds and height.

import MyCobot
import RigidBodyDynamics as RBD

const CENTER_MM = [170.0, -64.6, 310.0]   # in front of the robot (+x), wrist-offset y
const RADIUS_MM = 50.0
const LAPS = 2
const LAP_TIME = 4.0         # s per lap at full speed
const RAMP = 1.0             # s, speed ease in/out on the circle
const APPROACH_TIME = 3.0    # s, zero -> circle start and back
const DT = 0.002             # s, plan resolution
const LIMIT_MARGIN = 10.0    # deg
const MAX_JOINT_SPEED = 90.0 # deg/s
const MIN_Z_MM = 120.0
const OUTPUT = joinpath(@__DIR__, "..", "tools", "python", "plans", "circle.csv")

minjerk(s) = (s = clamp(s, 0, 1); s^3 * (10 - 15s + 6s^2))

# Angle along the circle: angular speed eased in and out, scaled to exactly LAPS turns
function circle_phase()
    total = LAPS * LAP_TIME + RAMP
    ts = collect(0:DT:total)
    env = @. minjerk(ts / RAMP) * minjerk((total - ts) / RAMP)
    θ = cumsum(vcat(0.0, env[2:end] .* DT))
    return ts, θ .* (LAPS * 2π / θ[end])
end

# Vertical circle in the y–z plane, starting at the right-most point
circle_point_mm(θ) = CENTER_MM .+ RADIUS_MM .* [0.0, -cos(θ), sin(θ)]

function plan()
    mechanism = MyCobot.load_mechanism()
    state = RBD.MechanismState(mechanism)
    R = RBD.rotation(MyCobot.flange_transform(state, zeros(6)))   # keep the zero-pose orientation
    goal(θ) = MyCobot.flange_goal(mechanism, R, circle_point_mm(θ))

    q_start, pe, re = MyCobot.inverse_kinematics(state, goal(0.0), [0, -30, -60, 90, 0, 0]; max_iters=300)
    @assert pe < 1e-4 && re < 1e-3 "IK failed for the circle start ($(1000pe) mm)"

    ts, θs = circle_phase()
    q_circle = zeros(length(θs), 6)
    q = q_start
    worst = 0.0
    for (i, θ) in enumerate(θs)
        q, pe, _ = MyCobot.inverse_kinematics(state, goal(θ), q)
        worst = max(worst, pe)
        q_circle[i, :] = q
    end
    @assert worst < 1e-4 "IK residual $(1000worst) mm"

    t_app = collect(0:DT:APPROACH_TIME - DT)
    s = minjerk.(t_app ./ APPROACH_TIME)
    q_in = s .* q_start'
    q_out = (1 .- s) .* q_circle[end, :]'
    q_all = vcat(q_in, q_circle, q_out, zeros(1, 6))
    t_all = (0:size(q_all, 1) - 1) .* DT

    # Checks
    lo, hi = MyCobot.joint_limits_deg(mechanism)
    @assert all(lo' .+ LIMIT_MARGIN .< q_all .< hi' .- LIMIT_MARGIN) "too close to a joint limit"
    dq = diff(q_all; dims=1) ./ DT
    @assert maximum(abs, dq) < MAX_JOINT_SPEED "joint speed $(maximum(abs, dq)) °/s too high"
    z = [MyCobot.flange_position_mm(state, q_all[i, :])[3] for i in 1:25:size(q_all, 1)]
    @assert minimum(z) > MIN_Z_MM "flange gets too low ($(minimum(z)) mm)"

    println("plan: $(round(t_all[end], digits=1)) s, IK residual ≤ $(round(1000worst, sigdigits=2)) mm, ",
            "flange z $(round(Int, minimum(z)))–$(round(Int, maximum(z))) mm")
    println("  joint range (°): ", [string(round(Int, a), "..", round(Int, b)) for (a, b) in zip(minimum(q_all; dims=1), maximum(q_all; dims=1))])
    println("  max joint speed (°/s): ", round.(vec(maximum(abs, dq; dims=1)), digits=1))
    return t_all, q_all
end

t_all, q_all = plan()
println("saved ", normpath(MyCobot.write_plan_csv(OUTPUT, t_all, q_all)))
