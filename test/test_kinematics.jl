import MyCobot

using Test

const RBD = MyCobot.RBD
const norm = MyCobot.LinearAlgebra.norm

@testset "Kinematics (RigidBodyDynamics.jl)" begin
    mechanism = MyCobot.load_mechanism()
    state = RBD.MechanismState(mechanism)

    lo, hi = MyCobot.joint_limits_deg(mechanism)
    @test hi ≈ [165.0, 140.0, 150.0, 150.0, 160.0, 135.0] atol = 0.1
    @test lo ≈ [-165.0, -140.0, -150.0, -150.0, -160.0, -225.0] atol = 0.1

    # Flange position vs the ATOM's get_coords at the same joint angles (mm). The URDF and
    # the ATOM put the tool point ~1 cm apart, so this checks joint directions and lengths.
    for (q, atom_xyz) in [([0.87, 0.35, -0.79, -0.61, -0.87, 0.7], [48.0, -63.3, 420.0]),
                          ([3.07, -139.57, 153.1, -153.1, 87.01, -30.05], [96.8, -12.6, 90.0])]
        @test norm(MyCobot.flange_position_mm(state, q) - atom_xyz) < 12
    end
    @test MyCobot.flange_position_mm(state, zeros(6)) ≈ [45.6, -64.6, 411.1] atol = 0.1

    # IK recovers a known pose from a different starting guess
    q_true = [10.0, -25, -70, 95, -8, 5]
    T = MyCobot.flange_transform(state, q_true)
    goal = MyCobot.flange_goal(mechanism, RBD.rotation(T), MyCobot.flange_position_mm(state, q_true))
    q, pos_err, rot_err = MyCobot.inverse_kinematics(state, goal, [0, -30, -60, 90, 0, 0])
    @test q ≈ q_true atol = 1e-4
    @test pos_err < 1e-6 && rot_err < 1e-6
end

# The URDFs and parameter tables are generated from mycobot_description (tools/gen_robot.py).
# Skipped without uv (the Pi): the generator needs xacro.
@testset "Robot description is up to date" begin
    uv = Sys.which("uv")
    if uv === nothing
        @info "no uv: skipping the robot description check"
    else
        @test success(`$uv run -q $(joinpath(@__DIR__, "..", "tools", "gen_robot.py")) --check`)
    end
end
