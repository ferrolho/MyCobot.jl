import MyCobot
import Random

using Test

@testset "IMU gravity calibration (simulated poses)" begin
    mechanism = MyCobot.load_mechanism()
    state = MyCobot.RBD.MechanismState(mechanism)
    rng = Random.MersenneTwister(3)
    Q = reduce(vcat, permutedims([150, 80, 120, 120, 150, 0] .* (2 .* rand(rng, 6) .- 1)) for _ in 1:50)
    # Known truth: 0.4° / -0.3° table tilt, a skewed IMU mounting, 1.5° pitch offset, 2 % scale, 20 mg bias.
    R0 = MyCobot.rotvec([0.1, -1.4, 0.3])
    p_true = [deg2rad(0.4), deg2rad(-0.3), 0, 0, 0, deg2rad(1.5), 0.02, -0.01, 0.015, 0.003, -0.002, 0.001, 0.02, -0.015, 0.01]
    acc = Matrix(MyCobot.imu_gravity_model(state, Q, p_true, R0)') .+ 0.0005 .* randn(rng, 50, 3)
    fit = MyCobot.fit_imu_gravity(Q, acc; mechanism=mechanism)
    @test rad2deg(fit.p[6]) ≈ 1.5 atol = 0.05
    @test rad2deg.(fit.p[1:2]) ≈ [0.4, -0.3] atol = 0.05
    @test fit.p[7:9] ≈ p_true[7:9] atol = 0.002
    @test fit.p[13:15] ≈ p_true[13:15] atol = 0.002
    @test fit.rms_g < 0.001
    @test all(fit.σ .< 0.01)
end
