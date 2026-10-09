# The ATOM link's message decoding (firmware 5.0: 6 or 7 joints), without the robot.
using Test
using MyCobot

# One TELEM sample as the firmware packs it: u32 t_us, u16 cmd[n], ref[n], pos[n], spd[n], load[n],
# i16 acc[3], gyro[3], u8 ok.
function telem_sample(t_us, cmd, ref, pos; ok=0x01)
    n = length(pos)
    steps(q) = [UInt16(MyCobot.angle_to_position(j, q[j])) for j in 1:n]
    b = UInt8[]
    append!(b, reinterpret(UInt8, [htol(UInt32(t_us))]))
    for a in (steps(cmd), steps(ref), steps(pos), zeros(UInt16, n), zeros(UInt16, n))
        append!(b, reinterpret(UInt8, htol.(a)))
    end
    append!(b, reinterpret(UInt8, htol.(Int16[0, 0, 4096, 0, 0, 0])))
    push!(b, ok)
    return b
end

@testset "ATOM telemetry: 6 and 7 joints (firmware 5.0)" begin
    q6 = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0]
    s6 = telem_sample(1000, q6, q6, q6)
    @test length(s6) == 77
    r6 = MyCobot.decode_telemetry([s6])
    @test size(r6, 2) == length(MyCobot.recording_header(6)) + 6
    @test isapprox(r6[1, 14:19], q6; atol=0.1)                 # q_1..q_6 where the analysis scripts read them

    q7 = vcat(q6, -30.0)
    s7 = telem_sample(2000, q7, q7, q7)
    @test length(s7) == 87
    r7 = MyCobot.decode_telemetry([s7])
    h7 = MyCobot.recording_header(7)
    @test size(r7, 2) == length(h7) + 6
    @test isapprox(r7[1, findfirst(==("q_7"), h7)], -30.0; atol=0.1)
    @test isapprox(r7[1, findfirst(==("q_plan_7"), h7)], -30.0; atol=0.1)
    @test r7[1, end - 3] ≈ 1.0                                 # acc_z: 1 g

    @test MyCobot.JOINT_LIMITS_DEG[7] == (-51.5039, 0.0)       # J7: the gripper's end stops
    @test MyCobot.position_to_angle(7, 2048) == 0.0            # J7: no encoder correction, sign +1
    @test MyCobot.angle_to_position(7, -51.5039) == 1462

    # A goal read from the ATOM (corrected angle) goes back to the same servo step (atom_move!).
    for j in 1:7, step in (1500, 2048, 2600)
        q = MyCobot.position_to_angle(j, step)
        @test MyCobot.angle_to_position(j, q) == step
    end
end
