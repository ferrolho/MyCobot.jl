import MyCobot

using Test

include("simulated_bus.jl")

hexbytes(s) = parse.(UInt8, split(s), base=16)

@testset "Feetech packets (bytes captured from the robot)" begin
    # READ 2 bytes at register 56 (position) from servo 1, and its reply (position 2176)
    @test MyCobot.ft_packet(1, MyCobot.Feetech.READ, [56, 2]) == hexbytes("ff ff 01 04 02 38 02 be")
    pkts = MyCobot.parse_status_packets(hexbytes("ff ff 01 04 00 80 08 72"))
    @test pkts == [(0x01, 0x00, UInt8[0x80, 0x08])]
    @test MyCobot.u16(0x80, 0x08) == 2176

    # The ATOM's SYNC READ of 15 bytes from register 56 for servos 1–6
    @test MyCobot.ft_packet(0xFE, MyCobot.Feetech.SYNC_READ, [56, 15, 1, 2, 3, 4, 5, 6]) ==
          hexbytes("ff ff fe 0a 82 38 0f 01 02 03 04 05 06 19")

    # One servo's reply to that request
    reply = hexbytes("ff ff 01 11 00 80 08 00 00 00 00 4c 19 00 00 00 80 08 00 00 78")
    (id, err, data), = MyCobot.parse_status_packets(reply)
    @test (id, err, length(data)) == (0x01, 0x00, 15)
    @test MyCobot.u16(data[1], data[2]) == 2176   # position
    @test data[7] == 76                            # voltage 7.6 V
    @test data[8] == 25                            # temperature 25 °C

    # The ATOM's SYNC WRITE for send_angles(zeros, 30): acceleration 50, goal 2048,
    # goal time 0, per-joint goal speed, starting at register 41
    io = RecordingIO()
    speeds = [300, 467, 510, 345, 360, 300]
    MyCobot.ft_sync_write(io, MyCobot.Feetech.REG_ACCELERATION,
                          (UInt8(j) => UInt8[50, MyCobot.le16(2048)..., 0, 0, MyCobot.le16(speeds[j])...] for j in 1:6))
    @test only(io.written) == hexbytes("ff ff fe 34 83 29 07 01 32 00 08 00 00 2c 01 02 32 00 08 00 00 d3 01 " *
                                       "03 32 00 08 00 00 fe 01 04 32 00 08 00 00 59 01 05 32 00 08 00 00 68 01 " *
                                       "06 32 00 08 00 00 2c 01 b9")
end

@testset "Feetech parser robustness" begin
    good = hexbytes("ff ff 01 04 00 80 08 72")
    atom = hexbytes("fe fe 0e 20 fb 9b c9 7b 3b ce c4 20 21 fd f4 43 fa")
    corrupt = copy(good); corrupt[end] = 0x00
    @test MyCobot.parse_status_packets(vcat(UInt8[0x00, 0xFF], atom, good)) == [(0x01, 0x00, UInt8[0x80, 0x08])]
    @test isempty(MyCobot.parse_status_packets(corrupt))
    @test isempty(MyCobot.parse_status_packets(good[1:end-1]))
    @test length(MyCobot.parse_status_packets(vcat(good, good))) == 2
end

@testset "Encodings and angle conversion" begin
    @test MyCobot.le16(2048) == UInt8[0x00, 0x08]
    @test MyCobot.decode_signed15(0x8064) == -100
    @test MyCobot.decode_signed15(0x0064) == 100
    @test MyCobot.decode_load(0x0421) ≈ -3.3        # bit 10 = direction, 0.1 % units
    @test all(MyCobot.angle_to_position(j, 0.0) == 2048 for j in 1:6)
    # Readings taken at the same pose: direct servo positions vs the ATOM's get_angles
    @test MyCobot.position_to_angle(1, 2134) ≈ -7.47 atol = 0.1
    @test MyCobot.position_to_angle(3, 3790) ≈ 153.1 atol = 0.1
    for j in 1:6, deg in (-90.0, -12.3, 0.0, 45.6)
        @test MyCobot.position_to_angle(j, MyCobot.angle_to_position(j, deg)) ≈ deg atol = 360 / 4096
    end
end

@testset "Simulated bus: requests" begin
    bus = SimulatedBus(positions=Dict(1 => 2100, 2 => 2048, 3 => 2048, 4 => 2048, 5 => 2048, 6 => 2000))
    @test MyCobot.ft_read(bus, 1, 56, 2) == MyCobot.le16(2100)
    @test MyCobot.ft_write(bus, 1, 41, [5]) == 0x00
    @test MyCobot.ft_read(bus, 1, 41, 1) == UInt8[5]
    s = MyCobot.read_state(bus)
    @test s.ok
    @test s.q[1] ≈ MyCobot.position_to_angle(1, 2100)
    @test s.q[6] ≈ MyCobot.position_to_angle(6, 2000)
    @test s.dq == zeros(6)

    # With goal speed 0 a new goal is accepted but nothing moves
    MyCobot.write_goals(bus, fill(10.0, 6))
    sleep(0.05)
    @test MyCobot.read_state(bus).q[1] ≈ MyCobot.position_to_angle(1, 2100)

    # enable_motion holds every joint first, then sets the speed cap
    MyCobot.enable_motion(bus; speed_cap=1000)
    @test all(get16(bus, id, 42) == get16(bus, id, 56) for id in SIM_IDS)
    @test all(get16(bus, id, 46) == 1000 for id in SIM_IDS)
    MyCobot.write_goals(bus, zeros(6))
    sleep(0.2)
    @test maximum(abs, MyCobot.read_state(bus).q) < 0.5
    MyCobot.disable_motion(bus)
    @test all(get16(bus, id, 46) == 0 for id in SIM_IDS)

    # A missing servo makes read_state report !ok instead of throwing
    delete!(bus.regs, 4)
    s = MyCobot.read_state(bus)
    @test !s.ok && isnan(s.q[4])
end

@testset "Trajectory player on the simulated bus" begin
    mechanism = MyCobot.load_mechanism()
    t_plan = collect(0:0.01:1.0)
    bump = @. 10 * sin(π * t_plan)^2                  # J1 out to 10° and back
    q_plan = zeros(length(t_plan), 6)
    q_plan[:, 1] = bump

    @test MyCobot.sample_trajectory(t_plan, q_plan, 0.5)[1] ≈ 10.0
    @test MyCobot.sample_trajectory(t_plan, q_plan, 0.505)[1] ≈ (q_plan[51, 1] + q_plan[52, 1]) / 2
    @test MyCobot.sample_trajectory(t_plan, q_plan, 5.0) == q_plan[end, :]

    bus = SimulatedBus()
    rec, aborted = MyCobot.play_trajectory(bus, t_plan, q_plan; tail=0.2, mechanism=mechanism)
    @test aborted === nothing
    @test size(rec, 2) == length(MyCobot.RECORDING_HEADER)
    @test size(rec, 1) > 100
    q1 = rec[:, findfirst(==("q_1"), MyCobot.RECORDING_HEADER)]
    @test maximum(q1) > 8                               # J1 actually moved out
    @test abs(q1[end]) < 0.5                            # and came back
    @test all(get16(bus, id, 46) == 0 for id in SIM_IDS) # goal speed reset at the end

    path = MyCobot.write_recording_csv(joinpath(mktempdir(), "rec.csv"), rec)
    @test startswith(readline(path), "t,q_plan_1,")

    # A stuck joint trips the tracking-error abort, and every joint then holds where it is
    bus = SimulatedBus(stuck=Set([1]))
    rec, aborted = MyCobot.play_trajectory(bus, t_plan, q_plan; tail=0.2, max_tracking_error=3.0, mechanism=mechanism)
    @test occursin("J1", aborted)
    @test all(get16(bus, id, 42) == get16(bus, id, 56) for id in SIM_IDS)
    @test all(get16(bus, id, 46) == 0 for id in SIM_IDS)

    # Refuses to start away from the zero pose, and rejects bad plans
    bus = SimulatedBus(positions=Dict(id => (id == 2 ? 2300 : 2048) for id in SIM_IDS))
    @test_throws ErrorException MyCobot.play_trajectory(bus, t_plan, q_plan; mechanism=mechanism)
    fast = copy(q_plan); fast[:, 1] .*= 20
    @test_throws ArgumentError MyCobot.check_plan(t_plan, fast, mechanism)
    offset = copy(q_plan); offset[:, 2] .+= 5
    @test_throws ArgumentError MyCobot.check_plan(t_plan, offset, mechanism)
end

@testset "Iterative learning control" begin
    # Plant: the command reaches the joint after a delay and a first-order lag (the shape
    # fitted to J1/J3), plus a sticking zone after reversals that a time shift can't fix.
    dt = 0.002
    t = collect(0:dt:6.0)
    w = MyCobot.taper_window(t, 0.5)
    ref = @. 20 * sin(2π * 0.25 * t) * w
    function plant(cmd; delay=0.04, T=0.08, stick=0.15)
        k = round(Int, delay / dt)
        u = vcat(fill(cmd[1], k), cmd[1:end-k])
        y = similar(u); y[1] = u[1]
        held = 0.0
        for i in 2:length(u)
            target = y[i-1] + dt / (T + dt) * (u[i] - y[i-1])
            reversing = i > 2 && sign(target - y[i-1]) != sign(y[i-1] - y[i-2]) && y[i-1] != y[i-2]
            reversing && (held = stick)
            held > 0 ? (held -= dt; y[i] = y[i-1]) : (y[i] = target)
        end
        return y
    end
    q_ref = repeat(ref, 1, 6)
    lag = fill(0.12, 6)
    q_cmd = reduce(vcat, permutedims([MyCobot.sample_trajectory(t, q_ref, ti + lag[j])[j] for j in 1:6]) for ti in t)
    q_cmd0 = copy(q_cmd)
    errors = Float64[]
    for k in 1:6
        y = plant(q_cmd[:, 1])
        push!(errors, sqrt(sum(abs2, y .- ref) / length(t)))
        q_cmd = MyCobot.ilc_update(t, q_ref, q_cmd, t, repeat(y, 1, 6); lead=lag)
    end
    @test errors[end] < 0.5 * errors[1]
    @test all(diff(errors) .< 0)                # improves every iteration
    @test q_cmd[1, :] ≈ q_cmd0[1, :] && q_cmd[end, :] ≈ q_cmd0[end, :]   # learning leaves the ends alone

    # Plan files with explicit commands round-trip
    path = MyCobot.write_plan_csv(joinpath(mktempdir(), "p.csv"), t, q_ref; q_cmd=q_cmd)
    t2, q2, c2 = MyCobot.read_plan_csv(path)
    @test t2 ≈ t && q2 ≈ q_ref && c2 ≈ q_cmd
    _, _, c3 = MyCobot.read_plan_csv(MyCobot.write_plan_csv(joinpath(mktempdir(), "p.csv"), t, q_ref))
    @test c3 === nothing
    println("ILC on the model plant, RMS error per iteration (°): ", round.(errors, digits=3))
end
