# Test signals: the Julia version (src/signals.jl) against the firmware version
# (firmware/atom_controller/test_signal.h), compiled for the laptop if a C++ compiler is present.

import DelimitedFiles

@testset "Test signals (Julia vs firmware)" begin
    chirp = MyCobot.SignalParams(1, "chirp"; amp=10.0, duration=24.0, vmax=120.0, amax=300.0)
    steps = MyCobot.SignalParams(2, "steps"; amp=5.0, duration=24.0, vmax=60.0, amax=400.0, base=[0, -30.0, -60, 0, 0, 0])
    starts = ([0.0, 0, 0, 0, 0, 0], [0.5, -0.3, 0.2, 0, 0, 0])

    # Plans pass the player checks
    m = MyCobot.load_mechanism()
    for (p, s) in zip((chirp, steps), starts)
        t, q = MyCobot.signal_plan(p, s)
        @test q[1, :] ≈ s
        @test q[end, :] ≈ s
        @test isnothing(MyCobot.check_plan(t, q, m; max_joint_speed=p.vmax + 1))
    end

    cxx = Sys.which("c++")
    if cxx === nothing
        @info "no C++ compiler: skipping the firmware comparison"
    else
        root = joinpath(@__DIR__, "..")
        exe = tempname()
        run(`$cxx -std=c++17 -O1 -I $(joinpath(root, "firmware", "atom_controller")) $(joinpath(root, "tools", "firmware-tests", "signal_dump.cpp")) -o $exe`)
        lines = filter(l -> !startswith(l, "#"), readlines(`$exe`))
        header = filter(l -> startswith(l, "#"), readlines(`$exe`))
        @test all(occursin("validate 0 start 0", h) for h in header)
        d = reduce(vcat, permutedims(parse.(Float64, split(l, ","))) for l in lines)
        for (c, (p, s)) in enumerate(zip((chirp, steps), starts))
            rows = d[d[:, 1] .== c - 1, :]
            err = maximum(maximum(abs.(MyCobot.signal_pose(p, s, r[2]) .- r[3:8])) for r in eachrow(rows))
            @test err < 0.01   # degrees (float32 in the firmware)
        end
    end
end
