# Firmware code that can run off the ATOM (on the Pi): test signals (test_signal.h), MOVE_TO/JOG
# (motion.h) and Improv WiFi (improv.h), checked by tools/firmware-tests/*.cpp. Skipped without a C++ compiler.

@testset "Firmware: test signals, MOVE_TO, JOG and Improv" begin
    @test length(MyCobot.pack_signal(MyCobot.SignalParams(1, "chirp"))) == 38   # sizeof(sig::Params)
    cxx = Sys.which("c++")
    if cxx === nothing
        @info "no C++ compiler: skipping the firmware tests"
    else
        root = joinpath(@__DIR__, "..")
        for test in ("test_signal_check.cpp", "test_motion_check.cpp", "test_improv_check.cpp")
            exe = tempname()
            run(`$cxx -std=c++17 -O1 -I $(joinpath(root, "firmware", "atom_controller")) $(joinpath(root, "tools", "firmware-tests", test)) -o $exe`)
            out = read(ignorestatus(`$exe`), String)
            print(out)
            @test occursin("all checks passed", out)
        end
    end
end
