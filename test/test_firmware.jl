# Firmware code that can run on the laptop: the onboard test signals (test_signal.h), checked by
# tools/firmware-tests/test_signal_check.cpp. Skipped without a C++ compiler.

@testset "Firmware: onboard test signals" begin
    @test length(MyCobot.pack_signal(MyCobot.SignalParams(1, "chirp"))) == 38   # sizeof(sig::Params)
    cxx = Sys.which("c++")
    if cxx === nothing
        @info "no C++ compiler: skipping the firmware tests"
    else
        root = joinpath(@__DIR__, "..")
        exe = tempname()
        run(`$cxx -std=c++17 -O1 -I $(joinpath(root, "firmware", "atom_controller")) $(joinpath(root, "tools", "firmware-tests", "test_signal_check.cpp")) -o $exe`)
        out = read(ignorestatus(`$exe`), String)
        print(out)
        @test occursin("all checks passed", out)
    end
end
