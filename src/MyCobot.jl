module MyCobot

include("serial/ProtocolCode.jl")

"""
    run_for_duration(fn::Function, duration::Real)

Run the given function for the specified duration.
"""
function run_for_duration(fn::Function, duration::Real)
    start_time = time()
    while time() - start_time < duration
        fn()
    end
end

end # module MyCobot
