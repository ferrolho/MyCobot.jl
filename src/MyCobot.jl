module MyCobot

include("serial/ProtocolCode.jl")
include("kinematics.jl")
include("feetech.jl")
include("ftdi.jl")
include("player.jl")
include("ilc.jl")

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

"""
    yieldsleep(t)

Yield the current thread for `t` seconds. This is useful because `sleep` blocks
the thread and struggles to be precise with values smaller than ~2 milliseconds.
`yieldsleep` works okay only for values greater than 10 microseconds.

# Examples
```julia
yieldsleep(0.020)  # Yield for 20 milliseconds
```
"""
function yieldsleep(t)
    # @assert t >= 1e-5
    t1 = time()
    while time() - t1 < t
        yield()
    end
end

end # module MyCobot
