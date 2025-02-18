"""
"""
function run_for_duration(fn::Function, duration::Real)
    start_time = time()
    while time() - start_time < duration
        fn()
    end
end
