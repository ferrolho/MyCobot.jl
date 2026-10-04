# Measure the mechanical range of one joint by hand.
#
#   julia --project=. scripts/measure_joint_range.jl JOINT|all [SECONDS] [--dry]
#
# Turns off the torque of JOINT (the other joints keep holding), or of all joints with "all",
# records the angles for SECONDS (default 30) while you move the joints by hand to their end
# stops, then holds them where they are and prints the ranges. --dry reads only: no torque change.
#
# SAFETY: support the arm before you release J2, J3 or J4, or all joints (gravity). Push gently and stop at the
# first hard stop. Don't send ATOM commands or press the ATOM button while this runs.
# After you turn J6 more than half a turn, power-cycle the arm before the next motion (J6 then
# turned away from its goal; see the docs, reference/gotchas).

import Dates
import LibSerialPort
import MyCobot
const FT = MyCobot          # bus functions
const REG = MyCobot.Feetech # register addresses

const PORT = "/dev/tty.usbserial-B00033ZX"
const BAUDRATE = 1_000_000
const OUT = joinpath(@__DIR__, "..", "tools", "python", "recordings", "joint_ranges.csv")

function main()
    args = filter(a -> !startswith(a, "--"), ARGS)
    dry = "--dry" in ARGS
    joints = args[1] == "all" ? collect(1:6) : [parse(Int, args[1])]
    duration = length(args) > 1 ? parse(Float64, args[2]) : 30.0
    all(j -> 1 <= j <= 6, joints) || error("JOINT must be 1–6 or all")

    println("FT232R latency timer: ", MyCobot.set_latency_timer(1), " ms")
    sp = LibSerialPort.open(PORT, BAUDRATE)
    q_min = fill(NaN, 6); q_max = fill(NaN, 6)
    released = Int[]
    try
        s = FT.read_state(sp)
        s.ok || error("not all servos replied: ", s.q)
        println("start: ", round.(s.q, digits=1))
        if !dry
            for j in joints
                FT.ft_write(sp, FT.SERVO_IDS[j], REG.REG_TORQUE_ENABLE, UInt8[0])
                push!(released, j)
            end
            println(">>> limp: ", join(("J$j" for j in joints), ", "), ". Move by hand to the end stops ($(duration) s).")
        end

        # Unwrap across the 0/4095 seam, so a joint that passes ±180° gives a continuous angle.
        q_prev = copy(s.q); q_cont = copy(s.q); q_min .= s.q; q_max .= s.q
        t0 = time(); last_print = 0.0
        while time() - t0 < duration
            st = FT.read_state(sp)
            for j in joints
                isnan(st.q[j]) && continue
                d = st.q[j] - q_prev[j]
                d > 180 && (d -= 360); d < -180 && (d += 360)
                q_prev[j] = st.q[j]; q_cont[j] += d
                q_min[j] = min(q_min[j], q_cont[j]); q_max[j] = max(q_max[j], q_cont[j])
            end
            if time() - last_print > 0.5
                last_print = time()
                println(rpad("  t=$(round(time() - t0, digits=1)) s", 12),
                        join(("J$j " * lpad(round(q_cont[j], digits=1), 6) * " [" * string(round(q_min[j], digits=1)) * ", " *
                              string(round(q_max[j], digits=1)) * "]" for j in joints), "  "))
            end
            sleep(0.02)
        end
    finally
        # Writing a goal turns the torque on: hold each released joint where it is now.
        for j in released
            try
                data = FT.ft_read(sp, FT.SERVO_IDS[j], REG.REG_PRESENT_POSITION, 2)
                FT.ft_write(sp, FT.SERVO_IDS[j], REG.REG_GOAL_POSITION, data)
            catch e
                println("!! J$j: could not re-enable torque: ", e)
            end
        end
        isempty(released) || println(">>> holding again: ", join(("J$j" for j in released), ", "))
        LibSerialPort.close(sp)
    end

    for j in joints
        println("J$j range: $(round(q_min[j], digits=1))° to $(round(q_max[j], digits=1))°")
    end
    if !dry
        new = !isfile(OUT)
        open(OUT, "a") do io
            new && println(io, "time,joint,min_deg,max_deg,seconds,limp")
            for j in joints
                println(io, Dates.now(), ",", j, ",", round(q_min[j], digits=2), ",", round(q_max[j], digits=2), ",", duration, ",",
                        length(joints) == 6 ? "all" : "one")
            end
        end
        println("appended to ", relpath(OUT))
    end
end

main()
