# Static pose campaign: MOVES THE ROBOT (unless --dry-run).
#
#   julia --project=. scripts/static_poses.jl [--n=48] [--seed=1] [--settle=6] [--atom=IP] [--dry-run]
#
# Moves the arm through --n random static poses (plus checks: J6 alone, and the zero pose every 10
# poses). At each pose it holds for --settle seconds and records the joint state and the IMU
# (STATE at ~65 Hz), then reads voltage, temperature and present current of each servo.
# Data for: the IMU gravity fit (pitch zero offsets, table tilt, IMU mounting, accelerometer
# calibration) and the gravity identification (sag and load against pose).
#
# Safety: every pose and every joint-space path between poses is checked at 50 points against the
# lab workspace (scripts/lab_workspace.jl). A path that fails goes through the zero pose.
# Moves are minimum-jerk at ≤ 30 °/s peak. Aborts (hold) on a failed move, a joint more than 6° from
# its goal while it holds, a servo above 60 °C, or a low supply (J1–J3 below 7.0 V, J4–J6 below 5.5 V:
# J4–J6 run at 6.0–6.6 V and dip to 5.9 V under load).
# Writes tools/python/recordings/static/<time>_static_samples.csv and _servos.csv.

import Dates
import Random
import MyCobot

opt(name, default) = (a = findfirst(startswith("--$name="), ARGS); a === nothing ? default : split(ARGS[a], "=")[2])
n_poses = parse(Int, opt("n", "48"))
seed = parse(Int, opt("seed", "1"))
settle = parse(Float64, opt("settle", "6"))
atom_ip = opt("atom", "192.168.1.107")
dry_run = "--dry-run" in ARGS

include("lab_workspace.jl")   # MIN_Z_MM, REACH_MM, pose_ok, path_ok

const VMAX = 30.0                                    # °/s, peak speed of each move
const RANGE = [150, 80, 120, 120, 150, 0]             # sampled joint range (± °); J6 stays at 0

zero6 = zeros(6)
rng = Random.MersenneTwister(seed)
poses = Vector{Vector{Float64}}()
tries = 0
while length(poses) < n_poses
    global tries += 1
    tries < 200_000 || error("could not find $n_poses safe poses")
    q = [RANGE[j] * (2rand(rng) - 1) for j in 1:6]
    path_ok(zero6, q) && push!(poses, q)
end
# Greedy nearest-neighbour order from zero (shorter moves).
order = Vector{Vector{Float64}}()
left = copy(poses); cur = zero6
while !isempty(left)
    i = argmin([maximum(abs.(p .- cur)) for p in left])
    global cur = popat!(left, i)
    push!(order, cur)
end
# The sequence: zero, J6 alone (±90°: the IMU must not see it if it is on the J5-J6 link), the
# poses with the zero pose every 10, zero at the end. A failed direct path goes through zero.
seq = [("zero", zero6), ("j6", [0, 0, 0, 0, 0, 90.0]), ("j6", [0, 0, 0, 0, 0, -90.0]), ("zero", zero6)]
for (i, q) in enumerate(order)
    path_ok(seq[end][2], q) || push!(seq, ("zero", zero6))
    push!(seq, ("pose", q))
    i % 10 == 0 && push!(seq, ("zero", zero6))
end
seq[end][1] == "zero" || push!(seq, ("zero", zero6))
for k in 2:length(seq)
    path_ok(seq[k-1][2], seq[k][2]) || seq[k][1] == "j6" || error("unsafe path at step $k")
end
durations = [max(2.0, 1.875 * maximum(abs.(seq[k][2] .- (k == 1 ? zero6 : seq[k-1][2]))) / VMAX) for k in 1:length(seq)]
total = sum(durations) + length(seq) * (settle + 1.5)
println("$(length(seq)) steps ($(count(s -> s[1] == "pose", seq)) poses), about $(round(total / 60, digits=1)) min")
for j in 1:6
    v = [s[2][j] for s in seq]
    println("  J$j: ", round(minimum(v), digits=1), " … ", round(maximum(v), digits=1), "°")
end
dry_run && exit()

stamp = Dates.format(Dates.now(), "yyyymmdd-HHMMSS")
outdir = joinpath(@__DIR__, "..", "tools", "python", "recordings", "static")
mkpath(outdir)
f_samples = open(joinpath(outdir, stamp * "_static_samples.csv"), "w")
f_servos = open(joinpath(outdir, stamp * "_static_servos.csv"), "w")
println(f_samples, join(vcat(["step", "kind", "t"], ["goal$j" for j in 1:6], ["q$j" for j in 1:6],
                             ["load$j" for j in 1:6], MyCobot.IMU_HEADER, ["ok"]), ','))
println(f_servos, "step,joint,voltage_V,temperature_C,current_raw,current_signed")

link = MyCobot.AtomLink(atom_ip)
abort = nothing
try
    ping = MyCobot.atom_ping(link)
    println("ATOM ", ping.version, ", ", ping.state)
    ping.state == "holding" || ping.state == "ready" || error("the ATOM is $(ping.state)")
    s0 = MyCobot.atom_state(link)
    maximum(abs.(s0.q)) < 3 || error("start at the zero pose (now $(round.(s0.q, digits=1)))")
    for (k, (kind, q_goal)) in enumerate(seq)
        done = MyCobot.atom_move_to(link, q_goal; duration=durations[k], max_tracking_error=8.0)
        if done.result != "done"
            global abort = "step $k: move ended with $(done.result) (J$(done.joint), $(round(done.error_deg, digits=1))°)"
            break
        end
        t0 = time()
        while time() - t0 < settle
            s = MyCobot.atom_state(link)
            println(f_samples, join(vcat([k, kind, round(time() - t0, digits=4)], q_goal, s.q, s.load, s.imu, [Int(s.ok)]), ','))
            if s.ok && maximum(abs.(s.q .- q_goal)) > 6
                global abort = "step $k: J$(argmax(abs.(s.q .- q_goal))) is $(round(maximum(abs.(s.q .- q_goal)), digits=1))° from its goal"
                break
            end
        end
        abort === nothing || break
        for id in 1:6
            v, temp = MyCobot.atom_read_reg(link, id, 62, 2)
            c = MyCobot.atom_read_reg(link, id, 69, 2)
            raw = Int(c[1]) | Int(c[2]) << 8
            println(f_servos, join([k, id, v / 10, temp, raw, MyCobot.JOINT_SIGN[id] * MyCobot.decode_signed15(raw)], ','))
            temp <= 60 || (global abort = "step $k: servo $id at $(temp) °C")
            v >= (id <= 3 ? 70 : 55) || (global abort = "step $k: servo $id supply $(v / 10) V")
        end
        flush(f_samples); flush(f_servos)
        println("step $k/$(length(seq)) $kind ", round.(q_goal, digits=1))
        abort === nothing || break
    end
finally
    abort === nothing || (MyCobot.atom_hold(link); println("ABORTED: ", abort))
    close(link); close(f_samples); close(f_servos)
end
println("wrote ", joinpath(outdir, stamp * "_static_*.csv"))
