# Encoder sweeps: MOVES THE ROBOT (unless --dry-run).
#
#   julia --project=. scripts/encoder_sweep.jl [JOINTS=1,2,3,4,5] [--vel=20] [--atom=IP] [--dry-run]
#
# Moves one joint at a time from the zero pose to −A, to +A and back to zero (minimum-jerk segments,
# peak --vel °/s), with 2 s holds at 0, −A and +A for the gyro bias. A is the largest amplitude
# (multiple of 5°) that keeps the sweep inside the lab workspace (scripts/lab_workspace.jl).
# Played on the ATOM at 500 Hz: the telemetry has the encoders and the gyro of the same cycle.
# The gyro measures the true rotation of the IMU link, so the integrated gyro against the encoder
# shows the encoder error (scripts/fit_encoder_sweep.jl). J6 is not visible: the IMU is before it.
# Writes tools/python/recordings/sweep/<time>_J<j>_sweep.csv.

import Dates
import MyCobot

opt(name, default) = (a = findfirst(startswith("--$name="), ARGS); a === nothing ? default : split(ARGS[a], "=")[2])
pos = filter(a -> !startswith(a, "--"), ARGS)
joints = isempty(pos) ? [1, 2, 3, 4, 5] : parse.(Int, split(pos[1], ","))
vel = parse(Float64, opt("vel", "20"))
atom_ip = opt("atom", "192.168.1.107")
dry_run = "--dry-run" in ARGS
vel <= 30 || error("--vel above 30 °/s")

include("lab_workspace.jl")   # pose_ok, path_ok

unit(j) = (e = zeros(6); e[j] = 1.0; e)
function amplitude(j)
    A = 5 * floor(Int, (WS_HI[j] - 12) / 5)
    while A > 0 && !(path_ok(zeros(6), A * unit(j)) && path_ok(zeros(6), -A * unit(j)))
        A -= 5
    end
    return float(A)
end

"Plan: zero → −A → +A → zero with holds; minimum-jerk segments at peak speed `vel`."
function sweep_plan(j, A; dt=0.01, hold=2.0)
    minjerk(x) = x^3 * (10 - 15x + 6x^2)
    t = Float64[]; q = Float64[]
    function seg(a, b, T)
        t0 = isempty(t) ? 0.0 : t[end] + dt
        for s in 0:dt:T
            push!(t, t0 + s); push!(q, a + (b - a) * (T == 0 ? 1.0 : minjerk(s / T)))
        end
    end
    seg(0.0, 0.0, hold)
    seg(0.0, -A, 1.875 * A / vel); seg(-A, -A, hold)
    seg(-A, A, 1.875 * 2A / vel); seg(A, A, hold)
    seg(A, 0.0, 1.875 * A / vel); seg(0.0, 0.0, hold)
    Q = zeros(length(t), 6); Q[:, j] = q
    return t, Q
end

plans = Dict(j => sweep_plan(j, amplitude(j)) for j in joints)
for j in joints
    t, Q = plans[j]
    println("J$j: ±$(maximum(Q[:, j]))°, $(round(t[end], digits=1)) s")
end
dry_run && exit()

outdir = joinpath(@__DIR__, "..", "tools", "python", "recordings", "sweep")
mkpath(outdir)
link = MyCobot.AtomLink(atom_ip)
try
    for j in joints
        for id in 1:6
            v, temp = MyCobot.atom_read_reg(link, id, 62, 2)
            temp <= 60 || error("servo $id is at $(temp) °C; let it cool")
            v >= (id <= 3 ? 70 : 55) || error("servo $id supply is $(v / 10) V")
        end
        t, Q = plans[j]
        rec, done = MyCobot.atom_play_trajectory(link, t, Q; plan_rate=25, max_tracking_error=10.0, timeout=t[end] + 30)
        path = joinpath(outdir, Dates.format(Dates.now(), "yyyymmdd-HHMMSS") * "_J$(j)_sweep.csv")
        MyCobot.write_atom_recording_csv(path, rec)
        println("J$j: $(done.result), $(size(rec, 1)) samples → $path")
        done.result == "done" || break
    end
finally
    close(link)
end
