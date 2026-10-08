# Plush-toy pick and place helpers (scratch, 2026-10-06). Run on the Pi: julia --project=~/myCobot/mycobot-280-lab
using MyCobot, LinearAlgebra, Printf
import RigidBodyDynamics as RBD
const M = MyCobot.load_mechanism()
const ST = RBD.MechanismState(M)
const LO, HI = MyCobot.joint_limits_deg(M)
const L = MyCobot.AtomLink("192.168.1.107")

# Gripper geometry in the flange frame (mm), from the model meshes (gripper_base: roll 90°, 34 mm out).
# Body: x ±29, y -38..13, z 2..64. Finger tips: z 97 (open) .. 115 (closed), y -2..18.5, x within ±36.
const PAD_X = 0.3    # 2026-10-06 22:50: midpoint of two fits (6.7 and -6.1); the finger axis tolerates the rest
const PAD_Y = 13.6
tip_z(opening) = 115.0 - 18.0 * opening / 1000      # rough, the tip swings on an arc
const BODY = [[x, y, z] for x in (-29.0, 29.0) for y in (-38.0, 13.0) for z in (2.0, 64.0)]
fingers(opening) = [[x, y, tip_z(opening)] for x in (-36.0, 36.0) for y in (-2.0, 18.5)]

const TABLE_Z = Ref(-34.0)          # closed fingertip z (model) at table contact: touch_table! 2026-10-08 at (230, 20): -34.3 (measured q), -34.0 (commanded)
const FLOOR = Ref(0.0)             # mm above TABLE_Z: no gripper point is commanded lower (zmin cannot go below it)
const GUARD = (zone=30.0, dload=45.0, dacc=0.6)   # 2026-10-08: friction reversal at the start of a motion moves J2 load by up to 25; stick-slip jolts reach 0.36 g at 2 mm/s; the mount broke at 90   # guarded play below TABLE_Z + zone: stop on a J2/J3 load change or an IMU jolt
const BOXES = Dict{String,Tuple{Vector{Float64},Vector{Float64}}}()   # name => (lo, hi) mm, base frame

flange(q) = MyCobot.flange_transform(ST, q)
function to_base(T, p)  # p in flange frame mm -> base mm
    R = RBD.rotation(T); t = 1000 .* Vector(RBD.translation(T))
    return t .+ R * p
end
tcp(q; opening=1000, d=tip_z(opening) - 10) = to_base(flange(q), [PAD_X, PAD_Y, d])
function body_points(q; opening=1000)
    T = flange(q)
    pts = [to_base(T, p) for p in vcat(BODY, fingers(opening))]
    for b in ("joint4", "joint5", "joint6")
        RBD.set_configuration!(ST, deg2rad.(q))
        push!(pts, 1000 .* Vector(RBD.translation(RBD.transform_to_root(ST, RBD.findbody(M, b)))))
    end
    return pts
end

"Problems with pose q (empty = ok). zmin: lowest allowed point height above TABLE_Z (mm)."
function problems(q; opening=1000, zmin=40.0, ignore=String[])
    out = String[]
    zmin = max(zmin, FLOOR[])
    all(LO .+ 2 .< q .< HI .- 2) || push!(out, "joint limits")
    for p in body_points(q; opening)
        p[3] < TABLE_Z[] + zmin && push!(out, @sprintf("low point z=%.0f", p[3]))
        r = hypot(p[1], p[2])
        r < 75 && p[3] < 200 && push!(out, @sprintf("near base column r=%.0f z=%.0f", r, p[3]))
        for (name, (lo, hi)) in BOXES
            name in ignore && continue
            all(lo .- 15 .< p .< hi .+ 15) && push!(out, "inside $name")
        end
    end
    return unique(out)
end
# the start is where the arm is (measured, with its sag): check from the first step on
path_problems(a, b; kw...) = unique(reduce(vcat, [problems(a .+ s .* (b .- a); kw...) for s in range(1 / 60, 1; length=60)]))

state() = MyCobot.atom_state(L)
"Lowest gripper point (mm, base z) at pose q."
gripper_low(q; opening=1000) = (T = flange(q); minimum(to_base(T, p)[3] for p in vcat(BODY, fingers(opening))))
path_low(a, b; opening=1000) = minimum(gripper_low(a .+ s .* (b .- a); opening) for s in range(0, 1; length=30))

"MOVES THE ROBOT. Joint-space minimum-jerk move with peak joint speed ≤ vmax °/s, after the checks.
Near the table (guard=:auto) the move is guarded: at the first contact sign the ATOM stops, the arm retreats, and it is an error."
function move!(q; vmax=25.0, opening=1000, zmin=40.0, ignore=String[], dry=false, guard=:auto)
    s = state(); s.ok || error("servo missing")
    pr = path_problems(s.q, q; opening, zmin, ignore)
    isempty(pr) || error("refused: " * join(pr, "; "))
    Δ = maximum(abs, q .- s.q)
    T = max(1.5, 1.875 * Δ / vmax)
    g = guard === :auto ? path_low(s.q, q; opening) < TABLE_Z[] + GUARD.zone : guard
    @printf("move%s: max Δ %.1f°, %.1f s, tcp %s -> %s\n", g ? " (guarded)" : "", Δ, T, round.(tcp(s.q; opening)), round.(tcp(q; opening)))
    dry && return nothing
    if g
        done, _, hit = move_guarded!(s.q, q, T)
        if hit !== nothing
            retreat!(; opening)
            error("contact: stopped and retreated")
        end
    else
        done = MyCobot.atom_move_to(L, q; duration=T, mechanism=M)
    end
    s2 = state()
    @printf("done: %s, err J%d %.2f°; q = %s; tcp = %s\n", done.result, done.joint, done.error_deg, round.(s2.q; digits=1), round.(tcp(s2.q; opening)))
    done.result == "done" || error("move ended: $(done.result)")
    return s2
end

"MOVES THE ROBOT. Minimum-jerk plan qa -> qb over T s, played with the contact guard. Returns (done, rec, hit)."
function move_guarded!(qa, qb, T; linear=false, ramp=0.4, kw...)
    minjerk(x) = (x = clamp(x, 0, 1); x^3 * (10 - 15x + 6x^2))
    t = collect(0:0.004:T)
    if linear   # constant speed with smooth ramps (no slow start: friction turns at once, not seconds later)
        v = [min(1.0, minjerk(ti / ramp), minjerk((T - ti) / ramp)) for ti in t]
        sp = cumsum(v); sp = (sp .- sp[1]) ./ (sp[end] - sp[1])
    else
        sp = minjerk.(t ./ T)
    end
    Q = reduce(vcat, permutedims(qa .+ si .* (qb .- qa)) for si in sp)
    for k in 1:3
        try
            MyCobot.atom_upload_plan(L, t, Q, Q); break
        catch e
            k == 3 && rethrow(); println("upload retry: ", e); sleep(0.5)
        end
    end
    return play_guarded!(; kw...)
end

"Play the uploaded plan and watch the telemetry: STOP at the first J2/J3 load change > dload or IMU jolt > dacc g,
against the first 0.1 s (mean over the last 50 ms). With base_t > 0 the load baseline is taken again at base_t s
(after the friction reversal at the start), and from then on dload applies; before it, GUARD.dload.
Returns (done, rec, hit); hit === nothing: no contact."
function play_guarded!(; dload=GUARD.dload, dacc=GUARD.dacc, base_t=0.0)
    sz = 77
    MyCobot.drain!(L)
    MyCobot.send(L, vcat(0x07, MyCobot.le(UInt16(500)), MyCobot.le(UInt16(2000)),
                         MyCobot.le(UInt16(round(Int, 20 * MyCobot.STEPS_PER_DEG))), MyCobot.le(UInt16(round(Int, 3 * MyCobot.STEPS_PER_DEG)))))
    samples = Vector{Vector{UInt8}}(); hit = nothing; base = nothing
    accn(r) = sqrt.(sum(r[:, 32:34] .^ 2; dims=2))[:]
    deadline = time() + 120
    while time() < deadline
        isready(L.inbox) || (sleep(0.0005); continue)
        m = take!(L.inbox); isempty(m) && continue
        if m[1] == 0x83 && m[2] == 0x07
            st = reinterpret(Int8, m[3]); (st == 0 || st == -3 || st == -4) || error("PLAY refused ($st)")
        elseif m[1] == 0x84
            for k in 0:m[6]-1
                push!(samples, m[7+sz*k:6+sz*(k+1)])
            end
            (hit === nothing && length(samples) >= 50) || continue
            if base === nothing
                rb = MyCobot.decode_telemetry(samples[1:50]); any(isnan, rb[:, 26:31]) && continue
                base = (ld=vec(sum(rb[:, 27:28]; dims=1)) ./ 50, acc=sort(accn(rb))[25])
            end
            r = MyCobot.decode_telemetry(samples[end-24:end]); any(isnan, r[:, 26:31]) && continue
            nb = round(Int, base_t * 500)
            if base_t > 0 && length(samples) >= nb + 50 && !haskey(base, :ld1)
                rb = MyCobot.decode_telemetry(samples[nb+1:nb+50])
                any(isnan, rb[:, 26:31]) || (base = merge(base, (ld1=vec(sum(rb[:, 27:28]; dims=1)) ./ 50,)))
            end
            late = haskey(base, :ld1)
            d = abs.(vec(sum(r[:, 27:28]; dims=1)) ./ 25 .- (late ? base.ld1 : base.ld)); da = maximum(abs.(accn(r) .- base.acc))
            if maximum(d) > (late || base_t == 0 ? dload : GUARD.dload) || da > dacc
                MyCobot.atom_stop(L)
                hit = (dload=d, dacc=da, t=r[end, 1] - MyCobot.decode_telemetry(samples[1:1])[1, 1])
                @printf("CONTACT at %.2f s: Δload J2 %.0f J3 %.0f, IMU jolt %.2f g -> stop\n", hit.t, d..., da)
            end
        elseif m[1] == 0x85
            done = (result=MyCobot.PLAY_RESULTS[m[2]+1], joint=Int(m[19]), error_deg=MyCobot.rd(Int16, m, 20) / MyCobot.STEPS_PER_DEG)
            return done, MyCobot.decode_telemetry(samples), hit
        end
    end
    MyCobot.atom_stop(L)
    error("no DONE from the ATOM")
end

"MOVES THE ROBOT. Lift the TCP straight up by dz mm (same tool rotation). Not checked against the scene: use it to back away."
function retreat!(; dz=20.0, opening=1000)
    try
        q0 = state().q; R = Matrix(RBD.rotation(flange(q0)))
        q = ik_to(tcp(q0; opening) .+ [0, 0, dz], R; q0, opening, n=5)
        q === nothing && (println("retreat: no IK"); return)
        MyCobot.atom_move_to(L, q; duration=max(1.5, dz / 15), mechanism=M)
        println("retreated ", dz, " mm: tcp ", round.(tcp(state().q; opening); digits=1))
    catch e
        println("retreat failed: ", e)
    end
end

"After an error: back away if the gripper is near the table."
function safe_retreat!()
    try
        g = stream(); q = state().q
        gripper_low(q; opening=g.opening) < TABLE_Z[] + GUARD.zone && retreat!(; dz=25.0, opening=g.opening)
    catch e
        println("safe retreat skipped: ", e)
    end
end

"MOVES THE ROBOT. Touch the table straight down with the closed fingertips at xy (2 mm/s, at most `over` mm past
TABLE_Z), stop at the first contact sign, retreat, and return the tip z at the load onset (does not set TABLE_Z)."
function touch_table!(xy; yaw, above=15.0, over=8.0, onset=6.0, tag="touch", dload=12.0, base_t=1.5, speed=2.0)
    gripper!(0); sleep(0.8)
    z0 = TABLE_Z[]
    q = ik_best([xy..., z0 + above + 10], 0, yaw; opening=0); q === nothing && error("no IK above")
    move!(q; opening=0, vmax=8, zmin=above - 3, guard=false)
    q1 = ik_best([xy..., z0 - over + 10], 0, yaw; q0=state().q, opening=0); q1 === nothing && error("no IK low")
    old = FLOOR[]; FLOOR[] = -over - 1
    z_on = nothing
    try
        pr = path_problems(state().q, q1; opening=0, zmin=-over - 1); isempty(pr) || error("refused: " * join(pr, "; "))
        done, rec, hit = move_guarded!(state().q, q1, (above + over) / speed; linear=true, dload, base_t)
        open("/home/henrique/scratch/cutlery/$tag.csv", "w") do io
            println(io, join(vcat(MyCobot.RECORDING_HEADER, MyCobot.IMU_HEADER), ","))
            for i in 1:size(rec, 1); println(io, join(rec[i, :], ",")); end
        end
        nb = round(Int, base_t * 500)
        # Contact signature (2026-10-08): the J4 tracking error (sag of the wrist, ~1.55°, noise 0.08°) falls as the
        # table takes the weight; J2/J3 loads rise later and move as much with friction at the start of a motion.
        e4 = rec[:, 11] .- rec[:, 17]; sm = [sum(e4[max(1, i-49):i]) / length(max(1, i-49):i) for i in 1:length(e4)]
        b4 = sum(e4[nb+1:nb+250]) / 250
        k4 = findfirst(i -> i > nb + 250 && sm[i] < b4 - 0.25, 1:length(sm))
        k4 === nothing || @printf("J4 onset at %.2f s: tip z %.1f\n", rec[k4, 1] - rec[1, 1], tip(rec[k4, 14:19]; opening=0)[3])
        ld = rec[:, 27:28]; b = vec(sum(ld[nb+1:nb+50, :]; dims=1)) ./ 50
        dev = [i <= nb ? 0.0 : maximum(abs.(ld[i, :] .- b)) for i in 1:size(rec, 1)]
        a = sqrt.(sum(rec[:, 32:34] .^ 2; dims=2))[:]; a0 = sort(a[1:50])[25]
        kl = findfirst(>(onset), [sum(dev[max(1, i-24):i]) / length(max(1, i-24):i) for i in 1:length(dev)])
        ka = findfirst(>(0.6), abs.(a .- a0))
        for (name, k) in (("load", kl), ("IMU", ka))
            k === nothing && (println(name, ": no onset"); continue)
            @printf("%s onset at %.2f s: tip z %.1f\n", name, rec[k, 1] - rec[1, 1], tip(rec[k, 14:19]; opening=0)[3])
        end
        k = k4
        k === nothing || (z_on = tip(rec[k, 14:19]; opening=0)[3])
        @printf("TABLE_Z %.1f; %s at tip z %.1f\n", z0, hit === nothing ? "no stop" : "stopped", tip(state().q; opening=0)[3])
    finally
        FLOOR[] = old
        retreat!(; dz=above + 10, opening=0)
    end
    return z_on
end

"IK for the TCP at p (mm, base) with the gripper pointing straight down; yaw (°) turns the finger axis."
function ik_down(p, yaw; q0=state().q, opening=1000, d=tip_z(opening) - 10)
    z = [0, 0, -1.0]; x = [cosd(yaw), sind(yaw), 0]; y = cross(z, x)
    R = hcat(x, y, z)
    pf = p .- R * [0.0, PAD_Y, d]
    q, pe, re = MyCobot.inverse_kinematics(ST, goal(R, pf), q0)
    return q, pe * 1000, rad2deg(re)
end

le16(x) = reinterpret(UInt8, [htol(x)])
"Gripper goal 0 (closed) .. 1000 (open)."
function gripper!(opening)
    MyCobot.drain!(L); MyCobot.send(L, vcat(0x11, le16(UInt16(opening))))
    m = MyCobot.receive(L, 0x83; timeout=0.3)
    m !== nothing && m[2] == 0x11 && error("GRIPPER refused: $(reinterpret(Int8, m[3]))")
end
"One STREAM message: (state, gripper present, opening, load, voltages)."
function stream()
    MyCobot.drain!(L); MyCobot.send(L, vcat(0x0C, le16(UInt16(20))))
    m = MyCobot.receive(L, 0x88; timeout=1.0)
    MyCobot.send(L, vcat(0x0C, le16(UInt16(0))))
    m === nothing && error("no STREAM")
    rdi(i) = ltoh(reinterpret(Int16, m[i+1:i+2])[1])
    return (state=m[6], control=m[74], gripper=m[75], opening=rdi(75), load=rdi(77), volt=Int.(m[50:55]), temp=Int.(m[44:49]))
end
function report()
    s = state(); g = stream()
    @printf("q = %s\nflange = %s mm, tcp(open) = %s mm\nimu = %s\ngripper: present %d opening %d load %d; volt %s temp %s control %d\n",
            round.(s.q; digits=1), round.(1000 .* Vector(RBD.translation(flange(s.q)))), round.(tcp(s.q)), round.(s.imu; digits=3),
            g.gripper, g.opening, g.load, g.volt, g.temp, g.control)
end

"IK for TCP at p with the gripper axis tilted `tilt`° from straight down, leaning outward (away from the base) , fingers across yaw."
function ik_tilt(p, tilt, yaw; q0=state().q, opening=1000, d=tip_z(opening) - 10)
    az = atand(p[2], p[1])
    out = [cosd(az), sind(az), 0.0]
    z = normalize(-cosd(tilt) .* [0, 0, 1.0] .+ sind(tilt) .* out)
    x0 = [cosd(yaw), sind(yaw), 0.0]
    x = normalize(x0 .- dot(x0, z) .* z); y = cross(z, x)
    R = hcat(x, y, z)
    pf = p .- R * [0.0, PAD_Y, d]
    best = nothing
    for q00 in (q0,)
        q, pe, re = MyCobot.inverse_kinematics(ST, goal(R, pf), q00; max_iters=300)
        if pe < 1e-3 && re < 1e-2 && all(LO .< q .< HI)
            best = q; break
        end
    end
    return best
end
goal(R, pf) = RBD.Transform3D(RBD.CartesianFrame3D("goal"), RBD.root_frame(M), [R pf ./ 1000; 0 0 0 1])
tip(q; opening=0) = to_base(flange(q), [PAD_X, PAD_Y, tip_z(opening)])
snap(name) = try run(`curl -fsS -o /home/henrique/scratch/cutlery/$name.jpg http://100.69.15.110:8280/snapshot.jpg`) catch e; println("snapshot $name failed") end
"Visit TCP points (gripper down, closed), take a snapshot at each, write calib.csv. MOVES THE ROBOT."
function calib_run(points; yaw_off=90, tag="cal", dry=false, zmin=100)
    qprev = state().q
    open("/home/henrique/scratch/plush/$tag.csv", dry ? "w" : "a") do io
        for (k, p) in enumerate(points)
            az = atand(p[2], p[1])
            q = ik_best(p, 0, az + yaw_off; q0=qprev, opening=0); q === nothing || (qprev = q)
            q === nothing && (println("no IK for $p"); continue)
            pr = problems(q; opening=0, zmin)
            println(k, " ", p, " q=", round.(q; digits=1), " ", pr)
            (dry || !isempty(pr)) && continue
            move!(q; opening=0, zmin)
            sleep(1.0)
            s = state(); t = tip(s.q)
            snap("$(tag)_$k")
            println(io, join(vcat(k, s.q, t, s.imu), ","))
        end
    end
end

"Pose of the TCP: (position, tilt from down °, yaw of the finger axis °)."
function tcp_pose(q; opening=1000)
    T = flange(q); R = RBD.rotation(T)
    return tcp(q; opening), acosd(clamp(-R[3, 3], -1, 1)), atand(R[2, 1], R[1, 1])
end
"Continuation IK: walk from q0 (its current TCP) to the TCP goal p (tilt, yaw) in n steps."
function ik_path(p, tilt, yaw; q0=state().q, opening=1000, n=20)
    p0, t0, y0 = tcp_pose(q0; opening)
    dy = mod(yaw - y0 + 180, 360) - 180
    q = q0
    for s in range(0, 1; length=n + 1)[2:end]
        q = ik_tilt(p0 .+ s .* (p .- p0), t0 + s * (tilt - t0), y0 + s * dy; q0=q, opening)
        q === nothing && return nothing
    end
    return q
end

function rotlog(R)
    c = clamp((tr(R) - 1) / 2, -1, 1); θ = acos(c)
    θ < 1e-9 && return zeros(3)
    w = [R[3, 2] - R[2, 3], R[1, 3] - R[3, 1], R[2, 1] - R[1, 2]] ./ (2sin(θ))
    return θ .* w
end
function rotexp(v)
    θ = norm(v); θ < 1e-12 && return Matrix(1.0I, 3, 3)
    k = v ./ θ; K = [0 -k[3] k[2]; k[3] 0 -k[1]; -k[2] k[1] 0]
    return I + sin(θ) * K + (1 - cos(θ)) * K * K
end
"Tool rotation: z axis tilted `tilt`° from straight down toward the horizontal direction `lean_az`°, x (finger axis) near yaw."
function tool_R(tilt, yaw; lean_az=yaw + 90)
    z = normalize(-cosd(tilt) .* [0, 0, 1.0] .+ sind(tilt) .* [cosd(lean_az), sind(lean_az), 0])
    x0 = [cosd(yaw), sind(yaw), 0.0]
    x = normalize(x0 .- dot(x0, z) .* z)
    return hcat(x, cross(z, x), z)
end
function ik1(p, R, q0; opening, d=tip_z(opening) - 10)
    q, pe, re = MyCobot.inverse_kinematics(ST, goal(R, p .- R * [PAD_X, PAD_Y, d]), q0; max_iters=200)
    return (pe < 5e-4 && re < 5e-3 && all(LO .+ 2 .< q .< HI .- 2)) ? q : nothing
end
"Continuation IK from q0 to TCP position p (mm) and tool rotation R."
function ik_to(p, R; q0=state().q, opening=1000, n=25)
    T = flange(q0); R0 = Matrix(RBD.rotation(T)); p0 = tcp(q0; opening)
    w = rotlog(R0' * R)
    q = q0
    for s in range(0, 1; length=n + 1)[2:end]
        q = ik1(p0 .+ s .* (p .- p0), R0 * rotexp(s .* w), q; opening)
        q === nothing && return nothing
    end
    return q
end
"Like ik_to, but tries the finger yaw and yaw+180 (same grasp) and keeps the J6 farther from its limits."
function ik_best(p, tilt, yaw; q0=state().q, opening=1000, lean_az=nothing)
    best = nothing
    for y in (yaw, yaw + 180)
        R = lean_az === nothing ? tool_R(tilt, y; lean_az=atand(p[2], p[1])) : tool_R(tilt, y; lean_az)
        q = ik_to(p, R; q0, opening)
        q === nothing && continue
        m = min(q[6] - LO[6], HI[6] - q[6])
        (best === nothing || m > best[2]) && (best = (q, m))
    end
    return best === nothing ? nothing : best[1]
end
"IK from a set of seeds (gripper-down pattern) plus q0; keeps the solution nearest q0 (weighted), J6 away from limits."
function ik_seeded(p, R; q0=state().q, opening=1000)
    az = atand(p[2], p[1])
    seeds = [q0]
    for j2 in (-40.0, -15, 10), j3 in (-80.0, -45, -10), j6 in (-180.0, -90, 0, 90)
        push!(seeds, [mod(az + 20 + 180, 360) - 180, j2, j3, -90 - j2 - j3, 0, j6])
    end
    best = nothing
    for s in seeds
        q = ik1(p, R, s; opening)
        q === nothing && continue
        c = maximum(abs, q .- q0) + (min(q[6] - LO[6], HI[6] - q[6]) < 20 ? 100 : 0)
        (best === nothing || c < best[2]) && (best = (q, c))
    end
    return best === nothing ? nothing : best[1]
end
const YAW_STRICT = Ref(false)   # true: only the given yaw (e.g. to keep the servo box away from an object)
function ik_best(p, tilt, yaw; q0=state().q, opening=1000, lean_az=atand(p[2], p[1]))
    best = nothing
    for y in (YAW_STRICT[] ? (yaw,) : (yaw, yaw + 180))
        q = ik_seeded(p, tool_R(tilt, y; lean_az), ; q0, opening)
        q === nothing && continue
        c = maximum(abs, q .- q0) + (-205 <= q[6] <= 115 ? 0 : 1000)   # J6 at least 20° inside the cable stops
        (best === nothing || c < best[2]) && (best = (q, c))
    end
    return best === nothing ? nothing : best[1]
end

# Obstacles from the first (rough) camera model, 2026-10-06; refined later.
BOXES["wall"] = ([-450.0, -700, -100], [-265.0, 700, 700])
BOXES["tissue box"] = ([-115.0, -215, -40], [35.0, -75, 85])
BOXES["plush"] = ([-235.0, -190, -40], [-100.0, -40, 45])


"MOVES THE ROBOT. Like move!, but plays the plan with telemetry (500 Hz: q, load, IMU). Returns (done, rec)."
function move_rec!(q; vmax=8.0, opening=1000, zmin=40.0, ignore=String[], min_T=1.5)
    s = state(); s.ok || error("servo missing")
    pr = path_problems(s.q, q; opening, zmin, ignore)
    isempty(pr) || error("refused: " * join(pr, "; "))
    T = max(min_T, 1.875 * maximum(abs, q .- s.q) / vmax)
    minjerk(x) = (x = clamp(x, 0, 1); x^3 * (10 - 15x + 6x^2))
    t = collect(0:0.004:T + 0.5)        # 0.5 s hold at the end
    Q = reduce(vcat, permutedims(s.q .+ minjerk(ti / T) .* (q .- s.q)) for ti in t)
    for k in 1:3
        try
            MyCobot.atom_upload_plan(L, t, Q, Q); break
        catch e
            k == 3 && rethrow(); println("upload retry: ", e); sleep(0.5)
        end
    end
    samples, done = MyCobot.atom_play(L)
    rec = MyCobot.decode_telemetry(samples)
    done.result == "done" || println("move ended: $(done.result) J$(done.joint) $(done.error_deg)°")
    return done, rec
end
"Static average over n STATE reads: (q, load, imu)."
function rest(n=20)
    S = [state() for _ in 1:n]
    return (q=sum(s.q for s in S) / n, load=sum(s.load for s in S) / n, imu=sum(s.imu for s in S) / n,
            load_sd=sqrt.(sum((s.load .- sum(x.load for x in S) / n) .^ 2 for s in S) / n))
end
# Refined with the calibrated camera (cal3, f = 1272 px) and the table touch (z = -30), 2026-10-06.
BOXES["tissue box"] = ([-90.0, -200, -40], [80.0, -30, 15])
BOXES["plush"] = ([-220.0, -175, -40], [-115.0, -20, 30])
"Fingertip positions (left, right) for a given opening (rough: pads 6..43 mm apart)."
function fingertips(q; opening=1000)
    g = 6 + 37 * opening / 1000
    T = flange(q)
    return [to_base(T, [PAD_X + s * (g / 2 + 5), PAD_Y, tip_z(opening)]) for s in (-1, 1)]
end
const CAM = [-278.0, -353, 449]   # camera centre in the base frame (cal3 fit, f = 1272 px), mm
"Horizontal unit vector from p toward the camera."
to_cam(p) = normalize((CAM .- p)[1:2])
"MOVES THE ROBOT. Move the TCP to p (gripper down, yaw), then re-command once with the measured error added (sag, play)."
const IGNORE_ALSO = Ref(String[])   # zones checked by hand for one task step (cleared after)
function move_tcp!(p, yaw; opening=1000, vmax=6, zmin=10, ignore=["plush"], correct=true)
    ignore = vcat(ignore, IGNORE_ALSO[])
    q = ik_best(p, 0, yaw; opening)
    q === nothing && error("no IK for $p")
    move!(q; opening, zmin, ignore, vmax)
    correct || return q
    e = p .- tcp(state().q; opening)
    norm(e) < 1.5 && return q
    q2 = ik_best(p .+ e, 0, yaw; q0=q, opening)
    q2 === nothing && return q
    move!(q2; opening, zmin, ignore, vmax)
    println("corrected by ", round.(e; digits=1), " -> tcp ", round.(tcp(state().q; opening); digits=1))
    return q2
end

"Close in steps and lift the arm by the fingertip descent of each step (the tips swing down 18 mm), so the
tips stay at one height. Stops stepping when the fingers stall on the object. MOVES THE ROBOT."
function grasp_arc!(yaw; steps=(800, 650, 500, 350, 200, 50, 0))
    g_prev = stream().opening
    for op in steps
        gripper!(op); sleep(0.6)
        g = stream().opening
        descent = tip_z(g) - tip_z(g_prev)
        @printf("goal %4d -> opening %4d, tips down %.1f mm\n", op, g, descent)
        if descent > 1.0
            p = tcp(state().q)
            move_tcp!([p[1], p[2], p[3] + descent], yaw; correct=false, vmax=4, zmin=0, ignore=["plush", "tissue box"])
        end
        g_prev = g
        g > op + 100 && (println("stalled at $g: holding"); gripper!(0); sleep(0.5); return stream().opening)
    end
    return stream().opening
end

"Lift in small steps while holding; stop if the opening falls (slipping) or a servo is hot. MOVES THE ROBOT."
function lift_check!(yaw, dz_total; step=5.0, hot=68, min_open=300)
    g0 = stream().opening
    for k in 1:ceil(Int, dz_total / step)
        p = tcp(state().q)
        move_tcp!([p[1], p[2], p[3] + step], yaw; correct=false, vmax=5, zmin=0, ignore=["plush", "tissue box"])
        g = stream()
        t7 = Int(MyCobot.atom_read_reg(L, 7, 63, 1)[1])
        @printf("lift %3.0f mm: opening %d, load %d, temps %s, J7 %d °C\n", k * step, g.opening, g.load, g.temp, t7)
        (maximum(g.temp) >= hot || t7 >= 70) && (println("HOT, stop"); return false)
        g.opening < min_open && (println("SLIPPING (", g0, " -> ", g.opening, ")"); return false)
        g.opening < 30 && (println("EMPTY"); return false)
    end
    return true
end

# Fixed: step from the COMMANDED target, not the measured TCP (the sag added up at every step: 13 mm drift).
const P_CMD = Ref([0.0, 0, 0])
function move_cmd!(p, yaw; correct=true, kw...)
    P_CMD[] = collect(float.(p))
    move_tcp!(P_CMD[], yaw; correct, kw...)
end
function grasp_arc!(yaw; steps=(800, 650, 500, 350, 200, 50, 0))
    g_prev = stream().opening
    for op in steps
        gripper!(op); sleep(0.6)
        g = stream().opening
        descent = tip_z(g) - tip_z(g_prev)
        @printf("goal %4d -> opening %4d, tips down %.1f mm\n", op, g, descent)
        if descent > 1.0
            P_CMD[] = P_CMD[] .+ [0, 0, descent]
            move_tcp!(P_CMD[], yaw; correct=false, vmax=4, zmin=0, ignore=["plush", "tissue box"])
        end
        g_prev = g
        g > op + 100 && (println("stalled at $g: holding"); gripper!(0); sleep(0.5); return stream().opening)
    end
    return stream().opening
end
function lift_check!(yaw, dz_total; step=5.0, hot=68, min_open=300)
    g0 = stream().opening
    for k in 1:ceil(Int, dz_total / step)
        P_CMD[] = P_CMD[] .+ [0, 0, step]
        try
            move_tcp!(P_CMD[], yaw; correct=false, vmax=5, zmin=0, ignore=["plush", "tissue box"])
        catch e
            occursin("no IK", sprint(showerror, e)) || rethrow()
            P_CMD[] = P_CMD[] .- [0, 0, step]; println("lift stops at the reach limit"); return true
        end
        g = stream(); t7 = try Int(MyCobot.atom_read_reg(L, 7, 63, 1)[1]) catch; -1 end
        @printf("lift %3.0f mm: opening %d, load %d, temps %s, J7 %d °C, tcp %s\n", k * step, g.opening, g.load, g.temp, t7, round.(tcp(state().q)))
        (maximum(g.temp) >= hot || t7 >= 70) && (println("HOT, stop"); return false)
        g.opening < min_open && (println("SLIPPING (", g0, " -> ", g.opening, ")"); return false)
    end
    return true
end

roidiff(a, b, roi) = parse(Float64, readchomp(`python3 /home/henrique/scratch/plush/roidiff.py /home/henrique/scratch/plush/$a.jpg /home/henrique/scratch/plush/$b.jpg $(roi[1]) $(roi[2]) $(roi[3]) $(roi[4])`))
"Poke: from start xy at TCP height z (gripper closed), step along dir; stop when the ROI changes. MOVES THE ROBOT."
function poke!(start, dir, z, yaw; step=4.0, maxd=80.0, roi=(740, 560, 850, 660), tag="poke")
    gripper!(0); sleep(0.8)
    move_cmd!([start..., 60.0], yaw; correct=false, vmax=10, zmin=0, ignore=["plush", "tissue box"])
    move_cmd!([start..., z], yaw; vmax=5, zmin=0, ignore=["plush", "tissue box"])
    snap("$(tag)_ref"); sleep(0.3); snap("$(tag)_ref2")
    noise = roidiff("$(tag)_ref", "$(tag)_ref2", roi)
    thr = 3noise + 3
    println("noise $noise, threshold $thr")
    for k in 1:ceil(Int, maxd / step)
        P_CMD[] = P_CMD[] .+ [step .* dir..., 0]
        move_tcp!(P_CMD[], yaw; correct=false, vmax=4, zmin=0, ignore=["plush", "tissue box"])
        snap("$(tag)_$k")
        d = roidiff("$(tag)_ref", "$(tag)_$k", roi)
        s = state(); t = tip(s.q)
        @printf("step %2d: cmd %s, tip meas %s, roi diff %.2f\n", k, round.(P_CMD[]), round.(t), d)
        if d > thr
            println("CONTACT at step $k: tip ", round.(t; digits=1))
            P_CMD[] = P_CMD[] .- [25 .* dir..., 0]
            move_tcp!(P_CMD[], yaw; correct=false, vmax=5, zmin=0, ignore=["plush", "tissue box"])
            move_cmd!([P_CMD[][1], P_CMD[][2], 60.0], yaw; correct=false, vmax=8, zmin=0, ignore=["plush", "tissue box"])
            return t
        end
    end
    return nothing
end

# Corks task (2026-10-06 evening): the box stands against the wall, left of the robot; the cow is gone.
BOXES["tissue box"] = ([-270.0, -105, -40], [-88.0, 62, 10])
delete!(BOXES, "plush")
const BOX_DROP = [-205.0, -35.0]   # run 2
"Pick the cork at xy (finger yaw), drop it in the box. MOVES THE ROBOT."
function pick_cork!(xy, yaw; tag, z_low=-6.0, z_carry=80.0, open=1000)
    gripper!(open); sleep(0.8)
    move_cmd!([xy..., 50], yaw; correct=false, vmax=12, zmin=0)
    move_cmd!([xy..., z_low], yaw; vmax=5, zmin=-10)
    snap("$(tag)_low")
    g = grasp_arc!(yaw)
    println("closed at ", g); snap("$(tag)_closed")
    g < 120 && (println("EMPTY"); move_cmd!([xy..., 50], yaw; correct=false, vmax=8, zmin=0); return false)
    ok = lift_check!(yaw, z_carry - P_CMD[][3]; step=10.0, min_open=100)
    snap("$(tag)_lifted")
    ok || (println("LOST"); gripper!(1000); return false)
    move_cmd!([-130.0, -110, z_carry], yaw; correct=false, vmax=10, zmin=0)   # waypoint: the direct path swings toward the wall
    move_cmd!([BOX_DROP..., z_carry], yaw; correct=false, vmax=10, zmin=0)
    println("over the box, opening ", stream().opening); snap("$(tag)_over_box")
    gripper!(1000); sleep(1.2); snap("$(tag)_dropped")
    return true
end
BOXES["board"] = ([-70.0, -300, -40], [300.0, -95, -15])   # white panel right of the corks (camera, 2026-10-06)

"Calibration points with the closed gripper pointing down: move (with correction), snapshot, log the fingertip (closed). MOVES THE ROBOT."
function calib_run2(points; tag="cal4", yaw=24.0)
    gripper!(0); sleep(0.8)
    open("/home/henrique/scratch/corks/$tag.csv", "w") do io
        for (k, p) in enumerate(points)
            try
                move_cmd!([p[1], p[2], p[3] + 30], yaw; correct=false, vmax=12, zmin=0)
                move_cmd!(p, yaw; vmax=8, zmin=0)
            catch e
                println(k, " skipped: ", sprint(showerror, e)); continue
            end
            sleep(0.8)
            s = state(); t = tip(s.q; opening=0)
            snap("$(tag)_$k")
            println(io, join(vcat(k, t), ","))
            println(k, " tip ", round.(t; digits=1))
        end
    end
end

"Descend without pushing: correct the sag 15 mm above, then go straight down with that offset. MOVES THE ROBOT."
function descend_precorrected!(xy, yaw, z_low; above=15.0)
    move_cmd!([xy..., z_low + above], yaw; correct=false, vmax=5, zmin=-10)   # measure the sag here (no move at the bottom)
    e = [xy..., z_low + above] .- tcp(state().q)
    P_CMD[] = [xy[1] + e[1], xy[2] + e[2], z_low + e[3]]
    move_tcp!(P_CMD[], yaw; correct=false, vmax=4, zmin=-10)
    println("pre-corrected by ", round.(e; digits=1), " -> tcp ", round.(tcp(state().q); digits=1))
end
function pick_cork2!(xy, yaw; tag, z_low, z_carry=80.0, open=700, drop_yaw=0.0)
    gripper!(open); sleep(0.8)
    move_cmd!([xy..., 50], yaw; correct=false, vmax=10, zmin=0)
    descend_precorrected!(xy, yaw, z_low)
    snap("$(tag)_low")
    g = grasp_arc!(yaw)
    println("closed at ", g); snap("$(tag)_closed")
    g < 120 && (println("EMPTY"); gripper!(1000); move_cmd!([xy..., 50], yaw; correct=false, vmax=8, zmin=0); return false)
    ok = lift_check!(yaw, z_carry - P_CMD[][3]; step=10.0, min_open=100)
    snap("$(tag)_lifted")
    ok || (println("LOST"); gripper!(1000); return false)
    move_cmd!([-130.0, -110, z_carry], yaw; correct=false, vmax=10, zmin=0)
    move_cmd!([BOX_DROP..., z_carry], drop_yaw; correct=false, vmax=10, zmin=0)   # turn so the servo box stays off the wall
    println("over the box, opening ", stream().opening); snap("$(tag)_over_box")
    gripper!(1000); sleep(1.2); snap("$(tag)_dropped")
    return true
end

# Corks run 2 (2026-10-06 19:05): the box moved left and up; corks in a group (camera model cam5).
BOXES["tissue box"] = ([-285.0, -75, -40], [-150.0, 95, 10])
delete!(BOXES, "board")
function run_cork(k, c, fy; open=1000, shift=12.0, tag="r2cork$k")
    xy = c .+ shift .* to_cam([c..., 0.0])
    z = -9.0 + (tip_z(open) - tip_z(1000))
    ok = pick_cork2!(xy, fy; tag, z_low=z, open=open)
    println("cork $k: ", ok ? "IN THE BOX" : "failed")
    return ok
end

# ---- Smooth grasp (the user's idea, 2026-10-06): one gripper close; the arm lifts by the measured fingertip
# extension while it closes (closed loop on the gripper's present position), with TRACK at 50 Hz.
"Julia angle (encoder-corrected) -> firmware angle (raw encoder), degrees."
fw_deg(j, q) = MyCobot.JOINT_SIGN[j] * (MyCobot.angle_to_position(j, q) - 2048) / MyCobot.STEPS_PER_DEG
send_track(q; vmax=60.0) = MyCobot.send(L, vcat(0x10, reduce(vcat, [le16(Int16(round(100 * fw_deg(j, q[j])))) for j in 1:6]), le16(UInt16(round(10 * vmax)))))
"Latest STREAM message in the inbox (or nothing): (opening, state)."
function latest_stream()
    m = nothing
    while isready(L.inbox)
        x = take!(L.inbox)
        (!isempty(x) && x[1] == 0x88 && length(x) >= 79) && (m = x)
        (!isempty(x) && x[1] == 0x83 && x[2] == 0x10) && println("TRACK refused: ", reinterpret(Int8, x[3]))
    end
    m === nothing && return nothing
    return (opening=ltoh(reinterpret(Int16, m[76:77])[1]), state=m[6])
end
"MOVES THE ROBOT. Close in one motion; lift the TCP by the measured fingertip extension. Returns the final opening."
function grasp_smooth!(yaw; goal=0, vmax=60.0, timeout=4.0, free_lift=0.0, free_above=200, R=nothing)
    p0 = copy(P_CMD[])
    g0 = stream().opening
    lut = Vector{Vector{Float64}}(); q = state().q
    for dz in 0:1.0:24   # R given (tilted tool): back off along the tool axis, keep the rotation
        q = R === nothing ? ik_best(p0 .+ [0, 0, dz], 0, yaw; q0=q) : ik1(p0 .- dz .* R[:, 3], R, q; opening=1000)
        q === nothing && error("no IK at +$dz mm")
        push!(lut, q)
    end
    qat(dz) = (d = clamp(dz, 0, 24); i = min(floor(Int, d), 23); f = d - i; lut[i+1] .+ f .* (lut[i+2] .- lut[i+1]))
    MyCobot.drain!(L)
    MyCobot.send(L, vcat(0x0C, le16(UInt16(50))))           # STREAM at 50 Hz
    t0 = time(); last_sub = t0; last_change = t0; g_last = g0; dz = 0.0; extra = 0.0
    gripper!(goal)
    while time() - t0 < timeout
        s = latest_stream()
        if s !== nothing
            dz = max(dz, tip_z(s.opening) - tip_z(g0))       # never move back down
            abs(s.opening - g_last) > 3 && (last_change = time(); g_last = s.opening)
        end
        # pads blocked from the start (on the table): raise slowly until they move (free_lift mm at most)
        stuck = time() - t0 > 0.35 && time() - last_change > 0.25 && g_last > free_above   # still, but wider than the object
        stuck && extra < free_lift && (extra += 0.2)
        dz = max(dz, extra)
        send_track(qat(dz); vmax)
        time() - last_sub > 0.5 && (MyCobot.send(L, vcat(0x0C, le16(UInt16(50)))); last_sub = time())
        time() - last_change > 0.3 && time() - t0 > 0.5 && !(g_last > free_above && extra < free_lift) && break   # still: stalled or closed
        sleep(0.02)
    end
    MyCobot.send(L, vcat(0x0C, le16(UInt16(0))))
    sleep(0.4)                                                  # TRACK deadman: the arm brakes and holds
    P_CMD[] = R === nothing ? p0 .+ [0, 0, dz] : p0 .- dz .* R[:, 3]
    g = stream().opening
    @printf("smooth close: opening %d -> %d, lifted %.1f mm (free lift %.1f) in %.1f s\n", g0, g, dz, extra, time() - t0)
    return g
end

"Calibration with several yaws (closed gripper, pointing down): log q too, to fit the tip offset in the flange frame. MOVES THE ROBOT."
function calib_yaws(points; tag="cal5")
    gripper!(0); sleep(0.8)
    open("/home/henrique/scratch/cutlery/$tag.csv", "w") do io
        for (k, (p, yaw)) in enumerate(points)
            try
                move_cmd!([p[1], p[2], p[3] + 25], yaw; correct=false, vmax=12, zmin=0)
                move_cmd!(p, yaw; vmax=8, zmin=0)
            catch e
                println(k, " skipped: ", sprint(showerror, e)); continue
            end
            sleep(0.8)
            s = state()
            snap("$(tag)_$k")
            println(io, join(vcat(k, yaw, s.q), ","))
            println(k, " yaw $yaw tip ", round.(tip(s.q; opening=0); digits=1))
        end
    end
end

"Pick with the smooth grasp (one close, arm lifts with the measured opening). MOVES THE ROBOT."
function pick_cork3!(xy, yaw; tag, z_low=-6.0, z_carry=80.0, open=1000, drop_yaw=0.0)
    gripper!(open); sleep(0.8)
    move_cmd!([xy..., 50], yaw; correct=false, vmax=10, zmin=0)
    descend_precorrected!(xy, yaw, z_low)
    snap("$(tag)_low")
    g = grasp_smooth!(yaw)
    snap("$(tag)_closed")
    g < 120 && (println("EMPTY"); gripper!(1000); move_cmd!([xy..., 50], yaw; correct=false, vmax=8, zmin=0); return false)
    ok = lift_check!(yaw, z_carry - P_CMD[][3]; step=10.0, min_open=100)
    snap("$(tag)_lifted")
    ok || (println("LOST"); gripper!(1000); return false)
    move_cmd!([-130.0, -110, z_carry], yaw; correct=false, vmax=10, zmin=0)
    move_cmd!([BOX_DROP..., z_carry], drop_yaw; correct=false, vmax=10, zmin=0)
    println("over the box, opening ", stream().opening); snap("$(tag)_over_box")
    gripper!(1000); sleep(1.2); snap("$(tag)_dropped")
    return true
end

# Espresso cup task (2026-10-06 20:20): box at the lower right; Jellycat espresso cup at (-210, -22).
BOXES["tissue box"] = ([-185.0, -272, -40], [10.0, -80, 10])
BOXES["cup"] = ([-245.0, -57, -40], [-178.0, 13, 45])
"Pick by a thin, soft part (the handle) with the smooth grasp; carry to `drop`. MOVES THE ROBOT."
function pick_soft!(xy, yaw; tag, z_low, drop, z_carry=90.0, drop_yaw=0.0, min_g=40, min_open=40)
    gripper!(1000); sleep(0.8)
    move_cmd!([xy..., 70], yaw; correct=false, vmax=10, zmin=0)
    descend_precorrected!(xy, yaw, z_low)
    snap("$(tag)_low")
    g = grasp_smooth!(yaw)
    snap("$(tag)_closed")
    g < min_g && (println("EMPTY ($g)"); gripper!(1000); move_cmd!([xy..., 70], yaw; correct=false, vmax=8, zmin=0); return false)
    ok = lift_check!(yaw, z_carry - P_CMD[][3]; step=10.0, min_open)
    snap("$(tag)_lifted")
    ok || (println("LOST"); gripper!(1000); return false)
    move_cmd!([drop..., z_carry], drop_yaw; correct=false, vmax=8, zmin=0)
    println("over the box, opening ", stream().opening); snap("$(tag)_over_box")
    gripper!(1000); sleep(1.5); snap("$(tag)_dropped")
    move_cmd!([drop..., z_carry + 30], drop_yaw; correct=false, vmax=10, zmin=0)
    return true
end

# Cutlery task (2026-10-06 21:00): new scene, the camera moved; old zones gone.
delete!(BOXES, "tissue box"); delete!(BOXES, "cup")
# The base was turned 180° (2026-10-06 21:05): the wall is now on the +x side.
BOXES["wall"] = ([265.0, -700, -100], [450.0, 700, 700])

"Tool rotation for a held utensil: fingers along a (horizontal unit), the utensil (from the grip toward its head, horizontal u) turned theta degrees down."
function tool_R_tilt(a, u, theta)
    Z = [0, 0, 1.0]
    z = -cosd(theta) .* Z .- sind(theta) .* u
    x = normalize(a .- dot(a, z) .* z)
    return hcat(x, cross(z, x), z)
end

const MUG = [67.9, 155.2]   # top centre (camera rim ellipse, 2026-10-08 run 2, after the push moved it 8 mm); rim z ≈ 65, outer ≈ 72 mm, opening ≈ 66 mm
BOXES["mug"] = ([MUG[1] - 36, MUG[2] - 36, -40.0], [MUG[1] + 36, MUG[2] + 36, 65.0])   # outer rim ± 36 mm (2026-10-08 run 2)
"Grasp a flat utensil at xy (fingers across, yaw), lift, carry over the mug, tilt it into the mug, release. MOVES THE ROBOT."
function utensil_to_mug!(xy, yaw; tag, z_low=-11.0, head_dir=210.0, tilt=75.0, z_drop=140.0, z_release=z_drop)
    gripper!(1000); sleep(0.8)
    move_cmd!([xy..., 60], yaw; correct=false, vmax=10, zmin=0)
    descend_precorrected!(xy, yaw, z_low)
    snap("$(tag)_low")
    g = grasp_smooth!(yaw)
    snap("$(tag)_closed")
    P_CMD[] = [xy..., 100.0]
    move_tcp!(P_CMD[], yaw; correct=false, vmax=6, zmin=-20, ignore=["mug"])
    g2 = stream(); println("lifted: opening ", g2.opening, " load ", g2.load); snap("$(tag)_lifted")
    move_cmd!([MUG..., z_drop], yaw; correct=false, vmax=8, zmin=0, ignore=["mug"])
    snap("$(tag)_over_mug")
    a = [cosd(head_dir + 90), sind(head_dir + 90), 0.0]; u = [cosd(head_dir), sind(head_dir), 0.0]
    q = ik_to([MUG..., z_drop], tool_R_tilt(a, u, tilt); q0=state().q, opening=200, n=40)
    q === nothing && error("no IK for the tilted drop")
    move!(q; vmax=8, zmin=0, ignore=["mug"])
    println("tilted: opening ", stream().opening); snap("$(tag)_tilted")
    if z_release < z_drop
        q3 = ik_to([MUG..., z_release], tool_R_tilt(a, u, tilt); q0=state().q, opening=200, n=20)
        q3 === nothing || (move!(q3; vmax=4, zmin=0, ignore=["mug"]); snap("$(tag)_lowered"))
    end
    gripper!(1000); sleep(1.5); snap("$(tag)_dropped")
    qv = ik_best([MUG..., z_drop], 0, head_dir + 90; opening=1000)
    qv === nothing || move!(qv; vmax=10, zmin=0, ignore=["mug"])
    return true
end
"Turn J1 with the arm folded high (a safe way between the back parking and the work area). MOVES THE ROBOT."
function fold_turn!(j1)
    q = state().q
    MyCobot.atom_move_to(L, [q[1], 0.0, -45.0, -45.0, 0.0, q[6]]; duration=4.0, mechanism=M)
    MyCobot.atom_move_to(L, [j1, 0.0, -45.0, -45.0, 0.0, q[6]]; duration=max(3.0, abs(j1 - q[1]) / 20), mechanism=M)
end
BOXES["wall"] = ([300.0, -700, -100], [450.0, 700, 700])   # wall foot at x ≈ 305–315 (camera, 2026-10-06 23:00)

"Grasp a utensil (it hangs from the grip), carry it over the mug, lower its end into the mug, release. MOVES THE ROBOT."
function utensil_drop!(xy, yaw; tag, z_low=-17.0, z_over=140.0, z_release=105.0)
    gripper!(1000); sleep(0.8)
    move_cmd!([xy..., 60], yaw; correct=false, vmax=10, zmin=0)
    descend_precorrected!(xy, yaw, z_low)
    snap("$(tag)_low")
    g = grasp_smooth!(yaw)
    snap("$(tag)_closed")
    P_CMD[] = [xy..., 100.0]
    move_tcp!(P_CMD[], yaw; correct=false, vmax=6, zmin=-20, ignore=["mug"])
    g2 = stream(); println("lifted: opening ", g2.opening, " load ", g2.load); snap("$(tag)_lifted")
    move_cmd!([MUG..., z_over], yaw; correct=false, vmax=8, zmin=0, ignore=["mug"])
    snap("$(tag)_over_mug")
    move_cmd!([MUG..., z_release], yaw; correct=false, vmax=5, zmin=0, ignore=["mug"])
    snap("$(tag)_in_mug")
    gripper!(1000); sleep(1.5); snap("$(tag)_dropped")
    move_cmd!([MUG..., z_over], yaw; correct=false, vmax=8, zmin=0, ignore=["mug"])
    return true
end

set_grip_torque(v) = (MyCobot.atom_write_reg(L, 7, 48, [v & 0xFF, v >> 8]); println("gripper torque limit -> ", v))
"Grip near one end, let the utensil swing to hang (lower grip torque), carry it high over the mug, lower the end in, release. MOVES THE ROBOT."
function utensil_hang_drop!(xy, yaw; tag, z_low=-17.0, hang_torque=200, z_high=200.0, z_in=150.0, drop_yaw=90.0, drop_tilt=20.0)
    gripper!(1000); sleep(0.8)
    move_cmd!([xy..., 60], yaw; correct=false, vmax=10, zmin=0, ignore=["mug"])
    descend_precorrected!(xy, yaw, z_low)
    snap("$(tag)_low")
    g = grasp_smooth!(yaw)
    snap("$(tag)_closed")
    P_CMD[] = [xy..., 60.0]
    move_tcp!(P_CMD[], yaw; correct=false, vmax=5, zmin=-20, ignore=["mug"])
    println("lifted: opening ", stream().opening)
    set_grip_torque(hang_torque); sleep(1.5)
    println("after torque drop: opening ", stream().opening); snap("$(tag)_hang")
    R = tool_R(drop_tilt, drop_yaw; lean_az=drop_yaw - 90)
    q = ik_to([MUG..., z_high], R; q0=state().q, opening=200, n=40)
    q === nothing && error("no IK for the high pose")
    move!(q; vmax=6, zmin=0, ignore=["mug"]); snap("$(tag)_high")
    q2 = ik_to([MUG..., z_in], R; q0=state().q, opening=200, n=20)
    q2 === nothing && error("no IK for the low pose")
    move!(q2; vmax=4, zmin=0, ignore=["mug"]); snap("$(tag)_in")
    gripper!(1000); sleep(1.5); snap("$(tag)_released")
    set_grip_torque(1000)
    move!(q; vmax=8, zmin=0, ignore=["mug"])
    return true
end

"Fork into the mug, v2 (2026-10-06 23:40): grip open only `open` (the mug is near), carry level, tilt the head
down about the actual finger axis with the TCP offset so the head end lands at the mug centre, correct the sag
in the tilted pose, lower, release. head_az = horizontal direction from the grip to the head end on the table.
MOVES THE ROBOT."
function utensil_to_mug2!(xy, yaw; tag, head_az, head_len, open=400, z_low=-17.0, tilt=65.0, z_drop=140.0, z_release=100.0, go=true)
    off = head_len * cosd(tilt)
    if go
        gripper!(open); sleep(0.8)
        IGNORE_ALSO[] = ["mug"]
        try
            move_cmd!([xy..., 60], yaw; correct=false, vmax=10, zmin=0)
            descend_precorrected!(xy, yaw, z_low)
            snap("$(tag)_low")
            grasp_smooth!(yaw)
            snap("$(tag)_closed")
            P_CMD[] = [xy..., 100.0]
            move_tcp!(P_CMD[], yaw; correct=false, vmax=6, zmin=-20, ignore=["mug"])
        finally
            IGNORE_ALSO[] = String[]
        end
        g = stream(); println("lifted: opening ", g.opening, " load ", g.load); snap("$(tag)_lifted")
    end
    # the head direction stays as on the table during a level carry (yaw is world-fixed)
    t = [cosd(head_az), sind(head_az), 0.0]
    p_over = [(MUG .- off .* t[1:2])..., z_drop]
    go && (move_cmd!(p_over, yaw; correct=false, vmax=8, zmin=0, ignore=["mug"]); snap("$(tag)_over_mug"))
    q0 = go ? state().q : ik_best(p_over, 0, yaw; opening=open)
    R0 = Matrix(RBD.rotation(flange(q0)))
    a = normalize([R0[1, 1], R0[2, 1], 0.0])
    u = normalize(t .- dot(t, a) .* a)          # head direction, square to the finger axis
    println("finger axis ", round(atand(a[2], a[1]); digits=1), "°, head ", round(atand(u[2], u[1]); digits=1), "°, offset ", round(off; digits=1), " mm")
    R = tool_R_tilt(a, u, tilt)
    op = go ? stream().opening : 50
    p1 = [(MUG .- off .* u[1:2])..., z_drop]
    q1 = ik_to(p1, R; q0, opening=op, n=40)
    q1 === nothing && error("no IK for the tilted pose")
    println("tilted pose q = ", round.(q1; digits=1), "  problems: ", problems(q1; opening=op, zmin=0, ignore=["mug"]))
    p2 = [p1[1], p1[2], z_release]
    q2 = ik_to(p2, R; q0=q1, opening=op, n=20)
    println("release pose q = ", q2 === nothing ? "none" : round.(q2; digits=1))
    go || return (q1, q2)
    move!(q1; vmax=8, zmin=0, ignore=["mug"])
    e = p1 .- tcp(state().q; opening=op); e[3] = 0
    println("tilted: opening ", stream().opening, ", sag error ", round.(e; digits=1)); snap("$(tag)_tilted")
    if norm(e) > 2
        q1c = ik_to(p1 .+ e, R; q0=state().q, opening=op, n=10)
        q1c === nothing || move!(q1c; vmax=4, zmin=0, ignore=["mug"])
        println("corrected -> tcp ", round.(tcp(state().q; opening=op); digits=1))
    end
    q2c = ik_to(p2 .+ e, R; q0=state().q, opening=op, n=20)
    q2c === nothing && error("no IK for the release pose")
    move!(q2c; vmax=4, zmin=0, ignore=["mug"]); snap("$(tag)_lowered")
    println("lowered: tcp ", round.(tcp(state().q; opening=op); digits=1))
    gripper!(1000); sleep(1.5); snap("$(tag)_dropped")
    q3 = ik_to(p1 .+ e, R; q0=state().q, opening=1000, n=10)
    q3 === nothing || move!(q3; vmax=6, zmin=0, ignore=["mug"])
    return true
end

"Grasp a utensil, lift it with the gripper level to z_lift and let it settle (it pivots in the pads). Prints the TCP and finger axis for the camera fit. MOVES THE ROBOT."
function utensil_lift!(xy, yaw; tag, open=400, z_low=-17.0, z_lift=150.0)
    gripper!(open); sleep(0.8)
    move_cmd!([xy..., 60], yaw; correct=false, vmax=10, zmin=0)
    descend_precorrected!(xy, yaw, z_low)
    snap("$(tag)_low")
    grasp_smooth!(yaw; free_lift=8.0)
    snap("$(tag)_closed")
    P_CMD[] = [xy..., z_lift]
    move_tcp!(P_CMD[], yaw; correct=false, vmax=5, zmin=-20)
    sleep(3.0)
    hang_report(tag)
end
function hang_report(tag)
    s = stream(); q = state().q; R = Matrix(RBD.rotation(flange(q)))
    snap(tag)
    p = tcp(q; opening=s.opening)
    @printf("HANG tcp %.1f %.1f %.1f axis %.1f %.1f %.1f opening %d load %d\n", p..., R[:, 1]..., s.opening, s.load)
end

"Align the TCP above xy in closed loop (re-command with the measured error until within tol), then go straight down by `above`. Sets P_CMD to the low command. MOVES THE ROBOT."
function descend_aligned!(xy, yaw, z_low; above=15.0, tol=1.5, opening=1000)
    p = [xy..., z_low + above]; cmd = copy(p)
    move_cmd!(cmd, yaw; correct=false, vmax=5, zmin=-10, opening)
    for k in 1:3
        e = p .- tcp(state().q; opening)
        @printf("align %d: error %s\n", k, string(round.(e; digits=1)))
        norm(e[1:2]) < tol && abs(e[3]) < 3 && break
        cmd = cmd .+ e
        q = ik_best(cmd, 0, yaw; q0=state().q, opening); q === nothing && error("no IK")
        move!(q; opening, zmin=-10, vmax=4, ignore=vcat(["plush"], IGNORE_ALSO[]))
    end
    P_CMD[] = cmd .- [0, 0, above]
    q = ik_best(P_CMD[], 0, yaw; q0=state().q, opening); q === nothing && error("no IK low")
    move!(q; opening, zmin=-10, vmax=3, ignore=vcat(["plush"], IGNORE_ALSO[]))
    println("low: tcp ", round.(tcp(state().q; opening); digits=1), " (target ", round.([xy..., z_low]; digits=1), ")")
end
"Like utensil_lift!, with the closed-loop alignment and free lift. MOVES THE ROBOT."
function utensil_lift2!(xy, yaw; tag, open=1000, z_low=-17.0, z_lift=150.0)
    gripper!(open); sleep(0.8)
    move_cmd!([xy..., 60], yaw; correct=false, vmax=10, zmin=0, opening=open)
    descend_aligned!(xy, yaw, z_low; opening=open)
    snap("$(tag)_low")
    grasp_smooth!(yaw; free_lift=8.0)
    snap("$(tag)_closed")
    P_CMD[] = [xy..., z_lift]
    move_tcp!(P_CMD[], yaw; correct=false, vmax=5, zmin=-20)
    sleep(3.0)
    hang_report(tag)
end

"Carry a hanging utensil so its lower end (vector v from the TCP, from the camera fit) is over the mug centre at height z_end. MOVES THE ROBOT."
function hang_over_mug!(v, yaw; tag, z_end=90.0, vmax=5)
    p = [MUG[1] - v[1], MUG[2] - v[2], z_end - v[3]]
    println("TCP target ", round.(p; digits=1))
    move_cmd!(p, yaw; correct=true, vmax, zmin=0, ignore=["mug"])
    sleep(3.0)
    hang_report(tag)
end

"For a hanging utensil (vector v from the TCP to its lower end): the end target over the mug so that, lowered
to z_in, the utensil crosses the rim plane about as far from the centre on one side as its end on the other."
function hang_aim(v; z_in=20.0, z_rim=65.0)
    h = hypot(v[1], v[2]); u = h < 1 ? [0.0, 0.0] : [v[1], v[2]] ./ h
    f = clamp((-v[3] - (z_rim - z_in)) / -v[3], 0, 1)   # fraction of v from the TCP to the rim crossing... (TCP above the rim)
    k = h * (1 - f) / 2                                  # end k mm past the centre, crossing k' before it
    return MUG .+ k .* u
end
"Move a hanging utensil (keep the wrist: continuation IK) so its end is at [xy, z_end]. MOVES THE ROBOT."
function hang_move!(v, xy, z_end; tag, vmax=4)
    s = stream(); q0 = state().q; R = Matrix(RBD.rotation(flange(q0)))
    p = [xy[1] - v[1], xy[2] - v[2], z_end - v[3]]
    q = ik_to(p, R; q0, opening=s.opening, n=30); q === nothing && error("no IK for $p")
    println("TCP -> ", round.(p; digits=1), " q ", round.(q; digits=1))
    move!(q; opening=s.opening, vmax, zmin=0, ignore=["mug"])
    e = p .- tcp(state().q; opening=s.opening)
    if norm(e) > 2
        q2 = ik_to(p .+ e, R; q0=state().q, opening=s.opening, n=10)
        q2 === nothing || move!(q2; opening=s.opening, vmax, zmin=0, ignore=["mug"])
    end
    sleep(2.5)
    hang_report(tag)
end

"Lowest z of a held utensil end (vt: end in the flange frame) while it is within dmax of the mug axis, on the joint path a -> b."
function end_path_low(a, b, vt; opening, dmax=48.0)
    worst = Inf
    for s in range(0, 1; length=60)
        q = a .+ s .* (b .- a); R = Matrix(RBD.rotation(flange(q)))
        e = tcp(q; opening) .+ R * vt
        hypot(e[1] - MUG[1], e[2] - MUG[2]) < dmax && (worst = min(worst, e[3]))
    end
    return worst
end


