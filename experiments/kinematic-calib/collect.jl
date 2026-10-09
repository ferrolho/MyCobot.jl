# Kinematic calibration, step 1: move the arm through sampled poses in free air and record, at each, a
# camera snapshot, the joint angles and goals, J7 and the IMU. Run with the pick-and-place helpers
# loaded (experiments/pick-and-place/pi/plush.jl). MOVES THE ROBOT.
#   poses = sample_poses(30); collect_poses(poses; tag="kc1")
using Random, Printf, LinearAlgebra

const KC_DIR = "/home/henrique/scratch/kincal"
const KC_CAM = (rvec=[0.378544667, 2.920413437, -1.06418754], tvec=[193.228348, -34.520422, 662.285854],
                f=1457.1855, cx=640.0, cy=480.0)   # lab-camera.json, 2026-10-08 (base frame, plate centred)
BOXES["laptop"] = ([0.0, -390, -40], [340.0, -5, 50])   # the laptop on its book, with things on top

function rodrigues(r)
    θ = norm(r); θ < 1e-12 && return Matrix(1.0I, 3, 3)
    k = r / θ; K = [0 -k[3] k[2]; k[3] 0 -k[1]; -k[2] k[1] 0]
    return I + sin(θ) * K + (1 - cos(θ)) * K * K
end
const KC_R = rodrigues(KC_CAM.rvec)
"Pixel (1280x960) of a base-frame point (mm), or nothing behind the camera."
function cam_px(p)
    c = KC_R * p .+ KC_CAM.tvec
    c[3] <= 0 && return nothing
    return [KC_CAM.f * c[1] / c[3] + KC_CAM.cx, KC_CAM.f * c[2] / c[3] + KC_CAM.cy]
end
inimage(px; m=60) = px !== nothing && m < px[1] < 1280 - m && m < px[2] < 960 - m

"Base-frame origin (mm) of a link at pose q."
function link_origin(q, name)
    RBD.set_configuration!(ST, deg2rad.(q))
    1000 .* Vector(RBD.translation(RBD.transform_to_root(ST, RBD.findbody(M, name))))
end

"Random poses in free air that the camera sees, with their gripper (closed) at least zmin above the table."
function sample_poses(n; seed=1, zmin=45.0)
    rng = MersenneTwister(seed)
    lo = [-40.0, -70, -130, -100, -100, -170]; hi = [80.0, 30, -10, 100, 100, 100]
    out = Vector{Vector{Float64}}()
    tries = 0
    while length(out) < n && tries < 200000
        tries += 1
        q = lo .+ rand(rng, 6) .* (hi .- lo)
        isempty(problems(q; opening=0, zmin)) || continue
        t = tcp(q; opening=0); f = link_origin(q, "joint6")
        (60 < t[3] < 260) || continue
        (inimage(cam_px(t)) && inimage(cam_px(f))) || continue
        all(norm(q .- p) > 25 for p in out) || continue   # spread out
        push!(out, q)
    end
    println("sampled ", length(out), " poses in ", tries, " tries")
    return out
end

"Order poses greedily by joint distance from q0."
function order_poses(poses, q0)
    left = copy(poses); out = similar(poses, 0); q = q0
    while !isempty(left)
        k = argmin([maximum(abs, p .- q) for p in left]); q = popat!(left, k); push!(out, q)
    end
    return out
end

"Visit the poses (checked paths), snapshot each, append rows to KC_DIR/tag.csv. MOVES THE ROBOT."
function collect_poses(poses; tag="kc1", settle=1.2, vmax=25.0)
    mkpath(KC_DIR)
    gripper!(0); sleep(0.8)
    file = joinpath(KC_DIR, "$tag.csv")
    new = !isfile(file)
    open(file, "a") do io
        new && println(io, "name,", join(["q$j" for j in 1:6], ","), ",", join(["g$j" for j in 1:6], ","), ",j7,ax,ay,az")
        for (k, q) in enumerate(order_poses(poses, state().q))
            pr = path_problems(state().q, q; opening=0, zmin=45)
            if !isempty(pr)   # through two high poses: rise where it is, turn J1 high, then go down to the pose
                c = state().q
                vias = [[c[1], -5.0, -60.0, -25.0, 0.0, c[6]], [q[1], -5.0, -60.0, -25.0, 0.0, q[6]]]
                legs = [(c, vias[1]), (vias[1], vias[2]), (vias[2], q)]
                bad = [first(path_problems(a, b; opening=0, zmin=45), 1) for (a, b) in legs]
                if all(isempty, bad)
                    try
                        for v in vias; move!(v; vmax, opening=0, zmin=45, guard=false); end
                    catch e
                        println("pose $k: via failed: ", e); continue
                    end
                else
                    println("skip pose $k: ", bad); continue
                end
            end
            try
                move!(q; vmax, opening=0, zmin=45, guard=false)
            catch e
                println("pose $k: move failed: ", e); continue
            end
            sleep(settle)
            name = @sprintf("%s_%02d", tag, k)
            run(`curl -fsS -o $KC_DIR/$name.jpg http://100.69.15.110:8280/snapshot.jpg`)
            s = MyCobot.atom_state(L)
            imu = s.imu[1:3]   # accelerometer (g), in the ATOM's frame
            println(io, name, ",", join(round.(s.q[1:6]; digits=3), ","), ",", join(round.(s.goal[1:6]; digits=3), ","), ",",
                    round(s.q[7]; digits=3), ",", join(round.(imu; digits=4), ","))
            flush(io)
            @printf("%s done: q %s\n", name, round.(s.q[1:6]; digits=1))
        end
    end
end
