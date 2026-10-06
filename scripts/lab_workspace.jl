# The safe workspace in the lab (Raspberry Pi desk, walls behind and to the right of the arm).
# Included by scripts that choose poses on their own (static_poses.jl, encoder_sweep.jl).
#
# Every link origin from joint4 on (and the flange) stays at least MIN_Z_MM above the table and
# inside the reach per azimuth sector (REACH_MM: the reach visited on the Control page before
# 2026-10-06, minus a 20 mm margin, at least 100 mm), and the wrist stays away from the base column.

import RigidBodyDynamics as RBD
import MyCobot

const MIN_Z_MM = 120.0
# Reach limit per 30° sector of azimuth (atan2(y, x) in the base frame), mm.
const REACH_MM = [183, 190, 100, 100, 182, 189, 177, 113, 106, 190, 190, 190]

const WS_MECHANISM = MyCobot.load_mechanism()
const WS_STATE = RBD.MechanismState(WS_MECHANISM)
const WS_LO, WS_HI = MyCobot.joint_limits_deg(WS_MECHANISM)
const WS_CHECKED = [b for b in RBD.bodies(WS_MECHANISM) if occursin(r"^(joint[4-6]|joint6_flange)$", string(b))]
const WS_WRIST = [b for b in WS_CHECKED if string(b) != "joint4"]

"Is the pose (degrees) 10° inside the joint limits and inside the table, reach and wrist rules?"
function pose_ok(q)
    all(WS_LO .+ 10 .< q .< WS_HI .- 10) || return false
    RBD.set_configuration!(WS_STATE, deg2rad.(q))
    for b in WS_CHECKED
        p = 1000 .* RBD.translation(RBD.transform_to_root(WS_STATE, b))
        r = hypot(p[1], p[2])
        p[3] >= MIN_Z_MM || return false
        k = floor(Int, mod(atand(p[2], p[1]), 360) / 30) + 1
        r <= REACH_MM[k] || return false
        b in WS_WRIST && r < 70 && p[3] < 250 && return false   # wrist folded down next to the base column
    end
    return true
end

"Is the straight joint-space path from `a` to `b` safe (checked at 50 points)?"
path_ok(a, b) = all(pose_ok(a .+ s .* (b .- a)) for s in range(0, 1; length=50))
