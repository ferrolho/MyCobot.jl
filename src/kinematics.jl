import LinearAlgebra
import RigidBodyDynamics as RBD

const URDF_PATH = joinpath(@__DIR__, "..", "mycobot_description", "urdf", "mycobot_280_arduino", "mycobot_280_arduino.urdf")
const FLANGE_BODY = "joint6_flange"

"""
    load_mechanism(; urdf=URDF_PATH)

Parse the myCobot 280 (for Arduino) URDF. Joint angles use the same convention as
the ATOM's `get_angles` (checked against `get_coords` to within ~1 cm).
"""
load_mechanism(; urdf::AbstractString=URDF_PATH) = RBD.parse_urdf(urdf, remove_fixed_tree_joints=false)

"""
    flange_transform(state, q_deg)

Pose of the flange in the base frame (`RBD.Transform3D`, metres) for joint angles in degrees.
"""
function flange_transform(state::RBD.MechanismState, q_deg::AbstractVector)
    RBD.set_configuration!(state, deg2rad.(q_deg))
    flange = RBD.findbody(state.mechanism, FLANGE_BODY)
    return RBD.transform_to_root(state, flange)
end

"""
    flange_position_mm(state, q_deg)

Flange position in the base frame, in millimetres.
"""
flange_position_mm(state::RBD.MechanismState, q_deg::AbstractVector) =
    1000 .* Vector(RBD.translation(flange_transform(state, q_deg)))

"""
    flange_goal(mechanism, rotation, position_mm)

A target flange pose for `inverse_kinematics`: `rotation` (3×3) and `position_mm`
are expressed in the base frame.
"""
flange_goal(mechanism::RBD.Mechanism, rotation::AbstractMatrix, position_mm::AbstractVector) =
    RBD.Transform3D(RBD.CartesianFrame3D("flange_goal"), RBD.root_frame(mechanism),
                    rotation, RBD.SVector{3}(position_mm ./ 1000))

"""
    joint_limits_deg(mechanism)

Lower and upper position limits of the six revolute joints, in degrees.
"""
function joint_limits_deg(mechanism::RBD.Mechanism)
    joints = filter(j -> RBD.num_positions(j) == 1, RBD.joints(mechanism))
    bounds = [only(RBD.position_bounds(j)) for j in joints]
    return rad2deg.(RBD.lower.(bounds)), rad2deg.(RBD.upper.(bounds))
end

"""
    inverse_kinematics(state, goal, q_init_deg; tol=1e-6, max_iters=100, damping=1e-3)

Damped least-squares IK for the full flange pose. `goal` is an `RBD.Transform3D`
from any frame to the base frame (metres). The error is the exponential coordinates
of the relative transform (`log`), and the step uses RBD's geometric Jacobian
expressed in the flange frame. Returns `(q_deg, position_error_m, rotation_error_rad)`.
"""
function inverse_kinematics(state::RBD.MechanismState, goal::RBD.Transform3D, q_init_deg::AbstractVector;
                            tol::Real=1e-6, max_iters::Integer=100, damping::Real=1e-3)
    mechanism = state.mechanism
    flange = RBD.findbody(mechanism, FLANGE_BODY)
    path = RBD.path(mechanism, RBD.root_body(mechanism), flange)
    q = deg2rad.(collect(Float64, q_init_deg))
    rot_err, pos_err = Inf, Inf
    for _ in 1:max_iters
        RBD.set_configuration!(state, q)
        T = RBD.transform_to_root(state, flange)
        ξ = log(inv(T) * goal)          # twist taking the flange to the goal, in flange coordinates
        e = vcat(RBD.angular(ξ), RBD.linear(ξ))
        rot_err, pos_err = LinearAlgebra.norm(RBD.angular(ξ)), LinearAlgebra.norm(RBD.linear(ξ))
        (rot_err < tol && pos_err < tol) && break
        J = RBD.transform(RBD.geometric_jacobian(state, path), inv(T))
        Jm = vcat(RBD.angular(J), RBD.linear(J))
        Δq = (Jm' * Jm + damping^2 * LinearAlgebra.I) \ (Jm' * e)
        q .+= clamp.(Δq, -0.1, 0.1)
    end
    return rad2deg.(q), pos_err, rot_err
end
