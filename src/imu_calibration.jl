# Gravity calibration of the end-effector IMU from static poses (scripts/static_poses.jl).
#
# At rest the accelerometer measures "up" (1 g) in its own axes. The model predicts it from the
# measured joint angles:
#   acc = C · R_SL · R_BL(q + δ)ᵀ · u_B + b
# u_B   world "up" in the base frame (2 parameters: the tilt of the base; the heading about
#       vertical is invisible to gravity)
# R_BL  forward kinematics of the IMU link ("joint6", between J5 and J6) in the base frame
# δ     joint zero offsets. J2, J3 and J4 are parallel, so only δ2 + δ3 + δ4 changes the IMU
#       orientation: it is fitted as δ2 (δ3 = δ4 = 0). δ1 merges with the base heading and δ5
#       with the IMU mounting, so neither is fitted.
# R_SL  how the IMU sits on its link (3 parameters, a rotation vector on top of an initial fit)
# C, b  accelerometer scale and cross-axis terms (C = I + symmetric matrix, 6 parameters) and
#       bias (3 parameters)

import LinearAlgebra as LA

const IMU_BODY = "joint6"
const IMU_PARAM_NAMES = ["tilt_x", "tilt_y", "mount_x", "mount_y", "mount_z", "delta_pitch",
                         "scale_x", "scale_y", "scale_z", "cross_xy", "cross_xz", "cross_yz",
                         "bias_x", "bias_y", "bias_z"]

skew(w) = [0 -w[3] w[2]; w[3] 0 -w[1]; -w[2] w[1] 0]

"Rotation matrix of the rotation vector `w` (rad)."
function rotvec(w::AbstractVector)
    θ = LA.norm(w)
    θ < 1e-12 && return Matrix{Float64}(LA.I, 3, 3) + skew(w)
    K = skew(w ./ θ)
    return LA.I + sin(θ) * K + (1 - cos(θ)) * K * K
end

"World up in the base frame for base tilts `tx`, `ty` (rad) about the base x and y axes."
base_up(tx, ty) = rotvec([tx, 0, 0]) * rotvec([0, ty, 0]) * [0.0, 0, 1]

"Up in IMU-link coordinates for joint angles `q_deg`, base up `u_B` and zero offsets `δ_deg`."
function up_in_link(state::RBD.MechanismState, q_deg::AbstractVector, u_B::AbstractVector, δ_deg::AbstractVector=zeros(6))
    RBD.set_configuration!(state, deg2rad.(q_deg .+ δ_deg))
    R_BL = Matrix(RBD.rotation(RBD.transform_to_root(state, RBD.findbody(state.mechanism, IMU_BODY))))
    return R_BL' * u_B
end

"Model accelerometer readings (3 × n, g) for joint angles `Q` (n × 6, degrees) and parameters `p`."
function imu_gravity_model(state::RBD.MechanismState, Q::AbstractMatrix, p::AbstractVector, R0::AbstractMatrix)
    u_B = base_up(p[1], p[2])
    R_SL = rotvec(p[3:5]) * R0
    δ = [0, rad2deg(p[6]), 0, 0, 0, 0]
    C = LA.I + [p[7] p[10] p[11]; p[10] p[8] p[12]; p[11] p[12] p[9]]
    b = p[13:15]
    return reduce(hcat, (C * (R_SL * up_in_link(state, Q[i, :], u_B, δ)) .+ b for i in 1:size(Q, 1)))
end

"Rotation that best maps the unit vectors in the columns of `U` onto those of `A` (Kabsch)."
function best_rotation(U::AbstractMatrix, A::AbstractMatrix)
    F = LA.svd(A * U')
    D = LA.Diagonal([1, 1, sign(LA.det(F.U * F.Vt))])
    return F.U * D * F.Vt
end

"""
    fit_imu_gravity(Q, acc; mechanism=load_mechanism(), iters=100)

Fit the gravity model to static poses: `Q` (n × 6) measured joint angles in degrees, `acc`
(n × 3) mean accelerometer readings in g. Levenberg-Marquardt with a finite-difference
Jacobian. Returns a named tuple with the parameters `p` (see `IMU_PARAM_NAMES`), their
standard deviations `σ`, the initial mounting `R0`, the residuals (g) and the angle residuals (°).
"""
function fit_imu_gravity(Q::AbstractMatrix, acc::AbstractMatrix; mechanism=load_mechanism(), iters::Integer=100)
    state = RBD.MechanismState(mechanism)
    A = Matrix(acc')
    U = reduce(hcat, (up_in_link(state, Q[i, :], [0.0, 0, 1]) for i in 1:size(Q, 1)))
    R0 = best_rotation(U, A ./ sqrt.(sum(abs2, A; dims=1)))
    resid(p) = vec(imu_gravity_model(state, Q, p, R0) .- A)
    p = zeros(15)
    r = resid(p)
    λ = 1e-3
    for _ in 1:iters
        J = jacobian_fd(resid, p, r)
        H = J' * J
        g = J' * r
        step = -(H + λ * LA.Diagonal(LA.diag(H) .+ 1e-12)) \ g
        r_new = resid(p .+ step)
        if sum(abs2, r_new) < sum(abs2, r)
            p .+= step; r = r_new; λ = max(λ / 10, 1e-12)
            LA.norm(step) < 1e-12 && break
        else
            λ *= 10
            λ > 1e8 && break
        end
    end
    J = jacobian_fd(resid, p, r)
    dof = length(r) - length(p)
    σ = sqrt.(max.(LA.diag(LA.pinv(J' * J)) .* sum(abs2, r) / dof, 0))
    M = imu_gravity_model(state, Q, p, R0)
    angle = [acosd(clamp(LA.dot(M[:, i], A[:, i]) / (LA.norm(M[:, i]) * LA.norm(A[:, i])), -1, 1)) for i in 1:size(A, 2)]
    return (p=p, σ=σ, R0=R0, residual=reshape(r, 3, :), angle_deg=angle, rms_g=sqrt(sum(abs2, r) / length(r)))
end

function jacobian_fd(f, p, f0; h=1e-7)
    J = zeros(length(f0), length(p))
    for k in eachindex(p)
        dp = zeros(length(p)); dp[k] = h
        J[:, k] = (f(p .+ dp) .- f0) ./ h
    end
    return J
end

"""
    static_pose_means(path; t_min=2.0)

Mean joint angles, accelerometer and gyro of each step of a static-pose recording
(`*_static_samples.csv`), from the samples at `t ≥ t_min` seconds after the move.
Returns `(step, kind, goal, q, acc, gyro, acc_std)` with one row per step.
"""
function static_pose_means(path::AbstractString; t_min::Real=2.0)
    data, header = DelimitedFiles.readdlm(path, ',', Any; header=true)
    col(name) = findfirst(==(name), vec(header))
    steps = unique(Int.(data[:, col("step")]))
    rows = [findall(i -> data[i, col("step")] == s && data[i, col("t")] >= t_min && data[i, col("ok")] == 1, 1:size(data, 1)) for s in steps]
    keep = findall(!isempty, rows)
    meanof(names, r) = [sum(Float64.(data[r, col(n)])) / length(r) for n in names]
    stdof(names, r) = [sqrt(sum(abs2, Float64.(data[r, col(n)]) .- sum(Float64.(data[r, col(n)])) / length(r)) / length(r)) for n in names]
    rowsof(f, names) = reduce(vcat, permutedims(f(names, rows[k])) for k in keep)
    return (step=steps[keep], kind=[string(data[rows[k][1], col("kind")]) for k in keep],
            goal=rowsof(meanof, ["goal$j" for j in 1:6]), q=rowsof(meanof, ["q$j" for j in 1:6]),
            acc=rowsof(meanof, ["acc_x", "acc_y", "acc_z"]), gyro=rowsof(meanof, ["gyro_x", "gyro_y", "gyro_z"]),
            acc_std=rowsof(stdof, ["acc_x", "acc_y", "acc_z"]))
end
