// twist.h — end-effector JOG (firmware 5.1): the client streams a twist of the TCP (the tool point), and the
// ATOM turns it into joint goals at 500 Hz. Plain C++: tested by tools/firmware-tests/, and used by the
// simulated ATOM (tools/atom_sim.py, through tools/twist_lib.cpp).
//
// - The kinematic chain is generated from the URDF (robot_params.h, tools/gen_robot.py).
// - A target pose moves with the twist. Damped least squares (adaptive damping near singular poses) gives the
//   joint speeds. One scale for all joints keeps them inside the speed limit, the acceleration limits and a
//   braking distance before their limits, so the TCP keeps its direction.
// - The arm stops 10° before the singular angles J3 = 0° (the arm straight) and J5 = ±90° (J4 and J6 align),
//   and brakes when the joints cannot give the twist (more than 20 % off).
// Units: mm, rad, rad/s inside; joint angles in degrees (as the rest of the firmware).
#pragma once
#include <math.h>
#include <string.h>
#include "motion_limits.h"

namespace twist {

const int N = robot::N_ARM;
const float D2R = 0.017453292519943295f;

struct Pose { float R[9]; float p[3]; };   // R: the tool axes in the base frame (columns), row-major; p in mm

inline void mul3(const float a[9], const float b[9], float c[9]) {
    float t[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) t[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
    memcpy(c, t, sizeof(t));
}
inline void mulv(const float a[9], const float v[3], float o[3]) {
    float t[3];
    for (int i = 0; i < 3; i++) t[i] = a[3 * i] * v[0] + a[3 * i + 1] * v[1] + a[3 * i + 2] * v[2];
    memcpy(o, t, sizeof(t));
}
inline void transpose(const float a[9], float t[9]) {
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) t[3 * j + i] = a[3 * i + j];
}
inline float norm3(const float v[3]) { return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

// Rotation by |w| about w (rad), Rodrigues.
inline void exp_so3(const float w[3], float R[9]) {
    const float th = norm3(w);
    if (th < 1e-9f) { const float I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; memcpy(R, I, sizeof(I)); return; }
    const float x = w[0] / th, y = w[1] / th, z = w[2] / th, s = sinf(th), c = cosf(th), t = 1 - c;
    const float r[9] = {c + x * x * t, x * y * t - z * s, x * z * t + y * s,
                        y * x * t + z * s, c + y * y * t, y * z * t - x * s,
                        z * x * t - y * s, z * y * t + x * s, c + z * z * t};
    memcpy(R, r, sizeof(r));
}

// Rotation vector of R (rad). The errors here are small (a few degrees), far from π.
inline void log_so3(const float R[9], float w[3]) {
    const float c = fmaxf(-1.0f, fminf(1.0f, (R[0] + R[4] + R[8] - 1) * 0.5f));
    const float th = acosf(c);
    const float v[3] = {R[7] - R[5], R[2] - R[6], R[3] - R[1]};
    const float k = th < 1e-4f ? 0.5f : th / (2 * sinf(th));
    for (int i = 0; i < 3; i++) w[i] = v[i] * k;
}

// Pose of the TCP (tcp: in the flange frame, mm) for joint angles (°), and each joint's axis and origin (base frame).
inline void fk(const float q[N], const float tcp[3], Pose& out, float axes[N][3], float origins[N][3]) {
    float R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}, p[3] = {0, 0, 0}, d[3];
    for (int j = 0; j < N; j++) {
        mulv(R, robot::CHAIN_P_MM[j], d);
        for (int i = 0; i < 3; i++) p[i] += d[i];
        mul3(R, robot::CHAIN_R[j], R);
        mulv(R, robot::CHAIN_AXIS[j], axes[j]);
        memcpy(origins[j], p, sizeof(p));
        float w[3], Rq[9];
        for (int i = 0; i < 3; i++) w[i] = robot::CHAIN_AXIS[j][i] * q[j] * D2R;
        exp_so3(w, Rq);
        mul3(R, Rq, R);
    }
    mulv(R, robot::FLANGE_P_MM, d);
    for (int i = 0; i < 3; i++) p[i] += d[i];
    mul3(R, robot::FLANGE_R, R);
    mulv(R, tcp, d);
    for (int i = 0; i < 3; i++) out.p[i] = p[i] + d[i];
    memcpy(out.R, R, sizeof(R));
}

// The geometric Jacobian of the TCP: rows 0-2 linear (mm/rad), rows 3-5 angular.
inline void jacobian(const float q[N], const float tcp[3], float J[6][N], Pose& pose) {
    float axes[N][3], org[N][3];
    fk(q, tcp, pose, axes, org);
    for (int j = 0; j < N; j++) {
        const float* z = axes[j];
        const float r[3] = {pose.p[0] - org[j][0], pose.p[1] - org[j][1], pose.p[2] - org[j][2]};
        J[0][j] = z[1] * r[2] - z[2] * r[1];
        J[1][j] = z[2] * r[0] - z[0] * r[2];
        J[2][j] = z[0] * r[1] - z[1] * r[0];
        J[3][j] = z[0]; J[4][j] = z[1]; J[5][j] = z[2];
    }
}

// Solve A x = b (6 × 6), Gaussian elimination with partial pivoting. A and b are changed.
inline void solve6(float A[6][6], float b[6], float x[6]) {
    for (int c = 0; c < 6; c++) {
        int piv = c;
        for (int r = c + 1; r < 6; r++) if (fabsf(A[r][c]) > fabsf(A[piv][c])) piv = r;
        if (piv != c) { for (int k = 0; k < 6; k++) { float t = A[c][k]; A[c][k] = A[piv][k]; A[piv][k] = t; } float t = b[c]; b[c] = b[piv]; b[piv] = t; }
        for (int r = c + 1; r < 6; r++) {
            const float f = A[r][c] / A[c][c];
            for (int k = c; k < 6; k++) A[r][k] -= f * A[c][k];
            b[r] -= f * b[c];
        }
    }
    for (int r = 5; r >= 0; r--) {
        float s = b[r];
        for (int k = r + 1; k < 6; k++) s -= A[r][k] * x[k];
        x[r] = s / A[r][r];
    }
}

// Smallest eigenvalue of a symmetric 6 × 6 matrix (cyclic Jacobi).
inline float min_eig6(const float A0[6][6]) {
    float A[6][6];
    memcpy(A, A0, sizeof(A));
    for (int sweep = 0; sweep < 12; sweep++) {
        float off = 0;
        for (int i = 0; i < 6; i++) for (int j = i + 1; j < 6; j++) off += A[i][j] * A[i][j];
        if (off < 1e-14f) break;
        for (int p = 0; p < 6; p++)
            for (int q = p + 1; q < 6; q++) {
                if (fabsf(A[p][q]) < 1e-20f) continue;
                const float th = (A[q][q] - A[p][p]) / (2 * A[p][q]);
                const float t = (th >= 0 ? 1.0f : -1.0f) / (fabsf(th) + sqrtf(th * th + 1));
                const float c = 1 / sqrtf(t * t + 1), s = t * c;
                for (int k = 0; k < 6; k++) { const float a = A[k][p], b = A[k][q]; A[k][p] = c * a - s * b; A[k][q] = s * a + c * b; }
                for (int k = 0; k < 6; k++) { const float a = A[p][k], b = A[q][k]; A[p][k] = c * a - s * b; A[q][k] = s * a + c * b; }
            }
    }
    float m = A[0][0];
    for (int i = 1; i < 6; i++) m = fminf(m, A[i][i]);
    return m;
}

// --- The controller ----------------------------------------------------------------------------------

const float L_REF = 100;            // mm: linear rows of the Jacobian in units of 100 mm
const float SIGMA_DAMP = 0.05f;     // the damping starts below this singular value
const float LAMBDA_MAX = 0.05f;     // the damping at a singularity
const float K_POSE = 4;             // 1/s: pulls the joint goals onto the target pose (drift of the linear step)
const float ANCHOR_MM = 5, ANCHOR_RAD = 3 * D2R;   // re-anchor the target beyond these errors
const float FIDELITY = 0.2f;        // brake if the joints give a twist more than 20 % off
const float LIMIT_INSIDE = 0.5f;    // ° inside the JOG range (2° inside the limits): float rounding never touches it
const float A_LIN = 300;            // mm/s², the TCP
const float A_ANG = 4;              // rad/s² (230 °/s²), the TCP
const float AMAX_SHARE = 0.8f;      // of the joint acceleration limits
// The smallest singular value (Jacobi sweeps) costs about 80 % of a step on the ATOM, which has no hardware
// float division or square root (2026-10-10: 470 µs per step). It changes slowly, so compute it every 4th step.
const int SIGMA_EVERY = 4;

// Singular angles (the smallest singular value of the Jacobian is 0): J3 = 0° and J5 = ±90°. When a joint is more
// than KEEP_OUT_DEG from one, that angle ± KEEP_OUT_DEG is a limit on its side. A joint that starts nearer (the
// zero pose has J3 = 0) can leave on either side.
const float KEEP_OUT_DEG = 10;
struct KeepOut { int joint; float angle; };
const KeepOut KEEP_OUT[] = {{2, 0}, {4, -90}, {4, 90}};

struct State {
    float q[N], qd[N];   // joint goals (°) and their speeds (°/s)
    float v[6];          // TCP twist now: mm/s, rad/s (base frame)
    Pose target;         // TCP target pose
    float qmin[N], qmax[N];
    float sigma;         // smallest singular value of the scaled Jacobian, from the last computation
    int sigma_age;       // steps since then
};

struct Info {
    float scale;         // 1 = the twist as wanted; < 1 slowed by a limit; 0 stopped
    int limit_joint;     // the joint (0-based) that brakes at its limit, or -1
    float sigma_min;     // smallest singular value of the scaled Jacobian
    bool singular;       // the damping is on
    bool blocked;        // the joints cannot give the twist: the arm brakes
};

inline void update_bounds(State& s) {
    for (int j = 0; j < N; j++) {
        s.qmin[j] = lim::MODEL_MIN_DEG[j] + lim::margin(j) + LIMIT_INSIDE;
        s.qmax[j] = lim::MODEL_MAX_DEG[j] - lim::margin(j) - LIMIT_INSIDE;
    }
    for (const KeepOut& k : KEEP_OUT) {
        const float q = s.q[k.joint];
        if (q >= k.angle + KEEP_OUT_DEG) s.qmin[k.joint] = fmaxf(s.qmin[k.joint], k.angle + KEEP_OUT_DEG);
        else if (q <= k.angle - KEEP_OUT_DEG) s.qmax[k.joint] = fminf(s.qmax[k.joint], k.angle - KEEP_OUT_DEG);
    }
}

// Start from joint goals q (°) and speeds qd (°/s; nonzero when another mode hands over a moving arm).
inline void reset(State& s, const float q[N], const float qd[N], const float tcp[3]) {
    memcpy(s.q, q, sizeof(s.q));
    memcpy(s.qd, qd, sizeof(s.qd));
    float J[6][N];
    jacobian(s.q, tcp, J, s.target);
    for (int i = 0; i < 6; i++) { s.v[i] = 0; for (int j = 0; j < N; j++) s.v[i] += J[i][j] * s.qd[j] * D2R; }
    update_bounds(s);
    s.sigma_age = SIGMA_EVERY;   // compute it in the first step
}

// The fastest speed (°/s) that can stop within `room` (°) at acceleration a, in steps of dt (c = a·dt):
// from speed v a joint moves at most dt·(v + c/2)²/(2c), and one step at this speed leaves room for v − c.
inline float brake_speed(float room, float a, float dt) { return fmaxf(0.0f, sqrtf(2 * a * fmaxf(0.0f, room)) - a * dt / 2); }

// One step of dt seconds toward the wanted twist (mm/s, rad/s): in the base frame, or with tool = true in the
// axes of the TCP. vmax: the joint speed limit (°/s). The new joint goals are in s.q.
inline Info step(State& s, const float want_in[6], bool tool, const float tcp[3], float vmax, float dt) {
    update_bounds(s);
    float J[6][N];
    Pose pose;
    jacobian(s.q, tcp, J, pose);

    // 1. The wanted twist in the base frame, then the TCP acceleration limits (linear and angular separately).
    float want[6];
    if (tool) { mulv(pose.R, want_in, want); mulv(pose.R, want_in + 3, want + 3); }
    else memcpy(want, want_in, sizeof(want));
    float v[6], dv[6];
    for (int i = 0; i < 6; i++) dv[i] = want[i] - s.v[i];
    const float nl = norm3(dv), na = norm3(dv + 3);
    const float kl = nl > A_LIN * dt ? A_LIN * dt / nl : 1, ka = na > A_ANG * dt ? A_ANG * dt / na : 1;
    for (int i = 0; i < 6; i++) v[i] = s.v[i] + dv[i] * (i < 3 ? kl : ka);

    // 2. The error from the goals to the target pose. A large error (a limit, a singularity) re-anchors.
    float eP[3], eR[3], Rt[9], Re[9];
    for (int i = 0; i < 3; i++) eP[i] = s.target.p[i] - pose.p[i];
    transpose(pose.R, Rt);
    mul3(s.target.R, Rt, Re);
    log_so3(Re, eR);
    if (norm3(eP) > ANCHOR_MM || norm3(eR) > ANCHOR_RAD) { s.target = pose; memset(eP, 0, sizeof(eP)); memset(eR, 0, sizeof(eR)); }

    // 3. Damped least squares in scaled units: q̇ = Jᵀ (J Jᵀ + λ² I)⁻¹ ξ.
    float Js[6][N], xs[6], A[6][6];
    for (int i = 0; i < 6; i++) {
        const float k = i < 3 ? 1 / L_REF : 1;
        for (int j = 0; j < N; j++) Js[i][j] = J[i][j] * k;
        xs[i] = (v[i] + K_POSE * (i < 3 ? eP[i] : eR[i - 3])) * k;
    }
    for (int a = 0; a < 6; a++)
        for (int b = 0; b < 6; b++) { float t = 0; for (int j = 0; j < N; j++) t += Js[a][j] * Js[b][j]; A[a][b] = t; }
    if (s.sigma_age >= SIGMA_EVERY) { s.sigma = sqrtf(fmaxf(0.0f, min_eig6(A))); s.sigma_age = 0; }
    s.sigma_age++;
    const float sigma = s.sigma;
    const float lambda2 = sigma < SIGMA_DAMP ? LAMBDA_MAX * LAMBDA_MAX * (1 - (sigma / SIGMA_DAMP) * (sigma / SIGMA_DAMP)) : 0;
    for (int i = 0; i < 6; i++) A[i][i] += lambda2;
    float y[6], rhs[6];
    memcpy(rhs, xs, sizeof(rhs));
    solve6(A, rhs, y);
    float qw[N];   // rad/s
    for (int j = 0; j < N; j++) { qw[j] = 0; for (int i = 0; i < 6; i++) qw[j] += Js[i][j] * y[i]; }
    // Near a singular pose the damped speeds may give another twist than the one wanted: brake then.
    float err2 = 0, x2 = 0;
    for (int i = 0; i < 6; i++) {
        float g = 0;
        for (int j = 0; j < N; j++) g += Js[i][j] * qw[j];
        err2 += (g - xs[i]) * (g - xs[i]);
        x2 += xs[i] * xs[i];
    }
    const bool blocked = x2 > 1e-6f && err2 > FIDELITY * FIDELITY * x2;
    float qdw[N];
    for (int j = 0; j < N; j++) qdw[j] = blocked ? 0 : qw[j] / D2R;

    // 4. One scale for all joints: speed limit, braking before the limits, acceleration limits.
    float smax = 1;
    int limit_joint = -1;
    for (int j = 0; j < N; j++) {
        const float a = fabsf(qdw[j]);
        if (a < 1e-6f) continue;
        smax = fminf(smax, vmax / a);
        const float room = qdw[j] > 0 ? s.qmax[j] - s.q[j] : s.q[j] - s.qmin[j];
        const float vb = brake_speed(room, AMAX_SHARE * lim::AMAX_DPS2[j], dt);
        if (a * smax > vb) { smax = vb / a; limit_joint = j; }
    }
    float lo = 0, hi = smax;
    for (int j = 0; j < N; j++) {
        const float a = qdw[j], b = s.qd[j], c = AMAX_SHARE * lim::AMAX_DPS2[j] * dt;   // |s·a − b| ≤ c
        if (fabsf(a) < 1e-6f) { if (fabsf(b) > c) hi = -1; continue; }
        float s1 = (b - c) / a, s2 = (b + c) / a;
        if (s1 > s2) { float t = s1; s1 = s2; s2 = t; }
        lo = fmaxf(lo, s1);
        hi = fminf(hi, s2);
    }
    const bool along = lo <= hi;
    const float sc = along ? hi : 0;
    float qd[N];
    if (along) {
        for (int j = 0; j < N; j++) qd[j] = qdw[j] * sc;
    } else {
        // No common scale (a sudden change near a singular pose, or blocked): all joints brake by one
        // factor, so the TCP slows along its present path. Each joint also stays on its braking curve.
        float k = 1;
        for (int j = 0; j < N; j++) if (fabsf(s.qd[j]) > 1e-9f) k = fminf(k, AMAX_SHARE * lim::AMAX_DPS2[j] * dt / fabsf(s.qd[j]));
        for (int j = 0; j < N; j++) {
            const float b = s.qd[j];
            const float room = b > 0 ? s.qmax[j] - s.q[j] : s.q[j] - s.qmin[j];
            qd[j] = copysignf(fminf(fabsf(b) * (1 - k), brake_speed(room, AMAX_SHARE * lim::AMAX_DPS2[j], dt)), b);
        }
    }

    // 5. Integrate. The target moves with the scaled twist, so it does not run away from a limit.
    for (int j = 0; j < N; j++) {
        const float q1 = s.q[j] + qd[j] * dt;
        // A joint outside its bounds (moved there by hand) only moves back in, without a jump.
        const float qn = s.q[j] > s.qmax[j] ? fminf(s.q[j], q1) : s.q[j] < s.qmin[j] ? fmaxf(s.q[j], q1)
                                            : fminf(s.qmax[j], fmaxf(s.qmin[j], q1));
        qd[j] = (qn - s.q[j]) / dt;
        s.q[j] = qn;
    }
    memcpy(s.qd, qd, sizeof(qd));
    if (along) {
        for (int i = 0; i < 6; i++) s.v[i] = v[i] * sc;
        for (int i = 0; i < 3; i++) s.target.p[i] += s.v[i] * dt;
        float w[3] = {s.v[3] * dt, s.v[4] * dt, s.v[5] * dt}, Rw[9];
        exp_so3(w, Rw);
        mul3(Rw, s.target.R, s.target.R);
    } else {
        // Braking: the twist is the one the joints give, and the target follows the goals.
        for (int i = 0; i < 6; i++) { s.v[i] = 0; for (int j = 0; j < N; j++) s.v[i] += J[i][j] * qd[j] * D2R; }
        float axes[N][3], org[N][3];
        fk(s.q, tcp, s.target, axes, org);
    }
    return {blocked ? 0 : sc, limit_joint, sigma, lambda2 > 0, blocked};
}

inline bool stopped(const State& s) {
    for (int j = 0; j < N; j++) if (fabsf(s.qd[j]) > 1e-3f) return false;
    return true;
}

}  // namespace twist
