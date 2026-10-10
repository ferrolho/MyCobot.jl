// Tests for firmware/atom_controller/twist.h (end-effector JOG, firmware 5.1), at the firmware's 500 Hz.
//   c++ -std=c++17 -O2 -I firmware/atom_controller tools/firmware-tests/test_twist_check.cpp -o t && ./t
// The FK reference (fk_reference.h) comes from src/kinematics.jl: tools/firmware-tests/fk_reference.jl.
#include <stdio.h>
#include <math.h>
#include <chrono>
#include <vector>
#include "twist.h"
#include "fk_reference.h"

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); } else printf("ok   "); printf(__VA_ARGS__); printf("\n"); } while (0)

using twist::N;
const float DT = 0.002f;
const float TCP[3] = {robot::GRIPPER_TCP_MM[0], robot::GRIPPER_TCP_MM[1], robot::GRIPPER_TCP_MM[2]};
const float R2D = 57.29577951f;

struct Sample { float q[N], qd[N], qd0[N]; twist::Pose pose; twist::Info info; };
struct Seg { float v[6]; float seconds; bool tool; };

static twist::Pose tcp_pose(const float q[N]) {
    twist::Pose p; float a[N][3], o[N][3];
    twist::fk(q, TCP, p, a, o);
    return p;
}
static float rot_deg(const float A[9], const float B[9]) {   // angle between two rotations
    float Bt[9], E[9], w[3];
    twist::transpose(B, Bt); twist::mul3(A, Bt, E); twist::log_so3(E, w);
    return twist::norm3(w) * R2D;
}

static std::vector<Sample> run(const float q0[N], const std::vector<Seg>& segs, float vmax) {
    twist::State s;
    float zero[N] = {0};
    twist::reset(s, q0, zero, TCP);
    std::vector<Sample> log;
    for (const Seg& g : segs)
        for (int k = 0; k < (int)lroundf(g.seconds / DT); k++) {
            Sample x;
            memcpy(x.qd0, s.qd, sizeof(x.qd0));
            x.info = twist::step(s, g.v, g.tool, TCP, vmax, DT);
            memcpy(x.q, s.q, sizeof(x.q)); memcpy(x.qd, s.qd, sizeof(x.qd));
            x.pose = tcp_pose(s.q);
            log.push_back(x);
        }
    return log;
}

// Limits: speed ≤ vmax, acceleration ≤ 0.8 × AMAX (+ 0.02 °/s per step for float rounding), inside the JOG range, finite.
static bool in_limits(const std::vector<Sample>& log, float vmax, const char** why) {
    for (const Sample& x : log)
        for (int j = 0; j < N; j++) {
            if (!isfinite(x.q[j])) { *why = "not finite"; return false; }
            if (fabsf(x.qd[j]) > vmax * 1.001f + 1e-3f) { *why = "speed"; return false; }
            if (fabsf(x.qd[j] - x.qd0[j]) > twist::AMAX_SHARE * lim::AMAX_DPS2[j] * DT + 0.02f)   // float: q/dt to about 0.01 °/s { *why = "acceleration"; return false; }
            if (x.q[j] < lim::MODEL_MIN_DEG[j] + lim::margin(j) - 1e-3f || x.q[j] > lim::MODEL_MAX_DEG[j] - lim::margin(j) + 1e-3f) { *why = "joint limit"; return false; }
        }
    *why = "";
    return true;
}

int main() {
    // --- Kinematics ---
    float worst_p = 0, worst_r = 0;
    for (int i = 0; i < FK_REF_N; i++) {
        twist::Pose p; float a[N][3], o[N][3]; const float zero[3] = {0, 0, 0};
        twist::fk(FK_REF_Q[i], zero, p, a, o);
        for (int k = 0; k < 3; k++) worst_p = fmaxf(worst_p, fabsf(p.p[k] - FK_REF_P[i][k]));
        for (int k = 0; k < 9; k++) worst_r = fmaxf(worst_r, fabsf(p.R[k] - FK_REF_R[i][k]));
    }
    CHECK(worst_p < 0.01f && worst_r < 1e-5f, "FK = src/kinematics.jl on %d poses: %.1e mm, %.1e", FK_REF_N, worst_p, worst_r);
    {
        const float q[N] = {10, -20, 30, -40, 50, -60};
        float J[6][N]; twist::Pose p0;
        twist::jacobian(q, TCP, J, p0);
        float worst = 0;
        for (int j = 0; j < N; j++) {
            float q2[N]; memcpy(q2, q, sizeof(q2)); const float h = 1e-2f; q2[j] += h * R2D;
            twist::Pose p1 = tcp_pose(q2);
            float Rt[9], E[9], w[3]; twist::transpose(p0.R, Rt); twist::mul3(p1.R, Rt, E); twist::log_so3(E, w);
            for (int k = 0; k < 3; k++) worst = fmaxf(worst, fmaxf(fabsf((p1.p[k] - p0.p[k]) / h - J[k][j]) / 100, fabsf(w[k] / h - J[k + 3][j])));
        }
        CHECK(worst < 0.02f, "Jacobian = finite differences (%.1e)", worst);
    }

    const char* why;
    const float DOWN[N] = {0, 0, -90, 0, 0, 0};   // the ready pose: the tool points down
    const twist::Pose P0 = tcp_pose(DOWN);
    {
        auto log = run(DOWN, {{{30, 0, 0, 0, 0, 0}, 2, false}, {{0, 0, 0, 0, 0, 0}, 1, false}}, 36);
        float off = 0, rot = 0;
        for (auto& x : log) { off = fmaxf(off, hypotf(x.pose.p[1] - P0.p[1], x.pose.p[2] - P0.p[2])); rot = fmaxf(rot, rot_deg(x.pose.R, P0.R)); }
        const float dx = log.back().pose.p[0] - P0.p[0];
        CHECK(off < 0.2f, "+x 30 mm/s for 2 s: %.3f mm off the line", off);
        CHECK(rot < 0.05f, "+x: the tool turns %.4f°", rot);
        CHECK(fabsf(dx - 60) < 1, "+x: %.2f mm (60 wanted)", dx);
        CHECK(in_limits(log, 36, &why), "+x: inside the limits %s", why);
        float rest = 0; for (int j = 0; j < N; j++) rest = fmaxf(rest, fabsf(log.back().qd[j]));
        CHECK(rest < 0.05f, "+x: at rest at the end (%.3f °/s)", rest);
    }
    {
        auto log = run(DOWN, {{{0, 0, 0, 0, 0, 0.5f}, 2, false}, {{0, 0, 0, 0, 0, 0}, 1, false}}, 36);
        float off = 0; for (auto& x : log) for (int k = 0; k < 3; k++) off = fmaxf(off, fabsf(x.pose.p[k] - P0.p[k]));
        CHECK(off < 0.2f, "turn 0.5 rad/s about the TCP: the TCP moves %.3f mm", off);
        CHECK(fabsf(rot_deg(log.back().pose.R, P0.R) - 57.3f) < 2, "turn: %.1f° (57.3 wanted)", rot_deg(log.back().pose.R, P0.R));
    }
    {
        auto log = run(DOWN, {{{0, 0, 30, 0, 0, 0}, 1, true}, {{0, 0, 0, 0, 0, 0}, 0.5f, true}}, 36);
        const float dz = log.back().pose.p[2] - P0.p[2];
        CHECK(fabsf(dz + 30) < 1, "tool frame +z (the approach) with the tool down: %.2f mm in z (−30 wanted)", dz);
    }
    {
        auto log = run(DOWN, {{{0, 0, -60, 0, 0, 0}, 8, false}}, 36);
        CHECK(in_limits(log, 36, &why), "down for 8 s: inside the limits %s", why);
    }
    {
        auto log = run(DOWN, {{{100, 0, 0, 0, 0, 0}, 6, false}, {{-100, 0, 0, 0, 0, 0}, 2, false}}, 36);
        float j3 = -999; for (auto& x : log) j3 = fmaxf(j3, x.q[2]);
        CHECK(in_limits(log, 36, &why), "out to the reach limit: inside the limits %s", why);
        CHECK(j3 <= -10 + 1e-3f, "out: J3 stops 10° before the arm is straight (at most %.2f°)", j3);
        CHECK(log.back().pose.p[0] < log[log.size() - 1001].pose.p[0] - 50, "out and back: it comes back");
    }
    {
        const float Q5[N] = {0, 0, -90, 0, 70, 0};
        auto log = run(Q5, {{{0, 0, 0, 0, 0.6f, 0}, 3, true}, {{0, 0, 0, 0, -0.6f, 0}, 3, true}, {{0, 0, 0, 0.6f, 0, 0}, 3, false}}, 36);
        float j5 = 0; for (auto& x : log) j5 = fmaxf(j5, fabsf(x.q[4]));
        CHECK(in_limits(log, 36, &why), "rotations near J5 = 90°: inside the limits %s", why);
        CHECK(j5 <= 80 + 1e-3f, "J5 stops 10° before ±90° (|J5| at most %.2f°)", j5);
    }
    {
        auto log = run(DOWN, {{{100, 0, 0, 0, 0, 1}, 1, false}, {{-100, 0, 0, 0, 0, -1}, 1, false}}, 90);
        CHECK(in_limits(log, 90, &why), "reversal at 100 mm/s and 1 rad/s: inside the limits %s", why);
    }
    {
        const float Z[N] = {0, 0, 0, 0, 0, 0};
        const twist::Pose PZ = tcp_pose(Z);
        auto ahead = run(Z, {{{25, 0, 0, 0, 0, 0}, 2, false}, {{0, 0, 0, 0, 0, 0}, 1, false}}, 36);
        auto down = run(Z, {{{0, 0, -25, 0, 0, 0}, 2, false}}, 36);
        float moved = 0; for (int k = 0; k < 3; k++) moved = fmaxf(moved, fabsf(down.back().pose.p[k] - PZ.p[k]));
        CHECK(ahead.back().pose.p[0] - PZ.p[0] > 35, "zero pose (singular), ahead: %.1f mm in 2 s", ahead.back().pose.p[0] - PZ.p[0]);
        CHECK(down.back().info.blocked && moved < 1, "zero pose, down: blocked, the TCP moves %.2f mm", moved);
    }
    unsigned seed = 1;
    auto rnd = [&]() { seed = seed * 1103515245u + 12345u; return ((seed >> 8) & 0xFFFF) / 32767.5f - 1; };
    {
        std::vector<Seg> segs;
        for (int i = 0; i < 600; i++) segs.push_back({{100 * rnd(), 100 * rnd(), 100 * rnd(), rnd(), rnd(), rnd()}, 1, rnd() > 0.5f});
        auto t0 = std::chrono::steady_clock::now();
        auto log = run(DOWN, segs, 60);
        const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / log.size();
        CHECK(in_limits(log, 60, &why), "10 min of random twists (base and tool frame): inside the limits %s", why);
        printf("     %.1f µs per step on this computer (with the test's own FK)\n", us);
    }
    {
        std::vector<Seg> segs;
        for (int i = 0; i < 300; i++) segs.push_back({{60 * rnd(), 60 * rnd(), 60 * rnd(), 0, 0, 0}, 1, false});
        auto log = run(DOWN, segs, 60);
        float rot = 0; for (auto& x : log) rot = fmaxf(rot, rot_deg(x.pose.R, P0.R));
        CHECK(rot < 6, "5 min of random translations, often at the limits: the tool turns %.2f° at most", rot);
    }
    {
        // A hand-over from another mode with the arm moving: the first steps stay inside the acceleration limits.
        twist::State s;
        const float qd[N] = {20, -15, 10, 30, 0, -20};
        twist::reset(s, DOWN, qd, TCP);
        float worst = 0, prev[N]; memcpy(prev, qd, sizeof(prev));
        const float zero[6] = {0};
        for (int k = 0; k < 500; k++) {
            twist::step(s, zero, false, TCP, 60, DT);
            for (int j = 0; j < N; j++) worst = fmaxf(worst, fabsf(s.qd[j] - prev[j]) / (twist::AMAX_SHARE * lim::AMAX_DPS2[j] * DT));
            memcpy(prev, s.qd, sizeof(prev));
        }
        CHECK(worst <= 1.001f && twist::stopped(s), "hand-over at speed, then zero twist: it brakes to rest (peak %.2f of the limit)", worst);
    }
    printf(failures ? "%d failed\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
