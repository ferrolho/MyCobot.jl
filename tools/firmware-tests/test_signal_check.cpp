// Property tests for firmware/atom_controller/test_signal.h (the only implementation of the
// onboard test signals). Run by test/test_firmware.jl, or by hand:
//   c++ -std=c++17 -O1 -I firmware/atom_controller tools/firmware-tests/test_signal_check.cpp -o t && ./t
#include <stdio.h>
#include <math.h>
#include "test_signal.h"

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static sig::Params make(int joint, int kind, float amp, float dur, float vmax, float amax) {
    sig::Params p = {};
    p.joint = joint; p.kind = kind; p.amp_deg = amp; p.f0_hz = 0.2f; p.f1_hz = 5;
    p.duration_s = dur; p.vmax_dps = vmax; p.amax_dps2 = amax;
    return p;
}

static void check_profile(const sig::Params& p, const float start[6], const char* name) {
    const float dt = 0.002f, T = sig::total_s(p);
    float q[6], prev[6], prev_v = 0;
    sig::eval(p, start, 0, q);
    for (int k = 0; k < 6; k++) CHECK(fabsf(q[k] - start[k]) < 1e-4f, "%s: starts at the start pose (J%d)", name, k + 1);
    sig::eval(p, start, T, q);
    for (int k = 0; k < 6; k++) CHECK(fabsf(q[k] - start[k]) < 1e-3f, "%s: ends at the start pose (J%d %.4f)", name, k + 1, q[k] - start[k]);
    const int j = p.joint - 1;
    const float t1 = sig::MOVE_S + sig::HOLD_S, t2 = t1 + p.duration_s;
    float vmax = 0, amax = 0, reach = 0;
    sig::eval(p, start, 0, prev);
    for (int i = 1; i * dt <= T; i++) {
        float t = i * dt;
        sig::eval(p, start, t, q);
        for (int k = 0; k < 6; k++) CHECK(fabsf(q[k]) <= sig::LIMIT_DEG[k] + 1e-3f, "%s: J%d inside the limits", name, k + 1);
        float v = (q[j] - prev[j]) / dt;
        if (t > t1 + 2 * dt && t < t2 - dt) {             // signal part only
            vmax = fmaxf(vmax, fabsf(v));
            if (p.kind == sig::CHIRP) amax = fmaxf(amax, fabsf(v - prev_v) / dt);
            reach = fmaxf(reach, fabsf(q[j] - sig::base_deg(p, j)));
        }
        prev_v = v;
        for (int k = 0; k < 6; k++) prev[k] = q[k];
    }
    CHECK(vmax <= p.vmax_dps * 1.05f, "%s: speed %.1f > vmax %.1f", name, vmax, p.vmax_dps);
    if (p.kind == sig::CHIRP) CHECK(amax <= p.amax_dps2 * 1.15f + 20, "%s: acceleration %.0f > amax %.0f", name, amax, p.amax_dps2);
    CHECK(reach <= p.amp_deg + 1e-3f, "%s: amplitude %.2f > amp %.2f", name, reach, p.amp_deg);
    CHECK(reach >= 0.5f * p.amp_deg, "%s: the signal moves (%.2f)", name, reach);
    printf("ok %s: total %.1f s, peak speed %.1f deg/s, peak accel %.0f deg/s2, reach %.2f deg\n", name, T, vmax, amax, reach);
}

int main() {
    const float zero[6] = {0, 0, 0, 0, 0, 0}, off[6] = {0.5f, -0.3f, 0.2f, 0, 0, 0};
    sig::Params chirp = make(1, sig::CHIRP, 10, 24, 120, 300);
    CHECK(sig::validate(chirp) == 0, "chirp is valid");
    check_profile(chirp, zero, "chirp J1");
    sig::Params steps = make(2, sig::STEPS, 5, 24, 60, 400);
    steps.base_cdeg[1] = -3000; steps.base_cdeg[2] = -6000;
    CHECK(sig::validate(steps) == 0 && sig::validate_start(steps, off) == 0, "steps are valid");
    check_profile(steps, off, "steps J2 at a base pose");
    sig::Params wrist = make(5, sig::CHIRP, 10, 24, 150, 1500);
    check_profile(wrist, zero, "chirp J5");

    // Rejections
    sig::Params bad = chirp; bad.joint = 7;          CHECK(sig::validate(bad) == 1, "joint 7 rejected");
    bad = chirp; bad.kind = 9;                       CHECK(sig::validate(bad) == 2, "kind rejected");
    bad = chirp; bad.amp_deg = 120;                  CHECK(sig::validate(bad) == 3, "amplitude rejected");
    bad = chirp; bad.duration_s = 200;               CHECK(sig::validate(bad) == 4, "duration rejected");
    bad = chirp; bad.vmax_dps = 400;                 CHECK(sig::validate(bad) == 5, "speed rejected");
    bad = chirp; bad.amax_dps2 = 5000;               CHECK(sig::validate(bad) == 6, "acceleration rejected");
    bad = chirp; bad.f1_hz = 0.1f;                   CHECK(sig::validate(bad) == 7, "frequencies rejected");
    bad = chirp; bad.base_cdeg[0] = 15000;           CHECK(sig::validate(bad) == 8, "limit rejected");
    const float far[6] = {0, 100, 0, 0, 0, 0};
    CHECK(sig::validate_start(chirp, far) == 10, "start too far from the base rejected");
    printf(failures ? "%d FAILURES\n" : "all checks passed\n", failures);
    return failures ? 1 : 0;
}
