// Property tests for firmware/atom_controller/motion.h (MOVE_TO and JOG, firmware 4.2+).
//   c++ -std=c++17 -O1 -I firmware/atom_controller tools/firmware-tests/test_motion_check.cpp -o t && ./t
#include <stdio.h>
#include <math.h>
#include "motion.h"

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

int main() {
    // MOVE_TO: validation and duration
    float ok_goal[6] = {100, -30, 60, 0, 0, 0}, bad_goal[6] = {0, 0, 149, 0, 0, 0};
    CHECK(motion::move_validate(ok_goal) == 0, "goal inside the limits");
    CHECK(motion::move_validate(bad_goal) == 3, "J3 at 149° is beyond 150 − 2");
    float start[6] = {0, 0, 0, 0, 0, 0}, goal[6] = {90, -40, 120, 5, 60, 170};
    float T = motion::move_min_duration(start, goal);
    float vmax = 0, amax_ratio = 0, prev[6] = {0}, prev_v[6] = {0};
    const float dt = 0.001f;
    for (int i = 1; i * dt <= T; i++) {
        float s = motion::minjerk(i * dt / T);
        for (int j = 0; j < 6; j++) {
            float q = start[j] + s * (goal[j] - start[j]), v = (q - prev[j]) / dt;
            vmax = fmaxf(vmax, fabsf(v));
            if (i > 1) amax_ratio = fmaxf(amax_ratio, fabsf(v - prev_v[j]) / dt / lim::AMAX_DPS2[j]);
            prev[j] = q; prev_v[j] = v;
        }
    }
    CHECK(vmax <= lim::MOVE_VMAX * 1.01f, "move speed %.1f within %.0f", vmax, lim::MOVE_VMAX);
    CHECK(amax_ratio <= 1.01f, "move acceleration within the limits (ratio %.2f)", amax_ratio);
    printf("ok move: T %.2f s, peak speed %.1f deg/s, peak accel %.0f %% of the limit\n", T, vmax, 100 * amax_ratio);

    // JOG: ramp, speed limit, braking at the limit, deadman ramp-down
    motion::Jog s = {};
    float target[6] = {100, 0, 0, 0, 0, -30};      // J1 asks for more than JOG_VMAX
    float t_full = -1, vpeak = 0, prev_v1 = 0, apeak = 0;
    for (int i = 1; i <= 20000; i++) {             // 40 s at 500 Hz: J1 reaches its limit
        motion::jog_step(s, target, 0.002f);
        vpeak = fmaxf(vpeak, fabsf(s.v[0]));
        apeak = fmaxf(apeak, fabsf(s.v[0] - prev_v1) / 0.002f); prev_v1 = s.v[0];
        if (t_full < 0 && s.v[0] >= lim::JOG_VMAX - 1e-3f) t_full = i * 0.002f;
        CHECK(fabsf(s.q[0]) <= lim::MODEL_LIMIT_DEG[0] - lim::JOG_MARGIN + 1e-4f, "J1 never passes its jog limit");
        CHECK(fabsf(s.q[5]) <= lim::MODEL_LIMIT_DEG[5] - lim::JOG_MARGIN + 1e-4f, "J6 never passes its jog limit");
    }
    CHECK(vpeak <= lim::JOG_VMAX + 1e-3f, "jog speed %.2f within %.0f", vpeak, lim::JOG_VMAX);
    CHECK(apeak <= lim::JOG_AMAX * 1.15f, "jog acceleration %.1f within %.0f (+15 %% for the last step at the limit)", apeak, lim::JOG_AMAX);
    CHECK(fabsf(t_full - lim::JOG_VMAX / lim::JOG_AMAX) < 0.01f, "reaches 30 deg/s after 0.15 s (%.3f)", t_full);
    CHECK(fabsf(s.q[0] - (lim::MODEL_LIMIT_DEG[0] - lim::JOG_MARGIN)) < 0.05f && fabsf(s.v[0]) < 1e-3f, "J1 stops at its limit (%.2f, v %.3f)", s.q[0], s.v[0]);
    printf("ok jog: full speed after %.3f s, peak %.1f deg/s, %.0f deg/s2, J1 stops at %.2f deg, J6 at %.2f deg\n", t_full, vpeak, apeak, s.q[0], s.q[5]);

    // Deadman: from full speed, a zero target ramps down at JOG_AMAX and stops
    motion::Jog d = {}; float go[6] = {30, 30, 30, 30, 30, 30}, zero[6] = {0};
    for (int i = 0; i < 200; i++) motion::jog_step(d, go, 0.002f);
    int n = 0;
    while (!motion::jog_stopped(d) && n < 1000) { motion::jog_step(d, zero, 0.002f); n++; }
    CHECK(motion::jog_stopped(d) && fabsf(n * 0.002f - lim::JOG_VMAX / lim::JOG_AMAX) < 0.01f, "deadman stop in 0.15 s (%.3f)", n * 0.002f);
    printf("ok deadman: stops in %.3f s\n", n * 0.002f);

    printf(failures ? "%d FAILURES\n" : "all checks passed\n", failures);
    return failures ? 1 : 0;
}
