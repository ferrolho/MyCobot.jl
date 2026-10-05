// Property tests for firmware/atom_controller/motion.h (MOVE_TO and JOG, firmware 4.2+; TRACK, 4.4+).
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

    // TRACK: to a fixed goal at full speed: speed and acceleration within the limits, no overshoot,
    // exact arrival, and faster than JOG. J1-J3 accelerate at 400, J4-J6 at 2000 deg/s².
    {
        const float dt = 0.002f;
        motion::Jog t = {};
        float g[6] = {120, -60, 90, 140, -100, 170};
        float vp[6] = {0}, ap[6] = {0}, pv[6] = {0}, over = 0, arrive[6];
        for (int j = 0; j < 6; j++) arrive[j] = -1;
        for (int i = 1; i <= 2000; i++) {             // 4 s
            motion::track_step(t, g, 90, false, dt);
            for (int j = 0; j < 6; j++) {
                vp[j] = fmaxf(vp[j], fabsf(t.v[j]));
                ap[j] = fmaxf(ap[j], fabsf(t.v[j] - pv[j]) / dt); pv[j] = t.v[j];
                float gl = fmaxf(-(lim::MODEL_LIMIT_DEG[j] - lim::JOG_MARGIN), fminf(lim::MODEL_LIMIT_DEG[j] - lim::JOG_MARGIN, g[j]));
                over = fmaxf(over, (t.q[j] - gl) * (gl > 0 ? 1 : -1));
                if (arrive[j] < 0 && t.q[j] == gl && t.v[j] == 0) arrive[j] = i * dt;
                CHECK(fabsf(t.q[j]) <= lim::MODEL_LIMIT_DEG[j] - lim::JOG_MARGIN + 1e-4f, "track J%d inside its limit", j + 1);
            }
        }
        for (int j = 0; j < 6; j++) {
            CHECK(vp[j] <= 90 + 1e-3f, "track J%d speed %.1f within 90", j + 1, vp[j]);
            CHECK(ap[j] <= lim::AMAX_DPS2[j] * 1.5f + 1, "track J%d acceleration %.0f within %.0f (+50 %% for the arrival step)", j + 1, ap[j], lim::AMAX_DPS2[j]);
            CHECK(arrive[j] > 0, "track J%d arrives exactly and stops", j + 1);
        }
        CHECK(over <= 0.01f, "track never passes a fixed goal by more than 0.01 deg, far below one servo step of 0.088 deg (%.4f)", over);
        printf("ok track: J1 120 deg in %.2f s (JOG at 30 deg/s: >4 s), J6 170 deg in %.2f s, peak %.1f deg/s\n", arrive[0], arrive[5], vp[0]);

        // Arrival without a creep: the last 0.3 deg take a few steps, not a 1.5 s tail.
        motion::Jog c = {}; float g3[6] = {10, 0, 0, 0, 0, 0}; int steps = 0;
        while (!(c.q[0] == 10 && c.v[0] == 0) && steps < 5000) { motion::track_step(c, g3, 90, false, dt); steps++; }
        CHECK(steps * dt < 0.4f, "10 deg on J1 in %.3f s", steps * dt);

        // A goal that reverses at full speed: the joint brakes, comes back, never passes its limit.
        motion::Jog r = {}; float far[6] = {200, 0, 0, 0, 0, 0}, back[6] = {-200, 0, 0, 0, 0, 0};
        for (int i = 0; i < 1500; i++) motion::track_step(r, far, 90, false, dt);
        for (int i = 0; i < 4000; i++) { motion::track_step(r, back, 90, false, dt); CHECK(fabsf(r.q[0]) <= 163 + 1e-4f, "reversal inside the limit"); }
        CHECK(r.q[0] == -163 && r.v[0] == 0, "reversal ends on the far limit (%.2f)", r.q[0]);

        // Stop (deadman, STOP, HOLD) from 90 deg/s: brakes at AMAX to zero.
        motion::Jog st = {}; float g4[6] = {150, 0, 0, 150, 0, 0};
        for (int i = 0; i < 300; i++) motion::track_step(st, g4, 90, false, dt);
        float v1 = st.v[0], v4 = st.v[3]; int k = 0;
        while (!motion::jog_stopped(st) && k < 2000) { motion::track_step(st, g4, 90, true, dt); k++; }
        CHECK(motion::jog_stopped(st) && fabsf(k * dt - v1 / lim::AMAX_DPS2[0]) < 0.01f, "stop J1 from %.0f deg/s in %.3f s", v1, k * dt);
        printf("ok track stop: J1 from %.0f deg/s and J4 from %.0f deg/s stop in %.3f s; reversal and creep ok\n", v1, v4, k * dt);
    }

    printf(failures ? "%d FAILURES\n" : "all checks passed\n", failures);
    return failures ? 1 : 0;
}
