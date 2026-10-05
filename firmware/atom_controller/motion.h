// motion.h — MOVE_TO and JOG (firmware 4.2+). Plain C++: tested by tools/firmware-tests/.
#pragma once
#include <math.h>
#include "motion_limits.h"

namespace motion {

inline float clampf(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }

// 0 = goal ok, otherwise 1 + the index of the first joint outside its limit.
inline int move_validate(const float goal[6]) {
    for (int j = 0; j < 6; j++)
        if (!(fabsf(goal[j]) <= lim::MODEL_LIMIT_DEG[j] - lim::JOG_MARGIN)) return j + 1;
    return 0;
}

// Shortest minimum-jerk duration (s) that keeps every joint within MOVE_VMAX and AMAX_DPS2:
// peak speed 1.875·d/T, peak acceleration 5.77·d/T².
inline float move_min_duration(const float start[6], const float goal[6]) {
    float T = 0.2f;
    for (int j = 0; j < 6; j++) {
        float d = fabsf(goal[j] - start[j]);
        T = fmaxf(T, fmaxf(1.875f * d / lim::MOVE_VMAX, sqrtf(5.77f * d / lim::AMAX_DPS2[j])));
    }
    return T;
}

inline float minjerk(float x) {
    x = clampf(x, 0, 1);
    return x * x * x * (10 - 15 * x + 6 * x * x);
}

struct Jog { float q[6]; float v[6]; };

// One JOG step of dt seconds: each joint's velocity goes toward its target (clamped to JOG_VMAX)
// at JOG_AMAX, brakes in time to stop JOG_MARGIN inside its limit, and the position integrates.
inline void jog_step(Jog& s, const float target[6], float dt) {
    const float a = lim::JOG_AMAX, dv_max = a * dt;
    // The fastest speed that can still stop within distance d, braking at a in steps of dt
    // (discrete form of sqrt(2·a·d); the continuous form leaves a speed step at the limit).
    auto stoppable = [&](float d) { return d <= 0 ? 0.0f : dv_max * (sqrtf(0.25f + 2 * d / (a * dt * dt)) - 0.5f); };
    for (int j = 0; j < 6; j++) {
        const float L = lim::MODEL_LIMIT_DEG[j] - lim::JOG_MARGIN;
        float t = clampf(target[j], -lim::JOG_VMAX, lim::JOG_VMAX);
        float up = stoppable(L - s.q[j]);
        float dn = stoppable(L + s.q[j]);
        t = clampf(t, -dn, up);
        s.v[j] += clampf(t - s.v[j], -dv_max, dv_max);
        float qn = s.q[j] + s.v[j] * dt;
        // Land exactly on the limit (float rounding can leave a small speed there).
        if (qn > L) { s.v[j] = (L - s.q[j]) / dt; qn = L; }
        else if (qn < -L) { s.v[j] = (-L - s.q[j]) / dt; qn = -L; }
        s.q[j] = qn;
    }
}

// One TRACK step (4.4+, the Control page's Live mode): each joint goes toward its goal (clamped
// JOG_MARGIN inside its limit) at up to vmax (≤ MOVE_VMAX) and its own AMAX_DPS2, and brakes to stop
// exactly on it. A goal that jumps closer than the braking distance makes the joint overshoot and
// come back, but never past its limit. With stop (deadman, STOP, HOLD): brake to zero at AMAX_DPS2.
inline void track_step(Jog& s, const float goal[6], float vmax, bool stop, float dt) {
    vmax = clampf(vmax, 0, lim::MOVE_VMAX);
    for (int j = 0; j < 6; j++) {
        const float a = lim::AMAX_DPS2[j], dv_max = a * dt;
        auto stoppable = [&](float d) { return d <= 0 ? 0.0f : dv_max * (sqrtf(0.25f + 2 * d / (a * dt * dt)) - 0.5f); };
        const float L = lim::MODEL_LIMIT_DEG[j] - lim::JOG_MARGIN;
        const float g = clampf(goal[j], -L, L), e = g - s.q[j];
        float t = stop ? 0.0f : copysignf(fminf(vmax, stoppable(fabsf(e))), e);
        t = clampf(t, -stoppable(L + s.q[j]), stoppable(L - s.q[j]));
        const float v0 = s.v[j];
        s.v[j] += clampf(t - s.v[j], -dv_max, dv_max);
        float qn = s.q[j] + s.v[j] * dt;
        // Arrive: the step reaches the goal from a speed that one step can stop. Land on it exactly.
        if (!stop && e != 0 && (g - qn) * e <= 0 && fabsf(v0) <= 1.5f * dv_max) { qn = g; s.v[j] = 0; }
        if (qn > L) { s.v[j] = (L - s.q[j]) / dt; qn = L; }
        else if (qn < -L) { s.v[j] = (-L - s.q[j]) / dt; qn = -L; }
        s.q[j] = qn;
    }
}

inline bool jog_stopped(const Jog& s) {
    for (int j = 0; j < 6; j++) if (fabsf(s.v[j]) > 1e-3f) return false;
    return true;
}

}  // namespace motion
