// test_signal.h — test signals computed onboard (PLAY_SIGNAL, firmware 3.1+), for system
// identification without uploading a plan. Plain C++ (no Arduino code), so that
// tools/firmware-tests/ can compile it on the laptop and compare it with MyCobot.test_signal.
//
// Timeline (seconds):  move start→base (MOVE_S, minimum jerk) | hold (HOLD_S) | signal on one
// joint (duration) | hold (HOLD_S) | move base→start (MOVE_S). Angles in degrees.
//
// chirp: logarithmic sweep f0→f1 over duration−2 s (1 s rest at each end), amplitude
//        ≈ min(amp, vmax/(2πf), amax/(2πf)²) (smooth minimum), with a smooth fade in/out of
//        max(0.5 s, 3.75·amp/vmax).
// steps: levels 0, +amp, 0, −amp, 0, +amp, −amp, 0, each duration/8, ramps at vmax.
#pragma once
#include <math.h>
#include <stdint.h>

namespace sig {

const float MOVE_S = 2.0f, HOLD_S = 1.0f;
const float LIMIT_DEG[6] = {155, 130, 140, 140, 150, 170};   // model limits − 10° (MyCobot URDF)
const float AMAX_DPS2[6] = {400, 400, 400, 2000, 2000, 2000}; // servo acceleration limits (≈ reg 85)
const float MAX_MOVE_DEG = 90;                                  // start→base distance per joint

enum Kind : uint8_t { CHIRP = 1, STEPS = 2 };

struct __attribute__((packed)) Params {
    uint8_t joint;            // 1..6
    uint8_t kind;             // Kind
    float amp_deg;            // chirp amplitude / step size
    float f0_hz, f1_hz;       // chirp frequencies
    float duration_s;         // signal part only
    float vmax_dps;           // speed limit of the signal
    float amax_dps2;          // acceleration limit of the chirp
    int16_t base_cdeg[6];     // base pose, 0.01°
};

inline float base_deg(const Params& p, int j) { return p.base_cdeg[j] * 0.01f; }

inline float total_s(const Params& p) { return 2 * MOVE_S + 2 * HOLD_S + p.duration_s; }

// Checks that do not need the start pose. 0 = ok, otherwise an error code.
inline int validate(const Params& p) {
    if (p.joint < 1 || p.joint > 6) return 1;
    if (p.kind != CHIRP && p.kind != STEPS) return 2;
    if (!(p.amp_deg > 0 && p.amp_deg <= 90)) return 3;
    if (!(p.duration_s >= 4 && p.duration_s <= 120)) return 4;
    if (!(p.vmax_dps > 0 && p.vmax_dps <= 150)) return 5;
    int j = p.joint - 1;
    if (!(p.amax_dps2 > 0 && p.amax_dps2 <= AMAX_DPS2[j])) return 6;
    if (p.kind == CHIRP && !(p.f0_hz > 0 && p.f1_hz > p.f0_hz && p.f1_hz <= 20)) return 7;
    for (int k = 0; k < 6; k++) {
        float reach = fabsf(base_deg(p, k)) + (k == j ? p.amp_deg : 0);
        if (reach > LIMIT_DEG[k]) return 8;
    }
    return 0;
}

// Check the start pose (degrees): each joint within MAX_MOVE_DEG of the base and inside the limits.
inline int validate_start(const Params& p, const float start[6]) {
    for (int k = 0; k < 6; k++) {
        if (fabsf(start[k]) > LIMIT_DEG[k]) return 9;
        if (fabsf(start[k] - base_deg(p, k)) > MAX_MOVE_DEG) return 10;
    }
    return 0;
}

inline float minjerk(float x) {
    x = x < 0 ? 0 : (x > 1 ? 1 : x);
    return x * x * x * (10 - 15 * x + 6 * x * x);
}

// The signal on its joint, t in [0, duration].
inline float signal(const Params& p, float t) {
    const float T = p.duration_s;
    if (p.kind == CHIRP) {
        const float Tc = T - 2.0f, tau = t - 1.0f;
        if (tau < 0 || tau > Tc) return 0;
        const float k = logf(p.f1_hz / p.f0_hz) / Tc;
        const float e = expf(k * tau);
        const float f = p.f0_hz * e;
        const float phase = 2 * (float)M_PI * p.f0_hz * (e - 1) / k;
        const float w2 = 2 * (float)M_PI * f;
        // Smooth minimum of the three amplitude limits ((Σ x⁻⁸)^(−1/8) ≤ min, at most 8 % lower
        // where two limits cross). A hard min() has corners that step the speed (~1.6°/s on J1).
        const float r1 = p.amp_deg, r2 = p.vmax_dps / w2, r3 = p.amax_dps2 / (w2 * w2);
        float a = powf(powf(r1, -8.0f) + powf(r2, -8.0f) + powf(r3, -8.0f), -0.125f);
        // Smooth fade in/out (minimum jerk), long enough that the fade alone uses at most half
        // of vmax: amp · 1.875 / tf ≤ vmax / 2. (A linear fade made speed steps of ~2·a·sin φ.)
        const float tf = fmaxf(0.5f, 3.75f * p.amp_deg / p.vmax_dps);
        float fade = fminf(minjerk(tau / tf), minjerk((Tc - tau) / tf));
        return fade * a * sinf(phase);
    }
    // steps
    const float levels[8] = {0, p.amp_deg, 0, -p.amp_deg, 0, p.amp_deg, -p.amp_deg, 0};
    const float hold = T / 8, ramp = 2 * p.amp_deg / p.vmax_dps;
    int k = (int)floorf(t / hold);
    if (k < 0) k = 0;
    if (k > 7) k = 7;
    float prev = k == 0 ? 0 : levels[k - 1];
    float s = (t - k * hold) / ramp;
    s = s < 0 ? 0 : (s > 1 ? 1 : s);
    return prev + s * (levels[k] - prev);
}

// Reference pose at time t (degrees) for all joints, starting from `start`.
inline void eval(const Params& p, const float start[6], float t, float out[6]) {
    const float t1 = MOVE_S + HOLD_S, t2 = t1 + p.duration_s, t3 = t2 + HOLD_S;
    for (int k = 0; k < 6; k++) {
        float b = base_deg(p, k);
        if (t < MOVE_S) out[k] = start[k] + minjerk(t / MOVE_S) * (b - start[k]);
        else if (t < t3) out[k] = b;
        else out[k] = b + minjerk((t - t3) / MOVE_S) * (start[k] - b);
    }
    if (t >= t1 && t < t2) out[p.joint - 1] += signal(p, t - t1);
}

}  // namespace sig
