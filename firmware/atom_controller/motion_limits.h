// motion_limits.h — the one table of joint and motion limits for the controller firmware (4.2+).
// Plain C++: also compiled by tools/firmware-tests/. (Not "limits.h": that would shadow <limits.h>.)
#pragma once

namespace lim {
// Model joint limits (°), as the URDF: round values inside the end stops measured by hand.
const float MODEL_LIMIT_DEG[6] = {165, 140, 150, 150, 160, 180};
// Servo acceleration limits (°/s²): factory register 85 (50 or 250) × 100 steps/s², rounded down.
const float AMAX_DPS2[6] = {400, 400, 400, 2000, 2000, 2000};
const float SIGNAL_MARGIN = 10;   // test signals stay this far inside the model limits
const float JOG_MARGIN = 2;       // JOG stops and MOVE_TO goals stay this far inside
const float MOVE_VMAX = 90;       // °/s, MOVE_TO
const float JOG_VMAX = 30;        // °/s, JOG
const float JOG_AMAX = 200;       // °/s², JOG (also the deadman ramp)
const float JOG_DEADMAN_S = 0.2f; // no JOG for this long: ramp the velocity down to zero
}
