// motion_limits.h — the one table of joint and motion limits for the controller firmware (4.2+).
// Plain C++: also compiled by tools/firmware-tests/. (Not "limits.h": that would shadow <limits.h>.)
#pragma once
#include "robot_params.h"   // generated from mycobot_description/config (tools/gen_robot.py)

namespace lim {
// Model joint limits (°), speed limit (°/s) and acceleration limits (°/s²): joint_limits.yaml.
using robot::AMAX_DPS2;
constexpr const float (&MODEL_MIN_DEG)[robot::N_JOINTS] = robot::LIMIT_MIN_DEG;
constexpr const float (&MODEL_MAX_DEG)[robot::N_JOINTS] = robot::LIMIT_MAX_DEG;
const float SIGNAL_MARGIN = 10;   // test signals stay this far inside the model limits
const float JOG_MARGIN = 2;       // JOG stops and MOVE_TO goals stay this far inside
const float MOVE_VMAX = robot::VMAX_DPS;   // °/s, MOVE_TO and TRACK
const float JOG_VMAX = 30;        // °/s, JOG
const float JOG_AMAX = 200;       // °/s², JOG (also the deadman ramp)
const float JOG_DEADMAN_S = 0.2f; // no JOG for this long: ramp the velocity down to zero
}
