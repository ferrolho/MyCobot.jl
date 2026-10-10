// The firmware's end-effector JOG controller (firmware/atom_controller/twist.h) as a C library, so that the
// simulated ATOM (tools/atom_sim.py) runs the same code as the ATOM. atom_sim.py builds it when needed:
//   c++ -std=c++17 -O2 -shared -fPIC -I firmware/atom_controller tools/twist_lib.cpp -o build/twist_lib.so
#include "twist.h"

extern "C" {
int twist_state_size() { return (int)sizeof(twist::State); }

void twist_reset(void* s, const float* q, const float* qd, const float* tcp) {
    twist::reset(*static_cast<twist::State*>(s), q, qd, tcp);
}

// info: scale, limit_joint, sigma_min, singular, blocked
void twist_step(void* s, const float* want, int tool, const float* tcp, float vmax, float dt, float* info) {
    twist::Info i = twist::step(*static_cast<twist::State*>(s), want, tool != 0, tcp, vmax, dt);
    info[0] = i.scale; info[1] = (float)i.limit_joint; info[2] = i.sigma_min; info[3] = i.singular; info[4] = i.blocked;
}

void twist_get(const void* s, float* q, float* qd) {
    const twist::State& t = *static_cast<const twist::State*>(s);
    for (int j = 0; j < twist::N; j++) { q[j] = t.q[j]; qd[j] = t.qd[j]; }
}

int twist_stopped(const void* s) { return twist::stopped(*static_cast<const twist::State*>(s)); }

// The TCP pose for joint angles (°): p (mm) and R (row-major), base frame.
void twist_fk(const float* q, const float* tcp, float* p, float* R) {
    twist::Pose pose; float a[twist::N][3], o[twist::N][3];
    twist::fk(q, tcp, pose, a, o);
    for (int i = 0; i < 3; i++) p[i] = pose.p[i];
    for (int i = 0; i < 9; i++) R[i] = pose.R[i];
}
}
