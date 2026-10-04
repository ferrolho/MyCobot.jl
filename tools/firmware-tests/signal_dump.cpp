// Prints test signals from firmware/atom_controller/test_signal.h for test/test_signals.jl.
//   c++ -std=c++17 -O1 -I firmware/atom_controller tools/firmware-tests/signal_dump.cpp -o signal_dump
//   ./signal_dump  ->  CSV: case,t,q1..q6
#include <stdio.h>
#include "test_signal.h"

int main() {
    sig::Params cases[2] = {};
    cases[0].joint = 1; cases[0].kind = sig::CHIRP; cases[0].amp_deg = 10; cases[0].f0_hz = 0.2f; cases[0].f1_hz = 5;
    cases[0].duration_s = 24; cases[0].vmax_dps = 120; cases[0].amax_dps2 = 300;
    cases[1].joint = 2; cases[1].kind = sig::STEPS; cases[1].amp_deg = 5; cases[1].f0_hz = 0.2f; cases[1].f1_hz = 5;
    cases[1].duration_s = 24; cases[1].vmax_dps = 60; cases[1].amax_dps2 = 400;
    cases[1].base_cdeg[1] = -3000; cases[1].base_cdeg[2] = -6000;
    const float starts[2][6] = {{0, 0, 0, 0, 0, 0}, {0.5f, -0.3f, 0.2f, 0, 0, 0}};
    for (int c = 0; c < 2; c++) {
        printf("# case %d validate %d start %d\n", c, sig::validate(cases[c]), sig::validate_start(cases[c], starts[c]));
        for (int i = 0; i * 0.01f <= sig::total_s(cases[c]); i++) {
            float t = i * 0.01f, q[6];
            sig::eval(cases[c], starts[c], t, q);
            printf("%d,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", c, t, q[0], q[1], q[2], q[3], q[4], q[5]);
        }
    }
    return 0;
}
