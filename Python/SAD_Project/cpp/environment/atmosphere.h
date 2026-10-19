#pragma once
/* Skeleton module: atmosphere (environment). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct ATMOSPHEREParams {
    std::vector<double> params;
};
struct ATMOSPHEREState {
    ATMOSPHEREParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct ATMOSPHEREInput {
    std::vector<double> u;
};
struct ATMOSPHEREOutput {
    std::vector<double> y;
};

void atmosphere_init(ATMOSPHEREState& s, const ATMOSPHEREParams& p);
void atmosphere_step(const ATMOSPHEREInput& u, ATMOSPHEREState& s, ATMOSPHEREOutput& y, double dt);
void atmosphere_reset(ATMOSPHEREState& s);
} // namespace sad
