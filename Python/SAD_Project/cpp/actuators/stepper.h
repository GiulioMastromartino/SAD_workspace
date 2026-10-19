#pragma once
/* Skeleton module: stepper (actuators). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct STEPPERParams {
    std::vector<double> params;
};
struct STEPPERState {
    STEPPERParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct STEPPERInput {
    std::vector<double> u;
};
struct STEPPEROutput {
    std::vector<double> y;
};

void stepper_init(STEPPERState& s, const STEPPERParams& p);
void stepper_step(const STEPPERInput& u, STEPPERState& s, STEPPEROutput& y, double dt);
void stepper_reset(STEPPERState& s);
} // namespace sad
