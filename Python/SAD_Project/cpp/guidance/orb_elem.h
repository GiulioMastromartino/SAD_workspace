#pragma once
/* Skeleton module: orb_elem (guidance). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct ORB_ELEMParams {
    std::vector<double> params;
};
struct ORB_ELEMState {
    ORB_ELEMParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct ORB_ELEMInput {
    std::vector<double> u;
};
struct ORB_ELEMOutput {
    std::vector<double> y;
};

void orb_elem_init(ORB_ELEMState& s, const ORB_ELEMParams& p);
void orb_elem_step(const ORB_ELEMInput& u, ORB_ELEMState& s, ORB_ELEMOutput& y, double dt);
void orb_elem_reset(ORB_ELEMState& s);
} // namespace sad
