#pragma once
/* Skeleton module: opnav (estimation). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct OPNAVParams {
    std::vector<double> params;
};
struct OPNAVState {
    OPNAVParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct OPNAVInput {
    std::vector<double> u;
};
struct OPNAVOutput {
    std::vector<double> y;
};

void opnav_init(OPNAVState& s, const OPNAVParams& p);
void opnav_step(const OPNAVInput& u, OPNAVState& s, OPNAVOutput& y, double dt);
void opnav_reset(OPNAVState& s);
} // namespace sad
