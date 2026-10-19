#pragma once
/* Skeleton module: spherical_harmonics (dynamics). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct SPHERICAL_HARMONICSParams {
    std::vector<double> params;
};
struct SPHERICAL_HARMONICSState {
    SPHERICAL_HARMONICSParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct SPHERICAL_HARMONICSInput {
    std::vector<double> u;
};
struct SPHERICAL_HARMONICSOutput {
    std::vector<double> y;
};

void spherical_harmonics_init(SPHERICAL_HARMONICSState& s, const SPHERICAL_HARMONICSParams& p);
void spherical_harmonics_step(const SPHERICAL_HARMONICSInput& u, SPHERICAL_HARMONICSState& s, SPHERICAL_HARMONICSOutput& y, double dt);
void spherical_harmonics_reset(SPHERICAL_HARMONICSState& s);
} // namespace sad
