#pragma once
/* Skeleton module: mag_field (environment). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct MAG_FIELDParams {
    std::vector<double> params;
};
struct MAG_FIELDState {
    MAG_FIELDParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct MAG_FIELDInput {
    std::vector<double> u;
};
struct MAG_FIELDOutput {
    std::vector<double> y;
};

void mag_field_init(MAG_FIELDState& s, const MAG_FIELDParams& p);
void mag_field_step(const MAG_FIELDInput& u, MAG_FIELDState& s, MAG_FIELDOutput& y, double dt);
void mag_field_reset(MAG_FIELDState& s);
} // namespace sad
