#pragma once
/* Skeleton module: nbody_gravity (dynamics). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct NBODY_GRAVITYParams {
    std::vector<double> params;
};
struct NBODY_GRAVITYState {
    NBODY_GRAVITYParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct NBODY_GRAVITYInput {
    std::vector<double> u;
};
struct NBODY_GRAVITYOutput {
    std::vector<double> y;
};

void nbody_gravity_init(NBODY_GRAVITYState& s, const NBODY_GRAVITYParams& p);
void nbody_gravity_step(const NBODY_GRAVITYInput& u, NBODY_GRAVITYState& s, NBODY_GRAVITYOutput& y, double dt);
void nbody_gravity_reset(NBODY_GRAVITYState& s);
} // namespace sad
