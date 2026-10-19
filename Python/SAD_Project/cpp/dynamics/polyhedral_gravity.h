#pragma once
/* Skeleton module: polyhedral_gravity (dynamics). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct POLYHEDRAL_GRAVITYParams {
    std::vector<double> params;
};
struct POLYHEDRAL_GRAVITYState {
    POLYHEDRAL_GRAVITYParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct POLYHEDRAL_GRAVITYInput {
    std::vector<double> u;
};
struct POLYHEDRAL_GRAVITYOutput {
    std::vector<double> y;
};

void polyhedral_gravity_init(POLYHEDRAL_GRAVITYState& s, const POLYHEDRAL_GRAVITYParams& p);
void polyhedral_gravity_step(const POLYHEDRAL_GRAVITYInput& u, POLYHEDRAL_GRAVITYState& s, POLYHEDRAL_GRAVITYOutput& y, double dt);
void polyhedral_gravity_reset(POLYHEDRAL_GRAVITYState& s);
} // namespace sad
