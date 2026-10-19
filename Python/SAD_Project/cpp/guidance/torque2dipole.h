#pragma once
/* Skeleton module: torque2dipole (guidance). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct TORQUE2DIPOLEParams {
    std::vector<double> params;
};
struct TORQUE2DIPOLEState {
    TORQUE2DIPOLEParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct TORQUE2DIPOLEInput {
    std::vector<double> u;
};
struct TORQUE2DIPOLEOutput {
    std::vector<double> y;
};

void torque2dipole_init(TORQUE2DIPOLEState& s, const TORQUE2DIPOLEParams& p);
void torque2dipole_step(const TORQUE2DIPOLEInput& u, TORQUE2DIPOLEState& s, TORQUE2DIPOLEOutput& y, double dt);
void torque2dipole_reset(TORQUE2DIPOLEState& s);
} // namespace sad
