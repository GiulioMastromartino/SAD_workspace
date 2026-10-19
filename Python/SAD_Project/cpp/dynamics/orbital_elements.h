#pragma once
/* Skeleton module: orbital_elements (dynamics). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct ORBITAL_ELEMENTSParams {
    std::vector<double> params;
};
struct ORBITAL_ELEMENTSState {
    ORBITAL_ELEMENTSParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct ORBITAL_ELEMENTSInput {
    std::vector<double> u;
};
struct ORBITAL_ELEMENTSOutput {
    std::vector<double> y;
};

void orbital_elements_init(ORBITAL_ELEMENTSState& s, const ORBITAL_ELEMENTSParams& p);
void orbital_elements_step(const ORBITAL_ELEMENTSInput& u, ORBITAL_ELEMENTSState& s, ORBITAL_ELEMENTSOutput& y, double dt);
void orbital_elements_reset(ORBITAL_ELEMENTSState& s);
} // namespace sad
