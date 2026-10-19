#pragma once
/* Skeleton module: sc_location (environment). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct SC_LOCATIONParams {
    std::vector<double> params;
};
struct SC_LOCATIONState {
    SC_LOCATIONParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct SC_LOCATIONInput {
    std::vector<double> u;
};
struct SC_LOCATIONOutput {
    std::vector<double> y;
};

void sc_location_init(SC_LOCATIONState& s, const SC_LOCATIONParams& p);
void sc_location_step(const SC_LOCATIONInput& u, SC_LOCATIONState& s, SC_LOCATIONOutput& y, double dt);
void sc_location_reset(SC_LOCATIONState& s);
} // namespace sad
