#pragma once
/* Skeleton module: sensor_comm (estimation). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct SENSOR_COMMParams {
    std::vector<double> params;
};
struct SENSOR_COMMState {
    SENSOR_COMMParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct SENSOR_COMMInput {
    std::vector<double> u;
};
struct SENSOR_COMMOutput {
    std::vector<double> y;
};

void sensor_comm_init(SENSOR_COMMState& s, const SENSOR_COMMParams& p);
void sensor_comm_step(const SENSOR_COMMInput& u, SENSOR_COMMState& s, SENSOR_COMMOutput& y, double dt);
void sensor_comm_reset(SENSOR_COMMState& s);
} // namespace sad
