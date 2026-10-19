#pragma once
/* Skeleton module: space_weather (environment). Auto-generated; refine physics as needed. */
#include "sad_types.h"
#include "sad_module.h"
#include <vector>

namespace sad {
struct SPACE_WEATHERParams {
    std::vector<double> params;
};
struct SPACE_WEATHERState {
    SPACE_WEATHERParams params;
    std::vector<double> state;
    double t = 0.0;
};
struct SPACE_WEATHERInput {
    std::vector<double> u;
};
struct SPACE_WEATHEROutput {
    std::vector<double> y;
};

void space_weather_init(SPACE_WEATHERState& s, const SPACE_WEATHERParams& p);
void space_weather_step(const SPACE_WEATHERInput& u, SPACE_WEATHERState& s, SPACE_WEATHEROutput& y, double dt);
void space_weather_reset(SPACE_WEATHERState& s);
} // namespace sad
