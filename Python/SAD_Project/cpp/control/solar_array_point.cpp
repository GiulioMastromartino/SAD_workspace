#include "solar_array_point.h"
#include <vector>

namespace sad {
void solar_array_point_init(SOLAR_ARRAY_POINTState& s, const SOLAR_ARRAY_POINTParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void solar_array_point_step(const SOLAR_ARRAY_POINTInput& u, SOLAR_ARRAY_POINTState& s, SOLAR_ARRAY_POINTOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void solar_array_point_reset(SOLAR_ARRAY_POINTState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
