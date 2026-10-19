#include "solar_flux.h"
#include <vector>

namespace sad {
void solar_flux_init(SOLAR_FLUXState& s, const SOLAR_FLUXParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void solar_flux_step(const SOLAR_FLUXInput& u, SOLAR_FLUXState& s, SOLAR_FLUXOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void solar_flux_reset(SOLAR_FLUXState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
