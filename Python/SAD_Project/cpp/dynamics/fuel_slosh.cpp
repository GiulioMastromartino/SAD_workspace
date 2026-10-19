#include "fuel_slosh.h"
#include <vector>

namespace sad {
void fuel_slosh_init(FUEL_SLOSHState& s, const FUEL_SLOSHParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void fuel_slosh_step(const FUEL_SLOSHInput& u, FUEL_SLOSHState& s, FUEL_SLOSHOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void fuel_slosh_reset(FUEL_SLOSHState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
