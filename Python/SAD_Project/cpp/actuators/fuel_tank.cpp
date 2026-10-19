#include "fuel_tank.h"
#include <vector>

namespace sad {
void fuel_tank_init(FUEL_TANKState& s, const FUEL_TANKParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void fuel_tank_step(const FUEL_TANKInput& u, FUEL_TANKState& s, FUEL_TANKOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void fuel_tank_reset(FUEL_TANKState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
