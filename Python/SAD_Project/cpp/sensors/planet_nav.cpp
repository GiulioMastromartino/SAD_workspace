#include "planet_nav.h"
#include <vector>

namespace sad {
void planet_nav_init(PLANET_NAVState& s, const PLANET_NAVParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void planet_nav_step(const PLANET_NAVInput& u, PLANET_NAVState& s, PLANET_NAVOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void planet_nav_reset(PLANET_NAVState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
