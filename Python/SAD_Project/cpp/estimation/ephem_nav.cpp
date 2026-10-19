#include "ephem_nav.h"
#include <vector>

namespace sad {
void ephem_nav_init(EPHEM_NAVState& s, const EPHEM_NAVParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void ephem_nav_step(const EPHEM_NAVInput& u, EPHEM_NAVState& s, EPHEM_NAVOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void ephem_nav_reset(EPHEM_NAVState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
