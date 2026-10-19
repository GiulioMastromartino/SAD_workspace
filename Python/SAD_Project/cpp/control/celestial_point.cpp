#include "celestial_point.h"
#include <vector>

namespace sad {
void celestial_point_init(CELESTIAL_POINTState& s, const CELESTIAL_POINTParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void celestial_point_step(const CELESTIAL_POINTInput& u, CELESTIAL_POINTState& s, CELESTIAL_POINTOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void celestial_point_reset(CELESTIAL_POINTState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
