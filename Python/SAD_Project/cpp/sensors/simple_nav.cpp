#include "simple_nav.h"
#include <vector>

namespace sad {
void simple_nav_init(SIMPLE_NAVState& s, const SIMPLE_NAVParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void simple_nav_step(const SIMPLE_NAVInput& u, SIMPLE_NAVState& s, SIMPLE_NAVOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void simple_nav_reset(SIMPLE_NAVState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
