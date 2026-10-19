#include "small_body_nav.h"
#include <vector>

namespace sad {
void small_body_nav_init(SMALL_BODY_NAVState& s, const SMALL_BODY_NAVParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void small_body_nav_step(const SMALL_BODY_NAVInput& u, SMALL_BODY_NAVState& s, SMALL_BODY_NAVOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void small_body_nav_reset(SMALL_BODY_NAVState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
