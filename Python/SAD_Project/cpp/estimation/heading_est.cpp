#include "heading_est.h"
#include <vector>

namespace sad {
void heading_est_init(HEADING_ESTState& s, const HEADING_ESTParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void heading_est_step(const HEADING_ESTInput& u, HEADING_ESTState& s, HEADING_ESTOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void heading_est_reset(HEADING_ESTState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
