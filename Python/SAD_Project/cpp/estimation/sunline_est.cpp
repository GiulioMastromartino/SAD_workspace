#include "sunline_est.h"
#include <vector>

namespace sad {
void sunline_est_init(SUNLINE_ESTState& s, const SUNLINE_ESTParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void sunline_est_step(const SUNLINE_ESTInput& u, SUNLINE_ESTState& s, SUNLINE_ESTOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void sunline_est_reset(SUNLINE_ESTState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
