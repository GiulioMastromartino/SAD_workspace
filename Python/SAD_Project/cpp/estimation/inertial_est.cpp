#include "inertial_est.h"
#include <vector>

namespace sad {
void inertial_est_init(INERTIAL_ESTState& s, const INERTIAL_ESTParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void inertial_est_step(const INERTIAL_ESTInput& u, INERTIAL_ESTState& s, INERTIAL_ESTOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void inertial_est_reset(INERTIAL_ESTState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
