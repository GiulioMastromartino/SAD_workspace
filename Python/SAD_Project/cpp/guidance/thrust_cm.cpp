#include "thrust_cm.h"
#include <vector>

namespace sad {
void thrust_cm_init(THRUST_CMState& s, const THRUST_CMParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void thrust_cm_step(const THRUST_CMInput& u, THRUST_CMState& s, THRUST_CMOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void thrust_cm_reset(THRUST_CMState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
