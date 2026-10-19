#include "nbody_gravity.h"
#include <vector>

namespace sad {
void nbody_gravity_init(NBODY_GRAVITYState& s, const NBODY_GRAVITYParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void nbody_gravity_step(const NBODY_GRAVITYInput& u, NBODY_GRAVITYState& s, NBODY_GRAVITYOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void nbody_gravity_reset(NBODY_GRAVITYState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
