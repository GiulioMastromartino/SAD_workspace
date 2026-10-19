#include "polyhedral_gravity.h"
#include <vector>

namespace sad {
void polyhedral_gravity_init(POLYHEDRAL_GRAVITYState& s, const POLYHEDRAL_GRAVITYParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void polyhedral_gravity_step(const POLYHEDRAL_GRAVITYInput& u, POLYHEDRAL_GRAVITYState& s, POLYHEDRAL_GRAVITYOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void polyhedral_gravity_reset(POLYHEDRAL_GRAVITYState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
