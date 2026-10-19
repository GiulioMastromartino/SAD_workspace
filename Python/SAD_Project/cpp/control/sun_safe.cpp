#include "sun_safe.h"
#include <vector>

namespace sad {
void sun_safe_init(SUN_SAFEState& s, const SUN_SAFEParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void sun_safe_step(const SUN_SAFEInput& u, SUN_SAFEState& s, SUN_SAFEOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void sun_safe_reset(SUN_SAFEState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
