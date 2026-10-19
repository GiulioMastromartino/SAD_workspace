#include "thermal.h"
#include <vector>

namespace sad {
void thermal_init(THERMALState& s, const THERMALParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void thermal_step(const THERMALInput& u, THERMALState& s, THERMALOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void thermal_reset(THERMALState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
