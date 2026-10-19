#include "ext_force.h"
#include <vector>

namespace sad {
void ext_force_init(EXT_FORCEState& s, const EXT_FORCEParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void ext_force_step(const EXT_FORCEInput& u, EXT_FORCEState& s, EXT_FORCEOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void ext_force_reset(EXT_FORCEState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
