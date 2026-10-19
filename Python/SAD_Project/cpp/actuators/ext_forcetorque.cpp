#include "ext_forcetorque.h"
#include <vector>

namespace sad {
void ext_forcetorque_init(EXT_FORCETORQUEState& s, const EXT_FORCETORQUEParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void ext_forcetorque_step(const EXT_FORCETORQUEInput& u, EXT_FORCETORQUEState& s, EXT_FORCETORQUEOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void ext_forcetorque_reset(EXT_FORCETORQUEState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
