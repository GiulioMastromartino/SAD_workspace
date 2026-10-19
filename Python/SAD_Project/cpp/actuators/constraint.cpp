#include "constraint.h"
#include <vector>

namespace sad {
void constraint_init(CONSTRAINTState& s, const CONSTRAINTParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void constraint_step(const CONSTRAINTInput& u, CONSTRAINTState& s, CONSTRAINTOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void constraint_reset(CONSTRAINTState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
