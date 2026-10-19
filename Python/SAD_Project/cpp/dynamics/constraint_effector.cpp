#include "constraint_effector.h"
#include <vector>

namespace sad {
void constraint_effector_init(CONSTRAINT_EFFECTORState& s, const CONSTRAINT_EFFECTORParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void constraint_effector_step(const CONSTRAINT_EFFECTORInput& u, CONSTRAINT_EFFECTORState& s, CONSTRAINT_EFFECTOROutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void constraint_effector_reset(CONSTRAINT_EFFECTORState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
