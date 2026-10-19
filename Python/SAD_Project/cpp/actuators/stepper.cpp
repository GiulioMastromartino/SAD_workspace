#include "stepper.h"
#include <vector>

namespace sad {
void stepper_init(STEPPERState& s, const STEPPERParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void stepper_step(const STEPPERInput& u, STEPPERState& s, STEPPEROutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void stepper_reset(STEPPERState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
