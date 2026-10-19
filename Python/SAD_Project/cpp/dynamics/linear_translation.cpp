#include "linear_translation.h"
#include <vector>

namespace sad {
void linear_translation_init(LINEAR_TRANSLATIONState& s, const LINEAR_TRANSLATIONParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void linear_translation_step(const LINEAR_TRANSLATIONInput& u, LINEAR_TRANSLATIONState& s, LINEAR_TRANSLATIONOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void linear_translation_reset(LINEAR_TRANSLATIONState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
