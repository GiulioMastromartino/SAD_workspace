#include "spinning_body.h"
#include <vector>

namespace sad {
void spinning_body_init(SPINNING_BODYState& s, const SPINNING_BODYParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void spinning_body_step(const SPINNING_BODYInput& u, SPINNING_BODYState& s, SPINNING_BODYOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void spinning_body_reset(SPINNING_BODYState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
