#include "ground.h"
#include <vector>

namespace sad {
void ground_init(GROUNDState& s, const GROUNDParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void ground_step(const GROUNDInput& u, GROUNDState& s, GROUNDOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void ground_reset(GROUNDState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
