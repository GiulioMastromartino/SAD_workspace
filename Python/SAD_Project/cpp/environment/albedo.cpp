#include "albedo.h"
#include <vector>

namespace sad {
void albedo_init(ALBEDOState& s, const ALBEDOParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void albedo_step(const ALBEDOInput& u, ALBEDOState& s, ALBEDOOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void albedo_reset(ALBEDOState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
