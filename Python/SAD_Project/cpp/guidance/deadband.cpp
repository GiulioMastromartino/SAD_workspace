#include "deadband.h"
#include <vector>

namespace sad {
void deadband_init(DEADBANDState& s, const DEADBANDParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void deadband_step(const DEADBANDInput& u, DEADBANDState& s, DEADBANDOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void deadband_reset(DEADBANDState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
