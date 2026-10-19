#include "atmosphere.h"
#include <vector>

namespace sad {
void atmosphere_init(ATMOSPHEREState& s, const ATMOSPHEREParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void atmosphere_step(const ATMOSPHEREInput& u, ATMOSPHEREState& s, ATMOSPHEREOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void atmosphere_reset(ATMOSPHEREState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
