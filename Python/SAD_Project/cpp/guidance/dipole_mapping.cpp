#include "dipole_mapping.h"
#include <vector>

namespace sad {
void dipole_mapping_init(DIPOLE_MAPPINGState& s, const DIPOLE_MAPPINGParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void dipole_mapping_step(const DIPOLE_MAPPINGInput& u, DIPOLE_MAPPINGState& s, DIPOLE_MAPPINGOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void dipole_mapping_reset(DIPOLE_MAPPINGState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
