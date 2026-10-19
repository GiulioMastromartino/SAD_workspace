#include "planet_ephem.h"
#include <vector>

namespace sad {
void planet_ephem_init(PLANET_EPHEMState& s, const PLANET_EPHEMParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void planet_ephem_step(const PLANET_EPHEMInput& u, PLANET_EPHEMState& s, PLANET_EPHEMOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void planet_ephem_reset(PLANET_EPHEMState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
