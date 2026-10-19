#include "formation.h"
#include <vector>

namespace sad {
void formation_init(FORMATIONState& s, const FORMATIONParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void formation_step(const FORMATIONInput& u, FORMATIONState& s, FORMATIONOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void formation_reset(FORMATIONState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
