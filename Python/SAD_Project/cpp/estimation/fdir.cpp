#include "fdir.h"
#include <vector>

namespace sad {
void fdir_init(FDIRState& s, const FDIRParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void fdir_step(const FDIRInput& u, FDIRState& s, FDIROutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void fdir_reset(FDIRState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
