#include "joint_thr_alloc.h"
#include <vector>

namespace sad {
void joint_thr_alloc_init(JOINT_THR_ALLOCState& s, const JOINT_THR_ALLOCParams& p) {
    s.params = p;
    s.state.assign(8, 0.0);
    s.t = 0.0;
}
void joint_thr_alloc_step(const JOINT_THR_ALLOCInput& u, JOINT_THR_ALLOCState& s, JOINT_THR_ALLOCOutput& y, double dt) {
    (void)dt;
    s.t += dt;
    // passthrough: output mirrors input (refine per module)
    y.y = u.u;
    if (y.y.empty()) y.y.assign(1, 0.0);
}
void joint_thr_alloc_reset(JOINT_THR_ALLOCState& s) {
    for (auto& v : s.state) v = 0.0;
    s.t = 0.0;
}
} // namespace sad
