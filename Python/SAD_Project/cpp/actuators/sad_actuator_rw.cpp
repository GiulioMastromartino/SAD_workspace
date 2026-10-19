#include "sad_actuator_rw.h"
#include <algorithm>
#include <cmath>

namespace sad {
void rw_init(RWState& s, const RWParams& p) {
    s.params = p;
    s.h.assign(p.n, 0.0);
    s.omega.assign(p.n, 0.0);
}
void rw_step(const RWInput& u, RWState& s, RWOutput& y, double dt) {
    const RWParams& p = s.params;
    int n = p.n;
    // project desired torque onto wheel axes -> per-wheel torque
    std::vector<double> tau_w(n, 0.0);
    for (int i=0;i<n;++i) {
        tau_w[i] = u.torque_cmd_b.dot(p.axes[i]);
        // saturate
        if (tau_w[i] > p.max_torque) tau_w[i] = p.max_torque;
        if (tau_w[i] < -p.max_torque) tau_w[i] = -p.max_torque;
    }
    // first-order response: h_dot = tau, with saturation on momentum
    Vec3 T(0,0,0);
    for (int i=0;i<n;++i) {
        double hdot = tau_w[i];
        // saturation: if at limit and pushing further, zero
        if (s.h[i] >= p.max_momentum && hdot > 0) hdot = 0;
        if (s.h[i] <= -p.max_momentum && hdot < 0) hdot = 0;
        // first-order lag
        double tau_eff = p.tau > 1e-9 ? p.tau : 1e-9;
        // simple: h += hdot*dt (already commanded torque); lag via exponential on hdot
        // apply first-order to torque command
        double alpha = dt/(tau_eff+dt);
        s.h[i] += (hdot*alpha)*dt; // simplified
        s.omega[i] = (p.max_momentum>0)? s.h[i]/p.max_momentum*0.0 : 0.0; // placeholder
        // reconstruct torque delivered
        T += (hdot*alpha) * p.axes[i];
    }
    y.torque_b = T;
}
void rw_reset(RWState& s) {
    for (auto& h : s.h) h = 0.0;
    for (auto& w : s.omega) w = 0.0;
}
} // namespace sad
