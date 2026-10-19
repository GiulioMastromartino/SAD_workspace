#include "sad_controller_quat_pd.h"
#include "sad_timing.h"

namespace sad {
void quatpd_init(QuatPDState& s, const QuatPDParams& p) { s.params = p; s.mode = CtrlMode::Detumble; }
void quatpd_step(const QuatPDInput& u, QuatPDState& s, QuatPDOutput& y, double dt) {
    SAD_TIME(TQuatPD);
    const QuatPDParams& p = s.params;
    s.mode = u.mode;
    double K, P, Ki;
    switch (u.mode) {
        case CtrlMode::Detumble: K = p.detumble_K; P = p.detumble_P; Ki = p.detumble_Ki; break;
        case CtrlMode::Slew:     K = p.slew_K;     P = p.slew_P;     Ki = p.slew_Ki;     break;
        case CtrlMode::Track:    K = p.track_K;    P = p.track_P;    Ki = p.track_Ki;    break;
    }
    Vec3 e = quat_error_vec(u.q_desired, u.q_current);
    // integral with anti-windup clamp
    s.integral += e * dt;
    for (int i = 0; i < 3; ++i) {
        if (s.integral[i] > 0.5) s.integral[i] = 0.5;
        if (s.integral[i] < -0.5) s.integral[i] = -0.5;
    }
    // Step-aware effective gains: the explicit torque below is algebraically
    // equivalent to a backward-Euler update of the per-axis attitude/rate loop.
    const bool implicit_allowed =
        p.implicit_dt && (!p.implicit_track_only || u.mode == CtrlMode::Track);
    for (int i = 0; i < 3; ++i) {
        double Ke = K, Pe = P, Kie = Ki;
        if (implicit_allowed && dt > 0.0 && p.inertia[i] > 0.0) {
            const double D = quatpd_implicit_scale(K, P, p.inertia[i], dt);
            Ke = K / D;
            Pe = (P + 0.5 * dt * K) / D;
            Kie = Ki / D;
        }
        y.torque_b[i] = -Ke * e[i] - Pe * u.omega[i] - Kie * s.integral[i];
    }
}
void quatpd_reset(QuatPDState& s) { s.mode = CtrlMode::Detumble; s.integral.setZero(); }
} // namespace sad
