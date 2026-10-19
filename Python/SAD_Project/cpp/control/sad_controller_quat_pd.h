#pragma once
#include "sad_types.h"
#include "sad_module.h"

namespace sad {
enum class CtrlMode { Detumble, Slew, Track };

// Backward-Euler effective-gain scale for the linearized per-axis attitude loop.
//
// The explicit PD update uses a zero-order-held torque, so its rate map is
//   w_{n+1} = (1 - dt*P/I) w_n,
// which is unstable for dt*P/I > 2.  Matching it to the implicit update
//   e_{n+1} = e_n + 0.5*dt*w_{n+1}
//   w_{n+1} = w_n - dt*(K/I)*e_{n+1} - dt*(P/I)*w_{n+1}
// gives K_eff = K/D, P_eff = (P + 0.5*dt*K)/D with
//   D = 1 + dt*P/I + 0.5*dt^2*K/I.
// The resulting explicit map is unconditionally stable for K,P,dt,I > 0.
// D is also used to scale Ki.  Returns 1 when the scaling is disabled.
inline double quatpd_implicit_scale(double K, double P, double I, double dt) {
    if (dt <= 0.0 || I <= 0.0) return 1.0;
    return 1.0 + dt * P / I + 0.5 * dt * dt * K / I;
}

struct QuatPDParams {
    double K = 0.5;   // proportional gain on quaternion error vector
    double P = 2.0;   // rate feedback gain
    double Ki = 0.0;  // integral gain
    double detumble_K = 2.0;
    double detumble_P = 4.0;
    double detumble_Ki = 0.0;
    double slew_K = 0.5;
    double slew_P = 2.0;
    double slew_Ki = 0.0;
    double track_K = 0.5;
    double track_P = 2.0;
    double track_Ki = 0.0;
    Vec4 q_slew_target = Vec4(0,0,0,1);
    // Nominal body inertia diagonal (kg m^2) used by the step-aware scaling.
    // The controller deliberately uses the nominal value, not the dispersed
    // plant inertia.  A non-positive component disables scaling on that axis.
    Vec3 inertia = Vec3(0.08666666666666667, 0.06666666666666667, 0.03333333333333333);
    // true  = backward-Euler equivalent gains at the current dt (stable at dt=1)
    // false = legacy explicit PD (A/B and regression path)
    bool implicit_dt = true;
    // Apply the scaling only in Track mode.  Detumble/slew run at dt=0.1 and
    // their legacy gains are already stable; scaling them slows the slew and
    // increases the (pre-existing) actuator-authority timeouts.  Set false to
    // scale every phase (needed when detumble/slew also run at dt=1).
    bool implicit_track_only = true;
};
struct QuatPDState {
    QuatPDParams params;
    CtrlMode mode = CtrlMode::Detumble;
    Vec3 integral = Vec3::Zero();
};
struct QuatPDInput {
    Vec4 q_current = Vec4(0,0,0,1);
    Vec3 omega = Vec3::Zero();
    Vec4 q_desired = Vec4(0,0,0,1);
    CtrlMode mode = CtrlMode::Detumble;
};
struct QuatPDOutput {
    Vec3 torque_b = Vec3::Zero();
};

void quatpd_init(QuatPDState& s, const QuatPDParams& p);
void quatpd_step(const QuatPDInput& u, QuatPDState& s, QuatPDOutput& y, double dt);
void quatpd_reset(QuatPDState& s);
} // namespace sad
