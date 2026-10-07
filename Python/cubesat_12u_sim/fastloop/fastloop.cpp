// Fast per-tick spacecraft control loop (all Monte Carlo lanes at once).
//
// Mirrors the Python closed loop in heyoka_attitude / heyoka_mc exactly:
// bias observer -> MRP error (short rotation) -> PD -> cached-pinv allocation
// -> effector quantizer -> wrench matvecs -> fuel bookkeeping. Operates on
// row-major (B,3)/(B,12) buffers with plain loops: no Eigen dependency,
// -O3 + NEON autovectorization friendly. Floating point is NOT bit-identical
// to the numpy path (different op order); parity is validated with tolerance.
#include <cmath>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace {

// ---- tiny 3-vector helpers (row-major stride-3 pointers) ----
inline double dot3(const double *a, const double *b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline void cross3(const double *a, const double *b, double *o) {
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}
inline double norm3(const double *a) { return std::sqrt(dot3(a, a)); }

// MRP difference sigma_BR from body/reference MRPs (short rotation),
// same formula as heyoka_attitude.mrp_diff.
inline void mrp_diff(const double *b, const double *r, double *e) {
    const double b2 = dot3(b, b), r2 = dot3(r, r);
    double cr[3];
    cross3(b, r, cr);
    const double den = 1.0 + b2 * r2 + 2.0 * dot3(b, r);
    for (int i = 0; i < 3; ++i)
        e[i] = (b[i] * (1.0 - r2) - r[i] * (1.0 - b2) + 2.0 * cr[i]) / den;
    const double n2 = dot3(e, e);
    if (n2 > 1.0)
        for (int i = 0; i < 3; ++i) e[i] = -e[i] / n2;
}

// ---- shared per-lane primitives (observer + PD/alloc/wrench/fuel) ----
struct CtrlParams {
    double K, P, obs_gain, obs_bias_gain, ctrl_scale;
    double dt_ctrl, dt_obs, g0;
    int n_thr;
    bool quantize;
};

inline void lane_observe(const double mm[3], double es[3], double bs[3],
                         bool &init, double obs_gain, double obs_bias_gain,
                         double dt_obs, double fb[3]) {
    if (!init) {
        es[0] = mm[0];
        es[1] = mm[1];
        es[2] = mm[2];
        init = true;
    } else {
        double r0 = mm[0] - es[0] - bs[0];
        double r1 = mm[1] - es[1] - bs[1];
        double r2 = mm[2] - es[2] - bs[2];
        es[0] += obs_gain * r0 * dt_obs;
        es[1] += obs_gain * r1 * dt_obs;
        es[2] += obs_gain * r2 * dt_obs;
        bs[0] += obs_bias_gain * r0 * dt_obs;
        bs[1] += obs_bias_gain * r1 * dt_obs;
        bs[2] += obs_bias_gain * r2 * dt_obs;
    }
    fb[0] = mm[0] - bs[0];
    fb[1] = mm[1] - bs[1];
    fb[2] = mm[2] - bs[2];
}

template <typename Pnom, typename MFx, typename MTx>
inline void lane_control(const double sg[3], const double fb[3],
                         const double rs[3], const CtrlParams &cp,
                         Pnom &Pn, MFx &Mf, MTx &Mt, int j,
                         double &Fx, double &Fy, double &Fz, double &Tx,
                         double &Ty, double &Tz, double *u, double &tx,
                         double &ty, double &tz) {
    double e[3];
    mrp_diff(sg, rs, e);
    tx = -cp.K * e[0] - cp.P * fb[0];
    ty = -cp.K * e[1] - cp.P * fb[1];
    tz = -cp.K * e[2] - cp.P * fb[2];
    for (int i = 0; i < cp.n_thr; ++i) {
        double v = Pn(i, 3) * (cp.ctrl_scale * tx) +
                   Pn(i, 4) * (cp.ctrl_scale * ty) +
                   Pn(i, 5) * (cp.ctrl_scale * tz);
        u[i] = v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
    }
    Fx = Fy = Fz = Tx = Ty = Tz = 0.0;
    for (int i = 0; i < cp.n_thr; ++i) {
        double q = u[i];
        if (cp.quantize)
            q = q <= 0.0 ? 0.0 : (q < 0.5 ? 1.0 / 6.0 : (q < 1.0 ? 5.0 / 6.0 : 1.0));
        Fx += Mf(j, 0, i) * q;
        Fy += Mf(j, 1, i) * q;
        Fz += Mf(j, 2, i) * q;
        Tx += Mt(j, 0, i) * q;
        Ty += Mt(j, 1, i) * q;
        Tz += Mt(j, 2, i) * q;
    }
}


}  // namespace

// One control tick for B lanes. All arrays row-major, contiguous.
//   sigma/omega/meas/ref/fb-est in: (B,3); P_nom: (12,6);
//   MF/MT: (B,3,12) per-lane effectiveness; tmax/isp: (B,).
// In-place: est_rate, bias_est (B,3); F_hold/T_hold (B,3); on_time (B,12);
// fuel/imp accumulators length-3 (phase index given).
// Out (for diagnostics): torque (B,3), uthr (B,12) clipped throttles.
void control_tick(
    int B, int n_thr,
    py::array_t<double, py::array::c_style | py::array::forcecast> sigma,
    py::array_t<double, py::array::c_style | py::array::forcecast> omega,
    py::array_t<double, py::array::c_style | py::array::forcecast> meas,
    py::array_t<double, py::array::c_style | py::array::forcecast> ref,
    py::array_t<double, py::array::c_style | py::array::forcecast> est_rate,
    py::array_t<double, py::array::c_style | py::array::forcecast> bias_est,
    py::array_t<bool, py::array::c_style | py::array::forcecast> est_init,
    py::array_t<double, py::array::c_style | py::array::forcecast> P_nom,
    py::array_t<double, py::array::c_style | py::array::forcecast> MF,
    py::array_t<double, py::array::c_style | py::array::forcecast> MT,
    py::array_t<double, py::array::c_style | py::array::forcecast> tmax,
    py::array_t<double, py::array::c_style | py::array::forcecast> isp,
    double K, double P, double obs_gain, double obs_bias_gain,
    double ctrl_scale, double dt_ctrl, double dt_obs, double g0, int phase,
    bool quantize,
    py::array_t<double, py::array::c_style> F_hold,
    py::array_t<double, py::array::c_style> T_hold,
    py::array_t<double, py::array::c_style> on_time,
    py::array_t<double, py::array::c_style> phase_imp,
    py::array_t<double, py::array::c_style> phase_fuel,
    py::array_t<double, py::array::c_style> torque,
    py::array_t<double, py::array::c_style> uthr) {
    auto s = sigma.unchecked<2>();
    auto o = omega.unchecked<2>();
    auto ms = meas.unchecked<2>();
    auto rf = ref.unchecked<2>();
    auto er = est_rate.mutable_unchecked<2>();
    auto be = bias_est.mutable_unchecked<2>();
    auto ei = est_init.mutable_unchecked<1>();
    auto Pn = P_nom.unchecked<2>();
    auto Mf = MF.unchecked<3>();
    auto Mt = MT.unchecked<3>();
    auto tm = tmax.unchecked<2>();
    auto sp = isp.unchecked<1>();
    auto Fh = F_hold.mutable_unchecked<2>();
    auto Th = T_hold.mutable_unchecked<2>();
    auto ot = on_time.mutable_unchecked<2>();
    auto tq = torque.mutable_unchecked<2>();
    auto uh = uthr.mutable_unchecked<2>();
    auto pi = phase_imp.mutable_unchecked<2>();
    auto pf = phase_fuel.mutable_unchecked<2>();

    CtrlParams cp{K, P, obs_gain, obs_bias_gain, ctrl_scale,
                  dt_ctrl, dt_obs, g0, n_thr, quantize};

    for (int j = 0; j < B; ++j) {
        double sg[3] = {s(j, 0), s(j, 1), s(j, 2)};
        double mm[3] = {ms(j, 0), ms(j, 1), ms(j, 2)};
        double rs[3] = {rf(j, 0), rf(j, 1), rf(j, 2)};
        double es[3] = {er(j, 0), er(j, 1), er(j, 2)};
        double bs[3] = {be(j, 0), be(j, 1), be(j, 2)};
        bool init = ei(j);

        double fb[3];
        lane_observe(mm, es, bs, init, obs_gain, obs_bias_gain, dt_obs, fb);
        ei(j) = init;
        double Fx, Fy, Fz, Tx, Ty, Tz, tx, ty, tz;
        double u[12];
        lane_control(sg, fb, rs, cp, Pn, Mf, Mt, j, Fx, Fy, Fz, Tx, Ty, Tz,
                     u, tx, ty, tz);
        tq(j, 0) = tx;
        tq(j, 1) = ty;
        tq(j, 2) = tz;
        double imp = 0.0;
        for (int i = 0; i < n_thr; ++i) {
            uh(j, i) = u[i];
            double on = u[i] * dt_ctrl;
            ot(j, i) += on;
            imp += on * tm(j, i);
        }
        Fh(j, 0) = Fx;
        Fh(j, 1) = Fy;
        Fh(j, 2) = Fz;
        Th(j, 0) = Tx;
        Th(j, 1) = Ty;
        Th(j, 2) = Tz;
        pi(j, phase) += imp;
        pf(j, phase) += imp / (sp(j) * g0);

        er(j, 0) = es[0];
        er(j, 1) = es[1];
        er(j, 2) = es[2];
        be(j, 0) = bs[0];
        be(j, 1) = bs[1];
        be(j, 2) = bs[2];
    }
}

// Fused whole-tick kernel: meas assembly + observer + control + fuel +
// pars pack, so per tick Python only calls this, propagate_for, and handles
// logging (shadow reset stays Python-side post-step, ~1us).
void tick_all(
    int B, int n_thr,
    py::array_t<double, py::array::c_style | py::array::forcecast> sigma,
    py::array_t<double, py::array::c_style | py::array::forcecast> omega,
    py::array_t<double, py::array::c_style | py::array::forcecast> bias_true,
    py::array_t<double, py::array::c_style | py::array::forcecast> noise_k,
    py::array_t<double, py::array::c_style | py::array::forcecast> ref,
    py::array_t<double, py::array::c_style | py::array::forcecast> est_rate,
    py::array_t<double, py::array::c_style | py::array::forcecast> bias_est,
    py::array_t<bool, py::array::c_style | py::array::forcecast> est_init,
    py::array_t<double, py::array::c_style | py::array::forcecast> P_nom,
    py::array_t<double, py::array::c_style | py::array::forcecast> MF,
    py::array_t<double, py::array::c_style | py::array::forcecast> MT,
    py::array_t<double, py::array::c_style | py::array::forcecast> tmax,
    py::array_t<double, py::array::c_style | py::array::forcecast> isp,
    double K, double P, double obs_gain, double obs_bias_gain,
    double ctrl_scale, double dt_ctrl, double dt_obs, double g0, int phase,
    bool quantize,
    py::array_t<double, py::array::c_style> F_hold,
    py::array_t<double, py::array::c_style> T_hold,
    py::array_t<double, py::array::c_style> on_time,
    py::array_t<double, py::array::c_style> phase_imp,
    py::array_t<double, py::array::c_style> phase_fuel,
    py::array_t<double, py::array::c_style> pars) {
    auto s = sigma.unchecked<2>();
    auto o = omega.unchecked<2>();
    auto bt = bias_true.unchecked<2>();
    auto nk = noise_k.unchecked<2>();
    auto rf = ref.unchecked<2>();
    auto er = est_rate.mutable_unchecked<2>();
    auto be = bias_est.mutable_unchecked<2>();
    auto ei = est_init.mutable_unchecked<1>();
    auto Pn = P_nom.unchecked<2>();
    auto Mf = MF.unchecked<3>();
    auto Mt = MT.unchecked<3>();
    auto tm = tmax.unchecked<2>();
    auto sp = isp.unchecked<1>();
    auto Fh = F_hold.mutable_unchecked<2>();
    auto Th = T_hold.mutable_unchecked<2>();
    auto ot = on_time.mutable_unchecked<2>();
    auto pi = phase_imp.mutable_unchecked<2>();
    auto pf = phase_fuel.mutable_unchecked<2>();
    auto pr = pars.mutable_unchecked<2>();

    CtrlParams cp{K, P, obs_gain, obs_bias_gain, ctrl_scale,
                  dt_ctrl, dt_obs, g0, n_thr, quantize};

    for (int j = 0; j < B; ++j) {
        double sg[3] = {s(j, 0), s(j, 1), s(j, 2)};
        double mm[3] = {o(j, 0) + bt(j, 0) + nk(j, 0),
                        o(j, 1) + bt(j, 1) + nk(j, 1),
                        o(j, 2) + bt(j, 2) + nk(j, 2)};
        double rs[3] = {rf(j, 0), rf(j, 1), rf(j, 2)};
        double es[3] = {er(j, 0), er(j, 1), er(j, 2)};
        double bs[3] = {be(j, 0), be(j, 1), be(j, 2)};
        bool init = ei(j);

        double fb[3];
        lane_observe(mm, es, bs, init, obs_gain, obs_bias_gain, dt_obs, fb);
        ei(j) = init;
        double Fx, Fy, Fz, Tx, Ty, Tz, tx, ty, tz;
        double u[12];
        lane_control(sg, fb, rs, cp, Pn, Mf, Mt, j, Fx, Fy, Fz, Tx, Ty, Tz,
                     u, tx, ty, tz);
        double imp = 0.0;
        for (int i = 0; i < n_thr; ++i) {
            double on = u[i] * dt_ctrl;
            ot(j, i) += on;
            imp += on * tm(j, i);
        }
        Fh(j, 0) = Fx;
        Fh(j, 1) = Fy;
        Fh(j, 2) = Fz;
        Th(j, 0) = Tx;
        Th(j, 1) = Ty;
        Th(j, 2) = Tz;
        pi(j, phase) += imp;
        pf(j, phase) += imp / (sp(j) * g0);

        // pars rows 0:6 (wrench); static rows preset once by the caller.
        pr(0, j) = Fx;
        pr(1, j) = Fy;
        pr(2, j) = Fz;
        pr(3, j) = Tx;
        pr(4, j) = Ty;
        pr(5, j) = Tz;

        er(j, 0) = es[0];
        er(j, 1) = es[1];
        er(j, 2) = es[2];
        be(j, 0) = bs[0];
        be(j, 1) = bs[1];
        be(j, 2) = bs[2];
    }
}

PYBIND11_MODULE(_fastloop, m) {
    m.doc() = "Batched spacecraft control tick (see heyoka_mc).";
    m.def("control_tick", &control_tick, "Run one control tick for B lanes.",
          py::arg("B"), py::arg("n_thr"), py::arg("sigma"), py::arg("omega"),
          py::arg("meas"), py::arg("ref"), py::arg("est_rate"), py::arg("bias_est"),
          py::arg("est_init"), py::arg("P_nom"), py::arg("MF"), py::arg("MT"),
          py::arg("tmax"), py::arg("isp"), py::arg("K"), py::arg("P"),
          py::arg("obs_gain"), py::arg("obs_bias_gain"), py::arg("ctrl_scale"),
          py::arg("dt_ctrl"), py::arg("dt_obs"), py::arg("g0"), py::arg("phase"), py::arg("quantize"),
          py::arg("F_hold"), py::arg("T_hold"), py::arg("on_time"),
          py::arg("phase_imp"), py::arg("phase_fuel"), py::arg("torque"),
          py::arg("uthr"));
    m.def("tick_all", &tick_all, "Fused whole-tick kernel (meas+observer+control+fuel+pars).",
          py::arg("B"), py::arg("n_thr"), py::arg("sigma"), py::arg("omega"),
          py::arg("bias_true"), py::arg("noise_k"), py::arg("ref"),
          py::arg("est_rate"), py::arg("bias_est"),
          py::arg("est_init"), py::arg("P_nom"), py::arg("MF"), py::arg("MT"),
          py::arg("tmax"), py::arg("isp"), py::arg("K"), py::arg("P"),
          py::arg("obs_gain"), py::arg("obs_bias_gain"), py::arg("ctrl_scale"),
          py::arg("dt_ctrl"), py::arg("dt_obs"), py::arg("g0"), py::arg("phase"),
          py::arg("quantize"), py::arg("F_hold"), py::arg("T_hold"),
          py::arg("on_time"), py::arg("phase_imp"), py::arg("phase_fuel"),
          py::arg("pars"));
}
