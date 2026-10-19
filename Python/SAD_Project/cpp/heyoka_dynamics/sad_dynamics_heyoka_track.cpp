#include "sad_dynamics_heyoka_track.h"

#include <heyoka/heyoka.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstddef>
#include <functional>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace sad {
namespace {

using expression = heyoka::expression;

static expression maxe(expression a, expression b) {
    return heyoka::select(heyoka::gt(a, b), a, b);
}

static std::vector<std::pair<expression, expression>> build_system() {
    auto vars = heyoka::make_vars(
        "rx", "ry", "rz", "vx", "vy", "vz",
        "qx", "qy", "qz", "qw", "wx", "wy", "wz",
        "zx", "zy", "zz", "fuel");

    expression rx = vars[0], ry = vars[1], rz = vars[2];
    expression vx = vars[3], vy = vars[4], vz = vars[5];
    expression qx = vars[6], qy = vars[7], qz = vars[8], qw = vars[9];
    expression wx = vars[10], wy = vars[11], wz = vars[12];
    expression zx = vars[13], zy = vars[14], zz = vars[15];
    expression fuel = vars[16];

    auto P = [](int i) { return expression(heyoka::par[i]); };

    expression Kx = P(3), Ky = P(4), Kz = P(5);
    expression Px = P(6), Py = P(7), Pz = P(8);
    expression Kix = P(9), Kiy = P(10), Kiz = P(11);
    expression tau_max = P(12);
    expression eps = P(13);
    expression zmax = P(14);
    expression Ixx = P(15), Iyy = P(16), Izz = P(17);
    expression ixx = P(18), iyy = P(19), izz = P(20);
    expression mass = P(21);
    expression mu = P(22);

    // Body-from-inertial DCM.
    expression R00 = 1.0 - 2.0 * (qy * qy + qz * qz);
    expression R01 = 2.0 * (qx * qy + qz * qw);
    expression R02 = 2.0 * (qx * qz - qy * qw);
    expression R10 = 2.0 * (qx * qy - qz * qw);
    expression R11 = 1.0 - 2.0 * (qx * qx + qz * qz);
    expression R12 = 2.0 * (qy * qz + qx * qw);
    expression R20 = 2.0 * (qx * qz + qy * qw);
    expression R21 = 2.0 * (qy * qz - qx * qw);
    expression R22 = 1.0 - 2.0 * (qx * qx + qy * qy);

    // Roll reference is pinned to the current body +Y, expressed in inertial:
    // a smooth state-dependent reference (no discontinuity in track).
    expression rrx = R10, rry = R11, rrz = R12;

    // Sun direction as a function of absolute time.
    const double PI = 3.14159265358979323846;
    const double YEAR = 365.25 * 86400.0;
    expression t = heyoka::time;
    expression ang = 2.0 * PI * t / YEAR + 0.5 * PI;
    expression sx = heyoka::cos(ang);
    expression sy = heyoka::sin(ang);
    expression sz = expression(0.0);

    // Frozen roll reference projected onto the plane perpendicular to the sun.
    expression dotr = rrx * sx + rry * sy + rrz * sz;
    expression rxp = rrx - dotr * sx;
    expression ryp = rry - dotr * sy;
    expression rzp = rrz - dotr * sz;
    expression rpn = heyoka::sqrt(rxp * rxp + ryp * ryp + rzp * rzp + 1e-24);
    expression rxu = rxp / rpn;
    expression ryu = ryp / rpn;
    expression rzu = rzp / rpn;

    // Orthonormal target basis: x=sun, z=x cross roll, y=z cross x.
    expression uzx = sy * rzu - sz * ryu;
    expression uzy = sz * rxu - sx * rzu;
    expression uzz = sx * ryu - sy * rxu;
    expression zpn = heyoka::sqrt(uzx * uzx + uzy * uzy + uzz * uzz + 1e-24);
    uzx = uzx / zpn; uzy = uzy / zpn; uzz = uzz / zpn;

    expression yx = uzy * sz - uzz * sy;
    expression yy = uzz * sx - uzx * sz;
    expression yz = uzx * sy - uzy * sx;

    // Target DCM rows: [x; y; z].
    expression TR00 = sx, TR01 = sy, TR02 = sz;
    expression TR10 = yx, TR11 = yy, TR12 = yz;
    expression TR20 = uzx, TR21 = uzy, TR22 = uzz;

    // Robust DCM -> quaternion, matching sad::dcm_to_quat (transpose included).
    expression tr = TR00 + TR11 + TR22;
    expression s0 = 2.0 * heyoka::sqrt(maxe(tr + 1.0, expression(1e-12)));
    expression qw0 = 0.25 * s0;
    expression qx0 = (TR12 - TR21) / s0;
    expression qy0 = (TR20 - TR02) / s0;
    expression qz0 = (TR01 - TR10) / s0;

    expression s1 = 2.0 * heyoka::sqrt(maxe(1.0 + TR00 - TR11 - TR22, expression(1e-12)));
    expression qw1 = (TR12 - TR21) / s1;
    expression qx1 = 0.25 * s1;
    expression qy1 = (TR01 + TR10) / s1;
    expression qz1 = (TR02 + TR20) / s1;

    expression s2 = 2.0 * heyoka::sqrt(maxe(1.0 - TR00 + TR11 - TR22, expression(1e-12)));
    expression qw2 = (TR20 - TR02) / s2;
    expression qx2 = (TR01 + TR10) / s2;
    expression qy2 = 0.25 * s2;
    expression qz2 = (TR12 + TR21) / s2;

    expression s3 = 2.0 * heyoka::sqrt(maxe(1.0 - TR00 - TR11 + TR22, expression(1e-12)));
    expression qw3 = (TR01 - TR10) / s3;
    expression qx3 = (TR02 + TR20) / s3;
    expression qy3 = (TR12 + TR21) / s3;
    expression qz3 = 0.25 * s3;

    expression c0 = heyoka::gt(tr, 0.0);
    expression c1 = heyoka::logical_and({heyoka::lte(tr, 0.0),
                                         heyoka::gt(TR00, TR11),
                                         heyoka::gt(TR00, TR22)});
    expression c2 = heyoka::logical_and({heyoka::lte(tr, 0.0),
                                         heyoka::gt(TR11, TR00),
                                         heyoka::gt(TR11, TR22)});

    expression qdx = heyoka::select(c0, qx0,
                      heyoka::select(c1, qx1,
                      heyoka::select(c2, qx2, qx3)));
    expression qdy = heyoka::select(c0, qy0,
                      heyoka::select(c1, qy1,
                      heyoka::select(c2, qy2, qy3)));
    expression qdz = heyoka::select(c0, qz0,
                      heyoka::select(c1, qz1,
                      heyoka::select(c2, qz2, qz3)));
    expression qdw = heyoka::select(c0, qw0,
                      heyoka::select(c1, qw1,
                      heyoka::select(c2, qw2, qw3)));

    // Normalize target quaternion.
    expression qdn = heyoka::sqrt(qdx * qdx + qdy * qdy + qdz * qdz + qdw * qdw + 1e-24);
    qdx = qdx / qdn; qdy = qdy / qdn; qdz = qdz / qdn; qdw = qdw / qdn;

    // Quaternion error e = vector part of qc * conj(qd), with shortest path.
    expression qex = -qw * qdx + qx * qdw - qy * qdz + qz * qdy;
    expression qey = -qw * qdy + qx * qdz + qy * qdw - qz * qdx;
    expression qez = -qw * qdz - qx * qdy + qy * qdx + qz * qdw;
    expression qew = qw * qdw + qx * qdx + qy * qdy + qz * qdz;

    expression sgn = heyoka::select(heyoka::lt(qew, 0.0), -1.0, 1.0);
    expression ex = sgn * qex;
    expression ey = sgn * qey;
    expression ez = sgn * qez;

    // Continuous PD + integral.
    expression tau_raw_x = -Kx * ex - Px * wx - Kix * zmax * heyoka::tanh(zx / zmax);
    expression tau_raw_y = -Ky * ey - Py * wy - Kiy * zmax * heyoka::tanh(zy / zmax);
    expression tau_raw_z = -Kz * ez - Pz * wz - Kiz * zmax * heyoka::tanh(zz / zmax);

    expression tau_cmd_x = tau_max * heyoka::tanh(tau_raw_x / tau_max);
    expression tau_cmd_y = tau_max * heyoka::tanh(tau_raw_y / tau_max);
    expression tau_cmd_z = tau_max * heyoka::tanh(tau_raw_z / tau_max);

    // Smooth pair allocator. Thrusts are positive; torque is preserved.
    expression s_x = heyoka::sqrt(tau_cmd_x * tau_cmd_x + eps * eps);
    expression Fpos_x = (s_x + tau_cmd_x) / (2.0 * 0.30);
    expression Fneg_x = (s_x - tau_cmd_x) / (2.0 * 0.30);

    expression s_y = heyoka::sqrt(tau_cmd_y * tau_cmd_y + eps * eps);
    expression Fpos_y = (s_y + tau_cmd_y) / (2.0 * 0.10);
    expression Fneg_y = (s_y - tau_cmd_y) / (2.0 * 0.10);

    expression s_z = heyoka::sqrt(tau_cmd_z * tau_cmd_z + eps * eps);
    expression Fpos_z = (s_z + tau_cmd_z) / (2.0 * 0.20);
    expression Fneg_z = (s_z - tau_cmd_z) / (2.0 * 0.20);

    // Torque produced by the smooth allocator equals tau_cmd in the unsaturated case.
    expression Tx = tau_cmd_x;
    expression Ty = tau_cmd_y;
    expression Tz = tau_cmd_z;

    // Gravity gradient torque with diagonal inertia.
    expression Rnorm = heyoka::sqrt(rx * rx + ry * ry + rz * rz);
    expression invR3 = 1.0 / (Rnorm * Rnorm * Rnorm);
    expression ax = -mu * rx * invR3;
    expression ay = -mu * ry * invR3;
    expression az = -mu * rz * invR3;

    expression rbx = R00 * rx + R01 * ry + R02 * rz;
    expression rby = R10 * rx + R11 * ry + R12 * rz;
    expression rbz = R20 * rx + R21 * ry + R22 * rz;

    expression Irx = Ixx * rbx;
    expression Iry = Iyy * rby;
    expression Irz = Izz * rbz;

    expression invR5 = invR3 / (Rnorm * Rnorm);
    expression kg = 3.0 * mu * invR5;
    expression tggx = kg * (rby * Irz - rbz * Iry);
    expression tggy = kg * (rbz * Irx - rbx * Irz);
    expression tggz = kg * (rbx * Iry - rby * Irx);

    // Euler equations with diagonal inertia.
    expression Iwx = Ixx * wx;
    expression Iwy = Iyy * wy;
    expression Iwz = Izz * wz;
    expression gx = -(wy * Iwz - wz * Iwy) + Tx + tggx;
    expression gy = -(wz * Iwx - wx * Iwz) + Ty + tggy;
    expression gz = -(wx * Iwy - wy * Iwx) + Tz + tggz;
    expression wdotx = gx * ixx;
    expression wdoty = gy * iyy;
    expression wdotz = gz * izz;

    // Quaternion kinematics.
    expression qdotx = 0.5 * (qw * wx + qy * wz - qz * wy);
    expression qdoty = 0.5 * (qw * wy - qx * wz + qz * wx);
    expression qdotz = 0.5 * (qw * wz + qx * wy - qy * wx);
    expression qdotw = 0.5 * (-qx * wx - qy * wy - qz * wz);

    // Smooth integral bound.
    expression zdotx = ex * (1.0 - heyoka::tanh(zx / zmax) * heyoka::tanh(zx / zmax));
    expression zdoty = ey * (1.0 - heyoka::tanh(zy / zmax) * heyoka::tanh(zy / zmax));
    expression zdotz = ez * (1.0 - heyoka::tanh(zz / zmax) * heyoka::tanh(zz / zmax));

    // Fuel rate.
    expression isp = expression(220.0);
    expression g0 = expression(9.80665);
    expression fuel_dot = (Fpos_x + Fneg_x + Fpos_y + Fneg_y + Fpos_z + Fneg_z) / (isp * g0);

    return {
        {heyoka::prime(rx) = vx, heyoka::prime(ry) = vy, heyoka::prime(rz) = vz,
         heyoka::prime(vx) = ax, heyoka::prime(vy) = ay, heyoka::prime(vz) = az,
         heyoka::prime(qx) = qdotx, heyoka::prime(qy) = qdoty,
         heyoka::prime(qz) = qdotz, heyoka::prime(qw) = qdotw,
         heyoka::prime(wx) = wdotx, heyoka::prime(wy) = wdoty, heyoka::prime(wz) = wdotz,
         heyoka::prime(zx) = zdotx, heyoka::prime(zy) = zdoty, heyoka::prime(zz) = zdotz,
         heyoka::prime(fuel) = fuel_dot}
    };
}

} // namespace

struct HeyokaTrack::Impl {
    std::vector<std::pair<expression, expression>> system;
    heyoka::taylor_adaptive<double> ta;

    Impl(double tol, bool compact)
        : system(build_system()),
          ta(system, {},
             heyoka::kw::tol = tol,
             heyoka::kw::compact_mode = compact,
             heyoka::kw::high_accuracy = false,
             heyoka::kw::fast_math = true,
             heyoka::kw::slp_vectorize = true,
             heyoka::kw::force_avx512 = true) {}
};

HeyokaTrack::HeyokaTrack(double tol, bool compact)
    : impl_(new Impl(tol, compact)) {}

HeyokaTrack::~HeyokaTrack() = default;

void HeyokaTrack::reset(const HeyokaTrackInit& init, double t0) {
    auto& ta = impl_->ta;
    ta.set_time(t0);

    double* st = ta.get_state_data();
    for (int i = 0; i < 3; ++i) st[i] = init.r[i];
    for (int i = 0; i < 3; ++i) st[3 + i] = init.v[i];
    st[6] = init.q[0]; st[7] = init.q[1]; st[8] = init.q[2]; st[9] = init.q[3];
    for (int i = 0; i < 3; ++i) st[10 + i] = init.w[i];
    for (int i = 0; i < 3; ++i) st[13 + i] = init.z[i];
    st[16] = init.fuel;

    double* p = ta.get_pars_data();
    for (int i = 0; i < 3; ++i) p[i] = init.roll_ref[i];
    for (int i = 0; i < 3; ++i) p[3 + i] = init.Keff[i];
    for (int i = 0; i < 3; ++i) p[6 + i] = init.Peff[i];
    for (int i = 0; i < 3; ++i) p[9 + i] = init.Kieff[i];
    p[12] = init.tau_max;
    p[13] = init.eps;
    p[14] = init.z_max;
    p[15] = init.inertia[0];
    p[16] = init.inertia[1];
    p[17] = init.inertia[2];
    p[18] = init.inv_inertia[0];
    p[19] = init.inv_inertia[1];
    p[20] = init.inv_inertia[2];
    p[21] = init.mass;
    p[22] = init.mu;
}

HeyokaTrackResult HeyokaTrack::propagate(double t_end, double sample_dt,
                                         const std::function<void(double, const std::array<double, 17>&)>& sample_cb) {
    (void)sample_dt;
    auto& ta = impl_->ta;
    HeyokaTrackResult out;

    if (sample_cb) {
        std::array<double, 17> st{};
        double* s = ta.get_state_data();
        for (int j = 0; j < 17; ++j) st[j] = s[j];
        sample_cb(ta.get_time(), st);
    }

    auto cb = [&sample_cb](heyoka::taylor_adaptive<double>& ta) -> bool {
        if (sample_cb) {
            std::array<double, 17> st{};
            double* s = ta.get_state_data();
            for (int j = 0; j < 17; ++j) st[j] = s[j];
            sample_cb(ta.get_time(), st);
        }
        return true;
    };

    double delta = t_end - ta.get_time();
    auto res = ta.propagate_for(delta, heyoka::kw::callback = cb, heyoka::kw::max_steps = 100000);
    out.steps = static_cast<int>(std::get<3>(res));
    out.min_step = std::get<1>(res);
    out.max_step = std::get<2>(res);
    double* s = ta.get_state_data();
    for (int j = 0; j < 17; ++j) out.final_state[j] = s[j];
    return out;
}

HeyokaTrack& heyoka_track_shared(double tol, bool compact) {
    struct CacheKey {
        double tol;
        bool compact;
        bool operator==(const CacheKey& o) const { return tol == o.tol && compact == o.compact; }
    };
    struct CacheKeyHash {
        std::size_t operator()(const CacheKey& k) const noexcept {
            std::size_t h = std::hash<double>{}(k.tol);
            h ^= std::hash<int>{}(k.compact ? 1 : 0) + 0x9e3779b9 + (h << 6) + (h >> 2);
            return h;
        }
    };
    thread_local std::unordered_map<CacheKey, std::unique_ptr<HeyokaTrack>, CacheKeyHash> cache;
    CacheKey key{tol, compact};
    auto it = cache.find(key);
    if (it == cache.end()) {
        it = cache.emplace(key, std::unique_ptr<HeyokaTrack>(new HeyokaTrack(tol, compact))).first;
    }
    return *it->second;
}

} // namespace sad
