#include "sad_dynamics_heyoka.h"

#include <heyoka/heyoka.hpp>

#include <cstddef>
#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace sad {
namespace {

using expression = heyoka::expression;

// Parameter layout shared by the Taylor system and reset()/step().
//   0..2   body force [N]
//   3..5   body torque [Nm]
//   6      mass [kg]
//   7..12  inertia, symmetric 3x3: Ixx Iyy Izz Ixy Ixz Iyz
//   13..18 inverse inertia, same layout
//   19     gravitational parameter mu [m^3/s^2]
//   20     gravity-gradient enable (0 or 1)
constexpr int P_FORCE = 0;
constexpr int P_TORQUE = 3;
constexpr int P_MASS = 6;
constexpr int P_INERTIA = 7;
constexpr int P_INV_INERTIA = 13;
constexpr int P_MU = 19;
constexpr int P_GG = 20;

static std::vector<std::pair<expression, expression>> build_system() {
    auto vars = heyoka::make_vars("rx", "ry", "rz", "vx", "vy", "vz",
                                  "qx", "qy", "qz", "qw",
                                  "wx", "wy", "wz");
    expression rx = vars[0], ry = vars[1], rz = vars[2];
    expression vx = vars[3], vy = vars[4], vz = vars[5];
    expression qx = vars[6], qy = vars[7], qz = vars[8], qw = vars[9];
    expression wx = vars[10], wy = vars[11], wz = vars[12];

    auto P = [](int i) { return expression(heyoka::par[i]); };
    expression Fx = P(P_FORCE + 0), Fy = P(P_FORCE + 1), Fz = P(P_FORCE + 2);
    expression Tx = P(P_TORQUE + 0), Ty = P(P_TORQUE + 1), Tz = P(P_TORQUE + 2);
    expression m = P(P_MASS);
    expression Ixx = P(P_INERTIA + 0), Iyy = P(P_INERTIA + 1), Izz = P(P_INERTIA + 2);
    expression Ixy = P(P_INERTIA + 3), Ixz = P(P_INERTIA + 4), Iyz = P(P_INERTIA + 5);
    expression ixx = P(P_INV_INERTIA + 0), iyy = P(P_INV_INERTIA + 1), izz = P(P_INV_INERTIA + 2);
    expression ixy = P(P_INV_INERTIA + 3), ixz = P(P_INV_INERTIA + 4), iyz = P(P_INV_INERTIA + 5);
    expression mu = P(P_MU);
    expression gg = P(P_GG);

    // Body-from-inertial DCM, exactly as sad::quat_to_dcm().
    expression R00 = 1.0 - 2.0 * (qy * qy + qz * qz);
    expression R01 = 2.0 * (qx * qy + qz * qw);
    expression R02 = 2.0 * (qx * qz - qy * qw);
    expression R10 = 2.0 * (qx * qy - qz * qw);
    expression R11 = 1.0 - 2.0 * (qx * qx + qz * qz);
    expression R12 = 2.0 * (qy * qz + qx * qw);
    expression R20 = 2.0 * (qx * qz + qy * qw);
    expression R21 = 2.0 * (qy * qz - qx * qw);
    expression R22 = 1.0 - 2.0 * (qx * qx + qy * qy);

    expression Rnorm = heyoka::sqrt(rx * rx + ry * ry + rz * rz);
    expression invR3 = 1.0 / (Rnorm * Rnorm * Rnorm);
    // R^T * F_body
    expression ax = -mu * rx * invR3 + (R00 * Fx + R10 * Fy + R20 * Fz) / m;
    expression ay = -mu * ry * invR3 + (R01 * Fx + R11 * Fy + R21 * Fz) / m;
    expression az = -mu * rz * invR3 + (R02 * Fx + R12 * Fy + R22 * Fz) / m;

    // Gravity-gradient torque, with the same full-inertia expression as
    // sad_dynamics.cpp.
    expression rbx = R00 * rx + R01 * ry + R02 * rz;
    expression rby = R10 * rx + R11 * ry + R12 * rz;
    expression rbz = R20 * rx + R21 * ry + R22 * rz;
    expression Irx = Ixx * rbx + Ixy * rby + Ixz * rbz;
    expression Iry = Ixy * rbx + Iyy * rby + Iyz * rbz;
    expression Irz = Ixz * rbx + Iyz * rby + Izz * rbz;
    expression invR5 = invR3 / (Rnorm * Rnorm);
    expression kg = 3.0 * mu * invR5 * gg;
    expression tggx = kg * (rby * Irz - rbz * Iry);
    expression tggy = kg * (rbz * Irx - rbx * Irz);
    expression tggz = kg * (rbx * Iry - rby * Irx);

    // Euler's equation: I*wdot + w x (I*w) = tau.
    expression Iwx = Ixx * wx + Ixy * wy + Ixz * wz;
    expression Iwy = Ixy * wx + Iyy * wy + Iyz * wz;
    expression Iwz = Ixz * wx + Iyz * wy + Izz * wz;
    expression gx = -(wy * Iwz - wz * Iwy) + Tx + tggx;
    expression gy = -(wz * Iwx - wx * Iwz) + Ty + tggy;
    expression gz = -(wx * Iwy - wy * Iwx) + Tz + tggz;
    expression wdotx = ixx * gx + ixy * gy + ixz * gz;
    expression wdoty = ixy * gx + iyy * gy + iyz * gz;
    expression wdotz = ixz * gx + iyz * gy + izz * gz;

    // Quaternion kinematics, matching sad::quat_derivative().
    expression qdotx = 0.5 * (qw * wx + qy * wz - qz * wy);
    expression qdoty = 0.5 * (qw * wy - qx * wz + qz * wx);
    expression qdotz = 0.5 * (qw * wz + qx * wy - qy * wx);
    expression qdotw = 0.5 * (-qx * wx - qy * wy - qz * wz);

    return {
        {heyoka::prime(rx) = vx, heyoka::prime(ry) = vy, heyoka::prime(rz) = vz,
         heyoka::prime(vx) = ax, heyoka::prime(vy) = ay, heyoka::prime(vz) = az,
         heyoka::prime(qx) = qdotx, heyoka::prime(qy) = qdoty,
         heyoka::prime(qz) = qdotz, heyoka::prime(qw) = qdotw,
         heyoka::prime(wx) = wdotx, heyoka::prime(wy) = wdoty,
         heyoka::prime(wz) = wdotz}
    };
}

struct CacheKey {
    double tol;
    bool compact;
    bool operator==(const CacheKey& o) const {
        return tol == o.tol && compact == o.compact;
    }
};

struct CacheKeyHash {
    std::size_t operator()(const CacheKey& k) const noexcept {
        std::size_t h = std::hash<double>{}(k.tol);
        h ^= std::hash<int>{}(k.compact ? 1 : 0) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

} // namespace

struct HeyokaDynamics::Impl {
    // Declared before ta: the Taylor integrator keeps a reference to the
    // expression system, so the system must outlive it.
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

HeyokaDynamics::HeyokaDynamics(double tol, bool compact)
    : impl_(new Impl(tol, compact)) {}

HeyokaDynamics::~HeyokaDynamics() = default;

void HeyokaDynamics::reset(const DynamicsState& s) {
    auto& ta = impl_->ta;
    ta.set_time(0.0);
    double* st = ta.get_state_data();
    for (int i = 0; i < 3; ++i) st[i] = s.r[i];
    for (int i = 0; i < 3; ++i) st[3 + i] = s.v[i];
    for (int i = 0; i < 4; ++i) st[6 + i] = s.q[i];
    for (int i = 0; i < 3; ++i) st[10 + i] = s.w[i];

    double* p = ta.get_pars_data();
    for (int i = 0; i < 6; ++i) p[P_FORCE + i] = 0.0;
    p[P_MASS] = s.params.mass;
    // Symmetric 3x3 inertia layout.
    p[P_INERTIA + 0] = s.params.inertia(0, 0);
    p[P_INERTIA + 1] = s.params.inertia(1, 1);
    p[P_INERTIA + 2] = s.params.inertia(2, 2);
    p[P_INERTIA + 3] = s.params.inertia(0, 1);
    p[P_INERTIA + 4] = s.params.inertia(0, 2);
    p[P_INERTIA + 5] = s.params.inertia(1, 2);

    p[P_INV_INERTIA + 0] = s.params.inv_inertia(0, 0);
    p[P_INV_INERTIA + 1] = s.params.inv_inertia(1, 1);
    p[P_INV_INERTIA + 2] = s.params.inv_inertia(2, 2);
    p[P_INV_INERTIA + 3] = s.params.inv_inertia(0, 1);
    p[P_INV_INERTIA + 4] = s.params.inv_inertia(0, 2);
    p[P_INV_INERTIA + 5] = s.params.inv_inertia(1, 2);
    p[P_MU] = s.params.mu;
    p[P_GG] = s.params.gravity_gradient ? 1.0 : 0.0;
}

void HeyokaDynamics::step(const DynamicsInput& u, DynamicsState& s, DynamicsOutput& y, double dt) {
    auto& ta = impl_->ta;
    double* p = ta.get_pars_data();
    for (int i = 0; i < 3; ++i) p[P_FORCE + i] = u.force_b[i];
    for (int i = 0; i < 3; ++i) p[P_TORQUE + i] = u.torque_b[i];

    ta.propagate_for(dt);
    double* st = ta.get_state_data();
    for (int i = 0; i < 3; ++i) s.r[i] = st[i];
    for (int i = 0; i < 3; ++i) s.v[i] = st[3 + i];
    Vec4 q(st[6], st[7], st[8], st[9]);
    q = quat_normalize(q);
    for (int i = 0; i < 4; ++i) {
        s.q[i] = q[i];
        st[6 + i] = q[i];  // keep the integrator state normalized too
    }
    for (int i = 0; i < 3; ++i) s.w[i] = st[10 + i];
    s.t += dt;

    y.r = s.r;
    y.v = s.v;
    y.q = s.q;
    y.w = s.w;
}

HeyokaDynamics& heyoka_dynamics_shared(double tol, bool compact) {
    thread_local std::unordered_map<CacheKey, std::unique_ptr<HeyokaDynamics>, CacheKeyHash> cache;
    CacheKey key{tol, compact};
    auto it = cache.find(key);
    if (it == cache.end()) {
        it = cache.emplace(key, std::unique_ptr<HeyokaDynamics>(
                                    new HeyokaDynamics(tol, compact))).first;
    }
    return *it->second;
}

} // namespace sad
