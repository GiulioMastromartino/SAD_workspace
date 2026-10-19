#include "sad_actuator_thruster.h"
#include "sad_timing.h"
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>

namespace sad {

void thruster_init(ThrusterState& s, const ThrusterParams& p) {
    s.params = p;
    s.on_time_accum.assign(p.n, 0.0);
    s.fuel_used = 0.0;
}

namespace {

// Shared post-processing: negative clamp, direction-preserving saturation
// scaling, then a final per-thruster clamp. `u` and `out` may alias.
inline void clamp_scale_copy(double* u, int n, const ThrusterParams& p, double* out) {
    // clamp negatives to 0
    for (int i=0;i<n;++i) if (u[i] < 0) u[i] = 0;
    // scale down if any thruster exceeds tmax (preserve direction)
    double scale = 1.0;
    for (int i=0;i<n;++i) {
        if (u[i] > 1e-12 && u[i] > p.tmax[i]) {
            double s = p.tmax[i] / u[i];
            if (s < scale) scale = s;
        }
    }
    if (scale < 1.0) for (int i=0;i<n;++i) u[i] *= scale;
    // final clamp (safety)
    for (int i=0;i<n;++i){
        double t = u[i];
        if (t < 0) t = 0;
        if (t > p.tmax[i]) t = p.tmax[i];
        out[i] = t;
    }
}

constexpr int kStackThrusters = 64;  // stack fast path; heap fallback above this

} // namespace

std::vector<double> thruster_allocate(const ThrusterParams& p, const Vec3& Fdes, const Vec3& Tdes) {
    SAD_TIME(TAlloc);
    int n = p.n;
    Eigen::MatrixXd A(6, n);
    Eigen::VectorXd b(6);
    {
        SAD_TIME(TAllocBuildA);
        for (int i=0;i<n;++i){
            Vec3 d = p.dir[i].normalized();
            Vec3 tau = p.pos[i].cross(d);
            for (int k=0;k<3;++k){ A(k,i)=d[k]; A(3+k,i)=tau[k]; }
        }
        for (int k=0;k<3;++k){ b[k]=Fdes[k]; b[3+k]=Tdes[k]; }
    }
    // pseudo-inverse via SVD
    Eigen::VectorXd u;
    {
        SAD_TIME(TAllocSVD);
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        u = svd.solve(b);
    }
    std::vector<double> thr(n);
    {
        SAD_TIME(TAllocClamp);
        clamp_scale_copy(u.data(), n, p, thr.data());
    }
    return thr;
}

void thruster_allocator_init(ThrusterAllocator& a, const ThrusterParams& p) {
    SAD_TIME(TAllocInit);
    const int n = p.n;
    Eigen::MatrixXd A(6, n);
    {
        SAD_TIME(TAllocBuildA);
        for (int i=0;i<n;++i){
            Vec3 d = p.dir[i].normalized();
            Vec3 tau = p.pos[i].cross(d);
            for (int k=0;k<3;++k){ A(k,i)=d[k]; A(3+k,i)=tau[k]; }
        }
    }
    a.n = n;
    {
        SAD_TIME(TAllocSVD);
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // Minimum-norm least-squares inverse: V_r * diag(1/s_r) * U_r^T.
        // rank() uses the same singularity threshold as JacobiSVD::solve(), so
        // the cached path reproduces the legacy results (up to rounding).
        const int r = svd.rank();
        const auto& sv = svd.singularValues();
        a.pinv = Eigen::MatrixXd::Zero(n, 6);
        for (int i=0;i<r;++i)
            a.pinv.noalias() += (1.0/sv(i)) * svd.matrixV().col(i) * svd.matrixU().col(i).transpose();
    }
}

void thruster_allocate_cached_into(const ThrusterAllocator& a, const ThrusterParams& p,
                                   const Vec3& Fdes, const Vec3& Tdes, double* out) {
    SAD_TIME(TAlloc);
    const int n = a.n;
    double b[6];
    for (int k=0;k<3;++k){ b[k]=Fdes[k]; b[3+k]=Tdes[k]; }

    double u_stack[kStackThrusters];
    std::vector<double> u_heap;
    double* u = u_stack;
    if (n > kStackThrusters) { u_heap.resize(n); u = u_heap.data(); }

    {
        SAD_TIME(TAllocSolve);
        // pinv is n x 6 column-major: P(i,k) = P[i + k*n].
        const double* P = a.pinv.data();
        for (int i=0;i<n;++i) {
            double s = 0.0;
            for (int k=0;k<6;++k) s += P[i + k*n] * b[k];
            u[i] = s;
        }
    }
    {
        SAD_TIME(TAllocClamp);
        clamp_scale_copy(u, n, p, out);
    }
}

std::vector<double> thruster_allocate_cached(const ThrusterAllocator& a, const ThrusterParams& p,
                                             const Vec3& Fdes, const Vec3& Tdes) {
    std::vector<double> thr(a.n);
    thruster_allocate_cached_into(a, p, Fdes, Tdes, thr.data());
    return thr;
}

void thruster_step(const ThrusterInput& u, ThrusterState& s, ThrusterOutput& y, double dt) {
    SAD_TIME(TThrusterStep);
    const ThrusterParams& p = s.params;
    int n = p.n;
    double thr_stack[kStackThrusters];
    std::vector<double> thr_heap;
    double* thr = thr_stack;
    if (n > kStackThrusters) { thr_heap.assign(n, 0.0); thr = thr_heap.data(); }
    if (u.is_on_time) {
        // u.u are on-time seconds within dt
        for (int i=0;i<n && i<(int)u.u.size();++i) thr[i] = u.u[i];
    } else {
        // throttle [0,1] -> thrust
        for (int i=0;i<n && i<(int)u.u.size();++i) thr[i] = u.u[i]*p.tmax[i];
    }
    // if u.u is shorter than n, zero the tail (the stack buffer is not initialized)
    for (int i=(int)std::min<size_t>(u.u.size(), (size_t)n); i<n; ++i) thr[i] = 0.0;
    Vec3 F(0,0,0), T(0,0,0);
    double mdot = 0.0;
    SAD_TIME(TThrNorm);
    for (int i=0;i<n;++i){
        double thrust = thr[i];
        if (!u.is_on_time) thrust = thr[i]; // already thrust
        // for on_time: thrust = (on_time/dt)*tmax
        if (u.is_on_time) thrust = (dt>0)? (thr[i]/dt)*p.tmax[i] : 0.0;
        if (thrust < 0) thrust = 0;
        if (thrust > p.tmax[i]) thrust = p.tmax[i];
        Vec3 d = p.dir[i].normalized();
        F += thrust * d;
        T += p.pos[i].cross(thrust * d);
        mdot += thrust / (p.isp * p.g0);
        s.on_time_accum[i] += (u.is_on_time ? thr[i] : 0.0);
    }
    y.force_b = F;
    y.torque_b = T;
    y.fuel_rate = mdot;
    s.fuel_used += mdot*dt;
}

void thruster_reset(ThrusterState& s) {
    for (auto& a : s.on_time_accum) a = 0.0;
    s.fuel_used = 0.0;
}
} // namespace sad
