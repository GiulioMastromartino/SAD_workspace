#include "sad_dynamics.h"
#include "sad_timing.h"
#include <cmath>
#include <vector>

namespace sad {

void dynamics_init(DynamicsState& s, const DynamicsParams& p) {
    s.params = p;
    s.r = p.r0; s.v = p.v0; s.q = quat_normalize(p.q0); s.w = p.w0; s.t = 0.0;
}

static void rhs(const DynamicsParams& p, const std::vector<double>& x, const std::vector<double>& u, std::vector<double>& dx) {
    SAD_TIME(TRhsTotal);
    Vec3 r(x[0],x[1],x[2]);
    Vec3 v(x[3],x[4],x[5]);
    Vec4 q(x[6],x[7],x[8],x[9]);
    Vec3 w(x[10],x[11],x[12]);
    q = quat_normalize(q);
    Vec3 Fb(u[0],u[1],u[2]);
    Vec3 Tb(u[3],u[4],u[5]);

    double Rn = r.norm();
    if (Rn < 1e3) Rn = 1e3;
    Vec3 a_grav = -p.mu * r / (Rn*Rn*Rn);

    Mat3 Rbi; { SAD_TIME(TRhsDcm); Rbi = quat_to_dcm(q); }
    Mat3 Rib = Rbi.transpose();
    Vec3 Fi = Rib * Fb;
    Vec3 ai = Fi / p.mass + a_grav;

    Vec3 tgg = Vec3::Zero();
    if (p.gravity_gradient) {
        SAD_TIME(TRhsGravityGrad);
        Vec3 rb = Rbi * r;
        double invR5 = 1.0/(Rn*Rn*Rn*Rn*Rn);
        double k = 3.0*p.mu*invR5;
        Vec3 Ir = p.inertia * rb;
        tgg = k * rb.cross(Ir);
    }
    Vec3 tau_b = Tb + tgg;

    Vec3 Iw = p.inertia * w;
    Vec3 wdot = p.inv_inertia * (tau_b - w.cross(Iw));

    Vec4 qdot; { SAD_TIME(TRhsQuatDeriv); qdot = quat_derivative(q, w); }

    dx[0]=v[0]; dx[1]=v[1]; dx[2]=v[2];
    dx[3]=ai[0]; dx[4]=ai[1]; dx[5]=ai[2];
    dx[6]=qdot[0]; dx[7]=qdot[1]; dx[8]=qdot[2]; dx[9]=qdot[3];
    dx[10]=wdot[0]; dx[11]=wdot[1]; dx[12]=wdot[2];
}

void dynamics_step(const DynamicsInput& u, DynamicsState& s, DynamicsOutput& y, double dt) {
    SAD_TIME(TDynamics);
    const DynamicsParams& p = s.params;
    double _t_alloc0 = g_timing.enabled ? sad_now_ns() : 0.0;
    std::vector<double> x(13);
    for (int i=0;i<3;++i){ x[i]=s.r[i]; x[3+i]=s.v[i]; }
    for (int i=0;i<4;++i) x[6+i]=s.q[i];
    for (int i=0;i<3;++i) x[10+i]=s.w[i];
    std::vector<double> uu(6);
    for (int i=0;i<3;++i){ uu[i]=u.force_b[i]; uu[3+i]=u.torque_b[i]; }
    int n=13;
    std::vector<double> k1(n),k2(n),k3(n),k4(n),xt(n);
    if (g_timing.enabled) g_timing.add(TDynamicsAlloc, sad_now_ns() - _t_alloc0);
    auto ev=[&](const std::vector<double>& xv, std::vector<double>& dxv){ rhs(p, xv, uu, dxv); };
    ev(x,k1);
    for(int i=0;i<n;++i) xt[i]=x[i]+0.5*dt*k1[i];
    ev(xt,k2);
    for(int i=0;i<n;++i) xt[i]=x[i]+0.5*dt*k2[i];
    ev(xt,k3);
    for(int i=0;i<n;++i) xt[i]=x[i]+dt*k3[i];
    ev(xt,k4);
    for(int i=0;i<n;++i) x[i]+=(dt/6.0)*(k1[i]+2*k2[i]+2*k3[i]+k4[i]);

    for (int i=0;i<3;++i){ s.r[i]=x[i]; s.v[i]=x[3+i]; }
    for (int i=0;i<4;++i) s.q[i]=x[6+i];
    for (int i=0;i<3;++i) s.w[i]=x[10+i];
    s.q = quat_normalize(s.q);
    s.t += dt;
    y.r=s.r; y.v=s.v; y.q=s.q; y.w=s.w;
}

void dynamics_reset(DynamicsState& s){ s.r.setZero(); s.v.setZero(); s.q=Vec4(0,0,0,1); s.w.setZero(); s.t=0; }

} // namespace sad
