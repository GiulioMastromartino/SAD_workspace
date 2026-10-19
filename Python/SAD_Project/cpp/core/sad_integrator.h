#pragma once
#include "sad_types.h"
#include <functional>
#include <vector>

namespace sad {
// Generic RK4 integrator for a state vector x (size n) with RHS f(t,x,u).
using RhsFn = std::function<void(double t, const std::vector<double>& x, const std::vector<double>& u, std::vector<double>& dx)>;

inline void rk4_step(RhsFn f, double t, std::vector<double>& x, const std::vector<double>& u, double dt) {
    int n = x.size();
    std::vector<double> k1(n), k2(n), k3(n), k4(n), xt(n);
    f(t, x, u, k1);
    for (int i=0;i<n;++i) xt[i]=x[i]+0.5*dt*k1[i];
    f(t+0.5*dt, xt, u, k2);
    for (int i=0;i<n;++i) xt[i]=x[i]+0.5*dt*k2[i];
    f(t+0.5*dt, xt, u, k3);
    for (int i=0;i<n;++i) xt[i]=x[i]+dt*k3[i];
    f(t+dt, xt, u, k4);
    for (int i=0;i<n;++i) x[i]+= (dt/6.0)*(k1[i]+2*k2[i]+2*k3[i]+k4[i]);
}
} // namespace sad
