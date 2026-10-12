// Gate H: symbolic graph vs numeric function on the COMBINED system
// (hub + 3 wheels + 12 thrusters). Taylor-advance (symbolic) must equal
// fine-RK4 (numeric state_derivative) to integrator accuracy; disagreement
// here means the two omni paths encode different ODEs.
#include "omni/system.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    std::cout << std::setprecision(17);
    omni::OmniSystem sys;
    sys.set_gravity(398600441800000.0);
    sys.set_use_gravity_gradient(true);
    sys.add_hub(20.0, {0.21666667,0,0, 0,0.21666667,0, 0,0,0.13333333}, {0,0,0});
    const double Js = 0.07957747154594767;
    sys.add_reaction_wheel({1,0,0}, Js, 0.0);
    sys.add_reaction_wheel({0,1,0}, Js, 0.0);
    sys.add_reaction_wheel({0,0,1}, Js, 0.0);
    double D[12][3] = {{-1,0,0},{-1,0,0},{1,0,0},{1,0,0},{0,0,-1},{0,0,-1},
                       {0,0,1},{0,0,1},{0,-1,0},{0,-1,0},{0,1,0},{0,1,0}};
    double P_[12][3] = {{0.1,0.1,0},{0.1,-0.1,0},{-0.1,0.1,0},{-0.1,-0.1,0},
                        {0.1,0,0.15},{-0.1,0,0.15},{0.1,0,-0.15},{-0.1,0,-0.15},
                        {0,0.1,0.15},{0,0.1,-0.15},{0,-0.1,0.15},{0,-0.1,-0.15}};
    std::vector<std::array<double,3>> d, p;
    std::vector<double> tm(12, 1.0);
    for (int i = 0; i < 12; ++i) {
        d.push_back({D[i][0], D[i][1], D[i][2]});
        p.push_back({P_[i][0], P_[i][1], P_[i][2]});
    }
    sys.add_thrusters(d, p, tm);
    std::cout << "states=" << sys.n_states() << " pars=" << sys.n_pars() << "\n";

    std::vector<double> x0 = {1.0e7, 0.0, 0.0, 0.0, 6300.0, 0.0,
                              0.1, 0.2, -0.3, 0.01, -0.02, 0.03,
                              0.0, 0.0, 0.0};
    std::vector<double> pars = sys.default_pars();
    // inputs: wheels-first registration -> [u1..u3, q0..q11]
    pars[0] = 0.01; pars[1] = -0.005; pars[2] = 0.008;

    // Taylor advance on the symbolic graph.
    const auto sysrows = sys.build_symbolic();
    heyoka::taylor_adaptive<double> ta(sysrows, x0, heyoka::kw::pars = pars,
                                       heyoka::kw::tol = 1e-14,
                                       heyoka::kw::compact_mode = false);
    std::vector<double> yt = x0;
    double* sd = ta.get_state_data();
    double* pd = ta.get_pars_data();
    const double dt = 0.1;
    for (int k = 0; k < 20; ++k) {
        for (size_t i = 0; i < yt.size(); ++i) sd[i] = yt[i];
        for (size_t i = 0; i < pars.size(); ++i) pd[i] = pars[i];
        ta.set_time(0.0);
        ta.propagate_for(dt);
        yt = ta.get_state();
        const double s2 = yt[6]*yt[6] + yt[7]*yt[7] + yt[8]*yt[8];
        if (s2 > 1.0) { yt[6] /= -s2; yt[7] /= -s2; yt[8] /= -s2; }
    }

    // Fine RK4 on the numeric function.
    std::vector<double> yn = x0;
    const double h = 1e-4;
    std::vector<double> k1, k2, k3, k4, ytmp;
    for (int k = 0; k < 20; ++k) {
        for (int s = 0; s < 1000; ++s) {
            k1 = sys.state_derivative(yn, pars);
            ytmp = yn;
            for (size_t i = 0; i < yn.size(); ++i) ytmp[i] += 0.5 * h * k1[i];
            k2 = sys.state_derivative(ytmp, pars);
            ytmp = yn;
            for (size_t i = 0; i < yn.size(); ++i) ytmp[i] += 0.5 * h * k2[i];
            k3 = sys.state_derivative(ytmp, pars);
            ytmp = yn;
            for (size_t i = 0; i < yn.size(); ++i) ytmp[i] += h * k3[i];
            k4 = sys.state_derivative(ytmp, pars);
            for (size_t i = 0; i < yn.size(); ++i)
                yn[i] += (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
            const double s2 = yn[6]*yn[6] + yn[7]*yn[7] + yn[8]*yn[8];
            if (s2 > 1.0) { yn[6] /= -s2; yn[7] /= -s2; yn[8] /= -s2; }
        }
    }
    double worst = 0.0;
    for (size_t i = 0; i < yt.size(); ++i)
        worst = std::max(worst, std::abs(yt[i] - yn[i]) / std::max(1.0, std::abs(yn[i])));
    std::cout << "[GATE H] Taylor(symbolic) vs RK4-fine(numeric), combined, 2 s: "
              << worst << "\n";
    if (!(worst < 1e-9)) { std::cout << "GATE H FAILED\n"; return 1; }
    std::cout << "GATE H: PASS\n";
    return 0;
}
