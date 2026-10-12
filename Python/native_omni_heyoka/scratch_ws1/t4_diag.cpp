#include "omni/system.hpp"
#include <iostream>
int main(){
    omni::OmniSystem sys;
    sys.set_gravity(3.986004418e14);
    sys.set_use_gravity_gradient(false);
    sys.add_hub(750.0, {900,0,0,0,800,0,0,0,600}, {0,0,0});
    for (const double* a : {std::array{1.,0.,0.}.data(), std::array{0.,1.,0.}.data(), std::array{0.,0.,1.}.data()})
        sys.add_reaction_wheel({a[0],a[1],a[2]}, 0.07957747154594767, 0.0);
    const auto P = sys.default_pars();
    std::vector<double> x = {1.0e7,2.0e6,-1.5e6,-700.,5500.,3200.,0.10,-0.20,0.30,0.01,-0.02,0.03,100.,200.,300.};
    const auto se = sys.build_symbolic();
    heyoka::taylor_adaptive<double> ta(se, heyoka::kw::pars=P, heyoka::kw::tol=1e-13, heyoka::kw::compact_mode=false);
    // fresh propagator, one step from x
    {
        double* sd = ta.get_state_data();
        for(int i=0;i<15;++i) sd[i]=x[i];
        ta.set_time(0.0);
        ta.propagate_for(0.01);
        const auto s1 = ta.get_state();
        std::cout << "step1 from x: " << s1[0] << " " << s1[3] << " " << s1[6] << "\n";
    }
    // Now from the SAME x but a brand new propagator (does state leak?)
    {
        double* sd = ta.get_state_data();
        for(int i=0;i<15;++i) sd[i]=x[i];
        ta.set_time(0.0);
        ta.propagate_for(0.01);
        const auto s2 = ta.get_state();
        std::cout << "step1 again : " << s2[0] << " " << s2[3] << " " << s2[6] << "  (should match)\n";
    }
    // RK4 substage: restart time and overwrite state
    {
        double* sd = ta.get_state_data();
        for(int i=0;i<15;++i) sd[i]=x[i];
        ta.set_time(0.0);
        ta.propagate_for(0.005);
        const auto sa = ta.get_state();
        double* sd2 = ta.get_state_data();
        for(int i=0;i<15;++i) sd2[i]=sa[i];
        ta.set_time(0.0);
        ta.propagate_for(0.01);
        const auto sb = ta.get_state();
        std::cout << "two 0.005/0.01 steps: " << sb[0] << " " << sb[3] << " " << sb[6] << "\n";
        std::cout << "  vs numeric substage: " << sys.state_derivative(sa,P)[0] << "\n";
    }
    return 0;
}
