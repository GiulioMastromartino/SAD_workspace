// Dump the TILTED-wheel symbolic system (3 arbitrary spin axes) for the
// Python driver. Same contributors, same solve, different gs vectors.
#include "omni/system.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>

int main() {
    const double s3 = 0.5773502691896258;
    omni::OmniSystem sys;
    sys.set_gravity(3.986004418e14);
    sys.set_use_gravity_gradient(false);
    sys.add_hub(750.0, {900, 0, 0, 0, 800, 0, 0, 0, 600}, {0, 0, 0});
    for (const double* a : {std::array{s3, s3, s3}.data(),
                            std::array{-s3, s3, s3}.data(),
                            std::array{s3, -s3, s3}.data()}) {
        sys.add_reaction_wheel({a[0], a[1], a[2]}, 0.07957747154594767, 0.0);
    }
    std::cout << "tilted: states=" << sys.n_states() << " pars=" << sys.n_pars() << "\n";
    for (auto n : sys.par_names()) std::cout << " " << n;
    std::cout << "\n";
    const auto rhs = sys.build_symbolic_rhs_strings();
    std::ofstream f("/tmp/omni_tilted_dump.txt");
    for (size_t i = 0; i < rhs.size(); ++i)
        f << sys.state_names()[i] << " = " << rhs[i] << "\n";
    f.close();
    std::cout << "wrote /tmp/omni_tilted_dump.txt\n";
    return 0;
}
