// ISC License - see native_rw/LICENSE.
//
// extern "C" Taylor-propagator API over OmniSystem.
//
// WHY THIS EXISTS (and why the string bridge was abandoned)
// ---------------------------------------------------------
// heyoka's expression string form TRUNCATES at ~1000 characters, appending
// "...".  Verified: an expression of 1020 chars prints as 1012 chars ending in
// "...", with no error and no warning.
//
// This silently breaks the design where the C++ core marshals the symbolic
// system to Python as strings: the aligned hub+RW system fits (omx = 926
// chars) so it round-trips perfectly, but a TILTED-wheel configuration
// produces omx = 1009 chars, which is silently truncated and yields a WRONG
// symbolic system.  Any configuration complex enough to overflow is corrupt,
// not rejected.
//
// Therefore the propagator stays in C++ and Python drives it through this
// ABI.  No strings, no parsing, no truncation, and no per-step Python
// interpretation of the EOM.
#include <cmath>
#include <cstring>
#include <optional>
#include <memory>
#include <string>
#include <vector>

#include "omni/system.hpp"

extern "C" {

// Opaque handle. Owns an OmniSystem plus its JIT'd Taylor integrator.
struct OmniTaylor {
    omni::OmniSystem sys;
    // optional: cannot be constructed in the member-init list, because the
    // symbolic system does not exist until the contributors are registered.
    std::optional<heyoka::taylor_adaptive<double>> ta;
    std::vector<double> state;
    std::vector<double> pars;

    OmniTaylor() = default;
};

static thread_local std::string g_last_error;

const char* omni_last_error() {
    return g_last_error.empty() ? "" : g_last_error.c_str();
}

// Build an integrator from a JSON-free config passed as flat arrays.
// axes: 9 doubles (3 axes x 3), Js: ndoubles wheels.
// Returns nullptr on failure; see omni_last_error().
OmniTaylor* omni_taylor_create(const double* I9, double mass,
                               const double* axes, int n_wheels,
                               const double* Js, double mu,
                               int use_gg, double tol, int compact_mode) {
    try {
        g_last_error.clear();
        auto* h = new OmniTaylor();
        h->sys.set_gravity(mu);
        h->sys.set_use_gravity_gradient(use_gg != 0);
        std::vector<double> I9v(I9, I9 + 9);
        h->sys.add_hub(mass, I9v, {0.0, 0.0, 0.0});
        for (int i = 0; i < n_wheels; ++i) {
            h->sys.add_reaction_wheel({axes[3 * i], axes[3 * i + 1], axes[3 * i + 2]},
                                      Js[i], 0.0);
        }
        h->state.assign(h->sys.n_states(), 0.0);  // caller sets the ICs
        h->pars = h->sys.default_pars();
        const auto sysrows = h->sys.build_symbolic();
        h->ta.emplace(sysrows, h->state, heyoka::kw::pars = h->pars,
                      heyoka::kw::tol = tol,
                      heyoka::kw::compact_mode = (compact_mode != 0),
                      heyoka::kw::high_accuracy = false);
        return h;
    } catch (const std::exception& e) {
        g_last_error = e.what();
        return nullptr;
    }
}

void omni_taylor_destroy(OmniTaylor* h) { delete h; }

int omni_taylor_n_states(OmniTaylor* h) { return int(h->state.size()); }
int omni_taylor_n_pars(OmniTaylor* h) { return int(h->pars.size()); }

double* omni_taylor_state(OmniTaylor* h) { return h->state.data(); }
double* omni_taylor_pars(OmniTaylor* h) { return h->pars.data(); }

// One Taylor step. Returns min(|h|) actually used, or <0 on error.
double omni_taylor_step(OmniTaylor* h, double dt) {
    try {
        g_last_error.clear();
        // Push BOTH the state and the parameter vector into the integrator.
        // Omitting the pars was a real bug: the propagator was built once with
        // the pars of construction time, so later set_pars() calls (i.e. the
        // motor torque) silently never reached the EOM -- the sim ran with
        // zero control and diverged from stock by ~120 deg.
        double* sd = h->ta->get_state_data();
        for (size_t i = 0; i < h->state.size(); ++i) sd[i] = h->state[i];
        double* pd = h->ta->get_pars_data();
        for (size_t i = 0; i < h->pars.size(); ++i) pd[i] = h->pars[i];
        h->ta->set_time(0.0);
        h->ta->propagate_for(dt);
        h->state = h->ta->get_state();
        return dt;
    } catch (const std::exception& e) {
        g_last_error = e.what();
        return -1.0;
    }
}

// Numeric accelerations from the SAME contributor code (cross-check / oracle).
// Bulk run entirely in C++: the honest dynamics-slice cost, with no per-step
// Python overhead.  pars3 is applied every step (nullptr = leave pars alone).
double omni_taylor_run(OmniTaylor* h, int nsteps, double dt,
                       const double* pars3) {
    try {
        g_last_error.clear();
        double* sd = h->ta->get_state_data();
        double* pd = h->ta->get_pars_data();
        for (int k = 0; k < nsteps; ++k) {
            for (size_t i = 0; i < h->state.size(); ++i) sd[i] = h->state[i];
            if (pars3 != nullptr) {
                for (int i = 0; i < 3; ++i) pd[i] = pars3[i];
            }
            h->ta->set_time(0.0);
            h->ta->propagate_for(dt);
            h->state = h->ta->get_state();
        }
        return double(nsteps);
    } catch (const std::exception& e) {
        g_last_error = e.what();
        return -1.0;
    }
}

int omni_taylor_accel(OmniTaylor* h, const double* st, double* out) {
    try {
        g_last_error.clear();
        std::vector<double> s(st, st + h->sys.n_states());
        const std::vector<double> a = h->sys.accelerations(s, h->pars);
        for (size_t i = 0; i < a.size(); ++i) out[i] = a[i];
        return int(a.size());
    } catch (const std::exception& e) {
        g_last_error = e.what();
        return -1;
    }
}

int omni_taylor_set_default_state(OmniTaylor* h, const double* st) {
    try {
        g_last_error.clear();
        if (h->state.size() != size_t(omni_taylor_n_states(h))) return -2;
        for (size_t i = 0; i < h->state.size(); ++i) h->state[i] = st[i];
        return 0;
    } catch (const std::exception& e) {
        g_last_error = e.what();
        return -1;
    }
}

}  // extern "C"
