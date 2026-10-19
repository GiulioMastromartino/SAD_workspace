#pragma once

#include "sad_dynamics.h"

#include <memory>

namespace sad {

// Heyoka Taylor integrator for the same 13-state plant used by dynamics_step:
//   x = [r(3), v(3), q(4, scalar-last), omega(3)]
//
// The force/torque wrench and the (possibly dispersed) mass/inertia are
// parameters of the compiled Taylor system, so one integrator can be reused
// across all Monte Carlo lanes and across optimizer calls.  The public
// interface mirrors dynamics_step as closely as practical.
class HeyokaDynamics {
public:
    HeyokaDynamics(double tol = 1e-12, bool compact = false);
    ~HeyokaDynamics();

    HeyokaDynamics(const HeyokaDynamics&) = delete;
    HeyokaDynamics& operator=(const HeyokaDynamics&) = delete;

    // Reset time, state and plant parameters.  Usually called once per run.
    void reset(const DynamicsState& s);

    // Advance one zero-order-held force/torque interval.
    void step(const DynamicsInput& u, DynamicsState& s, DynamicsOutput& y, double dt);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Process/thread-local integrator cache, keyed by tolerance and compact mode.
// Reusing the compiled Taylor system avoids paying Heyoka's JIT cost for every
// optimizer candidate while keeping the state reset per mission run.
HeyokaDynamics& heyoka_dynamics_shared(double tol = 1e-12, bool compact = false);

} // namespace sad
