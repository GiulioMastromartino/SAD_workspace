#pragma once
#include "sad_types.h"
#include "sad_module.h"

namespace sad {
struct ThrusterParams {
    int n = 8;
    std::vector<Vec3> pos;   // body-frame positions
    std::vector<Vec3> dir;   // thrust directions (exhaust) unit
    std::vector<double> tmax; // max thrust per thruster [N]
    double isp = 220.0;
    double g0 = 9.80665;
};
struct ThrusterState {
    ThrusterParams params;
    std::vector<double> on_time_accum; // per-thruster accumulated on-time [s]
    double fuel_used = 0.0; // kg
};
struct ThrusterInput {
    std::vector<double> u; // throttle [0,1] per thruster (or on-time seconds)
    bool is_on_time = false; // if true, u is on-time seconds per dt
    double dt = 0.1;
};
struct ThrusterOutput {
    Vec3 force_b = Vec3::Zero();
    Vec3 torque_b = Vec3::Zero();
    double fuel_rate = 0.0; // kg/s
};

// Precomputed least-squares allocator.
//
// The influence matrix A (6 x n, [force; torque] = A * thrust) depends only on
// the thruster geometry, not on the commanded wrench. Building the JacobiSVD of
// A on every control step is therefore redundant: factor once per run and turn
// the per-step allocation into a single n x 6 mat-vec (plus the clamp/scale
// heuristic). Unlike the legacy one-shot path this keeps the hot loop free of
// dynamic allocations.
struct ThrusterAllocator {
    int n = 0;
    Eigen::MatrixXd pinv;   // n x 6, u = pinv * [Fdes; Tdes]
};

void thruster_init(ThrusterState& s, const ThrusterParams& p);
void thruster_step(const ThrusterInput& u, ThrusterState& s, ThrusterOutput& y, double dt);
void thruster_reset(ThrusterState& s);

// allocation helper: solve pinv for desired force+torque given thruster geometry
// returns throttle vector (size n) in [0,1]
std::vector<double> thruster_allocate(const ThrusterParams& p, const Vec3& Fdes, const Vec3& Tdes);

// Build the cached pseudo-inverse for this geometry (one JacobiSVD per call).
void thruster_allocator_init(ThrusterAllocator& a, const ThrusterParams& p);

// Hot path: u = pinv * b, then the same negative-clamp / saturation-scale /
// final-clamp sequence as thruster_allocate. Writes n values into `out`.
void thruster_allocate_cached_into(const ThrusterAllocator& a, const ThrusterParams& p,
                                   const Vec3& Fdes, const Vec3& Tdes, double* out);

// Convenience wrapper for callers that want a fresh vector.
std::vector<double> thruster_allocate_cached(const ThrusterAllocator& a, const ThrusterParams& p,
                                             const Vec3& Fdes, const Vec3& Tdes);
} // namespace sad
