#pragma once
// SAD_Project lightweight timing registry.
// Purpose: fine-grained, low-overhead attribution of wall time inside the C++
// mission-batch kernel so we can see exactly where the seconds go.
//
// Design:
//  * A thread_local accumulator (one per worker process) collects ns + call
//    counts per TimingSlot, additionally bucketed by mission phase
//    (0=detumble, 1=slew, 2=track).
//  * Any module records time with the SAD_TIME(slot) scoped timer macro.
//  * Timing is opt-in: g_timing.enabled is set by the batch kernel from
//    BatchParams::timing, so production runs pay nothing.

#include <array>
#include <chrono>
#include <cstdint>

namespace sad {

enum TimingSlot : int {
    // ---- top-level subsystem calls (batch loop) ----
    TSun = 0,        // sun_step (whole)
    TSunPointQuat,   // sun_pointing_quaternion only
    TStarTracker,    // startracker_step (whole)
    TMission,        // mission_step (whole)
    TQuatPD,         // quatpd_step (whole)
    TAlloc,          // thruster_allocate (whole)
    TAllocBuildA,    //   building the 6xN influence matrix A
    TAllocSVD,       //   JacobiSVD construction + solve (one-shot / init only)
    TAllocClamp,     //   negative clamp + saturation scaling
    TAllocSolve,     //   cached-pinv mat-vec (hot path)
    TAllocInit,      // one-time cached allocator construction (SVD + pinv)
    TThrusterStep,   // thruster_step (whole)
    TThrNorm,        //   per-thruster dir.normalized()
    TDynamics,       // dynamics_step (whole, includes RK4)
    TDynamicsAlloc,  //   std::vector allocations in dynamics_step
    TRhsTotal,       // rhs() (all 4 RK4 stages)
    TRhsDcm,         //   quat_to_dcm inside rhs
    TRhsGravityGrad, //   gravity-gradient torque inside rhs
    TRhsQuatDeriv,   //   quat_derivative inside rhs
    TKpi,            // per-step KPI bookkeeping (cone check, orbit err push)
    // ---- batch orchestration ----
    TDispersion,     // per-run dispersion draws
    TPerRunSetup,    // per-run state/param construction
    TLoopOverhead,   // whole iteration minus nothing booked (residual)
    TCount
};

const char* timing_slot_name(int slot);

inline double sad_now_ns() {
    return static_cast<double>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

struct TimingAccum {
    std::array<double, TCount> ns{};
    std::array<long long, TCount> calls{};
    std::array<double, TCount * 3> phase_ns{};   // [slot*3 + phase]
    std::array<long long, TCount * 3> phase_calls{};
    std::array<long long, 3> phase_steps{};
    int phase = 0;            // 0 detumble, 1 slew, 2 track
    bool enabled = false;
    long long steps = 0;

    void reset() {
        ns.fill(0.0);
        calls.fill(0);
        phase_ns.fill(0.0);
        phase_calls.fill(0);
        phase_steps.fill(0);
        phase = 0;
        steps = 0;
    }
    inline void add(int slot, double dt_ns) {
        ns[slot] += dt_ns;
        calls[slot] += 1;
        if (phase >= 0 && phase < 3) {
            phase_ns[slot * 3 + phase] += dt_ns;
            phase_calls[slot * 3 + phase] += 1;
        }
    }
};

extern thread_local TimingAccum g_timing;

// Scoped timer. Records on destruction; no-op when timing disabled
// (still one branch + one clock read when enabled).
struct ScopedTimer {
    int slot;
    double t0;
    explicit ScopedTimer(int s) : slot(s) {
        t0 = g_timing.enabled ? sad_now_ns() : 0.0;
    }
    ~ScopedTimer() {
        if (g_timing.enabled) g_timing.add(slot, sad_now_ns() - t0);
    }
    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;
};

} // namespace sad

#define SAD_CONCAT_(a, b) a##b
#define SAD_CONCAT(a, b) SAD_CONCAT_(a, b)
#define SAD_TIME(slot) ::sad::ScopedTimer SAD_CONCAT(_sad_timer_, __LINE__)(slot)
