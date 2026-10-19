#pragma once
#include "sad_types.h"
#include "sad_timing.h"
#include "sad_dynamics.h"
#include "sad_actuator_thruster.h"
#include "sad_sensor_startracker.h"
#include "sad_env_sun.h"
#include "sad_controller_quat_pd.h"
#include "sad_mission_sequencer.h"
#include <array>
#include <cstdint>
#include <random>
#include <vector>

namespace sad {
struct BatchParams {
    // control gains
    double detumble_K=0.5, detumble_P=2.0, detumble_Ki=0.0;
    double slew_K=0.5, slew_P=2.0, slew_Ki=0.0;
    double track_K=0.5, track_P=2.0, track_Ki=0.0;
    double TS = 300.0;  // detumble+slew time budget (s)
    // orbit
    double R_ORBIT = 6978137.0; // 600km
    double MU = 3.986004418e14;
    // spacecraft
    double mass = 8.0;
    // dispersion flags
    bool disperse = true;
    // allocator path: true = precomputed pseudo-inverse (production),
    // false = legacy per-step JacobiSVD solve (kept for A/B validation)
    bool cached_alloc = true;
    // instrumentation
    bool timing = false;   // when true, collect the full timing registry
    // integration step during the track phase (s); detumble/slew always use 0.1
    double track_dt = 1.0;
    // step-aware (backward-Euler equivalent) PD gains; false = legacy explicit PD
    bool implicit_pd = true;
    // scale only the track phase (detumble/slew run at dt=0.1 and are stable)
    bool implicit_track_only = true;
    // per-axis body torque command limit (Nm); clipping well below the
    // allocator saturation region avoids clamp-induced slew limit cycles
    double tau_max = 0.1;
    // orbit injection dispersion (1-sigma-style uniform half-widths)
    double disp_r_m = 1000.0;    // +/- m on each ECI position axis
    double disp_v_mps = 0.5;    // +/- m/s on each ECI velocity axis
    // integrator backend: false = fixed-step RK4 (baseline), true = Heyoka
    bool use_heyoka = false;
    // thruster handling mode:
    //   0 = current pseudo-inverse + clamp allocator
    //   1 = direct commanded torque (no thruster allocation; fuel not comparable)
    //   2 = smooth torque-preserving pair allocator
    int thruster_mode = 0;
    double heyoka_tol = 1e-12;  // Taylor adaptive tolerance
    bool track_embedded = false;
    double track_sample_dt = 10.0;
    bool heyoka_compact = false;
};

// Fine-grained timing payload returned alongside the KPIs.
struct BatchTiming {
    bool enabled = false;
    double batch_ms = 0.0;
    long long steps = 0;
    std::array<long long, 3> phase_steps{};      // detumble, slew, track
    std::array<double, TCount> ns{};
    std::array<long long, TCount> calls{};
    std::array<double, TCount * 3> phase_ns{};   // [slot*3 + phase]
    std::array<long long, TCount * 3> phase_calls{};
    std::vector<double> run_ms;                  // per-run wall time
    std::vector<double> run_setup_ms;            // per-run construction
    std::vector<double> run_dispersion_ms;       // per-run dispersion draws
    std::vector<int> run_steps;                  // per-run integration steps
    std::vector<std::array<int, 3>> run_phase_steps; // per-run steps per phase
    long long heyoka_track_steps = 0;
    double heyoka_track_min_step = 0.0;
    double heyoka_track_max_step = 0.0;
    int failed_runs = 0;
    int reached_runs = 0;
};

struct BatchResult {
    std::vector<double> final_rate;
    std::vector<double> fuel_used;
    std::vector<double> detumble_settle;
    std::vector<double> slew_settle;
    std::vector<double> orbit_err;
    std::vector<double> box_max;
    // track pointing: body +X angle to the sun, degrees, over the track phase
    std::vector<double> track_ang_mean;
    std::vector<double> track_ang_max;
    std::vector<int> reached_track;
    std::vector<int> failed;
    BatchTiming timing;
};

void run_mission_batch(int N, unsigned seed, const BatchParams &bp, BatchResult &out);

// True when the library was compiled with the optional Heyoka backend.
bool heyoka_available();
} // namespace sad
