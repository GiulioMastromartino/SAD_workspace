# Diagnostics

Timing instrumentation for the C++ mission-batch kernel and the CMA-ES pipeline.

## How it works

* `cpp/core/sad_timing.{h,cpp}` — a `thread_local` registry (`sad::g_timing`) with
  per-slot ns/call counters, additionally bucketed by mission phase
  (0 = detumble, 1 = slew, 2 = track).
* Every subsystem records itself with the `SAD_TIME(slot)` scoped timer
  (dynamics, RK4 `rhs` and sub-parts, thruster allocation and its cached
  pseudo-inverse mat-vec, thruster step, sun, star tracker, mission,
  controller, KPI).
* Timing is **opt-in**: `sad_py.BatchParams.timing = True`. Production/optimizer
  runs pay zero overhead.
* `sad_py.run_mission_batch(...)` returns a `timing` dict with:
  `ns`, `calls`, `phase_ns`, `phase_calls`, `steps`, `phase_steps`,
  `run_ms`, `run_steps`, `run_phase_steps`, `batch_ms`.

Instrumentation costs ~35% when enabled (clock reads at ~460k steps for
N=50). Keep it off for real optimization runs.

## Scripts

Run from `Python/` with the usual `PYTHONPATH`:

```bash
export PYTHONPATH=$PWD/SAD_Project/python:$PWD
env/bin/python -m SAD_Project.diagnostics.profile_batch --runs 50
env/bin/python -m SAD_Project.diagnostics.profile_batch --runs 50 --track-dt 1.0
env/bin/python -m SAD_Project.diagnostics.profile_batch --runs 50 --track-dt 1.0 --legacy-pd
env/bin/python -m SAD_Project.diagnostics.profile_pipeline --workers 8
env/bin/python -m SAD_Project.diagnostics.bench_one_gen
env/bin/python SAD_Project/diagnostics/bench_integrators.py --N 300 --repeats 3
```

`profile_batch` prints the subsystem table, per-phase attribution, per-run
wall-time distribution, and a ranked "where to deep dive" list; it writes
`results_diagnostics/batch_profile.{json,png}`.  `--legacy-pd` disables the
implicit scaling, `--implicit-all` scales detumble/slew too (default is track
only).

`profile_pipeline` measures import cost, `evaluate()` scaling (kernel vs Python
overhead), one real CMA-ES generation (ask/map/tell, worker-sum, parallel
efficiency), and projects full-run wall time per worker count.

`bench_one_gen` runs two real generations (cold + steady) at pop 15, N=50,
8 workers and prints the per-subprocess breakdown.

## Integrator backends: RK4 and Heyoka

The mission batch now has two plant integrators behind the same C++ loop,
12-thruster allocator, quaternion PD, mission sequencer and dispersion model:

* **RK4** (default): fixed step, 0.1 s in detumble/slew and `TRACK_DT` in track.
* **Heyoka**: adaptive Taylor method, one `propagate_for(dt)` per control step.
  The 13-state quaternion plant is compiled once and its mass, inertia,
  inverse inertia, gravity-gradient flag, force and torque are parameters, so
  the same integrator is reused across Monte Carlo runs and optimizer calls.
  A thread-local cache avoids paying the JIT cost once per candidate.

CMake finds Heyoka in `Python/env` and enables it by default:

```bash
cmake -S SAD_Project -B SAD_Project/build -DSAD_WITH_HEYOKA=ON
```

If Heyoka is absent the RK4-only library still builds.  Python exposes
`sad_py.heyoka_available()` and the `BatchParams` fields `use_heyoka`,
`heyoka_tol` (default `1e-12`) and `heyoka_compact` (default `False`).

Benchmark (N=300, seed 42, three repeats, JIT warm-up excluded, Release build):

```bash
env/bin/python SAD_Project/diagnostics/bench_integrators.py \
    --N 300 --repeats 3 --seed 42
```

| backend | wall time (s) | ms/run | relative |
|---|---:|---:|---:|
| RK4 | 2.710 | 9.03 | 1.00x |
| Heyoka, tol 1e-12 | 5.082 | 16.94 | 1.88x slower |
| Heyoka, tol 1e-9 | 4.007 | 13.36 | 1.52x slower |
| Heyoka, tol 1e-12, compact | 12.495 | 41.65 | 4.75x slower |

Both backends reach track on 300/300 runs with zero failures.  With identical
seeds and dispersions the KPI differences are at integration-noise level:
final rate `8.9e-12 rad/s`, fuel `1.1e-10 kg`, `box_max` `4.6e-10 km`, and
maximum track angle `1.8e-6 deg`.  In other words, at this mission's 0.1 s /
1.0 s control steps the fixed-step RK4 is already accurate; Heyoka is useful
as a high-accuracy reference and parity check, not as a throughput win.
Compact mode is counterproductive here.  Results are saved under
`results_bench/`; `tests/test_heyoka_backend.py` checks the parity and cached
reuse.

`bench_allocator.cpp` is a native microbenchmark (no Python/timer overhead)
for the legacy per-step SVD vs the cached pseudo-inverse:

```bash
c++ -O3 -DNDEBUG -std=gnu++17 -arch arm64 -Icpp -Icpp/core -Icpp/actuators \
    -I../env/lib/python3.11/site-packages/bsk_sdk/include/eigen3 \
    diagnostics/bench_allocator.cpp \
    build/cpp/actuators/libsad_actuators.a build/libsad_core.a -o /tmp/bench_alloc
/tmp/bench_alloc
```

## Thruster allocator

`A` (6 x n, `[force; torque] = A * thrust`) depends only on the thruster
geometry. `thruster_allocator_init()` factors it once per run
(`ThrusterAllocator::pinv`, n x 6) and
`thruster_allocate_cached_into()` performs only a mat-vec + the clamp/scale
heuristic per step. The legacy `thruster_allocate()` is retained for one-shot
calls and A/B validation (`BatchParams.cached_alloc = False`).

For the standard 12-thruster geometry (4 on ±X, 4 on ±Z, 4 on ±Y, at the
face edges) `A` has full rank 6:

```
singular values [2, 2, 2, 0.3, 0.2, 0.1], cond = 20
geometric ± authority: |F| = 1.0 N, Tx = 0.15 Nm, Ty = 0.05 Nm, Tz = 0.10 Nm
```

The cached path still uses the same min-norm pseudo-inverse plus
negative-clamp/scale heuristic; for a pure wrench the clamp halves the
requested value (e.g. Fx = 0.5 N), but every axis now has both signs.  Cached
vs one-shot agree to 5.6e-17 N. Isolated cost: **~1.5 us -> 38 ns/call (~40x)**.
In the full kernel: **21.4 -> 6.2 ms/run (3.5x)**, because the cached path also
removes four heap allocations per control step.

## Attitude frame and sun target

The quaternion kinematics use `q_dot = 0.5 q ⊗ [omega, 0]` (body-from-inertial
quaternion), but `quat_to_dcm` previously returned the transpose
(inertial-from-body) matrix.  The sun-pointing target was built in the opposite
convention, so the track controller saw a stable 180° roll error.  Both C++
and Python now return the body-from-inertial DCM, and the sun target's free
roll is pinned to the current body +Y instead of a fixed inertial reference.
With the standard 12-thruster layout this changed the N=30 track error from
~3.14 rad and 0% cone compliance to **0.000174 rad mean (0.0100°), 100% cone
compliance** after the box/cone-cost optimizer run.

## Interpreting nesting

Indented rows are children of the row above (e.g. `alloc_jacobiSVD` is inside
`thruster_allocate`). The `excl.ms` column subtracts immediate children so you
can sum the exclusive times without double counting.

## Implicit PD (step-aware gains)

The track loop runs at `dt = 1.0`.  The legacy explicit PD has a damping map
`1 - dt*P/I`, stable only for `P < 2I/dt` (0.067 on z).  Defaults and the
optimized gains are far outside that, which produced the saturated limit cycle
(final rate ~1.5 rad/s).

`quatpd_step` now scales the per-axis gains by

```
D     = 1 + dt*P/I + 0.5*dt^2*K/I
K_eff = K / D
P_eff = (P + 0.5*dt*K) / D
Ki_eff = Ki / D
```

which makes the explicit torque algebraically equivalent to a backward-Euler
update of the linearized loop (unconditionally stable).  The scaling is applied
to the **track phase only** by default: detumble/slew run at `dt=0.1`, are
already stable, and scaling them slowed the slew and raised the pre-existing
slew-timeout rate.  `BatchParams.implicit_pd` and
`BatchParams.implicit_track_only` expose the A/B switches.

Result (N=300, dt=1.0, 250-gen gains): track end-rate **1.545 -> 0.046 rad/s**,
success-only median cost **3.00 -> 1.41**, failure count unchanged.  `dt=1.0`
and `dt=0.1` now give the same result, so the 10x step cost is gone.

## Standard 12-thruster configuration (2026-10-19)

`core/config.py`, the batch kernel (`NTHR=12`) and the geometry test now use
the 12-thruster 6-DOF layout ported from `cubesat_12u_sim`: four thrusters on
each ±X face (push ±X), four on each ±Z face (push ±Z), four on each ±Y face
(push ±Y), mounted at the face edges.  Tmax stays 0.5 N per thruster for the
6U bus.  The layout is rank 6 with ± authority on every axis, so the previous
missing `+Tx`, `Fy` and `Ty` authority is gone.

### Pre-optimizer A/B (old 4x250 gains)

With the 12-thruster layout and the pre-optimizer gains, the 0.1 Nm command
limit already improved the N=300 batch from 57/300 and 63/300 failures at
0.2 Nm to **34/300 (11.3%)** and **40/300 (13.3%)** at seeds 123/321.  All
remaining failures were slew timeouts, which motivated the fresh optimizer
run below.

The body torque command is clamped per axis to `ACS_TAU_MAX = 0.1 Nm`
(`BatchParams.tau_max`).  With the 12-thruster allocator, 0.2 Nm saturates the
clamp/scale path for many slew steps and produces limit-cycle timeouts.

### First 4x250 run (old cost, no box/cone term)

The first 12-thruster run used 250 generations, pop 15, N=50, four multistart
restarts and 8 workers: **1,000 generations / 750,000 runs / 1,468.6 s
(~24.5 min)**, best training cost **1.7636**.  Its gains were replaced by the
cost-shaped run below; the result is archived under `results_opt/archive/`.

### Piecewise box/cone cost

`optimization/objective.py` now scores the 5 km orbit box and the 15 deg sun
cone with the same shape:

```
pen(x, limit, w, k) = w * (x/limit)^2                  if x <= limit
                     = w * exp(k * (x/limit - 1))      if x > limit
```

- box: `x = box_max` km, `limit = 5 km`, `w = 1`, `k = 1`
- cone: `x = track_ang_max` deg, `limit = 15 deg`, `w = 1`, `k = 1`
- the penalty is evaluated on the per-run maximum excursion; the run cost is
  `final_rate + 10 if not reached + box + cone` and is capped at 50, the
  failure cost.

### Second 4x250 run (box/cone cost, old dispersion)

The earlier applied gains came from a 4x250 run with the box/cone objective and
the old ±5 km / ±5 m/s injection dispersion.  It used **1,000 generations /
750,000 runs / 1,668.9 s (~27.8 min)** and found training cost **2.2828**.  Those
gains were superseded by the single-run result below; the result file was
archived under `results_opt/archive/`.

### Third run: single 250-gen CMA-ES with the max-based cost

The optimizer was relaunched after the cost moved to the per-run maximum form
and the injection dispersion was tightened to **±1 km** position / **±0.5 m/s**
velocity.  Settings: 250 generations, pop 15, N=50, 8 workers, seed 42,
`sigma0=0.6`, one run, no restarts.  Runtime **383.9 s**, 187,500 mission runs,
best training cost **0.1411**.

```
detumble: K=5.339150  P=0.882814  Ki=0.027321709
slew:     K=1.286718  P=0.445776  Ki=0.003147821
track:    K=4.409346  P=49.859388 Ki=0.000102648
TS=278.2200
```

`TS` is an inactive timeout budget: actual slew settles in 4.8 s, and `TS` only
sets the abort threshold.  `ACS_TAU_MAX=0.1 Nm`, `IMPLICIT_TRACK_ONLY=True`,
`TRACK_DT=1.0`.

Validation with the applied config:

| metric | seed 42 | seed 123 | seed 321 |
|---|---:|---:|---:|
| N=300 mean cost | 0.207 | 0.215 | 0.205 |
| success-only median | 0.151 | 0.142 | 0.142 |
| failures | **0/300** | **0/300** | **0/300** |
| reached track | 300/300 | 300/300 | 300/300 |
| `box_max` | 4.67 km | 4.97 km | 4.66 km |
| cone max | 1.29° | 1.29° | 1.28° |
| final rate | ~3.5e-6 | ~3.6e-6 | ~3.6e-6 |
| fuel (kg) | 0.00054 | 0.00055 | 0.00055 |

The 5 km orbit box is met at all three seeds and in the N=30 MC: mean radial
error **1.056 km**, `box_max` **1.858 km mean / 3.657 km worst**.

The cost is now mostly the box quadratic term, with the cone inside its
quadratic region and no exponential tail: box **~0.203-0.211**, cone
**~0.0039**, final rate **~3.5e-6**, total **0.205-0.215** per run.

N=30 MC (`results_mc_cpp/mc_summary.json`): **100% reached track**, mean track
error **0.000174 rad (0.0100°)**, max track error 0.0162 rad mean (0.93°),
**100% cone compliance**, final rate **4.82e-6 rad/s**, slew settle **4.80 s**,
fuel **0.000548 kg**, detumble settle **1.90 s**.

N=50 single-run validation agrees: 0/50 failures, 50/50 reached track, mean
track error 0.000151 rad, max track error 0.0166 rad, final rate 4.19e-6 rad/s,
slew 4.80 s, fuel 0.000543 kg, orbit error 1.12 km, `box_max` 1.96 km.

Compared with the previous applied gains, the new run lowers the cost by ~5%,
cuts the worst cone error from 2.13° to 1.29° and slew settle from 5.5 s to
4.8 s.  The cost does not include fuel or time, so fuel rises from 0.00048 to
0.00055 kg and the final rate from ~5e-7 to ~3.5e-6 rad/s; both remain small.
Track error is slightly higher on average but the per-run maximum improves
from 0.0331 to 0.0162 rad.

Updated optimizer result: `results_opt/opt_result.json`.
Updated MC summary/cost map: `results_mc_cpp/mc_summary.json`,
`results_mc_cpp/cost_map.png`.  The optimizer cost map is
`results_cost_map/cost_map.png` (187.5k points).

## Current snapshot (Release, N=50, dt=1.0, implicit track-only, timing on)

| slot | share |
|---|---|
| `dynamics_step` | ~58% (rhs ~37%) |
| `sun_step` | ~7.5% |
| `star_tracker_step` | ~7% |
| `thruster_allocate` | ~6% (solve + clamp only; init is 0.1 ms/run) |
| `thruster_step` | ~4% |
| `quatpd_step` | ~1.2% |
| `mission_step` / KPI | ~2.4% |
| remainder | instrumentation / loop overhead |

Track phase is ~98% of all integration steps. With timing off the whole kernel
is **~9 ms/run** on the current 12-thruster model (N=50-300, Release); the
timing-on profile inflates small slots because each nested `SAD_TIME` pays two
clock reads.

End-to-end (pop 15, N=50, 8 workers): **~1.0-1.4 s/generation**, ~440-750
runs/s, 90% parallel efficiency.  The 4x250 run (1,000 generations) took
**1,420 s (~24 min)** versus the projected 3-4 h at `dt=0.1`.
