# Proposed layout for `Python/cubesat_12u_sim`

Status: **proposal only, nothing has been moved.** No file contents changed.

## Problems with the current flat layout

1. 45 `.py` files sit in one directory; the only groupings are the 4 pre-existing
   subpackages (`optimization`, `cppdrive`, `cxxdrive`, `rwdrive`, `fastloop`) and `tests`.
2. Nothing tells you *what a file does* before you open it. `heyoka_*`, `cpp_*`,
   `*_mc.py`, `run_*`, `run_rw_*` naming is overloaded:
   - `run_timing_compare.py` / `run_warm_compare.py` / `run_rw_timing.py` / `run_dissect.py`
     / `profiling.py` are all *timing/benchmark* tools, but live far apart.
   - `monte_carlo.py`, `heyoka_mc.py`, `native_mc.py`, `rw_monte_carlo.py` are the same
     concept with different physics backends.
   - `cpp_basilisk.py` / `cpp_native.py` / `cpp_rw.py` are three variants of one idea.
3. Built `.so` files, C++ sources, and `setup.py` for four extensions are mixed with
   analysis code, so `.gitignore` and build commands get messy.
4. `models/` (OBJ/MTL) is a runtime asset cache, not code — indistinguishable from modules.

## Proposed tree

```
Python/cubesat_12u_sim/
├── __init__.py                      # public API re-exports only
├── README.md
│
├── core/                            # physics + parameters, no backend deps
│   ├── config.py                    # single source of truth for constants
│   ├── dynamics.py                  # 6-DOF EOM, Euler DCM
│   ├── actuators.py                 # jets + reaction wheels -> wrench
│   ├── sensors.py                   # gyro model
│   ├── sun_model.py                 # sun ephemeris + pointing reference
│   └── propagate.py                 # scipy.solve_ivp harness
│
├── control/                         # guidance & control
│   ├── controller.py                # OrbitPID / SemiMajorPID
│   ├── hierarchical.py              # NMPC outer + PID inner
│   ├── nmpc.py                      # scipy POC NMPC, pluggable backend
│   └── adjoint.py                   # discrete-adjoint gradients for nmpc
│
├── backends/                        # "how do we integrate / simulate"
│   ├── basilisk/
│   │   ├── basilisk_sim.py          # stock Basilisk driver
│   │   ├── bsk_rw_stock.py          # stock scenarioAttitudeFeedbackRW runner
│   │   └── vizard_model.py          # OBJ/MTL asset generation (vis only)
│   ├── heyoka/
│   │   ├── heyoka_attitude.py       # MRP Taylor system + closed loop
│   │   ├── heyoka_propagate.py      # orbit-only Heyoka propagation
│   │   ├── heyoka_dynamics.py       # Basilisk spacecraft module (Python SysModel)
│   │   ├── heyoka_rw.py             # 15-state MRP + wheels
│   │   ├── heyoka_mc.py             # batch/ensemble MC
│   │   └── heyoka_sidecar.py        # live parity shadow propagator
│   └── cpp/
│       ├── cpp_rw.py                # Basilisk RW module via cxxdrive
│       ├── cpp_native.py            # native-compiled RW pipeline
│       └── cpp_basilisk.py          # native-compiled full tick loop
│
├── uncertainty/                     # Monte Carlo / dispersion studies
│   ├── monte_carlo.py               # reference whole-mission MC
│   ├── native_mc.py                 # MC over the native C++ backend
│   └── rw_monte_carlo.py            # dispersed RW MC across backends
│
├── tuning/                          # optimisation + timing instrumentation
│   ├── run_optimize.py              # CMA-ES 3-stage sweep
│   ├── profiling.py                 # hierarchical stage timing
│   ├── run_dissect.py               # per-component wall-time breakdown
│   ├── run_timing_compare.py        # cold wall-time ladder
│   ├── run_warm_compare.py          # warm wall-time ladder
│   └── run_rw_timing.py             # RW timing ladder
│
├── benchmarks/                      # accuracy/parity comparisons
│   ├── run_cpp_rw_compare.py
│   ├── run_rw_compare.py
│   ├── run_rw_forensic.py
│   ├── run_heyoka_pipeline_compare.py
│   └── run_nmpc_compare.py          # fuel: PID vs hierarchical NMPC
│
├── missions/                        # scenario entry points
│   ├── reference_mission_ACS3.py    # real-mission anchor (LEO SSO)
│   ├── main.py                      # free-fall demo
│   ├── run_stationkeeping.py
│   ├── run_3year.py
│   └── run_monte_carlo.py           # thin CLI over uncertainty/
│
├── optimization/                    # unchanged (already a package)
├── native/                          # C++ extension packages, unchanged internally
│   ├── cppdrive/  cxxdrive/  rwdrive/  fastloop/
│
├── assets/
│   └── models/                      # moved from ./models (runtime OBJ cache)
│
├── tests/                           # unchanged
└── results/                         # results_3yr, results_nmpc, results_mc*, ...
```

## How separation is expressed

| Layer | Rule |
|---|---|
| `core/` | pure numpy/scipy, importable with no Basilisk or heyoka installed |
| `control/` | imports only `core/` |
| `backends/<x>/` | the *only* places that import basilisk / heyoka / native_rw |
| `uncertainty/`, `tuning/`, `benchmarks/`, `missions/` | leaves: import core + control + chosen backend |

## Cost: this is NOT a zero-code-change move

Moving a module changes its import path, so every importer must be updated.
Measured in-repo references that would break:

- `from cubesat_12u_sim.<mod> import ...` in ~20 files (13 of them `from cubesat_12u_sim`
  bare, which resolve via `__init__.py` and would survive untouched)
- relative imports inside modules: `from . import config as C` in `dynamics.py`,
  `actuators.py`, `sensors.py`, `propagate.py`, `main.py` — these keep working
  unchanged if the subpackages all use `from ..core import config as C`
  (that one line *does* need editing)
- `native_rw`, `rwdrive`, `fastloop`, `cppdrive` extensions referenced by `setup.py`
  files and by `cpp_*.py` lazy imports — need the new `native/` path
- `README.md` documents `python -m cubesat_12u_sim.run_*` for every runner
- 4 `setup.py` files use relative source paths

### Two ways to execute it

**Option A — do it properly (recommended).**
1. `git mv` files into the new tree.
2. Add `__init__.py` to `core/ control/ backends/... uncertainty/ tuning/
   benchmarks/ missions/ assets/`.
3. Fix imports with a codemod (`sed` over the ~20 importer files) + the 5 relative-import lines.
4. Update the 4 `setup.py` files and README `python -m` examples.
5. `python -m pytest tests -q` + one smoke run per backend.
   ~1 focused session of work, no behaviour change.

**Option B — literally zero edits.**
Keep the modules where they are and add *no* new packages; instead only make the
existing structure legible: move `models/` -> `assets/models/`, move the four C++
extension packages under `native/`, and put a `LAYOUT.md` section in the README that
maps each prefix (`heyoka_*`, `cpp_*`, `*_mc`, `run_*`) to its role. This removes
most of the "what does this file do" confusion at near-zero risk, but the Python
side stays flat.

Tell me which option you want and I'll execute it.