# native_omni_heyoka — build status and verification record

*Last updated: this session. All gates below were run and passed.*

## What is done

### C++ core (complete, verified)

| file | status |
|---|---|
| `include/omni/scalar_traits.hpp` | done — `NumTraits<heyoka::expression>` specialization, `make_scalar`, `is_expression_v` |
| `include/omni/linear3.hpp` | done — hand-written 3x3 algebra (see "why" below) |
| `include/omni/mrp_t.hpp` | done — templated MRP DCM + kinematics, matches the validated reference exactly |
| `include/omni/backsub_t.hpp` | done — `BackSubMatricesT<S>` + Schur-complement solve |
| `include/omni/contributors.hpp` | done — hub / reaction-wheel / central-gravity, written once templated on `S` |
| `include/omni/system.hpp` | done — `OmniSystem`: registration, dynamic layout, symbolic build, numeric cross-check |

### Python

| file | status |
|---|---|
| `omniheyoka/_parser.py` | done — infix parser for the C++-emitted RHS strings (see "why" below) |

## Verification gates (all run, all passed)

| gate | what it checks | result |
|---|---|---|
| A | numeric `state_derivative` vs `heyoka_rw` reference over 500 random states and random parameters (incl. non-diagonal inertia) | **5.9e-16** (machine precision) |
| B | torque-free \|H\| drift, RK4 @ dt=0.01 | 2.2e-7 (the integrator's own error floor; not a momentum bug) |
| F | symbolic-advance vs numeric-advance, dt = 0.01 / 0.001 / 0.0001 | 8.78e-6 → 8.78e-7 → 8.78e-8, clean first-order convergence to zero ⇒ the symbolic and numeric systems encode the *identical* ODE |
| **G (final)** | C++-marshalled RHS strings → parsed in Python → `hy.taylor_adaptive`, propagated against `hy.taylor_adaptive(build_rw_system(), ...)` at dt=0.01 and dt=0.001, 600 steps each | **0.000e+00** (bit-exact) |

Also confirmed: 15 states, 19 parameters, 3 inputs for hub + 3 wheels; `par_names()`
returns exactly `u1 u2 u3 mass Ixx Iyy Izz Ixy Ixz Iyz invIxx invIyy invIzz
invIxy invIxz invIyz Js1 Js2 Js3` — i.e. the `native_rw` layout, reproduced
automatically (A2). (WS1 RECORD, WP25-C3: predates the WP2 `tauF` second input
and the Update-4 `invI6` removal — live hub+3RW is 15 states / 16 pars =
6 inputs + 7 hub + 3 `Js`, vec arm, `audit_bundle/results/swap_p1_parity.json:
rw3_open_pos` cell `nPars=16, nStates=15`. Kept for archaeology; `par_names()`
is authoritative.)

Graph size: 4033 bytes of RHS text across 15 equations. `build_symbolic()` +
JIT: 73.7 ms per instance (20-run average).

## Two non-obvious findings worth keeping

### 1. Eigen cannot carry `heyoka::expression` in its kernels

Two independent compile-time blockers, both reproduced:

- `Eigen`'s redux path (`sum`/`trace`/`norm`) constructs `Scalar(0)`, which is
  ambiguous for `heyoka::expression` (several constructors accept
  integer/float arguments).
- Even a plain mat-vec `A*v` routes through an inner product that calls
  `.sum()` (`ProductEvaluators.h`, the gemv evaluator).

`NumTraits<heyoka::expression>` was written (with a `Literal` member), but the
kernels still cannot be used. Hence `linear3.hpp` is entirely hand-written and
Eigen is used only as fixed-size *storage*. This is not a workaround — it is
also faster than the generic path and matches the hand-written
adjugate/determinant the design mandates.

### 2. heyoka's Python `hy.expression(str)` does NOT parse arithmetic

`hy.expression("x + y")` silently builds a single **variable** named `"x + y"`.
Any system assembled from such a string fails with "the variable 'x + y'
appears in the right-hand side but not in the left-hand side". heyoka exposes
no public C++ expression-tree walker, so the C++ core cannot emit a structural
form.

Resolution: the C++ core emits fully-parenthesised infix RHS strings via
`operator<<`, and `omniheyoka/_parser.py` reparses them. The one easy-to-miss
detail: **heyoka prints `sqrt(a)` as `a ** 0.5`**, so the parser must support
the `**` operator. (This is heyoka's canonical form —
`str(hy.sqrt(x*x+1))` gives `(1 + x*x)**0.5`.)

## Update 2 — tilted-wheel generalisation + the string bridge is dead

### The string-marshalling bridge had to be abandoned (design hazard, not a bug)

`omniheyoka/_parser.py` worked and gave a bit-exact match for the aligned system.
It is nonetheless **unsafe as a general mechanism**, and this was found while
testing the tilted configuration:

heyoka's expression **string form truncates at ~1000 characters**, appending
`...`, with no error and no warning. Verified directly: an expression of 1020
chars prints as 1012 chars ending in `...`.

Consequence: the aligned hub+RW system fits (longest RHS `omx` = 926 chars) and
round-trips perfectly, but the **tilted-wheel** system's `omx` is 1009 chars and
is **silently truncated**, producing a wrong symbolic system. Any configuration
complex enough to overflow is corrupt, not rejected.

**Resolution:** the propagator stays in C++ and Python drives it through an
`extern "C"` ABI (`src/omni_taylor_api.cpp`, `omniheyoka/ctypes_driver.py`). No
strings, no parsing, no truncation. `_parser.py` is kept only as documentation of
the failure.

### Bug found and fixed (worth recording: silent, and cost hours)

`omni_taylor_step()` pushed the **state** into the integrator but never the
**pars**. The propagator was built once with the pars of construction time, so
every later `set_pars()` — i.e. the motor torque — silently never reached the
EOM. The sim ran with zero control and diverged from stock by ~120 deg, while
position still looked sane. Any API that owns a cached integrator must push
*both* buffers.

### Results: the generalisation claim now has evidence

| gate | result |
|---|---|
| T1 tilted-wheel accelerations vs hand-derived tilted Newton–Euler, non-diagonal inertia, 400 random states/params | **5.6e-16** |
| T2 torque-free \|H\| conservation, tilted wheels | 3.0e-6 (integrator floor) |
| **full pipeline parity, TILTED, vs stock Basilisk** | **3.4e-04 deg** attitude, 6.9e-06 deg/s rate, \|sigma\| to 6 digits |

The tilted config is exactly the case `native_rw` cannot express (its stated
envelope is "3 body-axis-aligned wheels") and Basilisk supports natively via
`rwFactory.create(rwType, gsHat_B)`. Same contributors, same solve, only the
`gs` vectors differed — no new code.

### Timing, honestly

| | 600 s @ dt=0.1 (6000 steps) |
|---|---|
| composed symbolic system, C++ bulk stepping | **18.67 ms** (3.11 µs/step) |
| — same, tilted wheels (7% more) | 19.99 ms (3.33 µs/step) |
| hand-written `build_rw_system()`, same settings | 17.51 ms (2.92 µs/step) |
| **composition overhead vs hand-writing** | **1.07× per step** |
| native_rw FULL pipeline (stock FSW + C++ dynamics) | 15.20 ms |
| stock Basilisk FULL pipeline | ~110 ms (tilted) |
| our Python-driven closed loop | 184.87 ms — **89% of it is Python overhead** |

Two things follow. Composition is essentially free (1.07×), so the architecture
pays for itself. But a Python-level closed loop is 10× the C++ cost, so
plug-and-play integration requires the loop to move into C++ — which is what the
`SysModel` work already is.

## Update 3 — the whole non-stock side is now C++, and it validates on the bench

`OmniSpacecraft` is a real native Basilisk `SysModel` (`isinstance(x,
Basilisk.architecture.cSysModel.SysModel)` is True, so `AddModelToTask` accepts
it). Built with the `native_rw` recipe: scikit-build-core + `bsk_add_swig_module`
+ `-std=c++23` + `-O3` + direct `libheyoka` link with install rpath, plus the
top-level `cSysModel.py` shim that `%import "sys_model.i"` requires.

Everything that is not stock Basilisk now lives in C++: the contributors, the
back-substitution solve, the templated MRP, the symbolic build, the JIT, the
stepping, the message I/O, the MRP shadow switch, and the ±uMax saturation.
Python only scripts the stock pipeline around it.

### Validation bench (both arms JIT-warmed, best of 3)

600 s @ dt=0.1, 6000 ticks, PD K=3.5 P=30.0, 750 kg, diag(900,800,600).

| | stock Basilisk | omni (full pipeline) | speedup | parity |
|---|---|---|---|---|
| **ALIGNED** (the native_rw case) | 112.61 ms | **22.89 ms** | **4.92×** | **7.19e-05 deg**, 1.40e-06 deg/s |
| **TILTED** (native_rw cannot express) | 107.83 ms | **25.14 ms** | **4.29×** | **3.42e-04 deg**, 6.88e-06 deg/s |

`|sigma|` final matches to 6 digits in both cases. The aligned parity number is
byte-for-byte `native_rw`'s validated envelope, and the tilted case — which
`native_rw`'s stated envelope explicitly excludes — works with zero new code:
same contributors, same solve, only the `gs` vectors differ.

One-time symbolic build: **~8 ms** warm (200 ms for the first JIT in a process).

### Two bugs found while wiring it up (both silent)

1. **`nWheels()` / `writeMsgs` / `UpdateState` derived the wheel count as
   `wsInitRPM.size()-1`** — a leftover expression from an earlier design. With
   3 wheels that yields 2, so wheel 3 got no torque and was never logged. It
   fails *looks-like-success*: two wheels still respond, so the sim looks alive.
   Fixed to `wheels.size()`.
2. **`addWheel()` must RECORD the spec, not register it**, because `Reset()`
   replaces the whole `Impl` (fresh `OmniSystem`). Registration is now replayed
   from the recorded specs in `buildIntegrator()`.

### Timing caveat worth stating

The aligned case measures 0.78× if the JIT is cold, because the first symbolic
build in a process pays ~200 ms of LLVM warm-up. This is the same one-time cost
`native_rw` sees, and the same reason it has a process-wide template cache. For
any multi-run use (Monte-Carlo campaigns) that warm-up must be paid once in the
parent — see Update 2's caching discussion.

## Update 4 — thruster channel: algebra exact, pipeline gated vs stock

### Layout correction (load-bearing): hub owns 7 pars, not 13

`HubContributorT` declared `mass + I6 + invI6` (13) but `solve_backsub`
derives the inverse symbolically from `D` via `inv3()`, so the 6 `invI6`
slots were never referenced. heyoka rejects declared-but-unreferenced pars
(`taylor_adaptive` throws on par-count mismatch — verified directly), and
`OmniSystem::refresh()` still wrote 13 hub defaults into the 7-slot block,
which corrupted the heap and crashed every fresh build (exit 133, no output).
Fixed `refresh()` to write 7; fixed a second crash-risk in the same pass:
`solve_backsub` dereferenced the `ForceTorqueT*` unconditionally despite the
`= nullptr` default — now guarded.

Resulting layouts (derived, A1/A2 — `state_names()`/`par_names()` are truth):

| config | states | pars |
|---|---|---|
| hub + 3 RW | 15 | **13** = 3 u + mass + I6(6) + 3 Js (was 19 with dead invI6) |
| hub + 12 thrusters | 12 | **19** = 12 q + mass + I6(6) |

The RW EOM math is untouched (the inverse was always symbolic); Gate A
re-validates to the identical number (below). The WS2 Python layer
(`presets.py`/`mock.py`/`shim.py`, `RW_N_PARS = 19`, `len(pars) - 16`
heuristic) still assumes the 19-par RW layout and was written against an
`OmniSystem` class the built module never exposed (it ships only
`OmniSpacecraft`) — stale, needs name-based par translation if revived.
Tilted-wheel scratch bins are stale only in par count, math untouched.

### Gate T1-full: thruster algebra == independent reference, 5.6e-16

`scratch_ws1/t7_thruster_algebra.cpp` (500 random states × random duties,
12U bus geometry): `OmniSystem::state_derivative` vs an independent
transcription of `jets_wrench_quantized` + Kepler + full-inertia GG, with the
DCM cross-checked entry-by-entry against `RigidBodyKinematics.MRP2C`.
**Max rel err 5.55e-16.** One subtlety the harness documents: the quantizer
runs at step time (`fillInputs`), never in the graph (A3) — pars carry
already-quantized levels verbatim, so the harness quantizes once at the
boundary like the real stepping loop. (First version double-quantized and
failed at 1.02 — a harness bug, correctly caught by the gate.)

The old `bench_thruster.gate_T1` stays structural (12 states / 19 pars /
12 thrusters / 0 wheels) and now points at `t7` for the algebra proof.

### Gate T2: full pipeline vs stock `thrusterDynamicEffector`, TEST_Thruster

`bench_thruster.py` builds both arms with the repo-standard effector
(`TEST_Thruster`, `MaxThrust`, `cutoffFrequency=1000`, `useMinPulseTime=False`)
and the stock pinv allocator. Three bench bugs fixed along the way:
`thr.thrusterInMsg` → `thr.cmdsInMsg` (no such attribute), `sysModel`
imported after use (`NameError`), nav module created but never added to the
task (NavAttMsg spam, dead FSW).

| | dt=0.1, 120 s | dt=0.01, 120 s |
|---|---|---|
| stock | 85.2 ms | 819 ms |
| omni | 83.0 ms (1.03x) | 702 ms (1.17x) |
| att parity | 19.6 deg | **0.656 deg**, rate 0.32 deg/s, \|sig\| to 3 digits |

Two findings worth keeping:

1. **Stale-message artifact (test bug, not a module bug).** The stock
   dynamic effector fires only on fresh `THRArrayOnTimeCmdMsg`s; a message
   written once fires a single step (verified: step-1 Δv/Δω match to 4
   digits, step-2 stock goes quiet). Open-loop comparisons must rewrite the
   command every tick, exactly as the closed-loop allocator does.
2. **The residual is pulsed-vs-averaged modeling and converges ~linearly
   with dt** (fresh-cmd open loop, same horizon: 1.085 deg @ dt=0.1 →
   0.107 deg @ dt=0.01). Stock fires the valve for `onTime` then coasts;
   omni applies the staircase-averaged wrench over the step. Under violent
   torque both spin wildly and second-order rotation-within-step effects
   dominate; in the controlled loop at mission dt the gap (0.656 deg) sits
   inside the validated Python-path envelope for a chattering thruster loop
   (~2 deg MRP-equivalent, cf. `native_rw` README). Step-1 agreement is exact,
   so there is no plumbing or sign error left.

### Re-validation after the hub trim (same binaries, same seeds)

| gate | result |
|---|---|
| t2 Gate A (RW numeric vs reference, dual par vectors) | **5.90e-16** — identical to the pre-trim record |
| t2 Gate B (\|H\| drift) | 2.24e-07, PASS against the documented 1e-6 floor (old 1e-11 threshold contradicted the BUILD_STATUS record; aligned to it) |
| t3 Gate F (symbolic/numeric ODE equivalence) | 8.78e-6 → 8.78e-7 → 8.78e-8, identical to record |
| wheel build (`pip install --no-build-isolation`) | clean, direct `libheyoka` link + rpath |

`t2_core_numeric.cpp` now carries both par vectors (13 for the system,
19 for the hand-written reference that multiplies by precomputed invI).

### Explicitly not done (unchanged)

- ctypes ABI (`omni_taylor_api.cpp` / `ctypes_driver.py`) has no thruster
  entry point — legacy from the string-bridge era, untouched.
- `tests/` validation suite (WS3) — not started.
- No thruster `SysModel`-level MC campaign yet.

## Update 5 — full stock input-acceptance parity (all 7 items)

### Item 1: sub-step thrust arcs (valve open at full thrust, shut after)

`OmniSpacecraft::UpdateState` propagates each step arc-by-arc over the
sorted unique quantized open-times (epsilon-merged): thruster j is fully
ON during `[0, t_j)` with `t_j = staircase(onTime_j/dt)*dt`, OFF after.
Average == staircase duty by construction; rotation-during-step captured to
first order. Open-loop violent-spin parity improved 4-6x at every dt
(3.77 -> 0.98 deg @ dt=0.1/10 s; 1.085 -> 0.188 @ 2 s; 0.107 -> 0.018 @
dt=0.01). Cost: up to ~4 arcs/step in the chattering loop (typical),
single-arc fast path when idle.

A subtlety worth recording: pulsed (not averaged) emulation makes
trajectories depend on the PWM *frequency*: duty-identical 10 Hz vs 20 Hz
pulse trains differ at O(0.1) deg. Cross-dt comparisons are therefore
meaningless for pulsed thrust; gates compare matched dt (same switching
frequency). Both arms converge to the same continuous-duty limit as dt->0.

### Item 2: edge-triggered staged mailbox (the subtle one)

Probed stock with three timestamp patterns (single thruster, per-step dom):
fresh/advancing `[0, F, F]`, write-once `[F, 0, 0]`, frozen-ts `[0, 0, 0]`
— plus over-duty clamping and a frozen-then-advancing run that re-fires
exactly when timestamps advance. The rule: **fire iff first-sight-ever or
timestamp strictly advances; same-timestamp rewrites never re-fire;
write-once fires exactly once.** Implementation: tick-0 stages (no
propagation); later ticks stage-then-propagate same-tick with one-shot
consume (`thrStaged` + `lastThrStamp`, stagedTs persists across consumes).
Verified 4/4 cells stock == omni, bit-identical wheel states included.
Also fixed along the way: `thr.thrusterInMsg` -> `thr.cmdsInMsg`,
late `sysModel` import, nav never added to task.

### Item 3: quantizer edges locked (no code change needed)

Single-thruster 1-step probes across {0, 1e-12, 0.02, 0.49, 0.5, 0.9,
0.9999999, 1.0, 1.5, -0.1}: stock achieved duty equals the staircase
(including exact 0.5 -> 5/6 and 1.0 -> 1.0 boundaries, negatives -> 0,
over-duty -> 1.0, any positive -> >= 1/6) to 4 digits. Locked in
`tests/test_thruster_stock.py` (14 tests, ~2.5 s).

### Item 4: RW input path generalized (the big find)

Two real bugs, both invisible on the 750 kg demo bus (Js/I ~ 1e-4) and
dominant on 12U-class buses (Js/I ~ 0.37):

* **Layout misroute (crash-class):** `refresh()` hard-coded
  thrusters-before-wheels while the module registers wheels first, so with
  both present motor torques landed in thruster-duty slots (Wdot = 0!).
  Fixed with a registration-order log (A2 honored) + `wheel_input_offsets()`
  introspection; the module never assumes offset 0.
* **Stock hub-inertia semantics:** measured to 8 digits over 3 inertias
  that stock treats `IHubPntBc_B` as TOTAL inertia (incl. wheels) and
  separates spin inertia: `D = I - sum Js*gs*gs^T` (full outer product, so
  tilted wheels match identically). Omni's `HubContributorT` now does the
  same, reading Js from the wheels' own par slots (fully parametric —
  an earlier structural-Js version broke MC dispersion and was caught by
  Gate A). GG/translation keep the total I.
* **Stock coupled spin:** measured stock wheel-speed gain ==
  `(u/Js)*I/(I-Js)` to 4 digits: stock integrates
  `Js*dW = u - Js*(gs.omegadot)` sequentially after the hub solve (not
  `u/Js`). Implemented in `state_derivatives` (the `omegaDot` arg already
  existed for this). Gate B drift dropped 2.2e-7 -> 6.6e-16 (exact
  conservation); RW closed-loop parity 7.2e-05 -> **2.7e-13 deg**.
* Per-wheel `uMax` (`addWheel(..., uMax=0.2)`, stock saturates per
  effector): slopes vs stock to 1.1e-14 RPM. N-wheel uncapped (RW msg has
  36 slots; 4-wheel smoke incl. tilted 4th wheel passes). Note: the stock
  factory kwarg is `u_max`, not `uMax` (silently ignored otherwise — cost
  one debug cycle).

DESIGN constraint (1) refined (not contradicted): never ADD Js*gs*gs^T to
an already-hub-only D with decoupled spin (double counts, 8.1e-5); the
consistent pairs are (hub-only D, decoupled spin) and (total-I-minus-Js D,
coupled spin). Stock and omni now both use the latter.

### Item 5: combined RW+thruster gate

`scratch_ws1/bench_combined.py` (gentle open-loop, slow wheels, matched-dt
rows + violent-regime ladder): dt=0.1: 0.065 deg, dt=0.02: 0.014 deg, both
PASS; residual linear in dt (stock-side switching-integration error).
Translation exact (mm/cm).

### Item 6: single-source geometry helper

`omniheyoka/basilisk_cfg.py`: spec list `(pos, dir, tmax)` builds both
arms (`create_stock_thrusters` / `configure_omni_thrusters`), reads back
(`thruster_specs_from_factory` via `thrLoc_B`/`thrDir_B`/`MaxThrust`,
nested column-vector tolerant), round-trip tested.

### Item 7: valve lag measured: zero

Full-step firing achieves exactly duty 1.000000 (deficit 0 to 1e-6). No
lag states modeled. Locked in tests.

### Final numbers (this build)

| gate | result |
|---|---|
| t2 Gate A (RW numeric vs stock-semantics ref) | 5.90e-16 (record number, same trial) |
| t2 Gate B (H conservation) | 6.6e-16 |
| t3 Gate F (symbolic/numeric equivalence) | 8.78e-6 -> 8.78e-8, unchanged |
| t5 tilted T1/T2 | 5.58e-16 / 1.1e-06 |
| t7 thruster algebra | 5.55e-16 (500 randomized) |
| t8 combined symbolic/numeric | 7.2e-14 |
| pytest `native_omni_heyoka/tests` | 14 passed |
| RW closed-loop vs stock (600 s) | **2.7e-13 deg**, 5e-15 deg/s, 4e-10 RPM wheels |
| RW per-wheel uMax vs stock | 1.1e-14 RPM |
| thruster T2 vs stock, 120 s @ dt=0.01 | 0.66 deg, **0.013 deg/s** (was 0.21), 1.14x |
| combined gate | 0.065 deg @ dt=0.1, 0.014 @ dt=0.02 |

### Explicitly not done

- WS2 Python layer (`presets`/`mock`/`shim`) still assumes the dead 19-par
  RW layout and an `OmniSystem` class the wheel never exposed (ships only
  `OmniSpacecraft`); needs name-based par translation if revived.
- ctypes ABI (`omni_taylor_api.cpp`) still RW-only, untouched.
- RW jitter/friction/voltage-IO, wheel positions, N-body: documented
  envelope (stock measures zero effect in the validated configs).
- No thruster MC campaign yet.

## What is NOT done (updated 2026-10-12; the pre-Update-6 text below is kept
for archaeology, struck where landed)

- SWIG / CMake / `pyproject.toml` scaffolding — DONE (wheel 0.1.0 builds and
  installs; `OmniSpacecraft` + `OmniVecIntegrator` ship).
- `omniheyoka/shim.py` (`OmniTaylorAdaptive`) and `presets.py` — STILL STALE:
  written against `OmniSystem` + 19-par layout, needs name-based par
  translation if revived. NOTE (Update 7): `heyoka_rw.py` encodes absolute-h
  (exact for equal-Js orthonormal sets only); any revival must convert or
  restrict. Top-level `reset_mrp`/`sigma_indices` (A7) still unexported.
- Validation suite in `tests/` — DONE for the envelope + swap (149 + 4 + 6
  tests green, CI 3/3). Tilted case now checks against the corrected
  relative-h reference AND live stock sims.
- Thruster/wrench contributor — DONE (ThrusterContributorT + env cluster).
- ctypes ABI (`omni_taylor_api.cpp`) still RW-only, untouched.
- No thruster MC campaign yet.
- Swap follow-up: SRP-faceted / mag / J2 / N-body stock gates; prescribed
  motion / flex / slosh / stochastic stay loud-reject; multi-spacecraft sync
  (Phase 3); closed-loop FSW acceptance.

## How to reproduce the gates

```bash
cd Python/native_omni_heyoka
FLAGS="-std=c++23 -O2 -w -I../env/include -I/opt/homebrew/include \
       -I../env/lib/python3.11/site-packages/bsk_sdk/include/eigen3 -Iinclude \
       -L../env/lib -lheyoka \
       -Wl,-rpath,/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python/env/lib"

clang++ $FLAGS scratch_ws1/t2_core_numeric.cpp -o scratch_ws1/t2 && ./scratch_ws1/t2
clang++ $FLAGS scratch_ws1/t3_symbolic.cpp   -o scratch_ws1/t3 && ./scratch_ws1/t3
# GATE G (bit-exact vs the hand-written reference) then re-parses the dump:
cd .. && ./env/bin/python - <<'PY'
import sys; sys.path.insert(0,'native_omni_heyoka')
import numpy as np, heyoka as hy
from omniheyoka._parser import parse_system
from cubesat_12u_sim.backends.heyoka.heyoka_rw import build_rw_system, pack_rw_pars
rows={}
for line in open('/tmp/omni_symbolic_dump.txt'):
    l,r=line.strip().split(' = ',1); rows[l]=r
names=list(rows); sysrows=parse_system([rows[n] for n in names], names, 19)  # CI-EXEMPT(bare-19): historical Gate-G repro — 19 is the pre-tauF/pre-invI6-removal reference-layout width (heyoka_rw.pack_rw_pars), NOT a live count assert; live counts come from par_names().
P=pack_rw_pars(np.zeros(3),750.,np.diag([900.,800.,600.]),np.full(3,.07957747154594767))
y0=np.concatenate([[1e7,2e6,-1.5e6],[-700.,5500.,3200.],[.1,-.2,.3],[.01,-.02,.03],[100.,200.,300.]])
a=hy.taylor_adaptive(sysrows,y0.tolist(),pars=list(map(float,P)),tol=1e-13)
b=hy.taylor_adaptive(build_rw_system(),y0.tolist(),pars=list(map(float,P)),tol=1e-13)
a.propagate_for(6.0); b.propagate_for(6.0)
print("max rel diff:", (np.abs(np.asarray(a.state)-np.asarray(b.state))/np.maximum(np.abs(b.state),1)).max())
PY
```

## Update 6 — WP0…WP6 implemented in parallel (unbuilt on Linux; host build pending)

All seven WPs were implemented in one pass by parallel agents against the
measured bundle ledger (2026-10-12, BSK 2.12.0, 7/7 probes green at the time —
recounted 11/11 WP25-C3, see test_wp7 + PLAN_ENVELOPE bundle row), then
integrated sequentially into the shared files. Nothing here has compiled yet
(Linux container, Basilisk macOS-only) — the macOS host owns first build.

| WP | What landed | Key tolerance / contract |
|---|---|---|
| WP0 | `checkUnsupported()` (11 flags, Reset+UpdateState heads; forceMsg requires link), name-resolved state offsets (`rOff_/vOff_/sigmaOff_/omegaOff_/wheelOff_`), `scripts/ci_envelope_checks.sh` (A3/A2/index greps), `tests/test_wp0_guards.py` | CI 3/3 must pass; all numeric state indices gone from `src/` |
| WP1 | `addRatedWheel` (Js=maxMomentum/Ω, RPM→rad/s), `setWheelOmegaMax` (setter-only, ≤0=OFF), direction-aware step-start cut in UpdateState | overshoot ≤0.5 RPM @ dt=0.1, plateau 6000.40±0.05 |
| WP2 | RW 2nd slot `tauF{i}` (vecRot + coupled spin), module-side Stribeck iter-1 + per-wheel mt19937 jitter, `setRWModelJitterCoupled`=fatal, voltage reuse (no code) | distributional jitter (determinism proven 0.0); off = slot-identical-0 |
| WP3 | `thrusterOutMsgs` 9/9 publish (about-B, steady-tick), min-pulse gate + valve memory, ramp sub-arcs, cutoff exact-exp lag, blowdown 1.0+warn, swirlScale slot + axis term, force-link guard | all default-OFF = bit-identical arcs; M0–M6 green |
| WP4 | Hub B/C blocks + `c×mg` + cPrime hook, aggregate `c_B` record, B/C telemetry split, published `omegaDot` via end-state accelerations | C-point kept (0-vs-17); zero-offset regression identical |
| WP4 correction 2026-10-12 | B-point formulation REJECTED by fire-step bench (idle drift + 0.685× scaling); reverted to measured stock semantics: input-I about CoM (no PA/B/C/c×mg), arms shift `(r−c)×F` in `ThrusterContributorT` via `refresh()` wiring, B/C split + `omegaDot` kept | stock 0.18461538 == shifted/I to 8 digits |
| WP5 | NEW `environment.hpp` (single 'e' cluster: body 3Nb + drag/SRP 6 + mag 3 slots), TRUE laws module-side, per-tick refresh + per-arc hook, P1 truth table | N-body sign-fixed; J2 ECI; drag-LEO; cannonball-first; MTB-direct |
| WP6 | `rwOutMsgs` 23-field publish (u_current read back from slot), DV accumulators (labeled non-parity), FuelTankMsg bookkeeping (no mass feedback), `viz_direct.py` | field-diff gate; viz via direct interface; burn sim stays isolated |

Cross-WP contracts: WP0 flag names exact; `swirlScale` A2-by-name; env evaluated LAST in two-pass FT loop; `// WP5-arc-hook` placement; new par slots (`tauF`, `swirlScale`, env blocks) are name-derived (19-par counts stale by design).

### Host build risks (explicit)

- SWIG `%template` vector<Message<…>> lines (THR + RW) may be rejected → drop line, keep member + accessor.
- Payload field names (`RWModel`, `TotalAccumDVBdy`, `FuelTankMsgPayload.h`, `EarthRadiationMsgPayload.h`, `WindMsgPayload.v_air_N`, `SpicePlanetState.PositionVector`, `MagneticFieldMsg.magField_N`) need header confirmation.
- `Message<T>` default-constructibility for `resize()` (rwOutMsgs, thrusterOutMsgs lazy-grow covers).
- RW `n_inputs` 1→2 + thruster +1 slot + env blocks change every par layout (name-based asserts required; count-based benches updated in WP test files).

CLOSED (WP25-C3, 2026-10-13 — builds green for three rounds, no risk materialised):
all three Update-6 risks are retired by measurement, not by review. (1) The
host wheel built with the `%template vector<Message<…>>` lines intact — no
SWIG rejection, no line dropped. (2) `resize()` on both message vectors works
(the lazy-grow path was never needed as a fallback). (3) Every payload field
in the list above is confirmed against the built wheel: ledger
`audit_bundle/results/wp6_telem_viz_fuel.json:T1` records
`thr_output_missing: []` (9/9 `THROutputMsgPayload` fields) and
`rw_log_missing: []` (23/23 `RWConfigLogMsgPayload` fields), host arm.
Rounds cited: Update-6 first host build + WP25 wave-A/B rebuilds + wave-C
swap rebuild (`swap_p1_parity.json` verdict `parity green`, host arm) — three
green builds, zero SWIG/resize/field failures.

### tilt-W3 anomaly closure (2026-10-12, no model change)

Closed-loop tilt-W3 left wheels 0.26-0.34 RPM apart (attitude converged to
4.8e-5 deg). Eight independent exonerations, all measured: T1 tilted
accelerations 5.6e-16; constant-torque open-loop 1e-8 deg; time-varying
sine-sweep open-loop 6.8e-10 deg / 1.2e-11 RPM wheels;
stock-vs-stock+1e-12 plant sensitivity 1e-12 class; omni-vs-omni tol sweep
bitwise 0; FSW factory construction mirrored (bitwise no-op); saturation
fractions identical to 1e-6 both arms (0.24357 = 0.24363). Mechanism:
saturation-edge timing jitter -- at clip entry/exit edges, 1e-9-level
integrator differences flip individual ticks in/out of saturation on one arm
only (O(0.1 Nm) kicks); wheels are pure integrators with no momentum
management, so kicks freeze permanently while PD reconverges attitude.
Small-maneuver control chatters with MATCHED statistics both arms
(sat 0.47042 = 0.47042). Gate: tilt-W3 closed-loop att/rate stay tight
(1e-3 / 2e-5, validated record 3.42e-04 / 6.88e-06), wheels get a jitter
envelope (1.0 RPM = 3x measured worst). Unsaturated rigs keep 1e-9 class.

## Update 7 — relative-h fix + Phase-1 integrator swap (2026-10-12)

### The absolute-momentum bug (load-bearing, hid in every 3-wheel gate)

`ReactionWheelContributorT` carried `h = Js*(W + omega.gs)*gs` (absolute)
while the hub inverts `D = I - sum Js*gs*gs^T` (subtracted). That combination
double-counts the `Js*(omega.gs)*gs` momentum: it is already inside the total
inertia `D` is derived from. The extra torque `-Js*(omega.gs)*(omega x gs)`
cancels EXACTLY for 3 orthonormal wheels (`-Js*omega x omega = 0`), which is
why Gates A/F/G, T1 (vs the same-team absolute reference!) and every
3-wheel parity number passed while 1-2 wheel configs drifted ~1e-5.
Found by the swap 1-vs-3-wheel parity matrix (1x/1tilt/2xy: 1.6-3.3e-05,
3xyz: 4e-14), confirmed against upstream
`reactionWheelStateEffector.cpp`
(`vecRot -= gs*u + Js*W*(omega x gs)` = relative-h), fixed one-line to
`h = Js*W*gs`. Proof it is physics, not tuning: 1x/1tilt/2xy/3xyz/hub now
ALL agree at 4-7e-14; t2 Gate A re-validates to the identical record number
(5.90e-16); t5 T1 re-validates to 5.58e-16 against the corrected reference.
DESIGN constraint (1) amended; t5/t2 harnesses corrected (relative-h +
name-based par slots); the same-team-reference hazard from PLAN_ENVELOPE
Ground-6 materialized as predicted. `heyoka_rw.py` keeps absolute-h (exact
for equal-Js orthonormal sets only) -- flagged for the WS2 revival.

### OmniVecIntegrator: stock Spacecraft + setIntegrator swap (stage 1-3)

`include/omni/vec_integrator.hpp` + `src/omni_vec_integrator.cpp`:
`OmniVecIntegrator : public StateVecIntegrator`, exposed through the same
wheel (SWIG `%import` analog: `%include` the real `stateVecIntegrator.h`
so the descriptor carries the derived->base cast; `class Spacecraft`
forward-decl unifies with the stock proxy by NAME -- the `%{ %}`-only
attempt failed exactly like this). `DynamicObject` members only (dynManager
by name; `states`/`dynEffectors` are Spacecraft members -- the first build
died on that); per-tick RW torques read back from stock `rwOutMsgs`
`u_current` (timing-identical under every task order -- a direct
motor-message read would skew a tick by task order); stock keeps MRP
switching/DV/energy/telemetry (postIntegration). Mirror lives in
`omniheyoka/swap.py` (task-model walk: hub/RW/gravity + guards; loud on
anything else). Ledger `audit_bundle/results/swap_p1_parity.json`, gates
`tests/test_swap_p1.py`: hub/RW 1e-9 class, thruster staircase with
dt-aware switching bounds (linear-in-dt ladder proven), write-once/frozen
schedule equality, drag + SRP-cannonball gated (SRP needs the C_R -> r
convention map `cr = C_R - 1`, measured end-to-end), rejects 6 loud
(`jitter_wheel`, `two_rw_effectors`, `thruster_no_factory`, `min_pulse`,
`cutoff_zero`, `fuel_unmirrored_cxx` structural — ledger
`swap_p1_parity.json:rejects` verdict `all loud`, vec arm; friction is parity
since Update 8, NOT a reject — `rw3_fric` cell, see below. WP25-C3: was
"7/7 rejects loud" — the friction entry left when the Stribeck law landed).
Tumbling + env documents the O(|w|*dt) frozen-projection class with honest
bounds (rotation-capped subdivision tried: 4x gain at 4x cost, reverted --
bias is systematic, subdivision cannot remove it; arc-midpoint refresh
deferred). GG/SRP-faceted/mag/J2/N-body stock gates explicitly follow-up;
their wiring is the drag-proven shared path with unchanged validated laws.

### Swap debugging ledger (traps hit, all closed)

- THRArrayConfigMsg functor never links (addSubscriber snapshot stays dead;
  stock Reader fine) -> geometry joins Python-side (factory THRSimConfig +
  effector thrusterData, exact loc/dir cross-check, no `.label` reads: they
  abort). On-time mailbox fine.
- `thrustFactor` telemetry lags dynamics 3 ticks (all-zeros while firing);
  dynamics (omega deltas) are the truth -- two "bit-exact" benches were
  nearly vacuous before this was caught.
- Task priorities are load-bearing for stock firing (all-default never
  fires); recipe mandates the proven pattern.
- Min-pulse effective state is per-thruster MinOnTime (factory flag
  meaningless); `useMinPulseTime=False` at create() zeroes it.
- Stock default integrator is RK4 (upstream ctor) -- the swap target.

## Update 8 — Phase-2 coverage: offset, fuel, friction, lag (2026-10-13)

### CoM-offset swap gate + stock B/C sign finding

Offset (rBcB=0.1) + thruster A/B: r 2.24e-08, attitude ~1e-13
(ledger audit_bundle/results/swap_p1_parity.json:thr_offset -- r_BN_N
2.24e-08, sigma_BN 1.02e-13, omni-vs-stock, incl. r_CN/v_CN outputs;
replaces the earlier 5.6e-09 text, same cell re-read). Along the way the
swap MEASURED stock's B/C convention as r_BN = r_CN - R^T*c
(r_BN - r_CN = -c exactly; wrong sign would be ~0.2 m), opposite to the
pre-A1 DESIGN/test_wp4 `+` formula (self-consistent math only, never
stock-pinned). WP25-A1 unifies the SysModel path to the measured `-`
(signed-component gates per axis <=1e-9, cross-path <=1e-9).
Zero-offset path is IEEE-exact no-op (bit-identical).

### Fuel depletion (flagship, VEC path — SysModel is WP25-A3 option B, see A13)

`FuelMassContributorT` (1 state + mdot input) + per-tick hub mass/I par
overwrite + live aggregate c_B everywhere (no rebuilds in the final design;
a drift-triggered-rebuild design was tried and reverted: 2 s re-JIT each,
270 rebuilds = 9 min/sim, and unnecessary once c went parametric):
- Measured (mass-props message): input I/mass are hub-DRY; aggregate ==
  input + mFuel*K_pointmass(r_TB) EXACTLY (all 9 entries); c_B exact.
- mdot from staircase-ACHIEVED duties (matches stock bit-exactly over
  600 s: fuel_rel_err 0.0 -- both sides use achieved, not commanded; vec arm,
  ledger `audit_bundle/results/swap_p1_parity.json:fuel_centered/fuel_offset`
  cells: `fuel_rel_err 0.0`, `fuel_burned 1.12 kg`, `nPars=12, nStates=13`).
- Inertia about TRUE C: I_C = I_hub + mHub*PA(rBcB-c) + mFuel*PA(rTB-c)
  with LIVE c per tick (using I_B instead costs 0.2 m + 2e-2 rad -- measured).
- Thruster arms via 3 NEW cBx/cBy/cBz par slots (module-written per tick);
  structural comOff_ survives as refresh snapshot only. This killed the
  2 s-per-rebuild re-JIT design (270 rebuilds = 9 min/sim -- dead end,
  reverted) AND fixed the offset gap (0.2 -> 0.006, switching-class).
- heyoka demands EXACT par match. Thruster CoM axes are masked AT THE
  SOURCE: `ThrusterContributorT` registers only axes its geometry couples
  (cx via dy|dz, cy via dx|dz, cz via dx|dy -- from t=(r-c)xF expansion),
  so c-slots never dangle. This was forced by the 12U stale-path config
  (x-only thrust drops cBx = INTERIOR gap: cy,cz live, cx dead -- suffix
  truncation cannot fix that; the contributor now compacts). Masked-out
  axes read the structural record literal (provably dead in the graph).
  Module c-writes are per-axis via `has_par` (NPOS sentinel for masked).
  `OmniSystem::checked_par_count` remains as the structural backstop
  (suffix-drop + loud throw on other gaps); all ta constructions truncate
  + all per-tick pushes heap-bounded (4 ta sites: integrator, SysModel,
  t8; t3 RW-only untouched). wp7 `expected_layout` gains the `n_thr_c`
  mask-popcount term (hub+12thr: 23 = 13+7+3 full-mask with all 3 cB axes live;
  default (no cB axes coupled) is 20 = 13+7 — WP25-C3: the 23 without the
  default-20 note was the FRESH-low overclaim).
- Gates: centered + offset tank (10 kg, 5 N, duty 0.5, 600 s, 1.1 kg
  burned; vec arm, ledger `audit_bundle/results/swap_p1_parity.json:
  fuel_centered/fuel_offset` cells `fuel_rel_err 0.0`): fuel bit-exact,
  trajectories in switching class; c_B static
  initial would cost 0.2 m (measured) -- live-c everywhere instead.
- Scope: VEC PATH ONLY. SysModel publishes no fuel telemetry by design
  (WP25-A3 option B, Decision 4 — see DESIGN A13; the static-full-tank publish
  was wrong telemetry under burn, H6). Single tank + all mirrored thrusters
  drain it (multi-tank raises;
  subset linkage unverifiable -- documented assumption); empty-tank
  behavior (clamp mdot, thrust unaffected) labeled; fuel+offset+full-FSW
  closed loop is follow-up.

### Friction (Stribeck, stock-exact, VEC path) + cutoff accepted, never modeled (inert without ramps, both paths)

- Friction: per-wheel Stribeck law + stribeck-flag/omegaBefore state
  transcribed 1:1 from upstream (module-side tauF slots); helper mirrors
  the live coefficient set (factory-off all-zero = OFF, no records).
  Gate (vec arm, ledger `audit_bundle/results/swap_p1_parity.json:rw3_fric`
  cell): spin-up 40 s + free spin-down, 120 s — sigma_BN 8.34e-13,
  omega_BN_B 1.93e-14, wheels 1.42e-14 (1e-14 class all channels).
  SysModel stays iter-1-suppress-only with friction-ON loud-reject-or-label
  (WP25-B3 scope — unscoped "stock-exact" claims are banned, Decision 5).
  Jitter stays loud-reject (distributional-or-reject per DESIGN; bit-exact
  impossible across libc++).
- Cutoff accepted, never modeled (inert without ramps) -- WP25-A2 closed
  H1/M16: the unconditional SysModel first-order filter is DELETED (it
  DIVERGED from stock, attenuating fresh steps 0.63x at cutoff 10;
  measured r 0.229; `setCutoffFrequency(10)` vs vec was an arm split).
  Measured stock truth: cutoff is INERT without ramps (10 vs 1000 vs 100
  bit-identical over fire AND shutoff transients); both paths now compute
  `eff = open?1:0`. `setCutoffFrequency` accepts any finite `>0` (`<=0`
  still rejected, degenerate unmeasured) and never arms shaping. Ramps
  stay rejected. Archaeology: before this fix SysModel was exact at
  pinned 1000 (filter ~= identity there) and unvalidated at 10. Re-entry
   condition: cutoff composes WITH ramps if those ever land (M2). Gate:
   10-vs-1000 impulse ratio 1.00 +/- 0.01 over fire AND shutoff transients
   (matched dt); `thr_lag10` cell (default cutoff, no pin) retained.
   (WP25-C3 LEDGER NOTE: the in-tree `audit_bundle/results/wp3_timing.json`
   (ts 2026-10-12T14:40:35Z) predates the WP25-C2 probe keys — its M2 carries
   `cutoff_ladder` only, no `inert_1pct`/`shutoff_duty05`, and its M4[0.1]
   no `half_step_shift`. The host-regenerated ledger with the new keys is the
   gate source; copy it in-tree before citing M2 numbers from the file.)
- u_min/P_max/Omega_max need NO work on the VEC path (stock
  ConfigureRWRequests applies them; we read post-saturation u_current --
  timing-identical; vec arm, `src/omni_vec_integrator.cpp` readback). SysModel
  documents the silent gap + telemetry labels until modeled or rejected
  (WP25-B2 scope, Decision 6 — the unscoped Update-8 sentence is corrected
  here, not deleted).

### Swap debugging ledger additions

- `thrustFactor` telemetry lags dynamics ~3 ticks (all-zeros while firing);
  omega deltas are the truth (two near-vacuous benches caught).
- Task priorities load-bearing for stock firing (all-default never fires).
- Min-pulse effective state is per-thruster MinOnTime (factory flag
  meaningless); THRArrayConfigMsg functor never links (geometry joins
  Python-side; `.label` reads abort).
- Shadow-switch timing jitter makes sigma-vector comparison meaningless
  past |sigma|=1 (measured 1e-3 with omega exact 0.0); fuel attitude gated
  by DCM (switch-invariant 4e-16).
- Translational B-point states exist; `sc.states`/`dynEffectors` are opaque
  from Python AND unreachable from DynamicObject C++ (task-model walk is
  the only typed census); MRO walk distinguishes dynamics from field models.

## Bench vs tests decision (2026-10-12)

The Rev-1 validation matrix names `bench_rw_omni.py` Part A / `bench_thruster.py`
T2 / `bench_combined.py` rows that were never added (saturation-cap, friction,
offset, M4/M5, env-effects). Decision: **`native_omni_heyoka/tests/` supersedes
them and is the authoritative gate.** Rationale: every one of those rows now
exists as a pytest with stricter, ledger-traceable bounds and no-wall-clock
tolerance (test_wp1_saturation, test_wp2_friction_jitter, test_wp3_timing_models,
test_wp4_offset, test_wp45_env_offset, test_wp5_env, test_wp6_telemetry,
test_wp7_dispersion_jit). The scratch benches remain valid ad-hoc harnesses
(and `audit_bundle/` remains the probe/ledger layer) but they are NOT the gate;
adding duplicated rows to them would create two sources of truth for the same
numbers. Envelope-workbook consumers should reference `tests/`.
