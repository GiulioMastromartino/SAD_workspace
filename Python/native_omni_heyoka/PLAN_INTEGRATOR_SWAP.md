# Integrator-swap plan — `OmniVecIntegrator` as a stock `StateVecIntegrator`

Goal: a module that **completely exchanges Basilisk's default integrator**.
End state: in an otherwise unmodified stock sim script, replacing

```python
integrator = svIntegratorRK4(scObject)
scObject.setIntegrator(integrator)
```

with

```python
from omniheyoka import OmniVecIntegrator
scObject.setIntegrator(OmniVecIntegrator(scObject, tol=1e-12))
```

propagates the sim with heyoka-Taylor accuracy and speed, with measured
parity. This is a plan; nothing here is implemented yet. It follows the
repo's ground rules (probe ledger first, matched-dt, loud rejection,
no regression, JIT budget).

## Why this is a small interface with a large tail

The swap surface itself is tiny — verified against `bsk_sdk` headers
(BSK 2.12.0, `simulation/dynamics/_GeneralModuleFiles/`):

| Header fact | Consequence |
|---|---|
| `DynamicObject::integrator` is a `StateVecIntegrator*`, installed via `setIntegrator()` (`dynamicObject.h`) | One class to implement, one call to install. Same precedent as `OmniSpacecraft` inheriting `SysModel`. |
| `StateVecIntegrator::integrate(t, dt)` is the only virtual (`stateVecIntegrator.h`) | All step-loop semantics must live behind this one method. |
| Stock RK gathers/scatters via `ExtendedStateVector::fromStates / setStates / fromStateDerivs / setDerivatives`, drives trials via `dyn->equationsOfMotion(t, dt)`, advances via `dynManager.propagateStateVector(dt)` (`svIntegratorRK4.h`, `svIntegratorRungeKutta.h`) | The gather/scatter/advance protocol is reusable verbatim. The trial-evaluation protocol is **not** — see constraint below. |
| `DynamicObject::integrateState(t)` owns the call order: `preIntegration → integrator->integrate → postIntegration`; `syncDynamicsIntegration` / `isDynamicsSynced` couple multi-spacecraft stepping | The integrator must be correct under stock-owned ordering, not omni-owned `UpdateState` ordering. |

**Load-bearing constraint (shapes the whole plan):** heyoka's
`taylor_adaptive` requires a *symbolic* ODE graph. Stock
`equationsOfMotion` is numeric code that cannot supply Taylor
coefficients. Therefore the swap CANNOT be a generic stepper over
arbitrary stock effectors — it must go through omni's
runtime-composed symbolic system (Option A). The "swap" is:
*stock `Spacecraft` object + stock effector configuration →
mirrored into templated contributors → one symbolic build →
heyoka steps the full registered vector → scatter back into stock
states.* Any registered state no contributor claims must `bskError`
at setup (WP0 philosophy), never integrate silently wrong.

**This path is already validated — not hypothetical.** The
numeric↔symbolic equivalence the swap depends on is measured
(`BUILD_STATUS.md` gates, all green on host):

| Gate | Result | What it de-risks for the swap |
|---|---|---|
| F (t3) | symbolic-advance vs numeric-advance 8.78e-6 → 8.78e-7 → 8.78e-8, clean first-order convergence | mirrored symbolic system and numeric evaluation encode the *identical* ODE — the composition step adds no error |
| G | C++-built system vs hand-written `build_rw_system()`, 600 steps @ dt=0.01/0.001 | **bit-exact (0.000e+00)** — composed graph equals the reference graph |
| t8 | combined RW+thruster symbolic vs numeric | 7.2e-14 |
| A (t2) | numeric `state_derivative` vs stock-semantics reference, 500 randomized states/params | 5.90e-16 — the numeric side itself matches stock |

Consequence for phasing: the swap's residual risk is **not** "does
composition preserve the dynamics" (answered: exactly). It is
(a) mirror fidelity — are the stock config values copied correctly,
and (b) ordering — do staging/arcs/refresh behave identically under
stock-owned ticks. Phase 1 gates are therefore mirror-parity gates
(stock-config → mirrored-symbolic vs stock-RK4), reusing the Gate F
harness shape, not new composition research.

## Phase 0 — kill-or-confirm probes (COMPLETE 2026-10-12, all green)

Ledgers: `audit_bundle/results/swap_p0_{callback,census,swig,ordering}.json`;
gates: `tests/test_swap_p0.py` (4 tests, green; suite 153 passed).

1. **Callback-RHS: absent, confirmed.** All `taylor_adaptive`
   constructors take symbolic systems only (4 ctor shapes checked);
   `func` requires `taylor_decompose` LLVM rules; `callable` serves
   event callbacks, `step_callback` per-step observation. Shortcut
   door closed; mirror+compose stands on Gates F/G/t8/A.
2. **Census: names/sizes pinned, round-trip bit-exact (0.0).**
   hub-only 12 (`hubPosition/hubVelocity/hubSigma/hubOmega`, 3×1 each);
   hub+3RW 15 (+`reactionWheelOmegas` 3×1, BalancedWheels — no thetas);
   hub+fuel 13 (+`fuelMass` 1×1; stock default name is numbered,
   e.g. `fuelTankMass2` — mirror must read the live name, never assume
   it). `setState(same)→getState` delta exactly 0.0 in all configs.
3. **SWIG surface: C++-only confirmed.** `setIntegrator` accepts
   `StateVecIntegrator*` (stock RK4 ok, `int` → TypeError); Python
   subclass definition succeeds but instantiation raises (abstract, no
   directors) — Phase 1 needs the C++ build from day one, no Python
   prototype possible. `syncDynamicsIntegration` present;
   `bsk_sdk/.../stateVecIntegrator.h` present for `%import`.
4. **Ordering recorded (upstream v2.12.0, == host build).** Default
   integrator is RK4 (ctor). `UpdateState` = gravField →
   `integrateState` (preIntegration[all] →
   `integrate(timeBefore, timeStep)` → postIntegration[all]) →
   ref-override → messages. `setIntegrator` takes ownership, warns on
   synced, errors on null or foreign `dynPtrs[0]` (no dynamic_cast);
   old dynPtrs win (multi-sc preserved). Post-order: mass props → DV
   accumulators + FD omegaDot → energy/momentum → `modifyStates` (MRP
   switch) → force/torque refresh.
   Bonus source facts (same fetch): stock saturation rule matches
   omni's cut semantics; coupled spin + `matrixD -= Js·gs·gsᵀ` confirm
   Update-5 from upstream; `maxWheelAcceleration` is a CLAMP (answers
   the WP1 log-vs-clamp question); BalancedWheels contribute zero mass
   props; hub-only fast path + point-mass path exist; diffusion
   delegates to EOM (stochastic stays out).
   Two probe-side traps hit and fixed: effectors must be
   `AddModelToTask`'d or init SIGABRTs (pure-virtual); FuelTank needs
   `setTankModel` + `setNameOfMassState` (no `fuelMassInit` attr).

Phase 1 entry condition met: composition validated + swap surface
mapped + census pinned. First build: `OmniVecIntegrator` skeleton per
Phase 1, with mirror-parity gates reusing the Gate F harness shape.

## Phase 1 — `OmniVecIntegrator` skeleton over the covered envelope
(COMPLETE 2026-10-12; ledger `swap_p1_parity.json`, gates `test_swap_p1.py`)

Shipped: `include/omni/vec_integrator.hpp` + `src/omni_vec_integrator.cpp`
(`OmniVecIntegrator : StateVecIntegrator`), SWIG-exposed, wheel 0.1.0;
mirror in `omniheyoka/swap.py`. Recipe:
`oi = OmniVecIntegrator(sc); mirror_stock_config(sim, sc, oi, ...);
sc.setIntegrator(oi)` (+ env setters/subscriptions where used).

Design as planned, with these measured deviations:
- RW torques via stock `rwOutMsgs` `u_current` readback (NOT the motor
  message: direct reads skew a tick by task order; log readback is
  timing-identical under all three task orders, proven).
- Thruster mailbox = advance-rule + pre-staged tick-0 snapshot
  (Reset-equivalent staging): fresh/write-once/frozen/advance all match
  stock schedules bit-for-bit.
- GG flag from task-walk (stock has no hub switch); criminally important:
  the GG default (on) caused the first 6.9e-05 smoke gap, fixed by mirroring.
- Env ported whole (slots + per-tick/per-arc refresh); drag + SRP-cannonball
  gated (SRP needed the `cr = C_R - 1` convention map, measured); GG/faceted/
  mag/J2/N-body stock gates follow-up (shared wiring proven by drag).
- Found + fixed en route: absolute-h double count (Update 7; 1-wheel parity
  1.6e-05 -> 6e-14, all gates re-validated to record numbers).

Parity (matched dt=0.1): hub/RW 1e-9 class (measured ~1e-13); thruster
switching residual with proven linear-in-dt ladder; write-once/frozen
single-kink class; drag 4e-6 (stabilized) / 1e-4-tumbling-class bounded;
SRP 4e-8. Rejects: 7/7 loud. CI 3/3. Suite 159 passed.

**Scope:** sims whose effector set is exactly what `OmniSpacecraft`
already models (hub + balanced RW + staircase thrusters + central
gravity + WP5 env cluster). No new physics.

- New `include/omni/vec_integrator.hpp` + `src/omni_vec_integrator.cpp`:
  `class OmniVecIntegrator : public StateVecIntegrator` with
  `integrate(t, dt)` = gather (Phase-0 protocol) → mirror configs →
  symbolic build (cached, rebuilt on config-dirty like
  `buildIntegrator()`) → `propagate_for(dt)` → scatter →
  `modifyStates` equivalents (MRP shadow switch) → energy/momentum.
- Config mirror: read stock effector parameters (masses, inertias,
  wheel `gs`/`Js`/`uMax`, thruster geometry, env links) into the
  existing contributor setters. Reuse `OmniSystem`, do not fork it.
- Step-loop relocation: the arc loop, staged mailbox, per-tick env
  refresh, and saturation granularity currently keyed to
  `OmniSpacecraft::UpdateState` tick boundaries must be re-keyed to
  stock's `integrateState` ordering (Phase-0 item 4). Expect the
  saturation/overshoot and mailbox gates to need re-validation, not
  re-design.
- SWIG: expose via `omni_heyoka.i` (`%import` of the integrator
  interface; same vector-template risk pattern as Update 6 — drop the
  line, keep the member, if rejected).

DoD: on all validated benches (RW closed-loop, thruster T2,
combined), stock-`Spacecraft` + `OmniVecIntegrator` matches the
`OmniSpacecraft`-SysModel arm to ~1e-12 AND matches stock RK4 to the
existing parity numbers; `pytest` + CI greps green; JIT budget held
(≤2× baseline build ms / graph bytes).

## Phase 2 — coverage tail: every swap blocker becomes a WP or a loud reject

The swap is **envelope-scoped by construction**: any sim whose
registered states are all claimed runs; anything else `bskError`s at
first `integrate` (listing the unclaimed state names). Each family
below is a probe-first work package in the WP0–WP7 style; order by
demo need:

| Family | Today | Swap requirement |
|---|---|---|
| Fuel-tank mass depletion (`m/c_B/I` per substep, cf. upstream PR #1488) | labeled out, bookkeeping only (WP6) | `FuelMassContributorT<S>` with mass state + aggregate `c_B(t)`; mass-feedback parity gate |
| `useRWfriction` Stribeck, jitter, `u_min`/`P_max` | modeled module-side in `OmniSpacecraft` (WP1/2) | move into the mirrored contributor path; off-state = factory-off replica |
| Thruster ramps / cutoff lag / min-pulse / blowdown / swirl | default-off setters (WP3b) | same, re-validated under stock ordering |
| CoM offset `r_BcB_B ≠ 0` | modeled (WP4 correction) | mirror `rBcB_` from stock hub; B/C output kinematics on scatter |
| N-body / J2 / drag / SRP / magnetic | modeled (WP5) | mirror links; absent-link truth table (A13/P1) enforced at setup |
| Prescribed motion, flex, slosh, >36-slot packing | never modeled | loud reject (explicit non-goal until a WP claims them) |
| Stochastic diffusion (`equationsOfMotionDiffusion`) | n/a | loud reject, permanently out (Taylor ≠ SDE) |

DoD per family: probe ledger + parity gate + guard test, full
regression green. The envelope table in `DESIGN.md` gains a "swap
coverage" column; the integrator's setup error message cites it.

### Status 2026-10-13 (ledger `swap_p1_parity.json`, gates `test_swap_p1.py`)

| Family | Status |
|---|---|
| Fuel depletion | DONE: contributor + live mass/I_C/c-arms/B-C (no rebuilds); centered + offset tanks gated, fuel bit-exact; single-tank scope, empty-tank labeled |
| Friction (Stribeck) | DONE: stock-exact law + state, mirrored coeffs; spin-up/down gate 1e-14 class; jitter stays loud-reject (DESIGN-compliant) |
| u_min/P_max/Omega_max | DONE by construction (stock applies, we read post-saturation u_current) |
| Cutoff lag | Measured INERT without ramps (10 vs 1000 bit-exact over fire+shutoff); accepted unconditionally, never modeled; must compose with ramps if those land (M6/unconditional-filter reverted as divergent) |
| Min-pulse/ramps/blowdown/swirl | Still rejected (all default-off in stock; verified surface) |
| CoM offset | DONE: thr_offset gate (r 5.6e-09); B/C sign measured vs stock |
| drag / SRP-cannonball | DONE (gated); GG flag mirrored (no bench); faceted/mag/J2/N-body stock gates follow-up |
| Prescribed/flex/slosh | Structural loud-reject (C++ claim check proven by fuel_unmirrored_cxx) |
| Stochastic | Structurally out (single-integrator slot) |

## Phase 3 — drop-in acceptance on unmodified sim scripts

- Take the repo demo + one LEO-drag + one thruster-stationkeeping
  script **unmodified except the two integrator lines**; run
  full-mission parity (att/rate/wheels/fuel-bookkeeping) + speedup vs
  stock RK4, matched dt.
- Multi-spacecraft: `syncDynamicsIntegration` path green
  (`isDynamicsSynced` respected — synced objects must not double-step).
- FSW-in-the-loop: saturation-edge timing, allocator chatter, and
  MRP-switch counts compared tick-by-tick (the tilt-W3 lesson:
  wheels keep a jitter envelope, att/rate keep tight bounds).
- `tests/test_swap_*.py`: setup-reject tests for every out-of-envelope
  family (each trap provably raises before first propagation).

DoD: three scripts green with pre-registered tolerances; speedup
reported honestly (cold-JIT vs warm, per-step, Python overhead split —
same table shape as Update 2/3).

## Phase 4 — docs, packaging, commit

- `DESIGN.md` Addendum 3: swap contract (mirror table, ordering,
  envelope-scoping rule, stochastic-out).
- `BUILD_STATUS.md` Update 7: Phase 0–3 numbers (also repairs the
  stale "What is NOT done" section).
- Wheel ships `OmniVecIntegrator` next to `OmniSpacecraft`; top-level
  `omniheyoka.reset_mrp` / `sigma_indices` exported (outstanding gap).
- Commit the whole `native_omni_heyoka/` tree (currently untracked).

## Risks

- **Ordering mismatch**: stock owns the tick; omni's staged mailbox /
  arc loop assumed omni-owned ticks. Mitigation: Phase-0 item 4 +
  re-validation of every timing gate under stock ordering.
- **Mirror drift**: stock config read at setup can go stale if the
  script mutates effectors mid-run. Mitigation: config-dirty detection
  (rebuild symbolic system when mirror inputs change); document
  mid-run mutation as reject-or-rebuild.
- **Silent-coverage growth**: each new contributor widens the envelope;
  the setup-time claim check + CI grep on the coverage table keep the
  boundary loud.
- **JIT cost per distinct sim**: one ~200 ms first-build per process;
  document the parent-process warm-up pattern for campaigns.

## Explicit non-goals

- Stochastic integrators (SRA/SRI/SIESME paths) — Taylor cannot do SDE.
- Prescribed-motion branching, flex, slosh — reject until claimed.
- Replacing Basilisk's `DynamicObject`/`Spacecraft` classes themselves —
  the stock object stays; only the integrator is exchanged.
- In-place mutation of stock effector internals — the boundary is
  config-mirror + state gather/scatter + messages, same as the
  hijack-vs-rewrite verdicts in `PLAN_ENVELOPE.md`.
