# Envelope-completion plan — WP1…WP7 (Rev 1, adversarial-audit update)

Scope: close every gap listed in the input-acceptance audit so `omniheyoka`
becomes a drop-in stock-Basilisk replacement inside the demo's modeling class
(hub + balanced wheels + thrusters + central/N-body env forces, stock FSW
around a native integrator). **This is a plan; nothing here is implemented
yet.** Every work package ends with its own gate green *and* the full
regression sweep passing (nothing may regress).

> Rev 1 incorporates the 5-surface adversarial audit (RW / thruster /
> environment / telemetry-viz / build-graph boundary), all checked against
> `env/` + `bsk_sdk/` headers/wrappers + `src/include/omniheyoka`.
> Convention below: **SOLID** = verified contract, **FRAGILE** = plausible but
> unprobed, **FANTASY** = contradicted by stock source. Prior Rev 0 language
> that was FANTASY is struck and replaced; gates that were missing are added.

## Decision register (settled by the user, Rev 1 amended)

| # | Item | Decision |
|---|---|---|
| 1 | RW momentum saturation policy | **Exact replica**: direction-aware torque cut at `\|W\| >= Omega_max`, silent, instant resume, tick-detected overshoot (evaluation granularity to be probed: per-tick vs per-substep) |
| 2 | Saturation config API | **Stock-style pair**: `addRatedWheel(gs, maxMomentum, omegaMaxRPM, uMax)` deriving `Js`; `addWheel(gs, Js, uMax)` retained. Setters primary; `Omega_max` is **setter-only** (no config message carries it — see WP1) |
| 3 | Jitter / friction / voltage-IO | **Model Stribeck friction** (`useRWfriction` path with `fCoulomb/fStatic/betaStatic/cViscous/omegaLimitCycle`); jitter to **distributional parity or loud-reject** (bit-exact DoD struck — see WP2); voltage-IO by **reusing stock `MotorVoltageInterface`** (`motorTorqueOutMsg→rwMotorCmdInMsg`), no RW-side change |
| 4 | `useMinPulseTime` + on/off ramps + cutoff/blowdown/swirl | **Measure then model; raise loudly until modeled.** Split WP3a (landed hardening) / WP3b (probe-first). `cutoffFrequency` lag, blowdown/ISP/fuel-mass, swirl, force-msg path all in scope of the probe ledger, all must-raise until gated |
| 5 | `r_BcB_B` CoM offset | **Model the offset, rescoped to aggregate `c_B`**: B-vs-C state semantics, dynamics-about-CoM vs telemetry-about-B, `cPrime`/fuel coupling decided by probe before math lands (see WP4) |
| 6 | N-body / environment | **Full environment via message hijack + per-tick refresh**: N-body gravity (sign-fixed, offsets refreshed per tick, not frozen), J2, drag, SRP (cannonball-first, then faceted box), magnetic (MTB-gated). Earth-radiation **force cut** to field-only/analytic or deferred (no stock force consumer exists) |
| 7 | Vizard / telemetry | **Publish verified payload shapes; label fuel/viz honestly**: `THROutputMsg` 9/9 + `RWConfigLog` 23-field + `scState`/`rwSpeed` publish; caller-side fuel is **bookkeeping, not parity** (no mass feedback); viz via **direct `vizInterface`** (helper path is type-bound and unhijackable) |
| 8 | Build order | **Single batch, ordered waves** — WP0 guards + WP7 layout hygiene are hard predecessors of WP2/WP5; WP3 staging precedes WP5 arc hooks (same `UpdateState` loop). Revertibility holds except these stated orderings |

## Open decisions (RESOLVED, Rev 1 corrected)

| # | Item | Resolution |
|---|---|---|
| a | Jitter / voltage-IO | **Jitter: determinism probe first, then distributional parity or loud-reject.** Stream-exact/bit-exact struck: stock ships opaque `.so` with `RandomGaussianNoiseGenerator::generate`, `mt19937`, `normal_distribution`, plus `random_device`/`urandom` in `libdynamicsLib`; libc++ vs libstdc++ `normal_distribution` differs across hosts; draws/tick/order, per-wheel vs shared instance, `numRWJitter` census all unknown; `JitterFullyCoupled` adds theta states + `aOmega/bOmega/cOmega` that a torque slot cannot express. DoD is statistical (mean/var/correlation envelope) with `bskError` fallback, not `max\|Δτ\|==0`. Voltage-IO reframed: RW effector has **zero voltage bytes**; correct hijack is instantiating stock `MotorVoltageInterface` as-is and wiring its `motorTorqueOutMsg` to `rwMotorCmdInMsg`. Msg name is `ArrayMotorVoltageMsgPayload`. No back-EMF path in the effector; no `useRWVoltageIO` factory kwarg exists. |
| b | Magnetic | **Hijack the message + gate against `MtbEffector` directly.** Do not re-implement WMM — `ReadFunctor` on WMM / centered-dipole `MagneticFieldMsg.magField_N` (inertial N), `τ_B = m×B_B` with `B_B = dcm_BN(σ)·magField_N`. Rev 0 "no torque-bar module / indirect gate only" was wrong: `MtbEffector` exists (`mtbCmdInMsg/magInMsg/mtbParamsInMsg` required, `mtbOutMsg.mtbNetTorque_B`) and implements exactly the omni assembly for identity-`Gt` — use 1-bar identity-`Gt` dipole-within-limits as the **direct stock-τ gate** (mind row-major→col-major `Gt` transpose + `maxMtbDipoles` clamp). Keep `magnetometer` (`tamTrue_S`, identity `dcm_SB`, zero errors) as the independent second reader. Record WMM data-file path + epoch and dipole `g10/g11/h11`. |
| c | SRP eclipse | **Hijack the scalar; fix the contract.** `EclipseMsg` has **one field** (`shadowFactor`; `illuminationFactor` is a deprecated C `#define` alias — Python must use `shadowFactor`). Unlinked eclipse ⇒ **1.0 (full sun, ON)**, not off; `sunStateInMsg` (full `SpicePlanetStateMsg`, position + 1/r² flux scaling, not a bare direction) is **required** (`bskError` if unlinked). Eclipse needs a planet entry (demo Earth-central has none — probe must wire one); record `extrapolateScStateToStepMidpoint`. Stage **cannonball (`radiationPressure`) first** (1-line law, exists — Rev 0 "no cannonball" was wrong), then faceted box with the true two-term law (see WP5). Penumbra crossing needs a matched-dt ladder {0.1, 1, 10}s — 10 s ZOH smears the edge. |
| d | Thrust ramps / cutoff / min-pulse / blowdown | **Setters only — confirmed, and broadened.** `THRArrayConfigMsgPayload.thrusters[i]` (`THRConfigMsgPayload`) carries exactly `maxThrust, rThrust_B, swirlTorque, tHatThrust_B`; ramp times live in header-only `THRSimConfig.ThrusterOnRamp/OffRamp` (`vector<THRTimePair>`, piecewise — not "linear ramp"), `cutoffFrequency` (first-order filter, default 10 rad/s — benches pin 1000 to hide it), `MinOnTime` (default 0.020 s, zeroed when `useMinPulseTime=False`), blowdown coeffs, `MaxSwirlTorque`. FSW `getConfigMessage()` drops even `swirlTorque`. None is hijackable. All default to unprobed; all unmodeled settings must `bskError` (see WP3). |

**Message-contract facts established for WP6** (probed, Rev 1 corrected):
`THROutputMsgPayload` (in `msgPayloadDefCpp/`, not `DefC/`) fields =
`thrustFactor, thrustForce, thrustForce_B, thrustTorquePntB_B,
thrustBlowDownFactor, ispBlowDownFactor, maxThrust, thrusterLocation,
thrusterDirection` (9/9 exact; `thrusterLocation` units mislabeled "inertial"
in stock header — cosmetic); `rwOutMsgs` is **`std::vector<Message<RWConfigLogMsgPayload>*>`**
(Rev 0 `RWConfigLogMsgOutMsgsPtrVector` name was invented; fields deferred honestly —
now known: 23-field `RWConfigLogMsgPayload`: `rWB_B, gsHat_B, w2Hat0_B, w3Hat0_B,
mass, theta, Omega, Js, Jt, Jg, U_s, U_d, d, J13, u_current, frictionTorque,
u_max, u_min, u_f, Omega_max, P_max, linearFrictionRatio, RWModel`; `u_current`
is the closed-loop assert field); `RWArrayConfigMsgPayload` = `numRW,
GsMatrix_B, JsList, uMax` (**no `Omega_max`** — Rev 0 config-by-message for
saturation was circular: `Omega_max` lives only in the output log of the module
being replaced); `AtmoPropsMsg = neutralDensity + localTemp`; wind =
`WindMsgPayload.v_air_N` (full air velocity — consume this, not `v_wind_N`) +
`v_wind_N`; `MagneticFieldMsg.magField_N[3]` (inertial N); `EarthRadiationMsg =
albedoFlux/irFlux + albedoDir_N/irDir_N (flux-weighted, inertial) + timeTag`;
facet = `area, r_CopB_B (torque arm!), nHat_B, rotHat_B, c_diffuse, c_specular`
+ `ProjectedAreaMsg.area`. **Config-by-message viable for `uMax/Js/GsMatrix_B`
and geometry only** — not for `Omega_max`, ramps, cutoff, blowdown, friction
Stribeck set (live `RWConfigPayload` objects, not log messages, are the param
source in shadow setups).

## Ground rules (binding on all WPs)

1. **Graph purity (A3)**: no clamp/branch/sign inside the symbolic graph.
   Saturation, quantizer, staging, atmosphere density, eclipse fraction and
   drag/SRP/magnetic force evaluation stay module-side and enter via input
   parameter blocks (the thruster `q`-slot pattern). Only `+ - * / sqrt`
   (and their heyoka equivalents) in the graph. WP2 `tanh` (if any) is
   module-side only. Enforced by CI grep on symbolic paths, not just review.
2. **Derived layout (A2)**: parameters/inputs stay name-derived; every new
   contributor declares inputs; gates and benches use `par_names()`, never a
   hard-coded count. Existing assertions like `nPars() == 19` become
   name-based. **WP0/WP7 conversion must precede WP2/WP5** (new input slots
   break every count assert). Shipped-code index assumptions
   (`st[6+i]/[9+i]/[12+i]`) convert to `state_offset`-based or get a gate.
3. **Probe ledger first**: each WP begins with a stock measurement recorded
   in the WP's table (config, observable, value) **including pre-registered
   tolerance + matched-dt ladder**. No physics lands without a
   measurement that it reproduces. "Documented tolerance" fitted after the
   fact is rejected.
4. **Matched-dt comparisons** wherever switching/pulsed behavior exists
   (established in Update 5: pulse trains are PWM-frequency dependent);
   cross-dt only as convergence ladders.
5. **No regression**: after each WP, the whole sweep must pass:
   `t2, t3, t5, t7, t8` C++ gates, `pytest native_omni_heyoka/tests`,
   `bench_rw_omni.py`, `bench_thruster.py`, `bench_combined.py`.
6. Each WP independently revertible (separate commit, no cross-WP API
   dependencies) **except stated orderings**: WP0/WP7-layout → WP2/WP5;
   WP3-staging → WP5-arc-hooks; WP4-arm-spec → WP3/WP5 torques.
7. **Loud rejection (amended WP25-B1, Decision 3 locked)**: every bucket-(a)
   (fatal-today) setting `bskError`s rather than runs. Wave-A deliverable
   covers the full trap list (§Risks table, three buckets: (a) fatal-today,
   (b) modeled-behind-setter, (c) informational); each WP closes its own
   slice but may not widen the silent window. No WP is green while its trap
   slice is silent. **Behavioral raise tests (one per (a) item, pre-propagation)
   are the guard evidence; CI/probe-skip is hygiene only, never proof.**
9. **MC dispersion gate (new)**: parametric-`Js` + dispersed-geometry ensemble
   vs stock (Update-5 structural-`Js` regression class).

## Evidence already in hand (so no WP repeats a probe)

| Fact | Value |
|---|---|
| Stock `Omega_max` (HR16) | 6000 RPM, from `rwFactory` source. Kwarg in **RPM**; `RWConfigPayload.Omega_max` in **rad/s**; `RWSpeedMsg.wheelSpeeds` in **rad/s** — triple-unit trap |
| Stock saturation behavior | **MEASURED 2026-10-12 (bundle wp1_sat, BSK 2.12.0, demo bus 750 kg/diag(900,800,600), HR16, Driver rewrite-every-tick):** approach-from-below (5950 RPM, u=+0.2, dt=0.1, 60 ticks) → **overshoot 0.40 RPM, plateau 6000.40 RPM** (cuts faster than naive one-tick `u·dt/Js` = 2.4 RPM — Rev-0 "6003.10" struck); from-above (6050, +0.2) holds 6050 slope 0 (cut + coast); reverse-descent (6050, −0.2) under analysis (full trace in `audit_bundle/results/wp1_sat.json`); dt=0.02 never reaches cap in 1.2 s (5978.3 = 5950 + 24 RPM/s × 1.2 s ✓). Direction-aware cut + silent + instant-resume + tick-detected class confirmed; **tolerance locked: overshoot ≤ 0.5 RPM @ dt=0.1, plateau ±0.05 RPM** |
| Stock `u_min` / `P_max` (measured 2026-10-12, subprocess-isolated creates) | `u_min=1e-6` **accepted**; `P_max=200.0` **accepted**; custom-type spelling is lowercase **`custom`** (`Custom` capital → `RW type Custom is not implemented` exit-kill); `custom` + `Omega_max=-1.0` accepted (=off); **`custom` w/o `Omega_max` echoes `OMEGA_MAX=-1.0` → stays off, never inherits 6000 RPM** ✓. Source-scan default `u_min 0.0` stands (deadband-vs-`fStatic` separation is WP2-F1 business) |
| Jitter determinism (measured) | **J1: `max\|Δ wheelSpeeds\| = 0.0` across 2×400-tick `JitterSimple` runs, fixed `RNGSeed` (set on effector), 1 wheel** → verdict `deterministic-JitterSimple`; `random_device` fear dead for this path. Bit-exact DoD stays struck (libc++ portability); distributional parity evidence-backed. F1–F6 Stribeck one-knob traces + V1–V4 voltage-reuse census all `ok` (`audit_bundle/results/wp2_friction_jitter_voltage.json`); `RWModel` readback `null` in F-section (set-path under review, does not block) |
| Thruster M0–M6 (measured) | Mailbox matrix + quantizer + arc-phase + PWM + ramp-defaults + cutoff-ladder + min-pulse-histogram all green (`wp3_timing.json`); **M6: blowdown `[0.1]` runs with `no fuel tank attached` warning (informative), `swirl_create: accepted` (`MaxSwirlTorque=0.05`), `THRArrayCmdForceMsg_present: true` → force-link must-raise gate locked** |
| CoM-offset ledger (measured) | P-WP4.1 23/23 + `u_current/Omega/Js` tick sample; **P-WP4.2 `\|rBN−rCN\| = 0.100000000` exact, offset settable; consumers `r_BN_N` 0 files vs `r_CN_N` 17 files → C-point confirmed**; P-WP4.3 mass-props msg absent (`Spacecraft` has no `massPropsOutMsg` — aggregate-`c_B` needs another read path), fuel fields confirmed; P-WP4.4 steady-tick (last, not tick0) torque/force/factor sampling after fill-step fix |
| Env analytic gates (measured) | Central-sign inward ✓; Sun tide 5.37e-7; staleness bounds 2.35e-10 @ 0.1 s; `J2=−√5·C̄₂₀` ✓; drag/SRP-faceted/mag-frame numbers + ERP shape + P1 table all `ok` (`wp5_env.json`); module census now via `Basilisk.simulation.X` (prior "missing" was a wrong import path) |
| Telemetry/viz/fuel ledger (measured) | THR 9/9+2, RW 23+2, SCStates 14 + gap audit; **quantizer gap commanded 1.0 vs achieved 0.2 Ns (Δ 0.8)**; viz helper/type-check flags confirmed, direct-`vizInterface` needs plural `scStateInMsgs`/attr-dump path; **fuel-burn `ExecuteSimulation` segfaults on this host → T3 static-only by default (`WP6_RUN_FUEL_SIM=1` for isolated bisection); fuel-tank class census + `FuelTankMsg` fields recorded** |
| Stock `Js` derivation | `Js = maxMomentum / Omega_max(rad/s)` when not given; `Jt = 0.5*Js` (unused in dynamics); both+`Js` → hard `exit(1)`; `custom` with neither → hard error; `u_max` default **-1 = saturation OFF**; `custom` default `Omega_max=-1` = off — omni rated defaults (6000 RPM/0.2 Nm) apply to rated constructors only |
| Stock RW eased kwarg names (corrected) | `u_max`, `Omega_max`, `RWModel` (default `BalancedWheels`), `useRWfriction`, **`fCoulomb, fStatic, betaStatic, cViscous`** (Rev 0 `fViscous` did not exist), `useMinTorque/useMaxTorque, u_min (HR16 1e-5, default off), P_max, Js, rWB_B, Omega, label`. **No `useRWVoltageIO` kwarg exists**; `U_s/U_d` are wheel-type presets, not kwargs; `RNGSeed` is a `SysModel` base property; `numRWJitter` set outside factory |
| RW effector fields available for telemetry | `rwOutMsgs` (`vector<Message<RWConfigLogMsgPayload>*>`), `rwSpeedMsgBuffer`, `numRWJitter`, `writeOutputStateMessages`, `getMaxWheelAcceleration/set` (default 1e6), `getLargeTorqueThreshold/set` (default 10.0) — semantics (log vs clamp) unprobed, probe-first |
| Thruster effector messages | `cmdsInMsg` (`THRArrayOnTimeCmdMsg`), `thrusterOutMsgs` (`THROutputMsgPayload` vector). `THRArrayCmdForceMsg` path exists — omni subscribes OnTime only (must-raise on force link until modeled) |
| `THRSimConfig` defaults (new) | `MinOnTime` 0.020 s (zeroed when flag off); `cutoffFrequency` default 10 rad/s (benches pin 1000); `ThrusterOnRamp/OffRamp` empty (⇔ no ramp) unless set — all to be confirmed by static probe at implementation time |
| Env modules in this build (corrected) | `dragDynamicEffector` (`atmoDensInMsg` **required**, `windVelInMsg` optional), `exponentialAtmosphere`, `facetedSRPEffector` (`sunStateInMsg` **required**, `sunEclipseInMsg` optional→defaults 1.0) + `facetElementBodyInMsgs` + `facetProjectedAreaInMsgs` (via `facetedSpacecraftProjectedArea`), `eclipse` (sun+planet+sc-position required), `magneticFieldWMM`, `magneticFieldCenteredDipole`, **`radiationPressure` (cannonball SRP), `cannonballDrag`, `MtbEffector`, `magnetometer`, `zeroWindModel`, `planetEphemeris`, `spiceInterface`, `facetDragDynamicEffector`, `facetSRPDynamicEffector`, `earthRadiationModel` (field only)** |
| Rev 0 "No SRP-cannonball / no magnetic torque-bar" | **Wrong — struck.** Both exist; MTB upgrades the magnetic gate to direct stock-τ; cannonball stages SRP. What does not exist: residual-dipole disturbance effector, any ERP **force** consumer (`facetERPDynamicEffector` absent — only `earthRadiationModel` references the msg) |
| Gravity harmonics | `gravBodyFactory` supports `useSphericalHarmonicsGravityModel`; repo config keeps `USE_J2 = False`, `J2` constant present. Stock loads **normalized** `Cbar/Sbar` (`J2=-√5·C̄₂₀`); v2.10.0+ fixed 2nd-order loading — pin `maxDeg=2` + assert `cBar` |
| Stock semantics currently implemented | total-inertia `D = I - ΣJs·gs·gsᵀ`, coupled spin, staircase, staged mailbox, arcs, per-wheel uMax (Updates 4–5) |

---

## WP1 — RW momentum saturation (P0, unblocks real missions)

**Goal:** wheels cannot run past `Omega_max`; behavior bit-comparable to stock
within one tick of overshoot.

**Files**: `include/omni/spacecraft.hpp/.cpp` (module-side only; no
contributor change — the input slot is already there), `scratch_ws1/bench_sat.py`,
`tests/test_rw_saturation.py`.

**Design**
- Records per wheel: `omegaMaxRad` (from the stock-style pair, default HR16
  6000 RPM) + existing `Js`, `uMax`.
- Config hijack, corrected: `ReadFunctor` on **`RWArrayConfigMsg` for
  `uMax/JsList/GsMatrix_B` only** (SOLID; probe `GsMatrix_B` flat-layout
  row-vs-col against the allocator). **`Omega_max` is setter-only** —
  absent from the only config message; `RWConfigLog.Omega_max` is output-only
  and circular once stock is replaced. Shadow setups may scrape the live
  `RWConfigPayload.Omega_max` object pre-replacement; deployment uses setters.
- `addWheel(gs, Js, JsDirect...)` stays; new `addRatedWheel(gs,
  maxMomentum, omegaMaxRPM, uMax)` records `Js = maxMomentum/Omega_max` and
  `Omega_max` (over-load, not replacement — stock parity without API churn).
  `custom` (`Omega_max=-1`/`u_max=-1` = off) never inherits rated defaults.
- Per tick (and reused across arcs in the step, mirroring ZOH): for wheel
  *i*, with `W_i` from the state and commanded `u_i`:
  `saturated = (W_i*u_i > 0) & (|W_i| >= omegaMax_i)`; if saturated → `u_i=0`
  else pass through. Check at step start only ⇒ same overshoot class as stock
  (~u·dt/Js, tick-detected). **Granularity probe is load-bearing**: if stock
  evaluates in `ConfigureRWRequests` per tick but integrates substeps frozen,
  a per-arc omni cut under-overshoots — match the granularity, not just the rule.
- `maxWheelAcceleration` / `largeTorqueThreshold`: probe whether stock logs or
  clamps at these; if log-only, mirror with `bskLog` at the same thresholds;
  if clamps, WP1 grows the clamp. (Probe first — do not guess.)
- Unit guard: RPM (kwarg) vs rad/s (payload, speed msg) asserted in the probe.

**Probe vs stock** (single wheel, HR16 `create("Honeywell_HR16", gs,
maxMomentum=50.0, Omega=5950.0)` + `custom`+explicit-`Omega_max` variant,
constant `motorTorque=[0.2,…]`, log `rwSpeedOutMsg.wheelSpeeds` +
`rwOutMsgs[i].u_current/Omega` per tick @ dt=0.1/0.02, both spin directions,
approach from below/above, 1–4 wheels, one tilted axis; plus `Omega_max=-1`
off-path, `u_min` deadband (1e-6 Nm), `P_max` foldback, `GsMatrix_B` round-trip):
overshoot at crossing, plateau RPM, resume slope. Tolerance (locked 2026-10-12):
overshoot ≤ 0.5 RPM @ dt=0.1 (measured 0.40, plateau 6000.40 — Rev-0 2.4 RPM
struck), plateau ±0.05 RPM, resume-slope sign + order correct.

**DoD**: above probe passes for 1/2/3/4 wheels, tilted axes unaffected,
600 s closed-loop unchanged when never saturated (existing gates),
loud-reject on_force-link N/A (RW), tests + bench committed.

## WP2 — RW friction + jitter + voltage (P1, rescoped)

**Goal:** `useRWfriction=True` Stribeck parity; jitter distributional-or-reject;
voltage by reuse. Default off = factory-off replica (note: factory off zeroes
only `fCoulomb` — stray `cViscous/fStatic` persist; mirror the factory
off-state, not literal zeros).

**Probe first** (one knob at a time from HR16 defaults):
1. static: ramp u from negative to positive, find the stiction band vs
   `fCoulomb` vs `fStatic` vs `u_min`;
2. dynamic: constant u > band, equilibrium `W_eq(u)`;
3. free spin-down after u=0: `dW/dt` vs `W` (isolates `cViscous` +
   `omegaLimitCycle` linear zone);
4. hub coupling sign: measure `ωDot_BS` response to friction-only state
   (does friction torque react on the hub? what sign?);
5. **jitter determinism first**: `RWModel=JitterSimple` vs `BalancedWheels`,
   fixed `RNGSeed`, 2000 ticks, 1–3 wheels — determinism across identical runs
   (kills/confirms `random_device` fear); record `numRWJitter` post-Reset,
   `wheelThetas` activity, per-tick increment correlation (draws/tick/wheel).
   Only if deterministic does any replica work start. Also census theta states
   (`thetasState`) — omni has none.
6. friction-param address: read back `rwOutMsgs[i].frictionTorque/u_f` vs
   commanded `fCoulomb/fStatic/betaStatic/cViscous/omegaLimitCycle` (settles
   legacy `u_f/linearFrictionRatio` mapping — treat as legacy/stale until proven).
**Design**: friction torque computed module-side each tick from `W_i` as a
**Stribeck model** (static + velocity-weakening + limit-cycle zone + `u_min`
interplay — Rev 0 `−fC·tanh(W/k)−fV·W` first-guess struck as likely wrong;
budget two model iterations) injected as a **second input slot per wheel**
(`u_i`, `tauF_i`); `ReactionWheelContributorT` includes it in `vecRot` (and
the spin equation if the probe shows stock feeds it there). Jitter, if
deterministic: same per-wheel input slot as noise torque for `JitterSimple`
only; `JitterFullyCoupled` (back-sub `aOmega/bOmega/cOmega` + theta states)
**out of torque-slot scope — loud-reject**. Voltage: instantiate stock
`MotorVoltageInterface`, wire `motorTorqueOutMsg→rwMotorCmdInMsg`; omni
changes nothing (probe: `voltage2TorqueGain/scaleFactor/bias` recorded).
**DoD**: probes 1–4 + 6 reproduced both arms to gate tolerance;
jitter **distributional** (mean/var/correlation envelope) or documented
loud-reject with the determinism evidence; off-by-default verified against the
factory off-state by the current sweep; theta-model enablement loud-rejects.

## WP3a — Thruster landed hardening: mailbox + quantizer + arcs + telemetry + guards (P1)

**Goal**: lock what is measured; publish what is missing; reject what is not.

- Keep: staged mailbox (behavioral parity; mechanism claim flagged FRAGILE —
  extend matrix: same-content advance, two-writes-one-step, frozen-then-advance),
  staircase quantizer `{0,1/6,5/6,1}` (envelope today: `TEST_Thruster`,
  cutoff=1000, `useMinPulseTime=False`, dt=0.1 — broaden dt/type coverage),
  sub-step arcs `[0,t_j)` eps-merged (phase assumption start-firing recorded;
  discriminating half-step-shift probe added — M4).
- Publish `thrusterOutMsgs` (9/9 fields, about-B torque **unshifted** — see WP4).
- Guards (all `bskError` until WP3b models them): `useMinPulseTime=True`,
  non-empty `ThrusterOnRamp/OffRamp`, `cutoffFrequency<~100`, blowdown/ISP
  coeffs present, `swirlTorque/MaxSwirlTorque≠0`, `thrForceMapping` link,
  FSW `THRArrayConfigMsg` paths that drop swirl/ramps/cutoff.
- Probes M4 (arc phase) + M5 (over/negative-duty, 10 vs 20 Hz PWM, dt ladder
  to the continuous-duty limit with Richardson fit).

**DoD**: T2 chatter @ dt=0.01 holds; M4/M5 green; guards green (each trap
provably raises); telemetry field-diff green.

## WP3b — Min-pulse + ramps + cutoff + blowdown/swirl (P1, probe-first, no physics before ledger)

**Probe ledger M1–M3/M6** (all single-thruster, fresh-write, `thrusterOutMsgs`
+ `scState.omega` recorded):
- M1 ramp defaults+shape: static read-back of `ThrusterOnRamp/OffRamp` lens +
  step-response with a 2-pair 0→1/0.05 s ramp → `ThrustFactor(t)` trace
  (decides piecewise-vs-linear-vs-filter).
- M2 cutoff ladder: `cutoffFrequency ∈ {10 (default), 100, 1000}` step-response
  → fit lag τ; ramp×cutoff composition.
- M3 min-pulse histogram: `useMinPulseTime=True`, `MinOnTime=0.02`, dt=0.01,
  on-times {0.2,0.5,1.0,1.5}×MinOnTime + chatter pattern → fired-duration
  sequence (extend/merge/suppress rule from data; `THROperation`
  `fireCounter/PreviousIterTime` memory assumed until measured).
- M6 blowdown/swirl/offset truth: `thrBlowDownCoeff` + fuelTank run →
  `thrustBlowDownFactor` trace; SEP swirl run → axis torque; `r_BcB_B=[0.1,0,0]`
  thruster-only run → arm shift (joint with WP4 P-WP4.4).
**Design only after ledger**: valve memory in staging; ramp sub-arcs (`k`,
  `O(dt·k)` with data); lag state for cutoff; blowdown/fuel-mass state (or
  honest out-of-scope label in DoD); swirl axis torque; force-msg subscription
  if needed.

## WP4 — CoM offset `r_BcB_B` (P2, rescoped — probes before math)

**Goal**: aggregate-CoM-offset runs modelable; publishers distinguish B from C.

**Probes first** (each <1 day on working Basilisk env):
- P-WP4.1 payload census: `rwOutMsgs` full 23 fields + `RWSpeedMsg` on closed loop.
- P-WP4.2 B-vs-C: stock `r_BcB_B=[0.1,0,0]` run, log `r_BN_N` vs `r_CN_N`,
  `omegaDot_BN_B`, `TotalAccumDV*`; identify IMU/nav/CSS/viz consumers by code
  read + log diff. **Decides C-point vs B-point integrator design** — omni
  integrates C today; stock hub states are B. Porting block signs across that
  boundary is the likeliest sign bug; independent anchor required (stock
  finite-difference of `scState`, or BSK EOM derivation — same-team symbolic
  cross-check alone is correlated error).
- P-WP4.3 aggregate CoM: offset hub + offset wheels (`rWB_B≠0`) + RW mass +
  fuel-tank depletion → aggregate `c_B/cPrime_B` (via mass-props output).
  Expected: hub-only formulas fail → WP4 implements aggregate `c_B(t)`.
- P-WP4.4 moment-arm audit: single-thruster torque step with offset —
  `thrustTorquePntB_B` (about B, **must equal unshifted `r_thr×F`**) vs achieved
  `omegaDot` (about CoM, shifted by `(r_thr−c_B)×F`). Locks the
  dynamics-vs-telemetry distinction Rev 0 omitted.
**Design (after probes — CORRECTED 2026-10-12 by the fire-step bench):**
stock fire step 0.18461538 == shifted-arm/I to 8 digits, so the input `I` is
treated as ALREADY about the CoM: NO parallel-axis term in `D`, NO `c×mg`
gravity moment, NO B/C transport coupling (a B-point formulation with all
three was implemented first and REJECTED: idle drift + 0.685× fire scaling).
The offset enters ONLY through (a) shifted dynamics arms `(r−c_B)×F` at the
wrench consumers (thruster; env-force arms per the spec below) and (b) the
B/C output kinematics. Concretely: hub blocks UNCHANGED (A/D/I/GG as at zero
offset); `ThrusterContributorT` carries structural `comOff_` wired from
`hub_.rBcB` in `refresh()`; outputs `r_BN_N = r_CN_N + Rᵀ·c_B`,
`v_BN_N = v_CN_N + Rᵀ·(ω×c_B)` (R = C_BN); `omegaDot` **published** (Rev-0
"stays zero" struck); wheel `rWB_B` fenced (balanced-only negligible unless
rotor mass counts toward CoM — probe decides; jitter-imbalance makes it
load-bearing); friction is a shaft torque (**no moment arm**); thruster
**dynamics** arms shift, **telemetry** about-B does not; env-force arms
(drag/SRP/albedo) shift identically (spec added — was missing).
**DoD**: offset bench green; zero-offset unchanged; `add_hub` throw replaced by
aggregate path + guards.

## WP5 — Environment: gravity N-body + J2 + drag + SRP + magnetic (P2–P3)

**Goal**: the wrench-producing environment effects, all module-side with the
ForceTorque channel, one par-input block per effect (graph stays pure).

**Framework**: `EnvContributorT<S>` (no states) declaring a 6-vec input per
effect; module computes wrench from state + config each tick (per arc, for
accuracy where the effect is fast-varying); all general `+ - * / sqrt` work
stays in the graph only for gravity point masses. **WP3 staging first** —
WP5 hooks the arc interface (same `UpdateState` loop, not independently
revertible — sequenced).

| Effect | Model (corrected) | Stock gate |
|---|---|---|
| N-body gravity | **`a = Σ μᵢ(rᵢ−r)/\|rᵢ−r\|³` (sign fixed)**, offsets **refreshed per tick** from `SpicePlanetStateMsg`/`planetBodyInMsg` (ZOH staleness ~1e-10 m/s²; frozen-whole-run struck — Sun tide 1.4e-6 m/s² is fatal over 3 yr). Demo rotating `SunEphemeris` is the time-dependent source | `gravBodyFactory` multi-body (Earth central + Sun + Moon); central-at-origin special case; assert per-tick offset equality then ‖Δa‖ ≤ 1e-9 @ dt=0.1; sign check central-only first |
| J2 | **ECI-equatorial analytic** (zonal symmetry — no rotating-frame detour), config `J2`. Probe asserts loaded normalized `cBar` vs `C.J2` (`J2=−√5·C̄₂₀`), `maxDeg=2` pin (higher-degree leakage otherwise) | J2-only coefficient file; equatorial + inclined; tolerance from higher-degree residual (~1e-9 rel). Mission `USE_J2=False` — standalone bench |
| Drag | **`F_B = ½ρCdA\|v\|²(−v̂)`, `τ = comOffset×F`, `\|v\|≤1e-12→0`, `(1+densityCorrection)≥0`**, ρ hijacked from `AtmoPropsMsg.neutralDensity`, **`v_rel = v_BN_N − v_air_N`** (full field; unlinked wind ⇒ inertial velocity, ~6% LEO error — `zeroWindModel` corotation is the baseline probe). `atmoDensInMsg` required in stock (`bskError` if unlinked — omni off-by-default is a deliberate deviation, labeled) | `dragDynamicEffector` + `exponentialAtmosphere` + `zeroWindModel` on a **LEO fixture (~400 km)** — mission 12 000 km altitude has ~zero density; record `baseDensity/scaleHeight/planetRadius/envMaxReach` + `coreParams` |
| SRP | **Cannonball first** (`radiationPressure`: area, `coefficientReflection`, `sunEphmInMsg`, eclipse scalar), then faceted box with the **true law per facet: `F = −P·A_proj·[(1−c_spec)·ŝ + 2·(c_diff/3 + c_spec·cosθ)·n̂]`, `τ = r_Cop×F`, `P ∝ (AU/r_SB)²`, `cosθ = A_proj/A`**, all × hijacked eclipse scalar. Prove `facetedSpacecraftProjectedArea == max(0,n̂·ŝ)A` on the occlusion-free box before re-deriving; either consume `facetProjectedAreaInMsgs` or prove reduction | `radiationPressure` then `facetedSRPEffector` + occlusion module on 6-facet box (record all facet fields) + `eclipse` (planet entry + flag recorded); penumbra-crossing arc @ {0.1,1,10}s; assert per-facet F, total F+τ, tick-by-tick eclipse equality |
| Earth radiation | **CUT from force parity** (no stock force consumer in this build). Field-only probe (record `albedoFlux/irFlux/dirs/timeTag` over day-night orbit; omni asserts message equality) + optional analytic cannonball-with-flux-weighted-dirs gate with documented tolerance, no stock-parity claim. "Nearly free" struck | `earthRadiationModel` field equality only, or defer |
| Magnetic | **`τ_B = m×B_B`, `B_B = dcm_BN(σ)·magField_N`** hijacked from WMM / centered-dipole; epoch + coeffs recorded; per-tick ZOH + matched-dt ladder (attitude-rotated field is the fast variable) | **Direct**: `MtbEffector` 1-bar identity-`Gt` (dipole within limits) `mtbNetTorque_B` equality **+ independent**: `magnetometer` (identity `dcm_SB`, zero errors) `tamTrue_S` vs omni `B_B`; both field models |

**API**: `addEnvironmentBody(mu, offsetN-refreshed-per-tick)`, `setJ2(J2)`,
`setDragParams(...)`, `addSRPFacet(area, normal_B, r_Cop, c_diffuse, c_specular)`
(or cannonball preset), full `SpicePlanetStateMsg` sun source (not bare
direction — bare direction bakes ±3.4%/yr flux error), `setMagneticMoment(...)`
+ presets; par-input blocks appear in `par_names()` (`gX..`, `dragF..` etc.).
All field inputs via `ReadFunctor`.
**DoD**: per-effect bench (single effect on, both arms incl. −attitude sense,
matched dt + ladder {0.02,0.1,1.0}s) + combined one with **pre-registered**
tolerance; JIT assert green; absent-link truth table green (P1).

## WP6 — Telemetry + Vizard parity (P3, rewired)

**Goal**: stock-analyst tooling works against the omni arm — honestly labeled.

**Probe**: THR 9/9 (done statically) + RW 23-field dump (P-WP4.1) + viz
consumer read + fuel/DV truth.
**Design**: publish `thrusterOutMsgs` (all 9 fields, about-B torque unshifted)
and `rwOutMsgs` (full `RWConfigLog` shape incl. `u_current/Omega/frictionTorque`)
from `writeMsgs`; fill `omegaDot_BN_B` (currently zero — gap); fill or
explicitly zero-label `TotalAccumDV*/nonConservativeAccelpntB_B` (currently
silent zeros); keep `scStateOutMsg`/`rwSpeedOutMsg`. Fuel: **bookkeeping only**
— commanded-throttle accounting diverges three ways (commanded≠achieved via
staircase; no blowdown/ISP scaling; **no mass feedback** — omni `mass` fixed,
stock depletes `m/c_B/I` per substep, cf. upstream PR #1488); optionally publish
`FuelTankMsg`-shaped msg labeled non-parity. Config-by-message limited to
`uMax/JsList/GsMatrix_B` (WP1/WP5 geometry) — `Omega_max`-via-message struck.
Vizard: **bypass `vizSupport`** (`isinstance(sc, Spacecraft)` + `gravField`
harvest fails structurally for `OmniSpacecraft`) — configure `vizInterface`
directly (à la `test_dataFileToViz`: manual `scStateInMsg`/`thrInMsgs`/
`rwInMsgs` to omni msgs; this repo's demo passes `[scObject]`+`thrEffectorList`
and no `rwEffectorList` — RW viz already demo-out-of-scope). Record timestamps
(`CurrentSimNanos`+`moduleID`), rates, `thrusterLocation/Direction` echo
convention, B-vs-C render point.
**DoD**: message-field diff on a recorded closed-loop run (identical field
sets, values within gate), **direct-`vizInterface`** smoke green + helper-path
negative control recorded, fuel divergence sized for 600 s vs stationkeeping
horizons.

## WP0/WP7 — Guards + Test/Doc housekeeping (P0 hygiene, hard predecessor)

- **WP0 loud-reject guard (Wave-A deliverable, new)**: stock-config
  introspection (rwFactory kwargs, `THRSimConfig` ramps/cutoff/MinOnTime/
  blowdown/swirl, `USE_J2`, `r_BcB_B`, voltage subscription, force-msg link,
  jitter model) that `bskError`s when the omni arm can't model it. Today every
  trap runs silent — no WP lands without its guard slice.
- Name-based layout conversion (`nPars` → `par_names()`, `layout.py` helpers
  into benches/tests, `state_offset`-based indices, CI grep gates for A2/A3)
  **before** Wave B.
- `tests/test_nativerw.py`-style suite per new option (off-parity + on-parity).
- `BUILD_STATUS.md` Update 6 per WP; `DESIGN.md` A13 (env/telemetry contract)
  + A14 (absent-link truth table + B-vs-C + fuel/viz labels).

---

## Hijack-vs-rewrite evaluation (all WPs, Rev 1 corrected)

**Established by probe**: `bsk_sdk/include` ships declaration + base-class
headers (`atmosphereBase.h`, `magneticFieldBase.h`, …; concrete declarations
like `reactionWheelStateEffector.h`, `THRSimConfig.h` included) + msg payload
contracts in `architecture/msgPayloadDefC/` (+ `DefCpp` for THR-output), and
**no linkable libs** (`bsk_sdk/lib` = cmake only; zero `.a/.so`). Concrete
modules exist only as compiled Python extensions inside `Basilisk`. ⇒
**code-level hijack is impossible; the hijack boundary is (a) `SysModel`
subclassing via SDK helpers (already done) + (b) messages.** Anything whose
stock computation is an internal effector behavior must be rewritten. Verdicts:

| Item | Stock module | Consumable message | Verdict |
|---|---|---|---|
| FSW (nav/ref/err/ctl/alloc) | stock **compiled** modules (Rev 0 "scripts" fixed) | — | **Total hijack by reuse** (already the design) |
| WP1 saturation | internal to RW effector | `RWArrayConfigMsg` (`uMax`,`JsList`,`GsMatrix_B`) only — **no `Omega_max`** | **Rewrite** (measured); config hijack for geometry only; `Omega_max` setter-only |
| WP1 `u_min`/`P_max` | internal | params on live objects | **Rewrite or loud-reject** (new row) |
| WP2 friction | internal Stribeck | live `RWConfigPayload` Stribeck set (not `RWConfigLog` legacy `u_f`/`linearFrictionRatio`, not `fViscous`) | **Hijack params (corrected address) + rewrite physics** |
| WP2 jitter | internal (+ theta states for FullyCoupled) | `RWConfigLog.U_s, U_d` (presets); `RNGSeed` (base attr) | **Rewrite-distributional or reject** (bit-exact struck) |
| WP2 voltage-IO | separate `MotorVoltageInterface` module | `ArrayMotorVoltageMsgPayload` → `motorTorqueOutMsg` | **Reuse stock module as-is** (reframed; 0 omni change) |
| WP3a mailbox/quantizer/arcs | internal | — | **Rewrite — done in envelope** (phase probe outstanding) |
| WP3b min-pulse | internal state machine (`THROperation` memory) | — | **Rewrite probe-first** (was FANTASY-designed) |
| WP3b ramps/cutoff | internal (`THRSimConfig` piecewise + first-order lag) | none | **Rewrite probe-first + lag state** (was FANTASY-designed) |
| WP3b blowdown/ISP/fuel-mass/swirl/force-msg | internal / `FuelTank` StateEffector | none hijackable | **Rewrite or honest out-of-scope + must-raise** (new rows) |
| WP4 CoM offset | internal EOM (B-point) + aggregate `c_B`/`cPrime` | — | **Rewrite** (rescoped; symbolic cross-check + independent anchor) |
| WP5 N-body gravity | `gravBodyFactory` (mu + planet msgs) | config + positions per-tick | **Rewrite analytic (sign-fixed) + hijack offsets per tick** |
| WP5 J2 | harmonic (file-based, normalized) | — | **Rewrite analytic ECI + normalization/maxDeg gate** |
| WP5 drag | `exponentialAtmosphere` → `AtmoPropsMsg` (`neutralDensity`), `windVelInMsg` (`v_air_N`) | **yes** (density required-link; wind optional) | **Hijack inputs, rewrite assembly (arm+guard+correction)** |
| WP5 SRP cannonball (new stage) | `radiationPressure` → eclipse scalar + sun state | **yes** | **Hijack inputs, rewrite 1-line law** |
| WP5 SRP faceted | `facetedSRPEffector` + occlusion + `eclipse` → `shadowFactor` + sun state + facet msgs | **yes** | **Hijack inputs, rewrite two-term law + projection proof** |
| WP5 albedo/IR | `earthRadiationModel` → `EarthRadiationMsg` (no force consumer in build) | field yes, force no | **Field hijack only; force CUT or analytic-only** (was "cheap" — struck) |
| WP5 magnetic | WMM / centered-dipole → `MagneticFieldMsg` (`magField_N`) | **yes** | **Hijack message, rewrite `τ=m×B`**, gated direct via `MtbEffector` + independent via `magnetometer` |
| WP6 thruster/wheel telemetry | **we produce** (`THROutputMsgPayload` 9/9, `RWConfigLog` 23-field) | — | **Rewrite** (publish; about-B vs about-CoM distinguished) |
| WP6 fuel accounting | `fuelTank` **StateEffector** with own integration (not a message consumer) | no | Bookkeeping only (divergences sized); optionally publish `FuelTankMsg`-shaped msg labeled non-parity |
| WP6 Vizard | helper type-bound (`Spacecraft` + `gravField`); `vizInterface` direct-configurable; viz reads THR/RW/SC msgs | msgs yes, helper no | **Publish msgs + bypass helper** ("publish what viz reads" alone struck as sufficient) |
| Gates | stock effectors as shadow | — | **Hijack as reference generators** |

**Score**: every dynamics behavior the RW/thruster effectors own (saturation,
`u_min`/`P_max`, Stribeck friction, jitter, min-pulse, ramps, cutoff lag,
blowdown, swirl, CoM offset, EOM) is a rewrite — those have no observable
interface, only state outputs. Every *environment field* the demo can hit
(atmosphere density, full air velocity, eclipse scalar, sun state, magnetic
field, body mu/positions, Earth-radiation field, thruster/RW geometry, max
thrust, uMax, Stribeck coefficients) is a message hijack — the expensive model
code is reused, we only own the last-mile assembly. Net: WP5's physics surface
is `F_B = ½ρCdA|v|²(−v̂) + τ=comOffset×F`, the **two-term** facet law with
`r_Cop×F` and `(AU/r)²`, `τ = m×B` via `dcm_BN`, ECI zonal J2 — plus one honest
dt-ladder each.

**Design consequence**: message-hijacked inputs read via `ReadFunctor`
(exactly like `cmdsInMsg`/`rwMotorCmdInMsg` today); absent-link behavior
follows the **per-effect truth table** (new): drag-density unlinked → stock
`bskError` (omni off = labeled deviation); wind unlinked → inertial-velocity
(zero-fill); SRP-sun unlinked → `bskError`; eclipse unlinked → 1.0 (ON);
eclipse-no-sc/planet → `bskError`; MTB any-of-3 unlinked → `bskError`.
New input blocks appear in `par_names()` and are visible to the layout tests.

| Wave | WPs | Parallel? |
|---|---|---|
| A0 | WP0 loud-reject guard + WP7 layout hygiene (name-based, CI greps, `state_offset`) | first — hard predecessor of all below |
| A | WP1 (module-side saturation), env probe ledger P1–P10 | yes (after A0) |
| B | WP2 Stribeck + jitter-determinism + voltage-reuse; WP4 probes → math (contributor physics) | yes, different contributors (after A0) |
| C1 | WP3a hardening + telemetry publish + guards | after A0 |
| C2 | WP3b probe-first timing + WP5 env framework+effects (hooks WP3 arc interface) | sequenced C1→C2 (same `UpdateState` loop) |
| D | WP6 telemetry/viz/fuel-labels, final sweep, docs (A13/A14) | after B/C |

## Validation matrix at batch close

| Gate | Required after batch |
|---|---|
| t2 Gate A/B (RW numeric, H) | unchanged (5.9e-16 / <1e-6); +saturation probe variants (both arms, tilted, granularity) |
| t3 Gate F, t7 thruster algebra, t8 combined | unchanged numbers; t7 harness single-quantize boundary kept |
| t5 tilted | unchanged (5.6e-16) + offset variant (after WP4 rescoped tolerance) |
| `bench_rw_omni.py` Part A | ≤ 1e-12 deg; + saturation-cap part; + Stribeck part; + offset part |
| `bench_thruster.py` T2 | ≤ current envelope; + M4 phase + M5 PWM/duty rows; WP3b rows only after ledger |
| `bench_combined.py` | current matched-dt rows; + env-effects rows (LEO vs mission fixtures split) |
| `pytest native_omni_heyoka/tests` | all green, new suites per WP (off-parity + on-parity + guard-raise tests) |
| New per-effect env benches | each ≤ **pre-registered** tolerance vs stock module (N-body, J2, drag-LEO, SRP-cannonball, SRP-faceted+penumbra ladder, magnetic-direct+mag cross-check, ERP field-only) |
| Guards | every Risks-trap setting provably `bskError`s when unmodeled |
| JIT + MC | `[GRAPH]` bytes/build-ms ≤ 2× baseline; dispersed-`Js`/geometry MC ensemble vs stock |
| Bundle ledger (locked 2026-10-12) | `audit_bundle/results/bundle_summary.json`: **7/7 probes, 0 fail files**, BSK 2.12.0 / heyoka 7.12.1 / macOS arm64; wp1 edge re-run pending (tuple-accessor fix landed); wp6 T3 burn isolated (`WP6_RUN_FUEL_SIM=1`), viz sim isolated (`WP6_RUN_VIZ_SIM=1`) |

## Implementation order (locked from bundle evidence)

1. **WP0** — loud-reject guard + name-based layout + CI greps (unblocks everything; bundle proves every trap currently runs silent).
2. **WP1** — module-side saturation cut (tolerance locked above) + `addRatedWheel` + `RWArrayConfigMsg` geometry read + `maxWheelAcceleration`/`largeTorqueThreshold` log-vs-clamp verdict + `u_min`/`P_max` deadband/foldback (edge re-run first: `../../env/bin/python audit_bundle/probes/wp1_sat.py`).
3. **WP2 + WP4 probes→math (parallel)** — WP2: Stribeck module-side second input slot from F1–F6 traces (two model iterations budgeted), `JitterSimple` distributional slot (determinism proven, bit-exact stays struck), voltage reuse (no omni change); WP4: B-vs-C decision (C-point confirmed 0-vs-17 files) → aggregate-`c_B` blocks + about-B/about-CoM telemetry split + independent anchor.
4. **WP3a** — quantizer/mailbox/arcs already landed; add `thrusterOutMsgs` publish + all seven must-raise guards (`useMinPulseTime`, ramps, cutoff<~100, blowdown, swirl, force-link, config-drop).
5. **WP3b + WP5 (sequenced C1→C2)** — WP3b timing models from M1–M3/M6 ledger; WP5 per-effect benches from analytic gates (cannonball-first SRP, MTB-direct magnetic, LEO drag fixture, ECI J2, per-tick N-body refresh). ERP stays field-only.
6. **WP6 + sweep + docs** — publish THR-9/RW-23/SC gaps, direct-`vizInterface` smoke (`WP6_RUN_VIZ_SIM=1` on host), fuel bookkeeping labeled non-parity (burn sim stays isolated until the segfault is bisected: `WP6_ONLY=T3 WP6_RUN_FUEL_SIM=1`), A13/A14, full matrix green.

## Risks

- **Discretization mismatch**: stock computes environment effectors per tick;
  we mirror with per-tick/par-arc ZOH — expect O(dt) gaps, honest ladders only.
  Sharpest: penumbra (~10 s) vs `DT_CONTROL=10 s` smears the edge a full step.
- **JIT growth**: keep effects as par-inputs; enforced by §Ground-8 assert, not monitoring.
- **Friction/offset physics subtleties**: Stribeck + aggregate-`c_B` references
  written before the C++ lands (as in t7/t8) **plus independent anchor**
  (stock finite-diff / BSK derivation) — same-team transcription alone is
  correlated error.
- **Eclipse/facet SRP complexity**: cannonball-first; faceted scoped to
  occlusion-free box with projection proof; full `facetedSRPEffector` geometry
  stays out.
- **Silent-divergence traps → loud-reject table (WP25-B1 amended, three
  buckets; Decision 3 locked: amend-then-enforce). Bucket (a) every item
  raises `bskError` at the Reset + UpdateState heads before first
  propagation (`checkUnsupported()`); disarmed (default) configs run.
  Behavioral raise tests: `tests/test_wp0_guards.py` (one per (a) item).**

  Bucket (a) — fatal-today (default set; each row is enforced):

| Trap | Trigger | Required behavior |
|---|---|---|
| Force-msg link | `THRArrayCmdForceMsg` linked (`forceMsgOn_`) | `bskError` (no model by design) |
| `u_min` non-default | `setUMin(≠0.0)` (stock default 0.0 = off) | `bskError` (unmodeled deadband) |
| `P_max` non-default | `setPMax(≠200.0)` (stock default 200.0) | `bskError` (unmodeled foldback) |
| `maxWheelAcceleration` non-default | `setMaxWheelAcceleration(≠1e6)` (stock default 1e6) | `bskError` (log-vs-clamp unprobed) |
| `largeTorqueThreshold` non-default | `setLargeTorqueThreshold(≠10.0)` (stock default 10.0) | `bskError` (log-vs-clamp unprobed) |
| `JitterFullyCoupled` | `setRWModelJitterCoupled()` | `bskError` at the setter (setter-gated: no `RWModel` member exists on the SysModel surface for setup-time detection; stock-side values are rejected vec-side by `swap.py`) |
| Non-identity `Gt` | `setMtbGtMatrix()` non-identity (identity = disarmed) | `bskError` |
| `rotHat≠0` facet | `setSRPFacetRotHat()` nonzero (zero = disarmed) | `bskError` |
| `nBodies>2` unlinked | 3rd `addEnvironmentBody()` (no per-index msg vector; static offsets + sun/moon refresh only) | `bskError` |
| Prescribed motion | `setPrescribedMotionEnabled(True)` | `bskError` (unmodeled) |
| Flex | `setFlexEnabled(True)` | `bskError` (unmodeled) |
| Slosh | `setSloshEnabled(True)` | `bskError` (unmodeled) |
| Multi-tank | `addFuelTank()` 2nd registration (`setFuelParams()` is the single-tank bookkeeping surface) | `bskError` |
| Stochastic / diffusion | diffusion enable | (WP25-B2.4) — NOT enforced in B1 |
| Mid-run stock mutation | post-init stock-side edits | (WP25-B2.4) — NOT enforced in B1 |

  Bucket (b) — modeled-behind-setter (never fatal; each WP owns its model +
  tests): saturation (WP1); Stribeck friction `useRWfriction` iter-1 (WP2);
  `JitterSimple` distributional (WP2); `useMinPulseTime` suppress (WP3b);
  `ThrusterOnRamp`/`OffRamp` shape (WP3b); `cutoffFrequency` accepted
  (WP3b); blowdown hold-1.0 + fuel bookkeeping (WP3b/WP6 — WP25-C3 AMENDMENT
  LOCKED: warn-and-hold-1.0 SHIPPED, escalation to `bskError` REJECTED, per
  the decision record at `src/omni_spacecraft.cpp:1174-1180`: *"warn-and-hold-1.0
  SHIPPED. Coeffs are recorded but factors hold 1.0 without a tank; escalation
  to bskError was REJECTED (stock only warns on the no-tank config the M6 ledger
  cell pins green — raising would diverge from stock, not match)"*.
  Decision-4's "must-raise until gated" therefore does NOT apply to the
  no-tank blowdown config — the M6 `blowdown [0.1] runs with no fuel tank
  attached warning` ledger cell is the gate); swirl-global
  (WP3b); harmonics/`USE_J2` analytic (WP5); offset `r_BcB_B` (WP4);
  per-effect env truth table (WP5; `atmoDensInMsg`/`sunStateInMsg`/MTB links
  still `bskError` when required-but-unlinked).

  Bucket (c) — informational (never fatal): voltage reuse (stock
  `MotorVoltageInterface` stays in the task, `motorTorqueOutMsg` ->
  `rwMotorCmdInMsg`; zero omni-side change); unit-convention probes (RPM vs
  rad/s assert in the probe, not a guard).

## Explicitly out even after this batch

- Independent ephemeris propagation in omni (offsets **refreshed per tick**
  from stock messages — frozen-whole-run struck; no analytic Kepler propagator).
- Real effector-state API parity (`thrusterStateEffector`), prescribed
  motion, fuel slosh, >36-slot effector packing (`MAX_EFF_CNT=36` confirmed).
- Bit-exact jitter stream (struck to distributional-or-reject: libc++/
  `random_device`/draw-order opaque — loud rejection rather than a guessed stream).
- Earth-radiation force parity (no stock consumer in build — field-only or deferred).
- Fuel-mass-depletion dynamics + `vizSupport`-helper compatibility (bookkeeping +
  direct-`vizInterface` instead; labeled, not parity).
- Any stock behavior whose RNG/distribution contract turns out not to match
  in-process (WP2 probe 5 verdict: fall back to loud rejection rather than a
  guessed stream).
