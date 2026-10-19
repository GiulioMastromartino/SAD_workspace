# Audit_0-1 — Adversarial audit of `Python/native_omni_heyoka` committed work

Date: 2026-10-13. Method: 6 parallel adversarial subagents (physics core / RW
channel / thruster-timing / env+offset / swap+spacecraft module / build-graph-guards-hygiene),
then lead spot-checked every Critical/High claim against source before compiling.
No files were modified by the audit except this report. All `file:line` refs are
relative to `Python/native_omni_heyoka/`.

Scope: `DESIGN.md`, `BUILD_STATUS.md`, `PLAN_ENVELOPE.md`, `PLAN_INTEGRATOR_SWAP.md`,
`include/omni/*` (10 headers), `src/omni_spacecraft.cpp` (1298 L),
`src/omni_vec_integrator.cpp` (1025 L), `src/omni_heyoka.i`, `src/omni_taylor_api.cpp`,
`omniheyoka/*.py`, `tests/*.py` (13 files), `scripts/ci_envelope_checks.sh`,
`audit_bundle/probes/*` + `audit_bundle/results/*` (skimmed), `scratch_ws1/t*.cpp` harnesses.

Environment limit: no `git`/`python3` on this Linux container, no Basilisk build;
host-only (macOS + built wheel) claims are verified as code+ledger consistency, not
by re-execution. Every `importorskip`-gated sim test skips here by construction.

## Verdict summary

| Area | Verdict |
|---|---|
| Option-A core math (MRP DCM + kinematics, relative-h + `D=I−ΣJs·gs·gsᵀ` + coupled spin, Kepler+GG, layout A1/A2, gravity-first A5, autonomy A6, graph purity A3) | **Sound.** Code matches DESIGN Addenda + Update-7/8. Stale comments/docs contradict it (F3, F6, F13) but math is right. |
| RW saturation rule + tauF plumbing + mailbox + quantizer + arcs + force guard | **Sound** in code; several surrounding claims/gaps are not (see High/Medium). |
| B/C offset sign | **CRITICAL open contradiction:** two shipped paths implement opposite signs (C1). |
| Cutoff-lag model | **HIGH contradiction:** SysModel path ships the filter Update-8 proved divergent and reverted in the swap path (H1). |
| Loud-reject envelope (WP0) | **HIGH gap:** `checkUnsupported()` raises only for forceMsg; trap-table "every trap raises" is contradicted (H2). |
| Telemetry honesty (RW 23-field, DV, fuel) | **HIGH/MEDIUM gaps:** shape parity with placeholder values + frame bug (H3–H5). |
| Test strength (env/offset, jitter ledger) | **HIGH/MEDIUM gaps:** vacuous zeros-vs-zeros comparisons possible; SRP-shifted unproven; jitter horizon overstated. Ledger `ok` ≠ law matched in places. |
| Build/graph/hygiene | Graph purity + recipe + string-bridge removal + pars-push + `nWheels` + relative-h fixes all **verified closed**. WP0–WP6 **unbuilt on Linux by the project's own statement**; SWIG vectors / `Message` resize / payload fields are correctly flagged host risks, not green. WS2 layer stale-as-labeled; A7 exports missing; WS2 absolute-h hazard still armed. |
| Docs vs code | Systematically version-skewed (19→13→16 pars, 7/7→11/11 probes, B/C sign, cutoff). Math unaffected; consumers must read Addenda/Updates, not body text. |

## Critical

### C1. B/C sign: `OmniSpacecraft` (`+Rᵀc`) vs `OmniVecIntegrator` (`−Rᵀc`) — CONFIRMED
- Evidence: `src/omni_spacecraft.cpp:628-629`
  `r_BN = r_CN + Rᵀ·c`, `v_BN = v_CN + Rᵀ(ω×c)` vs
  `src/omni_vec_integrator.cpp:688-696` measured-stock comment
  (`r_BN−r_CN = −c` exactly, `r_BN = r_CN − Rᵀc`) + gather `727-728`
  (`y = rB + Rc`, i.e. `r_C = r_B + Rᵀc`) + scatter `990-998` (`rB = y − Rc`).
  `tests/test_wp4_offset.py:60-66` pins the `+` sign; integrator comment explicitly
  flags DESIGN/test_wp4 as "self-consistent math only, never stock-pinned".
- Impact: any `c_B≠0` run diverges by `2·Rᵀc` (0.2 m at `c=0.1`) across paths or vs
  stock B-point reads. Norm-only tests (`|rBN−rCN|=0.1`) cannot distinguish.
- Trigger: offset + thruster run compared SysModel-arm vs swap-arm.
- Fix: stock-pin the SysModel path (auditor recommendation: trust the measured
  `−` unless a second independent stock measurement says otherwise) and unify;
  add a signed-component gate (not norm-only).

## High

### H1. SysModel cutoff-lag filter is the diverged design Update-8 rejected — CONFIRMED
- Evidence: `src/omni_spacecraft.cpp:228-232` (`setCutoffFrequency` arms
  `cutoffLagOn_=true`) + `1020-1025` (`lag += (target−lag)·α`, `α=1−exp(−seg·cutoff)`),
  vs `src/omni_vec_integrator.cpp:946-954` ("NO cutoff-lag filter here by design…
  attenuates 0.63x — reverted") and `BUILD_STATUS.md:Update-8` (unconditional filter
  DIVERGES 0.63x at cutoff 10, stock INERT 10-vs-1000 bit-identical, resolution NO lag
  model). Default-off path is bit-identical (ledger M2 inert confirmed), but
  `setCutoffFrequency(10)` (stock default) enables the attenuating filter, and
  `tests/test_wp3_timing_models.py:715-718` codifies the 0.3–0.9 (~0.632x) ratio.
- Trigger: `setCutoffFrequency(10.0)`, duty 1.0, `dt=0.1` → omni impulse ~0.632x stock.
- Fix: remove/neutralize the SysModel filter when ramps are off (mirror the vec path);
  accept any `cutoff>0` without shaping; fix the ratio assertion; cutoff composes with
  ramps only if ramps ever land.

### H2. `checkUnsupported()` raises only for forceMsg; "11 flags / every trap raises" contradicted — CONFIRMED
- Evidence: `include/omni/spacecraft.hpp:266-276` declares 11 flags; but
  `src/omni_spacecraft.cpp:743-753` builds `active` ONLY from
  `forceMsgOn_ && thrForceInMsg.isLinked()` (comment: everything else "MODELED behind
  default-off setters… Only the unmodeled force path raises"). Called at Reset + UpdateState
  heads (verified). `setRWModelJitterCoupled:383-385` always `bskError`s, but at the
  setter, not in the guard. Env absent-link table (`1194-1282`) is separate and correct
  per A13. `tests/test_wp0_guards.py:22-42` asserts trap keywords appear in PLAN text +
  CI passes — zero behavioral raise tests. `results/wp0_guards.json` silent-traps verdict
  is `ok:true` by skip ("OmniSpacecraft unimportable").
- Consequence: friction/jitter/min-pulse/ramps/cutoff/blowdown/swirl/J2/offset NEVER raise
  even when on — by design per A14/DESIGN:539-546, contradicting PLAN_ENVELOPE:472-487
  ("model or bskError … until modeled") and Wave-A "no WP lands while its trap slice is
  silent". `u_min`/`P_max`/`maxWheelAcceleration`/`largeTorqueThreshold` have no flag/guard
  at all (RW audit F4) — silent full-torque runs.
- Fix: either implement per-trap raise-until-modeled or amend PLAN/DESIGN to the
  "modeled-behind-setter + force-only guard" position actually shipped, and add
  behavioral raise tests. CI 3/3 must not be cited as loud-reject proof.

### H3. RW log fabricates 15/23 fields; `u_min`/`P_max`/`Omega_max`/friction/theta vacuous — CONFIRMED
- Evidence: `src/omni_spacecraft.cpp:658-674`: only `Omega/Js/u_current` (read back from
  the par slot, never re-derived — good) + `u_max/gsHat` are real; `u_min=1e-6`,
  `Omega_max=6000 RPM`, `P_max=200`, `frictionTorque=0`, `theta/U_s/U_d/d/J13/u_f/…=0`,
  `RWModel=0`, `mass=0`, `rWB=0` are hard-coded. A custom-OFF or setter-capped wheel
  still reports a 6000 RPM cap; `setUMax()` post-registration is a silent no-op for
  existing wheels (`405-409` sets `this->uMax`, `815` only fills missing entries, `834`
  clamps from `wheelUMax[i]`); system-level `u_max_` (`system.hpp:195-198`) has zero reads.
  WP1 config-hijack (`RWArrayConfigMsg` → uMax/Js/Gs) is unimplemented (zero hits for
  `GsMatrix_B|JsList` in `src/`+`include/`); geometry enters only via setters.
- Trigger: stock `u_min` deadband / `P_max` foldback run vs omni → `u_current` diverges
  while both arms echo identical constants. FSW/telemetry consumers trusting
  `rwOutMsgs.Omega_max/u_min/P_max/frictionTorque` misbehave.
- Fix: source telemetry from config (or label each placeholder field); wire `setUMax`
  to existing wheels + `configDirty`; either implement the config-message read or
  loud-reject it; add behavioral tests (currently only `u_current/Omega/Js` gated).

### H4. Host env/offset parity tests admit vacuous zeros-vs-zeros passes — CONFIRMED (test-strength, not physics)
- Evidence: `tests/test_wp45_env_offset.py` docstring (14-22) demands firing/scale gates,
  but `test_host_drag_offset_direct_parity:297-313`, `test_host_srp_offset_direct_parity:398-410`
  (`out is None→return`, skip-as-pass), `test_host_mag_no_arm_smoke:456-460`
  (offset-run == zero-offset-run) all pass when both sides are zero. No
  `assert norm(sstep)>thr` / `drag_abs_max` gate in the test bodies (unlike the probe
  ledger's `drag_firing/drag_abs_max`). The exact vacuous class the docstring warns about.
- Fix: `assert norm(sstep)>1e-12` (drag is ~1e-7-class per combo ledger) + `assert firing`
  before every delta assert; turn `return`-on-unknown-spelling into failure.

### H5. SRP-shifted arm `(rCop−c)×F` ships without stock evidence; ledger row is `error` — CONFIRMED
- Evidence: `audit_bundle/results/wp45_combo.json`: `drag_offset.verdict=unshifted`
  (firing-proven, `drag_abs_max=6.87e-08`) vs `srp_offset.status=error`
  ("No valid heading source"). Code ships shifted unconditionally
  (`src/omni_spacecraft.cpp:1260-1265`, vec `625-627` with live-c). Host test asserts it
  at 5% but is host-only. The file's own 3-line-flip contingency note is the honest
  position — keep it, do not cite SRP-shifted as measured.
- Fix: land the facet+sizing stock link, then verdict; until then label FRAGILE.

### H6. Swap ledger-vs-gate: `rejects.verdict="SILENT: ['friction_wheel']"` breaks "7/7 loud" + `test_swap_p1.py:110` — CONFIRMED by ledger text
- `audit_bundle/results/swap_p1_parity.json` tail records silent friction (correct new
  behavior now friction is DONE) while `PLAN_INTEGRATOR_SWAP.md:137` / `BUILD_STATUS.md:532`
  claim 7/7 loud and the gate asserts `verdict=="all loud"`. Docs/gate stale, not physics.
- Fix: shrink the reject set to 6 + `fuel_unmirrored_cxx`, update texts + gate.

## Medium

- **M1. Stale absolute-h header comment vs relative-h code (trap for editors) — CONFIRMED.**
  `include/omni/contributors.hpp:273` (`h = Js*(W + omega.gs)*gs`) vs implementation
  `:322` (`h = Js*W*gs`) + `:326-329`. Code is correct (matches upstream quote at
  `:313-314`, Update-7, t2/t5 re-validation); the comment would reintroduce the bug.
  Fix: one-line comment correction.
- **M2. `frictionHubSign_` applied to hub but not wheel spin — CONFIRMED.**
  `:328` (`s*tauF` on hub) vs `:349` (bare `tauF` in `Wdot`). Exact at default `s=+1`;
  at `s=−1` momentum leaks at `2·tauF/Js`. Non-stock hook; fix by threading `s` into
  the spin equation or restricting the setter to `+1`.
- **M3. `r_BcB_B!=0` silently accepted; DESIGN "nonzero must throw" contradicted — CONFIRMED.**
  `system.hpp:41-63` validates mass/symmetry/`det>1e-30` and stores `rBcB` verbatim, no
  throw; `HubContributorT:179-243` never reads `c_B_` (documented B-point rejection).
  Standalone hub-only `add_hub(m,I,{0.1,0,0})` propagates identically to `r=0`. The
  Addendum/Update-8 semantics supersede the DESIGN table without striking it. Fix: strike
  the table row or re-add the throw for the hub-only path.
- **M4. No singular-D/Schur/`r=0` guard in graph path — CONFIRMED.**
  `linear3.hpp:185-192` (`adj/det`, no `det==0` check), `backsub_t.hpp:58,63`,
  gravity `1/(r2·rn)` (`contributors.hpp:419-424,236-239`, `environment.hpp:343-356`;
  module-side `environment.hpp:182` HAS `if(!(dn>0)) continue`, graph deliberately does
  not). `add_hub`'s `det(I)` check does not cover `det(D)` (`Js≈Ixx` aligned zeroes `D`).
  Double gives Inf/NaN silently; expression defers div-by-zero. Trigger: `r=0`,
  `Js=Ixx`, dispersed `mass<=0` via par overwrite. Also downgrades two claims:
  `cond(S)=1.0007` is stale anyway (no writer to `B/C` exists → `S=A=m·I`, `cond=1`;
  Info), and "identical arithmetic double vs expression" is overstated only through the
  zero-skip branch (`linear3.hpp:119-144`; finite inputs bit-agree per Gate A, non-finite
  diverge by construction; Info).
- **M5. `checked_par_count` suffix-only drop + printer assumption — CONFIRMED.**
  `system.hpp:406-432`: only trailing dead slots droppable; interior dead slot (e.g.
  thruster-before-wheels + degenerate `cBz` before live `Js`) throws "non-suffix par gap"
  and forces re-registration. Loud (safe) but order-dependent. Par-index recovery via
  `ostringstream<<p; s[0]=='p'` (`:413-415`, also `t9_parcount.cpp:74-86`) assumes heyoka
  prints `par[i]` as `p<i>`. Fix: document ordering constraint; pin with a printer test.
- **M6. N-body bodies `i≥2` frozen forever — SUBAGENT-VERIFIED (not re-read line-by-line).**
  `src/omni_spacecraft.cpp:1178-1192` refreshes only `i==0←sun`, `i==1←moon`; `i≥2` stays
  static — same frozen class DESIGN forbids (Sun tide 1.4e-6 fatal over mission).
  Fix: per-index msg vector or loud-reject `nBodies>2` without links. (Same pattern
  likely in vec path — check before closing.)
- **M7. SRP `rotHat_B` dropped; non-identity `Gt` unrepresentable, no reject — SUBAGENT-VERIFIED.**
  `addSRPFacet(area,n,rCop,cd,cs)` has no `rotHat` param/guard; mag clamps `|m|` only
  (`1289-1292`), no `Gt` member/API. Any stock facet with meaningful `rotHat` / MTB with
  non-identity `Gt` diverges silently. Fix: add params or loud-reject.
- **M8. Min-pulse is suppress-only; valve memory (`carryover/lastFireStep/fireCounter`) dead — CONFIRMED**
  by code inspection (`contributors.hpp:592-595`, `spacecraft.cpp:937-947`, `Impl:39-43`
  never read). Matches ledger M3 classes; do not claim extend/merge/memory until probed.
- **M9. Ramps present but unvalidated; off-ramp tail past `dt` truncated — CONFIRMED.**
  Setters + knot cuts + linear-vs-hold factors (`207-226, 989-995, 1010-1018, 1139-1153`)
  exist; `checkUnsupported` never raises (`rampsOn_` "modeled"); `addCut` rejects
  `t>=dt−eps` so `openT+offRamp>dt` tails are dropped (default-off: no current parity
  impact). Stock shape is explicit per-pair factors; omni's are implicit-even (admitted).
  No stock `ThrustFactor(t)` trace gate (M1 ledger is `OnRamp_len==0` only). Fix: bound
  with warning or propagate the tail; add the trace gate before claiming parity.
- **M10. SysModel fuel is static bookkeeping; "live mass/I/c without rebuild" is vec-only — CONFIRMED.**
  Vec `869-914` overwrites mass/I/cBx..cBz/mdot per tick; Spacecraft `481-485` bakes
  mass/I/rBcB once, `728-733` publishes `fuelMass_` static + `fuelMassDot=0`, comment
  `725-727` admits no mass feedback. SysModel `updateEnvInputs` SRP arm uses static
  `rBcB_` (`1262-1264`) vs vec live `cLive` (`625-627`) — fuel+offset+SRP diverges by
  `mFuel/(mDry+mFuel)·(rTB−rBcB)`. Update-8 flagship wording is over-broad; code labels
  are correct. Fix: scope the flagship to the vec path or port live-c to SysModel.
- **M11. DV accumulators copy body-frame impulse into inertial slots — CONFIRMED.**
  `src/omni_spacecraft.cpp:1075-1083`: identical `Fx,Fy,Fz` (body) added to `DVBdy`,
  `DV_CN_N`, `DV_BN_B`; `213`-area comment claims "frame split applied by caller" but C++
  does not rotate. Burn at non-identity attitude wrongs `TotalAccumDV_CN_N` by `R_BN`.
  "Labeled non-parity" does not cover a frame error. Fix: rotate or zero the inertial slot.
- **M12. Mid-run mutation / staleness holes — CONFIRMED pattern.**
  Spacecraft `buildIntegrator` fills `y` (`528-539`) before resolving offsets (`558-571`);
  `setUMax` never sets `configDirty` (`405-409` vs every other structural setter); vec
  `configDirty_` flips only on `oi.*` setters with a one-shot `mirror_stock_config`
  snapshot — direct stock-side edits mid-run are silently ignored (PLAN "reject-or-rebuild"
  not implemented for stock-side mutation). Stochastic diffusion adds no states so the
  state-name claim check (`vec:348-361`) cannot reject it (structurally out, silently
  ignored). `Reset()` does not clear DV/fuel accumulators (`755-770`). Fix in priority
  order: resolve-then-fill, dirty on `setUMax`, stock-mutation detection or documented
  freeze, diffusion loud-reject, Reset clears accumulators; `cOff==0` sentinel
  (`vec:51,474,910`) should become `SIZE_MAX`/bool before slot 0 is ever valid.
- **M13. Jitter/friction ledger overclaims — CONFIRMED reading.**
  J1 determinism `0.0` is 2×40 ticks, not 2×400 (`N_JITTER_TICKS=400` capped by
  `min(n,40)`; probe+test docstrings say 400). C++ `seed+GOLDEN+(i+1); splitmix64`
  (`spacecraft.cpp:858-861`) vs test `splitmix64(i+1); Random(seed^x)` differ —
  "mirrors the C++ spec" is false at bit level (harmless distributionally; omni's own
  `normal_distribution` inherits the libc++ portability caveat it cites against stock).
  F1–F6 `ok:true` with all-`null` readbacks (`'str' object has no attribute…`,
  `W0_assumed`, `"to be confirmed on host"`) means "sim ran", not "law matched".
  SysModel has no `omegaBefore`/stribeck latch (only vec `hpp:190-194, cpp:781,813`);
  `|W|<0.1·omegaLimitCycle` + `betaStatic>0` has no stiction band in SysModel iter-1
  (`tauF=−fC·tanh(W/k)−cV·W`, `fStatic/betaStatic` accepted-but-inert — honestly labeled
  `stribeck_iter1`, but any "Stribeck parity" reading is contradicted). FullyCoupled
  reject is setter-gated, not automatic; `RWSpeedMsg.wheelThetas` never written (zeros,
  unlabeled). Stale-`u` tail: 4-wheel rig with 3-slot `motorTorque` leaves wheel 4 at its
  previous-tick value (`spacecraft.cpp:825` loop bound); unlinked→all-zero is safe, short
  message is not.

## Low / hygiene (all confirmed or subagent-verified)

- **L1. Par-count version skew (19→13→16).** DESIGN body (`166,318-324`: `u3+mass+I6+invI6+Js3=19`)
  vs Update-4 (hub 13→7, RW 19→13, pre-WP2) vs live code (hub+3RW = 6 inputs `u+tauF` + 7
  hub + 3 Js = **16**, asserted `test_wp7:157` `(15,16)`). `BUILD_STATUS:33-36`
  ("15 states, 19 pars … reproduced") never struck though Update-6 admits counts "stale by
  design". `layout.py:17-24` canonizes the 19-layout (+hub-only 13, no `tauF`) and repeats
  the `W_i(3)` slip (actually 1; DESIGN A1 also says 3) — the file's own `(19)` literal
  would trip the A14 grep it documents. `presets.py/mock.py/shim.py` are clean (19 framed
  as historical reference). Fix: version-pin DESIGN body; fix `layout.py` docstring.
- **L2. A7 exports missing.** `omniheyoka/__init__.py:29-49` lacks `reset_mrp/sigma_indices`
  (exist only at `driver.py:77`, `layout.py:249`); DESIGN A7 requires top-level exports.
  `from omniheyoka import reset_mrp` → `ImportError`. Fix: export or strike A7.
- **L3. WS2 revival still armed on absolute-h.** C++ fixed and re-validated (Update-7);
  but `heyoka_rw.py` keeps absolute-h (exact for equal-Js orthonormal sets only) and
  `presets.py:40,418-424` + `mock.py:34,168` delegate ground truth to it. Any revival
  comparing dispersed-Js / tilted / 1–2-wheel configs against it reinstates Ground-6.
  Fix: convert or restrict before revival.
- **L4. CI gaps.** Graph-purity grep scans only `contributors.hpp`+`system.hpp`
  (`ci_envelope_checks.sh:86-87`), missing `environment.hpp`/`force_torque.hpp`/
  `backsub_t.hpp`/wrench path; state-index rule covers only `src/`, missing shipped
  `omniheyoka/ctypes_driver.py:72-80` hub literals; count-grep misses bare `19`
  (`scratch_ws1/bench_tilted.py:183` `parse_system(…,19)` stale-ad-hoc). Probe
  `GRAPH_FUNCS` set differs from CI's and emits 60 `if (` hits including double-side
  `std::abs` — probe `ok:true` vacuous on purity; CI is precise but incomplete.
  JIT tiers (`hub+4RW 3.3s, hub+3RW+env 5.0s` vs Ground-8 "fail if >2× baseline 0.548s")
  are honest (nonlinear LLVM JIT) but contradict Ground-8 text; `graph_bytes`
  assert-skips (`None`-tolerant, `spacecraft.hpp` exposes no RHS strings). Fix: extend CI
  scopes, reword Ground-8 to per-config tiers.
- **L5. Ledger counts + numbers.** PLAN "7/7 probes" vs `bundle_summary.json` 11/11 vs
  `test_wp7:183-184` 11/11 (growth: combo/fuel-bisect/threshold/telem-viz/tilt-debug —
  PLAN stale, test current). `thr_offset` status `5.6e-09` vs ledger `2.24e-08` (bound
  `2e-03` covers it; provenance does not). P-WP4.4 ledger rows are idle zeros
  (`torque/force=[0,0,0]`, `tel−expect=[0,0.5,0]`) — `test_wp4_offset.py:306-333` re-does it
  correctly with fresh-write + steady-tick; do not cite P-WP4.4 rows as arm evidence.
  Penumbra ladder `{0.1,1,10}` ships `{0.1,1.0}` by default (`FULL_LADDER=False`, Moon
  skipped); no eclipse-edge sub-arc in `sw` (cuts on `openT`/ramps only) so DT=10s ZOH
  smears the edge as labeled — keep the DT≤1s constraint for eclipse runs or add edge cuts.
  `densityCorrection` guard exists in law/helper but `spacecraft.hpp:242` exposes no setter
  (always 0.0) — expose or remove the claim. `setMagneticDipole`-alone registers a 15-slot
  empty cluster (bloat, harmless — graph references all env slots by construction, which
  correctly keeps the interior-gap throw unreachable). N-body module-vs-graph singularity
  guard differs (skip vs inf/NaN) — document or align; unreachable in practice.
  `Jt` third arg accepted-but-ignored without validation (`system.hpp:65`) — validate or
  document at the call site.

## Verified-correct (do not regress)

1. MRP→DCM `R=I+(8S²−4(1−|s|²)S)/(1+|s|²)²` (`mrp_t.hpp:44-68`, all six off-diagonals
   check; independent `t7:19-32` identical; denominator ≥1, `|σ|=1` safe) + kinematics
   `ṡ=¼Bω` (`mrp_t.hpp:90-98` term-by-term) + caller-owned shadow reset (`/=−n2`).
2. Relative-h + full-outer `D=I−ΣJs·gs·gsᵀ` + parametric-Js wiring (`system.hpp:763-796,827`)
   + coupled spin `(u+tauF−Js·gs·ωDot)/Js` sequential after hub solve
   (`contributors.hpp:200-214,301-350`); t2/T1/t7/t8 independent refs agree
   5.9e-16/5.6e-16/5.55e-16/7.2e-14 incl. non-diagonal I, unequal Js, tilted axes.
3. Hub Kepler + GG frames/signs (`vecTrans+=m·g`, `A=mI` → `r̈=g+F/m`; `−μr/|r|³`;
   `3μ/|r|⁵·(rB×I·rB)`, total-I); no PA/B-C/`c×mg` per measured-stock semantics (intentional).
4. A1/A2/A5/A6: registration-order states, inputs-first dynamic pars + `par_names()` authority
   (+`par_index`/`par_slice`), gravity-first both instantiations, no time symbol, env-LAST
   two-pass with thruster-overwrite preserved, `_B` telemetry-only (no double count),
   thruster `F_N=RᵀF_B`, `(r−c)×F` dynamics vs `r×F` telemetry + swirl axis term.
5. Hand-rolled `det3/adjugate3/inv3` + `+−*/sqrt`-only graph; Eigen storage-only
   (NumTraits rationale plausible); A3 holds in graph, clamps/tanh/exp correctly module-side.
6. Direction-aware saturation cut + step-start freeze + per-wheel `uMax`
   (`spacecraft.cpp:803-836`); `addRatedWheel` Js derivation + RPM→rad/s + OFF semantics.
7. Staged mailbox edge rule + tick-0 stage-only + one-shot consume + stamp persistence;
   quantizer edges incl. `0.5→5/6`, negatives→0, over→1.0; single-quantize boundary;
   sorted/epsilon-merged `[0,t_j)` arcs + idle/full single-arc path; min-pulse suppress
   classes; blowdown `1.0+warn`; swirl slot+axis; force-link guard raises (both heads).
8. Env laws as implemented: N-body sign, J2 ECI + `−√5·C̄₂₀`, drag law + `v_rel` + arms +
   floors, SRP `(AU/r)²` + two-term facet + eclipse scalar, mag `m×(R·B_N)`, ERP zero-force
   by construction; truth-table legs (atmo-error, wind-inertial, eclipse-1.0 single-field
   `shadowFactor`, mag-error, zero-dipole-error); per-tick + arc-hook placement (env-only).
9. Closed historical bugs stay closed: string-bridge truncation (nothing shipped imports
   `_parser.py`), `omni_taylor_step` pars-push (both buffers), `nWheels()-1`,
   registration-replay on Reset, name-resolved offsets at use sites, `u_current` readback
   (not motor msg), GG-flag mirror, stock-owned ordering (no duplicate MRP/DV/energy),
   `checked_par_count` + heap-bounded pushes at in-tree `ta` sites, `THRArrayConfigMsg`
   functor dead-end (Python-side join, no `.label`), single-tank scope + subset-linkage
   errors, `MinOnTime`/cutoff/blowdown/swirl swap-side rejects, `thrustFactor`-lag and
   task-priority traps avoided, `tests/`-supersedes-benches, tilt-W3 saturation-edge
   mechanism with widened wheel envelope (att/rate tight, wheels 1.0 RPM).
10. Build recipe: scikit-build-core + `bsk_add_swig_module`, heyoka order, `-std=c++23 -O3`
    no `-ffast-math`, direct `libheyoka` + rpath with explicit fallback, `cSysModel` shim.

## Must-fix before green (ordered)

1. Unify the B/C sign (C1); add a signed-component gate. Do not ship offset runs until one.
2. Remove or ramps-gate the SysModel cutoff filter per Update-8 (H1); fix the ratio test.
3. Reconcile `checkUnsupported` with the trap table (H2): per-trap raise-until-modeled or
   amend PLAN/DESIGN to the shipped force-only position + behavioral raise tests; cover
   `u_min`/`P_max`/`maxWheelAcceleration`/`largeTorqueThreshold`/config-message paths.
4. Source or label every RW log placeholder; fix `setUMax` staleness; implement or reject
   the config-message read (H3).
5. Add firing/scale gates to every host env/offset parity test (H4); land the SRP facet+sizing
   stock link before claiming shifted (H5); fix the swap rejects ledger+gate (H6).
6. One-line fixes: absolute-h header comment (M1), `frictionHubSign_` in spin (M2), strike or
   enforce the DESIGN `r_BcB` throw row (M3), document `det(D)`/`r=0` NaN + interior-gap
   ordering + printer assumption (M4–M5).
7. Close M6–M13 (N-body >2 bodies, rotHat/Gt, min-pulse memory label, ramp tail+trace gate,
   SysModel live-c/fuel scope, DV frame rotation, mutation/staleness/stochastic/Reset/cOff,
   jitter-ledger wording + latch gap + thetas label + short-message tail).
8. Hygiene (L1–L5): pin DESIGN 19-par body + PLAN probe counts, export or strike A7,
   convert/restrict WS2 off absolute-h, extend CI scopes, reword Ground-8 tiers; host-build
   WP0–WP6 and confirm SWIG vectors / `Message` resize / payload fields before any green.

## Appendix — audit provenance

- Subagent 1 (physics core): 10 findings F1–F10 + 6 verified-correct. Triage: F1→M1 confirmed
  (`contributors.hpp:273` vs `:322`); F2→M2 confirmed (`:328` vs `:349`); F3→M3 confirmed
  (`system.hpp:41-63`, DESIGN:139); F4→M4 confirmed (no `det==0` guard); F5→M5 confirmed
  (`system.hpp:406-432`); F6→L1; F7 Info (B=C=0, `cond=1`); F8 Info (zero-skip); F9→L5
  (`Jt` ignored); F10 Info (t3 weak, t8 strong; Gate G string-bridge dead).
- Subagent 2 (RW): F1 verified-correct (cut semantics + step-start freeze, host-only gates);
  F2→H3-part (`setUMax` no-op); F3→H3-part (config hijack absent); F4→H2/H3 (silent
  `u_min`/`P_max`/accel/threshold + fabricated telemetry `:666-669`); F5→H3-part;
  F6→M13 (iter-1 vs F-ledger nulls); F7 verified-correct (tauF plumbing); F8 labeled
  deviation; F9→M13 (no latch); F10/F11→M13 (streams, 40-vs-400); F12 partial reject;
  F13 verified-correct (n_inputs 1→2); F14→L1 (`layout.py` stale, others clean);
  F15 verified-correct (N-wheel/tilted + honest envelope); F16 fenced rWB_B + parametric Js
  verified; stale-u tail + fill-before-resolve + units/thetas per M12–M13.
- Subagent 3 (thruster/timing): mailbox/quantizer/single-quantize/A3/arcs/blowdown/swirl/force-guard
  verified-correct; F2 scoping note; F3 two-writes cell vacuous; F7→M9 (ramp tail);
  F8 M5-PWM + M4-phase mislabels; F9→M8 (memory dead); F10→H1 (cutoff divergence,
  `spacecraft.cpp:1020-1025` vs vec `946-954`); F11→M9; F12 blowdown warn (spec drift
  PLAN-must-raise vs BUILD-warn); F13 swirl global-only caveat; H1–H5 hunts per report.
- Subagent 4 (env+offset): cluster/two-pass/hook/N-body-sign/J2/drag/SRP-math/mag/ERP/truth-table
  verified-correct; F2 empty-cluster bloat; F7→M6 (bodies ≥2 frozen); F8 guard divergence;
  F11 densityCorrection dead setter; F13→M7 (rotHat/Gt); F15 static-sun bypass; C1→C1
  confirmed (`spacecraft.cpp:628` vs vec `691-696`); F17–F20 verified-correct splits;
  F19 live-c doc overclaim→M10; F21→H4; F22→H5; F23 P-WP4.4 idle rows→L5; F24 penumbra→L5;
  F25–F26 labeled/bounded.
- Subagent 5 (swap/spacecraft): V1–V13 verified-correct list adopted §Verified-correct(9);
  F1→C1; F2→M10; F3→H6; F4→M10; F5–F13→M12/H3/H4/L5 as mapped; F14–F15 info; hunt table
  (pars-push closed, heap closed in-tree, config-dirty/mutation open, ±0 no-op holds).
- Subagent 6 (build/graph/guards): F1 verified-correct (recipe); F2 unbuilt-as-stated
  (Update-6 vs Update-4/5 archaeology); F3–F5 host risks correctly flagged; F6–F7 A3
  verified-correct; F8→L4 (CI scope); F9→L4 (JIT tiers vs Ground-8); F10 verified-correct
  (accessors + CI structure); F11→L4 (`ctypes_driver` literals); F12→L2/L3 + H3 (WS2 stale,
  A7 unexported, `RW_N_PARS` name); F13→L1; F14→H2; F15→C1; F16→H1; F17→L1/L5;
  F18–F20 verified-closed; F21→L3 (Ground-6 residual in WS2).
- Lead validation: re-read `src/omni_spacecraft.cpp:620-753,1015-1124`,
  `src/omni_vec_integrator.cpp:685-744,975-1001`, `include/omni/contributors.hpp:270-367`,
  `omniheyoka/__init__.py`, `scripts/ci_envelope_checks.sh` — C1, H1, H2, H3, H5-frame-bug,
  M1, M2, L2 confirmed byte-for-byte; the rest accepted on subagent file:line evidence
  (each cited a precise location consistent with the lead's adjacent reads).
