# Phase 2.5 plan — close every Audit_0-2 finding (Rev 2)

Scope: remediate ALL still-open items from `Audit_0-2.md` (which already
regresses `Audit_0-1.md`), without regressing the X1–X5 fixed items. **This is a
plan; nothing here is implemented yet.** Each WP ends with its gate green *and*
the full regression sweep passing. Ground rules from `PLAN_ENVELOPE.md` (probe
ledger first, matched-dt, A3 graph purity, A2 name-derived layout, no regression,
JIT budget) stay binding; contradictions below resolve them explicitly.

Baseline: `Audit_0-2.md` §§ C1, H1–H8, M1–M16 (+blowdown/swirl/test_wp3 notes),
L1–L5 (+fresh lows). X1 (c-mask), X2 (rejects-ledger), X3/X4 (vec friction/fuel),
X5 (historical closures) are **do-not-regress** constraints, not work.

## Decision register (locked for 2.5)

| # | Item | Decision |
|---|---|---|
| 1 | B/C sign | **Vec `−` wins.** `swap_p1_parity.json:thr_offset` (offset 0.1, omni-vs-stock deltas 2.24e-08; wrong sign would be ~0.2 m) stock-pins `r_BN = r_CN − Rᵀc`. SysModel flips to `−`. Norm-only gates are banned for sign. |
| 2 | Cutoff | **No lag model on either path.** Vec already inert; SysModel filter is deleted (ramps-gated reintroduction only if ramps ever land, with its own probe). `setCutoffFrequency(10)` must equal 1000 to ~1.0. |
| 3 | Guards | **Amend-then-enforce.** `PLAN_ENVELOPE.md` trap table is amended to the shipped position (force-only fatal + named modeled-behind-setter + explicit silent-gap list in §Risks), AND behavioral raise tests land for every fatal that remains. CI/probe-skip is never cited as proof. |
| 4 | Fuel scope | **Vec owns live depletion; SysModel either integrates or unships the publisher.** No third option (static-publish-full-tank is a wrong-telemetry bug, not a label). Fuel×env needs combined cells or an explicit out-label. |
| 5 | Friction scope | **Vec owns stock-exact Stribeck; SysModel either ports it or stays labeled iter-1-suppress-only and rejects friction-ON.** Unscoped "stock-exact" claims are banned. |
| 6 | `u_min`/`P_max` scope | **Vec covered by construction (post-saturation `u_current` readback); SysModel documents the silent gap + telemetry labels until modeled or rejected.** The Update-8 sentence is scoped, not deleted. |
| 7 | Docs rule | **Every number cites cell+arm+ledger-ref.** Bare numbers ("1e-14 class", "7/7", "5.6e-09") without a pointer are rejected in review. |
| 8 | Multi-spacecraft sync | **Implement in 2.5 (WP25-A4); Phase 3 scope unchanged.** Vec hard-rejects `dynPtrs != 1` today (`vec:299-304`); Phase 3 DoD needs the sync path green, so 2.5 owns per-object gather/scatter + no-double-step + two-spacecraft parity gate. Deferral would require re-scoping Phase 3 — rejected. |

## Wave A — correctness (hard predecessor of B/C)

### WP25-A1 — B/C sign unification (closes C1, M10-arm part, L5-thr_offset-provenance)
**Files:** `src/omni_spacecraft.cpp:620-633`, `include/omni/spacecraft.hpp`
(record comment), `tests/test_wp4_offset.py:59-66,102-109,464-472`,
`audit_bundle/probes/wp4_offset.py` (+ results), `BUILD_STATUS.md:558-563`,
`PLAN_INTEGRATOR_SWAP.md:thr_offset`, `DESIGN.md:A13/B-C lines`.
**Design:**
1. Flip SysModel output to `r_BN = r_CN − Rᵀc`, `v_BN = v_CN − Rᵀ(ω×c)` (mirror
   `vec:698-703/734-735/997-1003`). Zero-offset stays exact no-op (`+0`/`−0` safe).
2. Replace norm-only asserts with signed-component gates: per-axis
   `r_BN−r_CN+Rᵀc == 0` to ≤1e-9 (static-attitude fixture, no rotation to hide sign),
   plus the existing `|d|==0.1` norm check kept as secondary.
3. Add a cross-path gate: same `c=0.1` fixture on SysModel-arm vs swap-arm,
   per-axis `Δr_BN ≤ 1e-9` (catches future `2·Rᵀc` splits).
4. Strike/annotate `DESIGN` `+`-sign lines + `test_wp4` self-check comments as
   superseded-by-measurement; correct `thr_offset` text to ledger `2.24e-08` (or
   re-measure and cite the new cell).
**Probe/gate (pre-registered):** offset 0.1 static-attitude, matched dt=0.1:
signed-component residual ≤1e-9 both arms vs stock; cross-path ≤1e-9.
**DoD:** C1 closed, signed gates green both arms, texts reconciled.

### WP25-A2 — Cutoff inert on both paths (closes H1, M16, L-fresh heading, test ratio)
**Files:** `src/omni_spacecraft.cpp:228-232,1020-1025`, `include/omni/spacecraft.hpp:230,271`,
`src/omni_vec_integrator.cpp:951-957` (reference, untouched),
`tests/test_wp3_timing_models.py:715-718`, `audit_bundle/probes/wp3_timing.py` M2,
`BUILD_STATUS.md:602-618`.
**Design:**
1. Delete the SysModel `lagState_` filter (or gate strictly behind `rampsOn_ &&
   offRamp/onRamp non-empty`, default-OFF path identical to vec
   `eff = open?1:0`). `setCutoffFrequency` accepts any `>0`, rejects `≤0`, never arms
   shaping. Remove `lagState_` member or keep only for the ramps-gated future.
2. Fix ratio test to assert inert: `0.99 < ratio(10 vs 1000) < 1.01` over fire AND
   shutoff transients (replaces `0.3–0.9` codified-divergence).
3. Fix heading to "cutoff accepted, never modeled (inert without ramps)"; keep the
   honest caveat "SysModel was exact only at pinned 1000 before this fix" as archaeology.
   Record "cutoff composes with ramps if those land (M2)" as the re-entry condition.
**Probe/gate:** M2 ladder {10,100,1000} fresh-write step-response, matched dt;
stock-vs-omni impulse ratio ≈1.0 (tol 1%); `thr_lag10==thr_fresh_05` bit-class retained.
**DoD:** H1/M16 closed, arm split gone, ratio test green.

### WP25-A3 — SysModel fuel honesty + fuel×env coverage (closes H6, H7, M10-SysModel)
**Files:** `src/omni_spacecraft.cpp:402,725-733` (+ per-tick block),
`omniheyoka/swap.py:_mirror_fuel` (reference), `src/omni_vec_integrator.cpp:876-920`
(reference), `audit_bundle/probes/swap_p1_parity.py:_env_fuel_cell`,
`tests/test_swap_p1.py` (fuel cells), `DESIGN.md:533-535`.
**Design (pick A or B, no third way):**
- A (preferred): port vec law to SysModel — per-tick `mdot=Σq·Tmax/(Isp·g0)+leak`
  from staircase-ACHIEVED duties, clamp `≤mFuel/dt`, `fuelMass−=mdot·dt`,
  publish live `fuelMass` + `fuelMassDot=−mdot`; thruster/SRP arms read live-c
  (same 3-slot pattern as vec `cOff`, or document why SysModel stays static-c with a
  measured cost bound).
- B: delete the SysModel `fuelTankOutMsg` publisher until modeled (loud absence beats
  wrong-telemetry presence).
2. Fuel×env: add fuel+drag and fuel+SRP combined cells (burn + env on, matched dt),
   or explicitly label drag-under-burn OUT with the `Δc×F_drag` order cited. Decide and
   record whether stock shifts `comOffset` with depletion (probe first — do not guess).
**Probe/gate:** 600 s centered + offset tank (existing scale: 10 kg, 5 N, duty 0.5,
1.1 kg burned): SysModel `fuel_rel_err == 0.0` (option A) or publisher-absent (option B);
fuel+env cells within pre-registered switching-class bounds.
**DoD:** H6 closed (no static-full-tank publish), H7 closed (cells or label), M10 scoped.

### WP25-A4 — Multi-spacecraft sync path (no audit ID; Phase-3 entry dependency)
**Files:** `src/omni_vec_integrator.cpp:299-304` (+ gather/scatter/stepping),
`include/omni/vec_integrator.hpp`, `omniheyoka/swap.py` (per-object mirror),
`tests/test_swap_multi.py` (new), `audit_bundle/probes/swap_multi.py` (+ results).
**Design:**
1. Lift the `dynPtrs != 1` reject: per-object census/claim/gather/scatter with
   name-derived slots per object. Shared symbolic graph iff configs are identical;
   per-object graphs otherwise — probe-first on build cost (JIT tiers still bind).
2. Honor `isDynamicsSynced`: synced objects must not double-step. Follow the Phase-0
   ordering record (`postIntegration[all dynPtrs]; skipped if isDynamicsSynced`,
   `swap_p0_ordering.py:47,73`).
3. Single-object path untouched by construction: existing swap cells re-green in the
   same commit (any delta fails the WP).
**Probe/gate (pre-registered):** two-spacecraft sim, distinct ICs, matched dt,
mission-scale leg: per-object state deltas vs stock in switching class + a step-count
audit proving no double-step on synced objects.
**DoD:** Phase 3 multi-spacecraft bullet unblocked; single-object parity unchanged.

## Wave B — envelope honesty (after A; parallel within B)

### WP25-B1 — Guards: amend-then-enforce (closes H2 incl. 4-knob gap)
**Files:** `src/omni_spacecraft.cpp:743-753` (+heads), `include/omni/spacecraft.hpp:266-276`,
`PLAN_ENVELOPE.md:472-487,90-93`, `DESIGN.md:539-546`, `tests/test_wp0_guards.py`,
`scripts/ci_envelope_checks.sh` (untouched — not proof), `audit_bundle/.../wp0_guards.*`.
**Design:**
1. Amend PLAN trap table to three buckets: (a) fatal-today (forceMsg + any knob in the
   silent-gap list the team chooses NOT to model in 2.5 — default: `u_min`/`P_max`/
   `maxWheelAcceleration`/`largeTorqueThreshold` non-default, `JitterFullyCoupled`,
   non-identity `Gt`, `rotHat≠0` facets, `nBodies>2` unlinked, stochastic/diffusion,
   prescribed/flex/slosh, multi-tank, mid-run stock mutation); (b) modeled-behind-setter
   (saturation/friction-SysModel-iter-1/jitter-Simple/min-pulse-suppress/ramps-shape/
   cutoff-accepted/blowdown-hold/swirl-global/J2-analytic/offset-`−`/env-truth-table);
   (c) informational (voltage reuse). Every (a) item raises `bskError` at Reset+UpdateState.
2. Behavioral raise tests: one per (a) item (arm the setter/link, expect raise;
   disarm, expect run). Delete the keyword-in-text + CI-exit + skip-`ok` citations as
   proof (keep the tests as hygiene, not as guard evidence).
**Probe/gate:** each (a) trap provably raises before first propagation; full sweep green.
**DoD:** H2 closed; 4-knob gap either modeled (see B3) or fatal.

### WP25-B2 — Telemetry/DV/Reset/staleness (closes H3, H8+M2, M11, M12-part)
**Files:** `src/omni_spacecraft.cpp:405-409,528-571,646-675,755-770,825,1075-1083`,
`include/omni/system.hpp:195-198,65`, `include/omni/contributors.hpp:273,323-349,364-366`,
`tests/test_wp6_telemetry.py`, `tests/test_wp1_saturation.py:199-219`,
`src/omni_vec_integrator.cpp` + `include/omni/vec_integrator.hpp` (switch count only).
**Design:**
1. RW log: source every field or label it — `Omega_max/u_min/P_max` from config
   (OFF→`-1`/unset echo, never 6000/1e-6/200 defaults for an OFF wheel);
   `frictionTorque` live (vec law reference) or explicitly zero-labeled "unmodeled on
   this path"; `theta/U_s/U_d` labeled; fix `test_wp6_telemetry.py:145` (`≈0`
   scoped "while unmodeled on SysModel" or pointed at vec). WP1 hijack: implement
   `RWArrayConfigMsg` read or loud-reject the link (no silent third state).
2. `setUMax`: update existing `wheelUMax[i]` + `configDirty=true` (or document
   build-time-only + reject post-Reset use). Same for `setFrictionHubSign`: thread
   through `OmniSystem` to `ReactionWheelContributorT::frictionHubSign_` (cf.
   `set_com_offset` wiring) AND apply `s` in spin (`:349` → `(u+s·tauF−Js·gs·ωDot)/Js`),
   or restrict setter to `+1`/reject `−1`. Fix header `:273` absolute-h one-liner in the
   same edit (drop `+ omega.gs`). Correct `system.hpp:u_max_` dead accessor (wire or delete).
3. DV: rotate body impulse by `R_BN` into `DV_CN_N` (or zero the inertial slot with a
   comment); keep body slot as-is. `Reset`: clear `accumDV_*`, `nonConsAccelB_`,
   `fuelMass_` (+ document `lagState_/rampClock_/lastFilt_/lastQ_` fresh-`Impl` behavior).
   Reorder `buildIntegrator` to resolve-then-fill (`:558-571` before `:528-539`).
4. Stochastic: loud-reject diffusion enable at setup (state-name check is insufficient
   by construction). Mid-run stock mutation: document freeze at `InitializeSimulation`
   + detect-or-rebuild, or explicitly reject post-init stock-side edits.
5. Vec `MRPSwitchCount` (no audit ID; Phase-3 FSW dependency): count shadow-set
    crossings per tick in the integrator, expose via getter, publish on the swap path
    (mechanism probe-first — scatter hook vs helper publish). SysModel already tracks
    `mrpSwitches`; the vec path publishes nothing today.
**Probe/gate:** custom-OFF wheel reports OFF (not 6000); post-registration `setUMax`
reported-effective; `s=−1` either rejected or momentum-consistent; DV burn at
non-identity attitude rotated; double-`Reset` DV-clean; diffusion-enabled sim raises;
vec `MRPSwitchCount == stock` tick-by-tick on a switching leg.
**DoD:** H3/H8/M2/M11/M12-part closed; vec switch-count source exists.

### WP25-B3 — Friction/jitter/`u_min` paths (closes M13, R2-`u_min`, F-ledger, M8)
**Files:** `src/omni_spacecraft.cpp:346-359,841-881`, `src/omni_vec_integrator.cpp:771-823`,
`include/omni/vec_integrator.hpp:190-194`, `omniheyoka/swap.py:161-173`,
`tests/test_wp2_friction_jitter.py`, `audit_bundle/probes/wp2_*` + results.
**Design:**
1. Scope in stone: vec = stock-exact (keep `rw3_fric` 1e-14 gate); SysModel = EITHER
   port the flag/latch/`omegaBefore`/limit-cycle law OR stay iter-1-suppress-only with
   (a) friction-ON loud-reject-or-envelope label, (b) `fStatic`/`betaStatic` inert
   comment kept, (c) near-zero stiction-band gap documented with its probe. Off-state
   deviation (slot-0 vs factory stray-`cViscous`) stays labeled.
2. Jitter: extend probe to true 2×400 (remove `min(n,40)` cap) or relabel all 400-claims
   to 40; fix "mirrors C++" to distributional-only; SysModel FullyCoupled: auto-detect
   or document setter-gated limitation; label `wheelThetas` zeros. F-ledger: amend the
   all-`null` F1–F6 rows (or supersede-pointer to `rw3_fric`; no unamended "law matched"
   rows). Min-pulse: keep suppress-only + dead-memory label (no memory claim).
**Probe/gate:** vec `rw3_fric`-class retained; SysModel friction-ON either matches the
ported law or raises/labels; jitter determinism at the claimed horizon only.
**DoD:** M13 (all four) + M8-label closed; no unscoped "parity" reading possible.

## Wave C — tests + robustness + hygiene (after B for gates that depend on it)

### WP25-C1 — Env/offset tests + missing stock links (closes H4, H5, M6, M7, L5-env)
**Files:** `tests/test_wp45_env_offset.py:297-313,398-410,456-460,24-33`,
`tests/test_wp4_offset.py`, `tests/test_wp5_env.py`, `audit_bundle/probes/wp5_env.py:42-43`,
`wp4_offset.py`, `src/omni_spacecraft.cpp:1178-1192,1260-1265,1290-1292,1212-1214`,
`src/omni_vec_integrator.cpp:556-570,586-588,627-638,660-661`,
`include/omni/spacecraft.hpp:130-132,242`, `include/omni/environment.hpp:182,215-217,339-370`.
**Design:**
1. Firing/scale gates in the three host tests: `assert norm(sstep)>thr` (drag ~1e-7-class)
   + `assert firing` before every delta; `None→return` becomes failure. Keep swap vacuity
   guards as reference.
2. SRP-shifted: land facet+sizing stock link, then verdict; until then keep FRAGILE +
   3-line-flip note, never cite as measured. Cannonball-only `env_srp` stays scoped.
3. N-body: per-index msg vector or loud-reject `nBodies>2` without links (both paths).
   rotHat/Gt: add params or loud-reject (`rotHat≠0`, non-identity `Gt`; `|m|`-only clamp
   stays + documented). `densityCorrection`: expose setter or remove the claim.
   Penumbra: default ladder gains the 10 s leg on host (or DT≤1 s constraint for eclipse
   runs recorded + eclipse-edge sub-arcs added to `sw`). P-WP4.4 idle rows: supersede-pointer
   to `test_wp4_offset.py:306-333`, never cited as arm evidence.
**Probe/gate (pre-registered):** firing-proven drag/SRP/mag legs; penumbra {0.1,1,10}s;
N-body 3-body reject-or-refresh; rotHat/Gt reject-or-model.
**DoD:** H4/H5/M6/M7/L5-env closed.

### WP25-C2 — Robustness residuals (closes M1, M3–M5-rest, M9, M14, M15, blowdown/swirl, test_wp3 labels)
**Files:** `contributors.hpp:273,479-487,507-523,570-573,583-628`,
`system.hpp:14-15,41-65,324-330,406-440`, `linear3.hpp:117-144,185-192`,
`backsub_t.hpp:58,63`, `environment.hpp:182`, `spacecraft:937-947,981-995,1010-1018,1136-1153,906-911,969-972, Impl:39-43`,
`vec:204-211,880-884,889-890,913-914,825-826,870-875`,
`tests/test_wp3_timing_models.py:603-606,715-739,752-774`, `probes/wp3_timing.py:81-160`.
**Design:**
1. One-liners: M1 header (`:273`→relative), M3 (strike DESIGN:139 or re-add hub-only
   throw), M4 (document `det(D)`/`r=0`/`dn==0` NaN posture at call sites; qualify
   DESIGN:110-112 identical-arithmetic + `:111-112` `cond(S)`→`cond=1` degenerate note),
   M15 (replace `vec:870-875` static-c + `:825-826` friction-off comments),
   `tmax==0.0`→`<=0`/finite (note-only).
2. M5-rest: document c-ordering constraint; pin `p<i>` printer with a test.
3. M9: bound off-ramp tails (warn or propagate) + `ThrustFactor(t)` trace gate before any
   ramp-parity claim (M1 ledger is `len==0` only). M8 stays suppress-only + dead-memory
   label. Blowdown: amend PLAN to warn-and-hold-1.0 (shipped) or escalate to raise —
   pick one. Swirl: document global-slot scope (per-thruster vector iff ever needed).
4. M14: `<P.size()` guards on `mdotOff/massOff/IO` + build-time
   `fuelIsps.size()==thrTmax.size()` check (missing→raise, not silent `0.0`).
5. test_wp3 labels: M4 omni-vs-omni→stock-vs-omni phase discrimination (half-step-shift
   probe per PLAN M4); M5 true `dt=0.05` 20 Hz train (not same-tick proxy); two-writes
   true same-stamp-different-payload cell; `thrustFactor` lag alignment gate
   (telemetry-timing only; dynamics stay omega-delta truth).
**Probe/gate:** printer test; ramp trace gate; blowdown/swirl scope tests; M14 OOB-proof
(try mismatched isp list → raise); phase/PWM/vacuity labels corrected.
**DoD:** all Medium residuals closed; no silent-drop/OOB path remains unguarded.

### WP25-C3 — Docs/CI/build/ledgers (closes L1–L5-rest, X2-remainders, FRESH lows)
**Files:** `DESIGN.md:139,166,178-182,289,391-401`, `BUILD_STATUS.md:33-36,404-406,532,558-593,602-620`,
`PLAN_ENVELOPE.md:94-96,448,472-487`, `PLAN_INTEGRATOR_SWAP.md:138,191-204`,
`omniheyoka/__init__.py:29-49`, `layout.py:7-24`, `presets.py/mock.py/shim.py` (verify),
`scripts/ci_envelope_checks.sh:38-39,87,107-135`, `ctypes_driver.py:72-80`,
`scratch_ws1/bench_tilted.py:183`, `tests/test_wp7:*`, `tests/test_swap_p1.py:10-11,52-53,124`,
`swap_p1_parity.py:705,716`, `system.hpp:65`, `spacecraft:669`.
**Design:**
1. Numbers with pointers: scope Update-8 flagship/friction/`u_min` to vec path; attach
   1e-14 cell+arm; correct `thr_offset` to 2.24e-08; note `23=13+7+3` full-mask vs
   default 20; `test_wp7` name ("dispersion_jit" overpromises friction/jitter — scope or rename).
2. Strikes/exports (closes L1-part, L2, L3): DESIGN 19-body + `:139` + `cond(S)` struck or version-pinned; A2
   stays authoritative (16 live: hub+3RW `6+7+3`; hub+12thr `13+7+{0..3}`; hub-only 7);
   `system.hpp:14-15` fixed for `tauF`/`swirl`/`cB`; `layout.py` docstring fixed
   (`W_i(1)`, 19→16/13→7); A7 exported or struck (`reset_mrp`/`sigma_indices` top-level);
   WS2 absolute-h: convert-or-restrict before any revival (presets/mock stay flagged);
   `Jt_ignored` validated-or-documented at `system.hpp:65`; `setUMax` dirty (with B2).
3. CI (closes L4): extend purity scan to `environment.hpp`/`force_torque.hpp`/`backsub_t.hpp`/
   `linear3.hpp`+wrench path; state-index rule to `ctypes_driver.py`; count-grep to bare
   `19`; reconcile probe `GRAPH_FUNCS` with CI bodies (or demote probe to non-gate);
   reword Ground-8 to per-config tiers (0.548/3.3/5.0s); expose module RHS-bytes accessor
   or record the `graph_bytes` skip as explicit tech-debt (not silent pass).
4. Ledgers/tests (closes L5, X2-remainders): PLAN 7/7→11/11; X2 docstrings struck; probe duplicate removed;
   half-pinned swap shapes fully pinned; host-build the tree and confirm SWIG
   `vector<Message<…>>`, `Message` resize, and every payload field in the Update-6 risk
   list before green.
5. Phase-3 script→envelope map (no audit ID; Phase-3 entry gate): enumerate the
    feature set of each Phase-3 script (demo, LEO-drag, stationkeeping — cutoff
    values, min-pulse, tank count, jitter, wheels, env links) and pin every feature
    to a post-2.5 envelope cell or a loud reject. Anything unmapped blocks Phase 3
    entry — no "unmodified script hits a new fatal" surprises.
**Probe/gate:** CI extended scopes green; ledger-count asserts match; host build clean;
script map reviewed with zero unmapped.
**DoD:** L1–L5 closed; no bare number without a pointer remains; script map complete.

## Validation matrix at 2.5 close (pre-registered tolerances)

| Gate | Required |
|---|---|
| B/C signed-component, both arms vs stock, `c=0.1`, matched dt | ≤1e-9 per axis; cross-path ≤1e-9 |
| Cutoff ladder {10,100,1000} impulse ratio | 1.00 ± 0.01 fire + shutoff |
| SysModel fuel 600 s centered+offset | `fuel_rel_err == 0.0` (A) or publisher absent (B); fuel+env cells in switching class |
| Stribeck vec `rw3_fric` | retained 1e-14 class; SysModel friction-ON ported-or-rejected/labeled |
| Guard behavioral suite | every fatal-today trap raises pre-propagation; modeled-set green |
| RW telemetry | OFF→OFF echo; `setUMax` effective; DV rotated; double-Reset clean; diffusion raises; vec switch count leg-total ±1 vs stock, tick trace recorded |
| Env/offset host trio | firing-proven (drag ~1e-7-class gates); SRP-shifted FRAGILE until link lands |
| N-body/rotHat/Gt/penumbra/densityCorrection | reject-or-model each; {0.1,1,10}s penumbra |
| Robustness | printer test; ramp trace; M14 raise; phase/PWM true cells |
| Regression | t2/F/T1/t7/t8 record numbers unchanged; `pytest` full green; CI extended green; JIT tiers held; `has_par`/NPOS/c-mask untouched semantics |
| Docs | zero bare numbers; strikes/exports/CI scopes per C3; script map zero-unmapped |
| Multi-spacecraft two-object leg (A4) | per-object deltas in switching class; step audit proves no double-step; single-object cells unchanged |

## Implementation order (locked)

1. **A1 → A2 → A3 → A4** (sign, cutoff, fuel, sync — A3/A4 re-baseline layout and
   stepping; nothing else lands mid-wave-A). A1/A2/B1 may overlap (disjoint regions,
   separate commits); A3 follows A1/A2 and A4 follows A3 (settled base for churn).
2. **B1 → B2 → B3** (guards first so B2/B3 additions are born-gated; telemetry before friction-gates that read it).
3. **C1 → C2 → C3** (tests before robustness-probes before doc-sweep; C3 strike-list last so it records what actually shipped).
Each WP its own commit, independently revertible except the stated chain; every WP
re-runs the full sweep.

## Risks

- **Sign-flip blast radius:** A1 touches every offset expectation (tests, ledger texts,
  DESIGN). Mitigation: A1 lands alone with signed gates; cross-path gate catches strays.
- **Cutoff deletion vs hidden reliance:** some driver may depend on the attenuating
  filter. Mitigation: grep `setCutoffFrequency` callers pre-delete; default-10 runs must
  improve toward stock, never regress.
- **Fuel-port blast radius (SysModel):** per-tick mass/I/c writes change every SysModel
  par layout. Mitigation: name-derived slots only (`par_index`/`has_par`); `checked_par_count`
  + bounded pushes; mask-popcount expectations updated in the same commit.
  wp7 layout caps re-baselined in the same commit if the port grows the graph.
- **Sync-path blast radius:** A4 touches gather/scatter/stepping shared with the
  single-object path. Mitigation: single-object cells re-green in the same commit;
  per-object graphs isolated; step-count audit proves no double-step.
- **Guard tightening vs existing rigs:** new fatals can break currently-green configs.
  Mitigation: B1 ships with the amended table + per-trap messaging citing the envelope;
  sweep must be green *because* rigs were updated, not because guards were softened.
- **JIT growth:** live-c/fuel/Stribeck stay par-inputs, never graph terms (Ground-8 tiers
  enforced, not monitored).

## Explicitly out of 2.5 (labeled, not silent)

- Multi-tank depletion, per-tank thruster subsets, empty-tank re-ignition dynamics,
  closed-loop FSW fuel acceptance; faceted-occlusion SRP; ERP force parity; bit-exact
  jitter streams (libc++ barrier stands); prescribed/flex/slosh/stochastic modeling;
  in-place stock-effector mutation support; independent ephemeris propagation.
