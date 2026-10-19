# Audit_0-2 — Adversarial re-audit of `Python/native_omni_heyoka` after new commit

Date: 2026-10-13 (round 0-2). Method: re-read `Audit_0-1.md` as regression
baseline, then 6 parallel adversarial subagents (physics core / RW / thruster-timing /
env+offset / swap+spacecraft / build-guards-docs), each ordered to return
FIXED / STILL-OPEN / REGRESSED with `file:line` proof plus fresh findings. Lead
spot-checked the Critical/High and all newly-claimed-fixed items against source
(`grep` + targeted reads) before compiling. No files modified except this report.
All refs relative to `Python/native_omni_heyoka/`.

New commit scope (by mtime + size vs 0-1 baseline): `include/omni/contributors.hpp`
(→669 L), `include/omni/system.hpp` (→845 L), `src/omni_vec_integrator.cpp` (→1030 L),
`omniheyoka/swap.py` (→302 L), `BUILD_STATUS.md` (→649 L, Update-8 extended),
`tests/test_wp7_dispersion_jit.py` (→49 kB), `audit_bundle/results/swap_p1_parity.json`
(ts 2026-10-13T08:54:25Z, new cells). `src/omni_spacecraft.cpp` UNCHANGED (2026-10-13
07:19, before 0-1) — every SysModel-path 0-1 finding is therefore still in the code
unless fixed header-side.

Environment limit (unchanged): no `git`/`python3`/Basilisk build on this Linux
container; host-only wheel/sim claims verified as code+ledger consistency, not
re-execution. `importorskip` sim tests skip here by construction.

## Verdict summary (delta vs 0-1)

| Area | 0-1 | 0-2 |
|---|---|---|
| Option-A core math | Sound | Sound, unchanged |
| Thruster c-axis interior gap (x-only `cBx` dead) | Medium open (M5) | **FIXED — source masking verified by 3 agents** (generic suffix-only rule + printer assumption remain) |
| Swap rejects "7/7 loud" vs silent friction | High open (H6) | **FIXED mechanically** (friction→parity, ledger `all loud`, gate checks 6; doc remnants stay) |
| Vec friction Stribeck-exact + `rw3_fric` 1e-14 | Open | **FIXED vec-only** (SysModel still iter-1) |
| Vec fuel bit-exact + live mass/`I_C`/c + NPOS sentinel | Open/partial | **FIXED vec-only** (SysModel static) |
| B/C sign (`+` vs `−`) | Critical open | **Still open — fix direction now decided by signed-component evidence favoring vec `−`** |
| SysModel cutoff filter (divergent 0.63x) | High open | Still open — relabeled, not fixed |
| Loud-reject envelope (force-only guard) | High open | Still open, byte-identical |
| RW placeholders / `u_min`·`P_max` silence / DV frame / vacuous env tests / SRP-shifted / M1–M4 / L1–L5 | Open | All still open (details below) |
| New Highs this round | — | SysModel fuel never decreases; fuel×env arm split; `setFrictionHubSign` dead; vec unbounded fuel writes + isp-length silence; stale vec comments; cutoff asymmetry; BUILD_STATUS heading/scope overclaims |

## Fixed (verified, do not regress)

### X1. Thruster CoM-axis interior gap closed at source — VERIFIED by 3 agents
- Source mask `contributors.hpp:479-487`: `cx⇐dy|dz`, `cy⇐dx|dz`, `cz⇐dx|dy` — exact
  against `t=(r−c)×F` (`tx=(ry−cy)Fz−(rz−cz)Fy`, `ty=(rz−cz)Fx−(rx−cx)Fz`,
  `tz=(rx−cx)Fy−(ry−cy)Fx`). Union over thrusters; `pos` correctly ignored;
  `tmax==0` skip unreachable (`system.hpp:87-88` rejects `≤0`).
- Only coupled axes registered (`:507-523`); masked axes read structural `comOff_`
  literal (`:570-573`) — provably dead in graph (coupled `F` component is 0), so the
  literal is exact, not a drop. x-only → mask `{F,T,T}` (`cBy,cBz` live); stale 12U
  interior gap cannot recur. Diagonal/full → all live; near-axis tiny-nonzero
  over-registers harmlessly.
- Module writes per-axis via `has_par` + `NPOS=SIZE_MAX` (`vec:27,52,477-480,918-919`,
  guard `cOff[j]<P.size()` fails for NPOS — no write, no OOB). `has_par`
  (`system.hpp:324-330`) non-throwing. `checked_par_count` (`:414-440`) retained as
  backstop; all pushes heap-bounded (`spacecraft:973,1035`, `vec:965-966`).
- Consequence: thruster c-subset is always trailing own-pars (env/fuel `n_pars=0`), so
  the mask is always suffix-safe; generic interior gaps from other sources still throw
  loudly (safe posture). `test_wp7:121-147,1096` mask-popcount (`hub+12thr 23=13+7+3`,
  default 20 with `n_thr_c=0`) consistent. A3-clean (mask from doubles at construction).
- Residual (not physics): document the ordering constraint; pin the `p<i>` printer
  assumption (`:419-431`) with a test.

### X2. Swap rejects H6 fixed mechanically — VERIFIED
- Ledger `swap_p1_parity.json:230-238`: 6 loud (`fuel_unmirrored_cxx`, `jitter_wheel`,
  `two_rw_effectors`, `thruster_no_factory`, `min_pulse`, `cutoff_zero`),
  `verdict="all loud"` — no `friction_wheel` entry (correct: friction is DONE).
  Gate `test_swap_p1.py:100-110` checks exactly those 6. New `rw3_fric` cell
  (`:67-76`: σ 8.3e-13, ω 1.9e-14, wheels 1.42e-14) proves friction-as-parity.
- Remainder (docs): `PLAN_INTEGRATOR_SWAP.md:138`, `BUILD_STATUS.md:532`,
  `test_swap_p1.py:10-11` docstring still say 7/7 / list friction as reject. Strike them.

### X3. Vec friction stock-exact + `rw3_fric` 1e-14 — VERIFIED vec-only
- `vec:771-823`: full upstream law (0.10·`omegaLimitCycle`+`betaStatic` gate, sign
  latch, Gaussian+tanh branch, limit-cycle rescale, `omegaBefore` update),
  `FricState` (`vec_integrator.hpp:190-194`), mirror all-zero→OFF (`swap.py:161-173`,
  `betaStatic` default −1.0, enable iff `>0` — consistent).
- SysModel path is NOT fixed (`spacecraft:873-875` byte-identical iter-1; no
  latch/`omegaBefore`; `fStatic`/`betaStatic` inert). Update-8 heading "Friction
  (Stribeck, stock-exact)" (`:602-604`) unscoped — SysModel readers will mis-apply.
  F-ledger null rows (`wp2…json:161-173` all-`null` readbacks under `ok:true`) unamended.

### X4. Vec fuel bit-exact + live mass/`I_C`/c + NPOS — VERIFIED vec-only
- `vec:681-693` live `c=(mHub·rBcB+mFuel·rTB)/mT` (no-fuel exact to record);
  `:876-889` `mdot=Σduty·Tmax/(Isp·g0)+leak`, clamp `≤mFuel/dt`, non-negative;
  `:890-915` `m=mDry+mFuel`, `I_C=I_hub+PA(rBcB−c)+mFuel·PA(rTB−c)`; `:918-919` c-slots
  track live c. Ledger `fuel_centered`/`fuel_offset`: `fuel_rel_err 0.0`,
  `burned 1.12 kg` (`:188-213`). Scope caveats correctly labeled (single-tank raises,
  empty-tank labeled, subset-linkage assumption, closed-loop follow-up).
- `cOff` NPOS init/guards (`:27,52,477-480,918-919`) replace the `==0` sentinel.
  SysModel has no live-c writes (grep: zero `cBx/cOff/massOff` hits) — static snapshot
  exact while unfueled, latent gap under burn.

### X5. Still-closed historical fixes — RE-VERIFIED
String-bridge removal (no shipped `_parser` import), `omni_taylor_step` pars+state push,
`nWheels()=wheels.size()`, registration replay, name-resolved offsets, `u_current`
readback, GG-flag mirror, stock-owned ordering, `checked_par_count`+bounded pushes,
config functor dead-end, single-tank guards, `MinOnTime`/cutoff/blowdown/swirl
swap-side rejects, `thrustFactor`/priority traps avoided, `tests/`-supersedes-benches,
tilt-W3 envelope, build recipe. `test_wp7` static + `test_js_flows_per_run` keep the
same-team-reference gate closed.

## Critical (unchanged, direction now decided)

### C1. B/C sign: SysModel `+Rᵀc` vs vec `−Rᵀc` — STILL OPEN, fix direction decided
- SysModel `spacecraft:628-629` (`r_BN=r_CN+Rᵀc`) byte-identical to 0-1; vec
  `vec:698-703` comment + gather `:734-735` (`y=rB+Rc`) + scatter `:1002-1003`
  (`rB=y−Rc`) unchanged (line drift only). `test_wp4_offset.py:59-66` still pins `+`
  with a self-check (`:464-469`, omni-vs-omni, not stock).
- NEW signed-component evidence (vec side only): `swap_p1_parity.json:152-162`
  `thr_offset` (offset 0.1) records omni-vs-stock deltas `r_BN=r_CN=2.24e-08` — wrong
  sign would give `2·Rᵀc≈0.2 m`. Vec `−` matches stock component-wise to 2e-8.
  SysModel `+` has no comparable stock-pinned cell.
- Impact/trigger: any `c≠0` SysModel-vs-swap or vs-stock B-point run diverges 0.2 m.
- Fix: flip SysModel to `−`; replace norm-only (`|d|==0.1`) gates with signed-component
  gates. Do not ship cross-path offset runs until unified.

## High (all still open unless noted)

### H1. SysModel cutoff filter (0.63x divergent) — OPEN, relabeled not fixed
- `spacecraft:228-232` arms `cutoffLagOn_`, `:1020-1025` ships unconditional
  `lag+=(target−lag)·α` — the exact design Update-8 body rejects (vec `:951-957`
  "NO filter… attenuates 0.63x — reverted", ledger `thr_lag10==thr_fresh_05`
  bit-identical `:143-151`, M2 norms identical). `setCutoffFrequency(10)` (stock
  default) attenuates ~0.632x on SysModel, 1.0x on vec — arm split.
  `test_wp3_timing_models.py:715-718` still asserts `0.3<ratio<0.9` as PASS (codifies
  the bug; should assert ≈1.0). Update-8 caveat "SysModel exact at pinned 1000,
  unvalidated at 10" (`:616-617`) is honest labeling: at 1000 the filter ≈ identity.
  Heading "cutoff lag (default-ON)" (`:602`) contradicted by body "NO lag model"
  (`:610-618`) and by `spacecraft.hpp:230/271` (`cutoffFreq_=10` but `cutoffLagOn_=false`
  default-OFF). Fix: delete/ramps-gate the SysModel filter; fix heading + ratio test.

### H2. Loud-reject envelope (force-only guard) — OPEN, byte-identical
- `spacecraft:743-753` builds `active` only from forceMsg link; comment declares the
  rest "Modeled/informational (never fatal)". Heads verified; coverage unchanged.
  `DESIGN A14:539-546` codifies shipped force-only; `PLAN:472-487` + Ground-7 still
  demand per-trap raise-until-modeled — contradiction unresolved.
  `test_wp0_guards.py:22-42` still keyword-in-text + CI-exit asserts, zero behavioral
  raise tests; `wp0_guards.json` silent-trap `ok:true` by skip. `u_min`/`P_max`/
  `maxWheelAcceleration`/`largeTorqueThreshold` still have no member/flag/guard (grep:
  zero dynamics hits; telemetry literals `:666-667` only). CI 3/3 still must not be
  cited as loud-reject proof. Fix per 0-1 (implement per-trap raise or amend PLAN +
  behavioral tests).

### H3. RW log placeholders + `setUMax` staleness + unimplemented hijack — OPEN
- `spacecraft:658-674` unchanged: only `Omega/Js/u_current/u_max/gsHat` real;
  `u_min=1e-6`, `Omega_max=6000 RPM`, `P_max=200`, `frictionTorque/theta/U_s/U_d/…=0`,
  `RWModel=0`, `mass/rWB=0` hard-coded. `setUMax:405-409` still no `wheelUMax` update,
  no `configDirty` (lead `grep`-confirmed; contrast every structural setter which sets
  it). `system.hpp:195-198` `u_max_` still zero reads. `GsMatrix_B|JsList` still zero
  hits (geometry via setters only; `Omega_max` setter-only honored).
- Update-8 "u_min/P_max/Omega_max need NO work (we read u_current)" (`:619-620`) holds
  vec-only (`vec:757-770` post-`ConfigureRWRequests` readback) and is FALSE SysModel-side
  (`spacecraft:821-836` raw motor message + Omega cut + uMax clamp only). Scope the claim.

### H4. Vacuous host env/offset gates — OPEN, nothing added
- `test_wp45_env_offset.py:297-313` (drag), `:398-410` (`None→return` skip-as-pass),
  `:456-460` (offset==zero-offset) still pass on both-zero; docstring `:16-22` still
  demands firing/scale gates the bodies lack. Partial mitigation lives only in swap
  probes (`:781-796` vacuity guards), not the three host tests. Fix per 0-1
  (`norm(sstep)>thr` + `assert firing`; failure not `return`).

### H5. SRP-shifted without stock evidence — OPEN, no new evidence
- `wp45_combo.json:299-397` `srp_offset` still `status:error` ("No valid heading
  source"). Both paths still ship `(rCop−c)×F` unconditionally (spacecraft static-c
  `:1260-1265`, vec live-c `:627-638`). `env_srp` ledger cell is cannonball-only,
  zero-offset — does not cover the faceted claim. Keep the 3-line-flip contingency and
  FRAGILE label; land facet+sizing link before claiming.

### H6. NEW — SysModel fuel never decreases (not even bookkeeping) — CONFIRMED by lead grep
- `fuelMass_` assigned once (`spacecraft:402`), published verbatim (`:730`),
  `fuelMassDot=0.0` (`:731`); zero decrement hits in file. DESIGN:533-535 promises
  "bookkeeping from staircase-ACHIEVED thrust". A 600 s burn publishes constant tank
  while stock depletes 1.12 kg (vec ledger scale). Worse than labeled non-parity.
- Trigger: any SysModel run with `setFuelParams` + firing thrusters.
- Fix: integrate `mdot=−F_ach/(Isp·g0)` per tick (vec `:876-890` reference) or remove
  the publisher until modeled. SEVERITY High.

### H7. NEW — fuel×env arm split, zero coverage — SUBAGENT-VERIFIED
- Vec: drag arm static `envComOff_` (`:586-588`) vs SRP `rCop−cLive` (`:632-634`) vs
  thruster live `cOff` (`:918-919`) — under burn `cLive` drifts while drag does not
  (order `Δc×F_drag`), with no stock verdict on which is right. SysModel: both frozen
  (+ mass frozen → `F/m` wrong ~0.15% growing to 1.1 kg-scale). No fuel+drag or
  fuel+SRP cell exists (`_env_fuel_cell` is thrusters+tank only). Per-tick fuel vs
  per-arc env-hook ordering itself is clobber-safe and ZOH-lagged-negligible
  (env-only zeroing both paths; `mdot·dt≈2e-4 kg` unmodeled — correctly negligible).
- Fix: add fuel+env combined cells (or label drag-under-burn out); decide whether stock
  shifts `comOffset` with depletion. SEVERITY High.

### H8. NEW — `setFrictionHubSign()` dead setter (escalates M2) — CONFIRMED by lead grep
- `spacecraft:366` stores `frictionHubSign_` (single hit in file; no `configDirty`,
  no propagation to `ReactionWheelContributorT::frictionHubSign_`). Contributor sits at
  default `+1` forever: `setFrictionHubSign(−1)` + friction-on silently computes `+1`
  physics, not the audited `2·tauF/Js` leak (which assumed the sign arrives). Default
  parity unaffected; the F4 reaction-sign probe path is untestable.
- Fix: thread through `OmniSystem` (cf. `set_com_offset`) or restrict/reject `−1`.
  SEVERITY Medium-High.

## Medium (delta)

- **M1 absolute-h header comment — STILL OPEN** (`contributors.hpp:273` absolute vs
  `:322` relative; file top `:11-15` already relative — self-contradiction). One-liner.
- **M2 hub-only sign — STILL OPEN as far as it reaches** (`:328` `s·tauF` hub vs
  `:349` bare `tauF` spin); now compounded by H8 (sign never arrives). Fix both together.
- **M3 `r_BcB` throw row — STILL OPEN** (`system.hpp:41-63` stores verbatim; hub
  `:179-243` never reads `c_B_`). Strike DESIGN:139 or re-add throw for hub-only.
- **M4 singular guards / `cond(S)` / zero-skip — STILL OPEN, all four sub-items**
  (no `det==0`, no graph `r=0`/`dn>0`, stale `cond=1.0007`, overstated "identical
  arithmetic"). Document NaN posture; qualify DESIGN:110-112.
- **M5 generic rule — PARTIALLY FIXED** (c-class closed per X1; suffix-only +
  `p<i>` printer assumption remain, loud-safe).
- **M6 N-body `i≥2` frozen — OPEN unchanged** (spacecraft `:1178-1192`, vec `:556-570`;
  no per-index vector, no `>2` reject).
- **M7 rotHat/Gt — OPEN unchanged** (no param/member on either path; `|m|`-only clamp).
- **M8 min-pulse memory — OPEN as labeling gap** (suppress `:937-947` correct per M3
  ledger; `carryover/lastFireStep/fireCounter` write-only; `contributors:622-628`
  pure suppress). Do not claim memory.
- **M9 ramps — OPEN** (tail truncated at `dt−eps` `:981-994`; implicit-even shape
  `:1139-1153` vs explicit stock pairs; no `ThrustFactor(t)` trace gate).
- **M10 flagship scope — OPEN as worded** (vec DONE per X4; SysModel static per H6;
  SRP-arm static-vs-live split; Update-8 "everywhere" unscoped).
- **M11 DV frame bug — OPEN** (`spacecraft:1075-1083` body `F` into `DV_CN_N`/`DV_BN_B`
  unrotated; "non-parity" label does not cover frame error).
- **M12 staleness/mutation — OPEN** (fill-before-resolve `:528-539` vs `:558-571`
  benign-today; `setUMax` dirty per H3; vec stock-side mutation undetected
  `:668-674`; stochastic structurally silent `:349-361`; Reset `:755-770` still keeps
  DV/fuel accumulators — outer members survive fresh `Impl`).
- **M13 jitter/friction ledger — OPEN** (J1 40-not-400 ticks; split-`mirror` bit-false;
  FullyCoupled setter-gated only on SysModel, covered on vec via `swap:152-156`;
  `wheelThetas` zeros unlabeled; F-rows all-`null` under `ok:true`; SysModel no latch
  vs vec latch). Stale-`u` short-message tail downgraded to unreachable (36-slot drivers).
- **NEW M14 — vec unbounded fuel writes + isp-length silence (Medium).**
  `P[mdotOff/massOff/IO]` (`vec:889-890,913-914`) lack the `<P.size()` guard every
  sibling write has; `setThrusterIsps:204-211` + `880-884` read missing entries as
  `0.0` (mdot undercounts while thrust fires). Safe today (fuel pars mid-vector, never
  suffix-dropped; mirror always sets full list) but one refactor from heap OOB / silent
  divergence. Add bounds + build-time length check.
- **NEW M15 — stale vec comments (Medium-low, lead-grep-confirmed).**
  `vec:870-875` "c_B stays build-time static" contradicts live-c `:681-693,891-919`;
  `vec:825-826` "Friction slots stay 0 (stage 1…)" sits after the DONE friction block.
  Delete/replace (editor-trap class).
- **NEW M16 — cutoff asymmetry load-bearing (Medium, same fix as H1).**
  Vec-accepted-10 vs SysModel-attenuated-10 now proven by `thr_lag10==thr_fresh_05`.
- **Blowdown warn-vs-must-raise + swirl global-only — OPEN as drift/fence** (warn+hold
  `1.0` vs PLAN must-raise; single `swirlScale` vs per-thruster stock).
- **`test_wp3` codified-ratio + M4/M5 probe labels — OPEN test-strength**
  (ratio `0.3–0.9`, omni-vs-omni M4, same-tick 20 Hz proxy, vacuous two-writes,
  `thrustFactor` lag ungated; geometry helper + matched-dt + vec achieved-duty stay fixed).

## Low / hygiene (delta)

- **L1 counts — OPEN.** DESIGN body 19-par (`:166,318-324`) vs live 16 (hub+3RW
  `6+7+3`, `test_wp7:168`) / 23 full-mask (`test_wp7:171`); `system.hpp:14-15` now says
  13/19 but ignores `tauF`/`swirlScale`/`cB` (harness `t7:122-123` expects 23).
  `layout.py:7-24` docstring unchanged (`W_i(3)`, 19/`invI6`/13). Presets/mock/shim
  headers modernized (16 + interleaved `tauF`, no `invI6` — verified clean).
  `Jt` still `/*Jt_ignored*/` (`system.hpp:65`, any value silent; telemetry now
  `0.5*Js` `:669` — split unlabeled).
- **L2 A7 exports — OPEN.** `__init__:29-49` still no `reset_mrp`/`sigma_indices`
  (`driver.py:77`, `layout.py:249` only); DESIGN A7 still requires them.
- **L3 WS2 absolute-h — STILL ARMED, correctly labeled** (`presets:40,418-424`,
  `mock:34,168` serve absolute reference; flagged at `BUILD:404-406`).
- **L4 CI gaps — ALL OPEN.** Purity scan misses env/force/backsub/linear files;
  state-index rule misses `ctypes_driver.py:72-80` hub literals; count-grep misses bare
  `19` (`bench_tilted.py:183`, `BUILD:437` repro); probe-vs-CI `GRAPH_FUNCS` mismatch
  (probe 60 `if (` hits incl double-side `std::abs`, verdict `ok:true` vacuous);
  JIT tiers honest-in-code (`hub+3RW 0.548s`, `4RW/12thr 3.3s`, `3RW+env 5.0s`) vs
  Ground-8 "fail if >2×" flat text; `graph_bytes` None-tolerant skip (no module RHS
  accessor though `OmniSystem:444` has it).
- **L5 counts/numbers — MIXED.** Test 11/11 current (`test_wp7:183-197` matches
  `bundle_summary.json`), PLAN 7/7 stale (`:448`); `thr_offset` 5.6e-09 text vs
  2.24e-08 ledger (bound covers, provenance doesn't); P-WP4.4 idle rows, penumbra
  default `{0.1,1.0}` + no eclipse-edge sub-arcs, `densityCorrection` setter absent
  (guard exists, control doesn't), empty-cluster bloat — all unchanged.
- **FRESH Low:** `BUILD:593` cites `23=13+7+3` without noting default 20; `BUILD:602`
  heading vs `:610-618` body contradiction (see H1); `swap_p1_parity.py:705+716`
  duplicate `rw3_fric` assignment (wasted 120 s, no correctness impact);
  `test_swap_p1.py:52-53,124` half-pinned shapes; thruster mask `tmax==0.0` exact
  compare (no trigger — setters reject `≤0`).

## Must-fix before green (ordered delta — 0-1 items not repeated except where scope changed)

1. C1: flip SysModel B/C to measured `−`; signed-component gate. (Fix direction new.)
2. H6+H7: SysModel fuel integration or publisher removal; fuel+env combined cells or
   drag-under-burn label. (New.)
3. H1/M16: delete/ramps-gate SysModel cutoff filter; fix heading + ratio test. (Relabel
   insufficient.)
4. H2/H3/H4/H5 + M11/M12: per 0-1 (guards/test-gates/telemetry-frame/Reset/mutation/
   stochastic). `setUMax` dirty flag rides with H3.
5. H8+M2: thread `frictionHubSign_` through `OmniSystem` or reject `−1`; document.
   M14: bound fuel writes + isp-length check. M15: fix the two stale vec comments.
6. Docs/hygiene: scope Update-8 flagship + friction + `u_min` claims to vec path; attach
   the 1e-14 ledger cell+arm; strike PLAN 7/7 + docstrings, Ground-8 tiers, DESIGN
   19-body + `:139` + `cond(S)`, `layout.py` docstring, A7 export-or-strike, CI scope +
   bare-19 + ctypes literals. Host-build the tree; confirm SWIG vectors / `Message`
   resize / payload fields before green.

## Appendix — provenance

- Subagent 1 (core): M1/M2/M3/M4 STILL-OPEN, M5 PARTIALLY-FIXED (c-mask proof:
  `contributors:479-487` expansion + `:507-523` registration + `:570-573` reads +
  `vec:27,52,477-480,918-919` writes + heap bounds), L1 STILL-OPEN, fuel graph law +
  MRP/backsub/env-graph verified-correct, NPOS-fixed + mask edge notes fresh.
- Subagent 2 (RW): R1–R7 regressions (Omega setter-only FIXED; `setUMax`/hijack/`u_min`
  OPEN; vec friction FIXED 1e-14 vs SysModel iter-1 OPEN; jitter all-OPEN; `layout.py`
  OPEN vs presets/mock/shim FIXED; stale-u downgraded; DV/Reset/cutoff OPEN) + F1 dead
  setter (lead-confirmed single-hit, no wiring) / F2 placeholder-gate / F3 `test_wp7`
  clean / F4 mirror edge verified.
- Subagent 3 (thruster): H1/M8/M9/blowdown/swirl/force + mailbox/quantizer/phase/PWM +
  H4-adjacent + c-mask proof (expansion/geometries/reads/writes/suffix-safety) with
  SysModel-no-live-c scope gap — all adopted above.
- Subagent 4 (env+offset): C1 OPEN with new signed evidence (`thr_offset` 2.24e-08);
  H4/H5/M6/M7/M10-SysModel/L5 OPEN; F1 fuel-static + F2 fuel×env coupling HIGH new;
  F3 ordering verified-correct.
- Subagent 5 (swap/spacecraft): new-code map + C1/H6(fixed-mechanically)/M10/M11/H3/
  Reset/`thr_offset`/NPOS-fixed/`setUMax`/mutation/stochastic/empty/multi-sc + NEW
  F1–F8 (stale comments lead-confirmed at `vec:825,875`; unbounded writes; isp-length;
  cutoff asymmetry; mask-`tmax` note; probe duplicate; half-pinned asserts).
- Subagent 6 (build/guards): `test_wp7` mask/JIT/`graph_bytes`/ledger + Update-8
  c-mask/friction-1e-14/cutoff-`u_min` claim audits + R1–R7 regressions — all adopted.
- Lead validation: `grep` confirmed H8 (single `frictionHubSign_` hit, no dirty/wiring),
  H6 (two `fuelMass_` hits, static), `setUMax` (no dirty), vec `:825`/`:875` stale
  comments, NPOS guards; re-read `BUILD:554-649`, `contributors:265-374`,
  `system:400-440`, `swap.py`, `swap_p1_parity.json` (new cells + `all loud`).
