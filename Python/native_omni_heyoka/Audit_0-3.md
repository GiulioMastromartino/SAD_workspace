# Audit_0-3 — Adversarial re-audit of `Python/native_omni_heyoka` at Phase-2.6 HEAD

Date: 2026-10-14 (round 0-3). Method: re-read `Audit_0-2.md` as regression
baseline, then 4 parallel adversarial subagents (A: integrator core + swap path;
B: SysModel path; C: env/offset; D: docs/CI/meta), each ordered to return
FIXED / STILL-OPEN / REGRESSED with `file:line` proof plus fresh findings. Lead
spot-checked the highest-stakes claims against source (`grep` + targeted reads:
drag arms both paths, B/C `−`, cutoff deletion, fuel NEVER-write, `arm:"static"`
hardcode, phantom citation, BUILD:410 remnant) before compiling. No files
modified except this report. All refs relative to `Python/native_omni_heyoka/`.

HEAD: `0bed2956e55ba345331e449fe45c6168236f7058` ("WP-H closed-loop fuel
acceptance, single-tank (green)"). Source: `.git/logs/HEAD` tail (no `git`
binary in container to confirm otherwise; hash corroborated by `.git/HEAD` →
`refs/heads/optimize/nmpc-pid-params` + `COMMIT_EDITMSG` matching the WP-H
subject). Everything cited is against this hash.

Commit scope (by mtime + reads vs 0-2 baseline): `src/omni_spacecraft.cpp` and
`src/omni_vec_integrator.cpp` both rewritten 2026-10-14 07:06:29 (B/C `−` flip,
drag-arm `live/static-c minus comOffset` both paths, cutoff-filter deletion,
SysModel Stribeck port, fuel option-B); headers + `swap.py` unchanged
(2026-10-13 22:11); `test_wp45_env_offset.py` rewritten to 856 lines (07:08:58);
fresh ledgers `swap_p1_parity.json` (07:09:56Z), `swap_multi.json` (07:10:23Z),
`closed_loop_fuel.json` (07:42:49Z), `vec_mrp_switch_trace.json` (07:43:54);
status map regen 07:12:18; docs `DESIGN`/`BUILD_STATUS` updated 07:09–07:39.

Environment limit (unchanged): no `git`/`python3`/compiler on this Linux
container (lead-verified: `command -v` all exit 1); host-only wheel/sim claims
verified as code+ledger consistency, not re-execution. `importorskip` sim tests
skip here by construction. Timing asserts are flagged unverifiable-here, never
cited as regressions. Load averages: N/A (no runs executed).

## Verdict summary (delta vs 0-2)

| Area | 0-2 | 0-3 |
|---|---|---|
| B/C sign (C1) | Open, direction decided (vec `−` favored) | **FIXED — SysModel flipped to measured `−` (`spacecraft:709-710`), source-pinned** |
| SysModel cutoff filter (H1/M16) | Open, relabeled | **FIXED — filter DELETED (`spacecraft:1315-1324`), ratio test asserts 0.99–1.01** |
| SysModel fuel constant-publish (H6) | Open (High) | **FIXED as option-B absence — publisher NEVER written (`:819-827`) + warn** |
| `setFrictionHubSign` dead (H8) | Open | **FIXED as loud reject (`:397-409`, `-1` raises)** |
| SysModel Stribeck (was iter-1) | Open | **FIXED — full port landed (`:1058-1147`, latch + `omegaBefore`)** |
| DV frame (M11) | Open | **FIXED — inertial slot rotated (`:1375-1405`)** |
| Guards bucket-(a) (H2) | Open, keyword-only | **FIXED — 14 behavioral raise-before-propagation tests** |
| M6/M7 N-body/rotHat/Gt | Open | **FIXED — behavioral raise + disarmed-run pairs** |
| H4 vacuous gates (`test_wp45`) | Open | **FIXED for that file — firing-gated host benches, `None→return` gone** |
| M14 unbounded fuel writes + isp silence | Open (0-2 NEW) | **FIXED — all writes `<P.size()`-guarded, short-Isp raises at build** |
| M15 stale vec comments | Open (0-2 NEW) | **FIXED — both rewritten** |
| Drag arm rule in code | Split (vec `−Rᵀc` vs SysModel `+`) | **FIXED in code both paths (`F×(rBcB−com)`), degeneracy confessed in-tree** |
| Drag-arm *evidence* (ledger) | Open | **STILL OPEN — no discriminating stock row in any checked-in ledger (new H9)** |
| Fuel×env arm split (H7) | Open, unlabeled | **Labeled but unresolved across paths; status-map lift OVERCLAIMS (new H11/H12)** |
| SRP faceted link | FRAGILE/open | **Code landed; ledger cell still open; strike half-justified (M23)** |
| Jitter NO-GO rationale | Open (WP26-D) | **FIXED — reproducible from `jitter_spike.md` numbers** |
| J1 channel blindness | Open (in M13) | **STILL OPEN in probe/test (spike-only label)** |
| X1/X2/X3/X4/X5 seeds | Fixed | **All re-verified, no regressions (lines drifted)** |
| New this round | — | H9–H12, M17–M24, L6–L17 (below) |
| Criticals | C1 open | **C1 closed; no new Critical (sequence held at C2)** |

## Fixed (verified, do not regress)

- **C1. B/C sign unified to measured `−`.** SysModel
  `src/omni_spacecraft.cpp:709-710` (`sc.r_BN_N[i] = ...st... − Rbn·c`,
  `v_BN_N` likewise with `−Rᵀ(ω×c)`; lead-grep-confirmed) + header contract
  `spacecraft.hpp:291-295` + source-pin test `test_wp4_offset.py:193-203`
  (greps both `−` spellings present, both `+` absent). Vec gather/scatter now
  `:1387-1395`/`:1674-1687`. Residual: SysModel `−` has no SysModel-vs-stock
  signed cell of its own (one-sided, was two-sided).
- **H1/M16. Cutoff filter deleted both paths.** SysModel
  `src/omni_spacecraft.cpp:1315-1324` (`double filt = target; ... filter is
  DELETED (it DIVERGED from stock, 0.632x ...)`; lead-grep-confirmed);
  setter `:241-250` records but never arms (`cutoffLagOn_` stays false,
  `spacecraft.hpp:309-310`). Ratio test rewritten
  (`test_wp3_timing_models.py:768-811`: old 0.3–0.9 bound removed, behavioral
  `0.99 < nb/na < 1.01` over fire AND shutoff). `wp3_timing.json` gains
  `cutoff_ladder` + `inert_1pct` + `shutoff_duty05` keys. Residuals: M20
  (ledger predates fix), L7 (lag carcass).
- **H6. SysModel fuel fixed as option-B absence.** `fuelMass_` hits exhaustive:
  `:449` record+warn (`WP25-A3 option B ... use the vec/swap path`), `:819-827`
  NEVER-write block (lead-grep-confirmed: zero `.write` on `fuelTankOutMsg`),
  `:974` Reset restore. Second tank raises (`:890-896`). Residuals: M18, L8.
- **H8. `setFrictionHubSign` fixed as loud reject.** `:397-409` (`s != 1.0` →
  `bskError(... graph bakes +1 ...)`); behavioral test
  (`test_wp25_b2.py:102-114`: `-1` raises, `+1` accepted). Threading explicitly
  deferred (needs rebuild+spin-equation change) — documented, not silent.
- **SysModel Stribeck port landed.** `:1058-1147` (gate `:1107-1108`, latch
  `:1109-1118`, Gaussian+tanh `:1121-1132`, rescale `:1138-1139`,
  `omegaBefore` update `:1141`); `FricState{stribeck, omegaBefore}` at `:39-40`;
  contract test pins port tokens + asserts iter-1 line absent
  (`test_wp2_friction_jitter.py:484-491`). Global-5-knob vs per-wheel vec recs
  noted (`:1070-1071`) — single-valued configs coincide exactly.
- **M11. DV frame fixed.** `:1375-1405` (inertial slot `R_BN·F·k` from
  end-of-step sigma, body slots `F·k`); test burns +z at σ=(0.3,0,0) asserting
  `DV_CN == RᵀF·k`, slots-differ, per-tick uniformity
  (`test_wp6_telemetry.py:478-541`).
- **M12. Reset/build staleness fixed (with leftover M18).** Reset `:966-974`
  zeroes DV triad + accel; build is resolve-then-fill (`:593-612`); `setUMax`
  `:456-469` rewrites `wheelUMax` + sets `configDirty`; double-reset test with
  burn/re-burn vacuity (`test_wp6_telemetry.py:545-594`). Stochastic/diffusion
  silence rationale documented SysModel-side (`:937-956`; vec enforcement
  unverified here — out of scope + no execution).
- **H2. Bucket-(a) guards behavioral.** `checkUnsupported:908-927` raises on
  exactly 13 traps at both heads (Reset `:959-960`, UpdateState `:988-989`);
  14 behavioral tests (`test_wp0_guards.py:164-355`, each `pytest.raises` at
  `InitializeSimulation` + disarmed twin runs 2 ticks); hygiene tests labeled
  HYGIENE-ONLY (`:78-111`). `u_min`/`P_max`/`maxWheelAccel`/`largeTorque`
  telemetry echoes labeled config-echo with armed-non-defaults raising first.
  Recommend a test-count == trap-count cross-pin (future trap could land
  test-less).
- **M6/M7. N-body/rotHat/Gt behavioral.** Gt `:861-875`, rotHat `:877-882`,
  nBodies>2 `:919` arm loudly; raise + identity/disarmed-run pairs
  (`test_wp0_guards.py:248-285`) + post-init head proof (`:337-349`).
- **H4 (wp45 file scope).** Old vacuous trio gone (file rewritten to 856
  lines); every host leg dual-firing-gated (`1e-10 < ‖step‖ ≤ 1e-7`, bands
  derived from ledger scales) with `withholding = FAILURE`
  (`test_wp45_env_offset.py:421-429, 592-616, 663-676, 707-719, 853-856`).
- **M14. Fuel writes bounded + Isp length-checked.** All three write sites
  `<P.size()`-guarded (`vec:1554-1555, 1579-1580, 1584-1585`); short-Isp raises
  at build (`vec:979-983`); `:1544` 0.0-fallback fenced unreachable.
- **M15. Stale vec comments rewritten** (`vec:1482-1484`, `:1533-1534`).
- **Drag arm in code both paths.** Vec live-c `vec:1187-1189`
  (`dragArm{cLive−envComOff}`, lead-grep-confirmed) + SysModel static-c
  `spacecraft:1526-1536` (`rBcB_−envComOff`, lead-grep-confirmed) + shared
  helper `environment.hpp:220-234` (`T = F×arm`, old order documented-rejected
  `:217-219`). Degeneracy confessed in-tree (`vec:1181-1184`).
- **SRP faceted sizing in code.** `projectedArea` + two-term `srpFacet`
  (`environment.hpp:250-271`); per-facet area/normal/CoP/cd/cs wired both
  paths (`spacecraft:580-582, 1574-1587`; `vec:942-944, 1229-1245`); `wp5_env`
  6-facet box pins per-facet `A_proj` analytically
  (`wp5_env.json:465-592`, `test_wp5_env.py:183-195`).
- **Jitter NO-GO rationale reproducible.** `jitter_spike.md` (2026-10-13T22:21):
  stock v2.12.0 zero RNG draws in jitter path vs port 1 normal/wheel/tick
  (stream-matching categorically impossible) + empirical table (same-seed 0.0,
  diff-seed 0.0, J1 cross-model 0.0, hub 3.02e-09/2.02e-08, 0-RPM 0.0) +
  three-bar NO-GO + DESIGN-ban-grounds correction (`DESIGN:603-616`).
- **X1/X2/X3/X4/X5 re-verified, no regressions** (all owned-file line refs
  drifted — Appendix tables — content intact; X2 doc remnants struck except
  L6; A7 exports landed in code, L6 doc remnant excepted).
- **Counts/ledger asserts fixed.** 11/11 vs 7/7 (annotated archaeology only);
  mask-popcount 23=13+7+3 with default-20 note (`BUILD:622-625`,
  `test_wp7:177-178`); `thr_offset` 2.24e-08 replaces 5.6e-09 in docs
  (traced N6); CI L4 all fixed (purity scope, ctypes `LAYOUT-OK` fence,
  bare-19 `CI-EXEMPT`, GRAPH_FUNCS demoted to non-gate, graph_bytes labeled
  debt, Ground-8 per-config tiers).

## High (new)

### H9. Drag-arm rule has no checked-in discriminating stock row — UNPROVEN in ledger
- Code landed both paths (Fixed §), but every in-ledger fixture is
  degenerate-or-idle: combo `drag_offset` (h=[0.1,0,0], com=[0.05,0,0]) sits at
  h=2·com where old and new rules coincide at `0.05·|F|≈6.47e-6 N·m`, AND its
  `"unshifted"` verdict (`wp45_combo.json:229`, `median_zero_rel 0.0106`) is
  VOID (expectation used flat rho=2e-12 while the leg flew rho≈2e-14;
  retraction math `test_wp45_env_offset.py:11-14` checks out dimensionally).
  `drag_zero_com` idle (`firing:false`, norms ~1e-17). `wp5_env` drag analytic
  at c=0 (rules identical). The sole firing combined cell `fuel_drag`
  (`swap_p1_parity.json:214-230`, offset tank [0.3,0,0.2], com [0.05,0,0],
  `dc_norm 5.32e-4`) is sensitivity-limited BY ITS OWN BOUND
  (`shift_sig_dcm 3.56e-5 < dcm_max 4.45e-4`, `:240 vs :216`) and the test
  concedes `G=1 NOT excluded` (`test_swap_p1.py:157-163`); old-static was also
  green there (probe `:594-596`). WP-H leg cannot discriminate (centered tank,
  no env). The two discriminating legs (5-point sweep `:679-719` with zero at
  h==c + 3x/2x/−1x ratios; tilted facet `:840-856` excluding unshifted at ~19x
  miss/match) are host-only `importorskip` with NO recorded rows in any JSON.
- Trigger: citing `fuel_drag` green or any in-ledger cell as proof of the new arm.
- Fix (WP-sized): check in host-measured rows (5-point z-steps + tilted-facet
  steps with firing proofs) as a ledger appendix + a stabilized-IC leg with
  `shift_sig_dcm > dcm` residual, pre-registered.

### H10. Arm label now contradicts code; arm gate is tautological
- Probe hardcodes `d["arm"] = "static"` (`swap_p1_parity.py:850`,
  lead-grep-confirmed) and the test asserts `c["arm"] == "static"`
  (`test_swap_p1.py:169`, lead-grep-confirmed) — passes for ANY code, including
  the new live-c `vec:1187-1189` and a hypothetical revert. Post-fix ledger
  `arm "static"` (`swap_p1_parity.json:224,241`) mislabels a live-c build;
  probe docstrings (`:593-604, :693-706`, `:594` "stays STATIC",
  lead-grep-confirmed) describe the pre-fix arm.
- Trigger: any revert or arm change keeps the suite green while the ledger lies.
- Fix (WP-sized): derive `arm` from behavior or demote to recorded string +
  add a discriminating numeric gate (per H9); rewrite the STATIC docstrings.

### H11. Fuel×env arm split persists across paths by construction, no stock verdict
- SysModel drag uses static `rBcB_` (`spacecraft:1531-1533`) and SRP facet arm
  `{f.rCop − rBcB}` (`:1581-1583`); vec uses live `cLive` for both
  (`vec:1187-1189`, `:1239-1241`). Under measured depletion drift
  (`dc_norm 5.32e-4 m`, `swap_p1_parity.json:223,240`) the paths disagree at
  order `|dc×F|` with no stock verdict on which is right. Labeled (SysModel
  option-B comment `:1528-1530`) but unresolved; the `fuel_drag` green was
  measured tumbling (signature buried — probe notes stabilized ICs DIVERGE at
  dcm≈2.0, `:603-606`, so the green depends on tumbling ICs). No fuel+drag or
  fuel+SRP cell on stabilized ICs exists on either path.
- Trigger: any stabilized fuel+drag burn compared across paths or vs stock.
- Fix (WP-sized): fuel+env combined cell on stabilized ICs, or label
  drag-under-burn out-of-envelope for one path.

### H12. H7 status-map `verified` overclaims a sensitivity-limited null
- `generate_status_map.py:63` marks H7 `verified, p:1.0` ("static drag arm
  stands with 25x sensitivity bound"), yet the bound
  (`|dc×F_drag|≈1.6e-7 N·m → dcm≈3.6e-5`, 25x below the 9.2e-4 residual floor;
  `fuel_drag dcm_max 4.45e-4` vs `shift_sig_dcm 3.56e-5`) means a 1:1 tracking
  arm hides INSIDE the residual. "Verified" is true for parity-in-switching-
  class, not for the arm model; the badge will be misread as "arm proven."
- Trigger: Phase-3 scope decisions reading the map.
- Fix (WP-sized): split status to `verified (parity) / sensitivity-limited
  (arm)`, or gate H7-verified on a stabilized fixture with recomputed bound.

## Medium (new)

- **M17. WP-H negative leg lives only in probe+log, not JSON/test.**
  `closed_loop_fuel.log:1` preserves the divergent candidate (r 1.12,
  dcm 0.116, wheels 0.24, MRP 27/27, off-center tank without burn-line keys) vs
  accepted `log:2` (r 3.82e-3, dcm 9.56e-10, MRP 11/11); JSON carries only the
  accepted leg (`closed_loop_fuel.json:28-76`, `verdict "parity green"` `:81`).
  Scope honestly labeled in probe (`closed_loop_fuel.py:13-25` with mechanism
  + numbers) but ledger/test readers see unconditional green. Fix: add the
  rejected fixture to JSON as a documented non-parity cell + cite in test
  docstring. Firing proofs verified both arms (`burned 1.12038 > 0.1`,
  `disp 3761828.87 > 1e5` both arms, `fuel_rel_err 0.0`).
- **M18. Reset restores FULL tank, not `m0`.** `spacecraft:974`
  (`fuelMass_ = fuelMax_`); comment `:967-969` admits the `m0` record is
  "owned by S4 (leftover)". Trigger: `setFuelParams(m0<max)` + Reset publishes
  full tank. Fix: store `fuelM0_` at `setFuelParams`, restore in Reset.
- **M19. J1 still wheelSpeeds-only in probe rev 2.** `wp2` probe `:310-350` +
  test `:308-313` assert determinism solely on the spike-proven blind channel
  (cross-model `max|d|=0.0`, spike check #3); 400-tick upgrade lengthens a
  vacuous metric; committed ledger still rev-1 2x40. Fix: add hub-channel
  (`omega_BN_B`/`sigma_BN`) determinism cell per spike §5 + regenerate ledger.
- **M20. Post-fix ledgers stale where it matters.** `wp3_timing.json`
  (2026-10-13T22:19) `inert_1pct` keys predate the 10-14 07:06 filter deletion;
  0.99–1.01 behavioral gate host-only (skips here). `wp2` ledger still rev-1
  (2x40, all-null F rows) while probe is rev-2. Trigger: citing committed
  ledgers as proof of the deleted filter / Stribeck port. Fix: regenerate both
  on host post-fix. (All 7 SysModel-side ledgers stale — freshness table,
  Appendix.)
- **M21. Combo shape test is void-tolerant.** `test_combo_ledger_verdict_shape`
  (`test_wp45_env_offset.py:180-192`) passes on the void `"unshifted"` word and
  skips when the ledger is absent. Trigger: ledger refresh re-emitting the void
  verdict stays green. Fix: assert the retraction pointer (verdict ≠ void word,
  or require `superseded` marker / fresh timestamp).
- **M22. Void verdicts ship without in-JSON void flags.** `wp45_combo.json:229`
  (`"verdict": "unshifted"`), `wp4_offset.json:337-368` (idle-zero rows,
  `tel_minus_expect [0,0.5,0]` artifact), `srp_offset status:error :393-396` —
  retractions live only in code/test/docs. No live citer found (all four void
  cells inventoried, none cited as proof). Trigger: future agent/doc citing raw
  JSON at face value. Fix: add retraction fields or retire the files.
- **M23. FRAGILE strike lacks a checked-in faceted stock number.**
  Facet-link wiring genuinely closed in test code (module path + `setNumFacets`
  + `sunInMsg` pinned, no-skip-fallback, firing bands both arms, `:493-537`);
  but the `0.5%` numbers (`5.45093125e-10` vs `5.42305167e-10`) live only in
  comments (`:516-518, :581-583`). `swap_p1_parity.json:179-187` SRP cell is
  cannonball-scale, not the faceted leg. Strike JUSTIFIED for wiring mode,
  PREMATURE as parity claim. Fix: record the faceted rows (same fix as H9).
- **M24. Phantom ledger citation for torque-sign numbers.**
  `environment.hpp:214-216` attributes the 8-digit
  `omni [0,0,−1.07812560e-09] vs stock [0,0,+1.07812559e-09]` comparison to
  `"ledger wp45_combo.json"` (lead-read-confirmed), but that JSON contains no
  such numbers (drag leg is stock-only FD-proxy, no omni arm — full-read +
  `1.078` grep miss). True home: unrecorded host run. Fix: point at the host
  test or check in the rows (same fix as H9).

## Low / hygiene (new)

- **L6. BUILD:410 A7 remnant.** `"reset_mrp/sigma_indices still unexported"`
  contradicts `omniheyoka/__init__.py:28-29,35-36` exports
  (lead-grep-confirmed). Strike the line.
- **L7. `lagState_` carcass.** Declared `:52`, zeroed `:1168`, never read
  (lead-grep: only size/assign hits; shaping uses `lastFilt_/lastQ_/rampClock_`).
  Remove or tombstone-comment.
- **L8. `fuelTankOutMsg` declared, never written.** `spacecraft.hpp:85` +
  `.i:63-64` expose it; zero writes (option B). Attached recorder yields
  all-zeros silently. Label UNWRITTEN or remove member + SWIG include.
- **L9. Closed-loop `build_s` tier gap.** `closed_loop_fuel.json:75`
  `build_s 7.0517` exceeds loosest tier (5.0s) for an unbound config
  (`nPars 21 nStates 16` = hub+3RW+thr+fuel); test asserts no cap. No
  violation — record a tier or explicitly unbind. No load averages in any
  ledger (only `build_s`) — provenance incomplete but honest.
- **L10. Status manifest stale counts.** `status_manifest.json` (07:12:18)
  predates `closed_loop_fuel.json` (07:42:49); parity cells cover `swap_p1`
  only (`generate_status_map.py:138-178`); docstring claims 233/184+19+30 vs
  actual 267/216+21+30. Regen + fix counts.
- **L11. BUILD stale wp3 note.** `BUILD:667-671` claims in-tree
  `wp3_timing.json` lacks `inert_1pct/shutoff_duty05` — false against
  `:167-178,299-344`. Strike the note.
- **L12. CI probe-scope gap.** `ci_envelope_checks.sh:134,174` scopes (b)/(e)
  to `omniheyoka tests scratch*` — new `probes/closed_loop_fuel.py`,
  `swap_multi.py` unscanned (currently clean by grep). Extend or record
  exclusion. (Possible `cr`-convention note `swap_p1_parity.py:804/819/859`
  referred to swap owner — unverified here.)
- **L13. Wiring-unknown skips.** `test_wp45_env_offset.py:376,653`
  (`atmoDensInMsg`/`magFieldInMsg` "wiring unknown") would skip-as-pass on host
  on link drift; contrast SRP leg which correctly fails (`:593-595`). Convert
  to failure per H4 rule.
- **L14. Closed-loop lacks steps==ticks audit.** JSON has `ticks` but no
  `steps`/`calls` (contrast `swap_multi.json` steps+calls==ticks on all four
  object-legs; MRP shared 5/5+19/19, split 5/5+20/20, closed-loop 11/11,
  switch-trace 3v3 all exact). Single-spacecraft path — Low.
- **L15. `swap_multi.log` append hygiene.** 4 full runs appended; JSON keeps
  the last (`build_s` 2.41/3.05/18.6/0.372 → JSON 0.3723). Values consistent
  to 3 sig figs (no cherry-pick evidence) — rotate/truncate per run.
- **L16. Stale names/words.** `dragWrench(... comOffset_B ...)`
  (`environment.hpp:220-221`) receives the *arm* at both call sites —
  invites a "verbatim" revert; rename to `dragArm_B`. Test docstring `:36-43`
  states the fixture-special-case (`T = F×comOffset`) as if general (true only
  at h=2·com) — reword as sign-flip record with fixture note.
- **L17. Global line-ref re-baseline.** Every Audit_0-2 `spacecraft.cpp` ref
  drifted (file →1617 lines; e.g. cutoff `:228-232`→`:241-250`,
  guard `:743-753`→`:908-927`, DV `:1075-1083`→`:1357-1407`,
  friction `:873-875`→`:1058-1147`, B/C `:628-629`→`:709-710`; vec fuel block
  `:681-919`→`:1335-1586` via A4 refactor; per-agent drift tables in
  Appendix). No content regression found; downstream audits re-baseline to
  THIS report. In-tree tests citing block names (not numbers) are resilient.

## Must-fix before green (ordered, WP-sized)

1. **H9/M23/M24 (ledger proof):** check in host-measured discriminating rows
   (5-point drag sweep + tilted-facet steps, firing proofs both arms) as a
   ledger appendix; fix the `environment.hpp:214-216` citation to point at it.
2. **H10 (label honesty):** stop hardcoding `arm="static"` (probe `:850` +
   test `:169`); derive from behavior or demote + add discriminating numeric
   gate; rewrite STATIC docstrings (`:593-604, :693-706`).
3. **H11/H12 (fuel×env scope):** stabilized-IC fuel+drag/SRP combined cell, or
   label drag-under-burn out-of-envelope for one path; split H7 map status to
   `verified (parity) / sensitivity-limited (arm)`.
4. **M20/M19 (host regen):** regenerate `wp3_timing.json` + `wp2` rev-2 ledger
   post-fix on host (with load averages recorded); add hub-channel J1 cell.
5. **M17/M21/M22 (void discipline):** rejected WP-H fixture into JSON as
   non-parity cell + test-docstring cite; harden shape test against the void
   word; add in-JSON retraction flags (or retire `wp45_combo`/`wp4` stale files).
6. **M18/L7/L8/L6/L11 (code+doc leftovers):** `fuelM0_` Reset restore; remove/
   tombstone `lagState_`; label/remove `fuelTankOutMsg`; strike BUILD:410 and
   BUILD:667-671 notes.
7. **L12/L13/L9/L10/L13–L16 (hygiene):** extend CI probe scope or record
   exclusion; wiring-unknown skips → failure; tier-or-unbind closed-loop
   build; regen status map post-WP-H; rename `comOffset_B` → `dragArm_B`.
8. **Phase-3 gate:** host-build the tree; confirm SWIG vectors / `Message`
   resize / payload fields; re-run the behavioral suites on host (all
   FIXED-in-code items above are code+ledger-verified, NOT host-executed).

## Appendix

**Host.** Container Linux, no BSK build, no `git`/`python3`/toolchain (lead-
verified); BSK version for cited host numbers: v2.12.0 (per `jitter_spike.md`;
macOS arm64 for JIT provenance pins). Platform/load during runs: no runs
executed here — load N/A; no ledger in tree records load averages (only
`build_s`) — flagged L9, never cited as regression.

**Commit.** `0bed2956e55ba345331e449fe45c6168236f7058` (WP-H), from
`.git/logs/HEAD` tail + `COMMIT_EDITMSG`; parent chain WP-E `7132e0d…`,
WP26-C `250576f…`, drag-fix `e62adb8…` all present in log order.

**Ledger freshness (ledger ts vs src 2026-10-14 07:06:29).**
FRESH (post-fix): `swap_p1_parity.json` 07:09:56Z, `swap_multi.json`
07:10:23Z, `closed_loop_fuel.json` 07:42:49Z, `vec_mrp_switch_trace.json`
07:43:54. STALE (pre-fix, do not cite as physics): `wp0_guards`,
`wp1_sat`, `wp1_threshold`, `wp2_friction_jitter_voltage` (all 10-12),
`wp4_offset` (10-12), `wp45_combo` (10-12, +VOID), `wp5_env` (10-13 22:11),
`wp3_timing` (10-13 22:19 — new keys predate deletion), `wp6_*` (10-12),
`bundle_summary` (10-12). No stale-ledger-vs-new-build for the FRESH numbers;
staleness bites via labels (H10), void verdicts (M22), and pre-fix keys (M20).

**Per-agent provenance.**
- A (integrator core + swap): X1/X4 STILL-FIXED, M14/M15 FIXED; F-A1–F-A6 →
  H9/H10/M17/L14/L15/L17. Ledger-freshness + vacuity + drift tables adopted.
- B (SysModel): C1/H1-M16/H6/H8/Stribeck/M11/M12/H2/jitter-rationale FIXED
  (code+ledger); J1-blindness STILL-OPEN; F-B1–F-B8 → L7/M18/L8/M19/M20/H11.
  All 7 cited ledgers STALE; new FRESH ledgers vec/swap-side.
- C (env/offset): drag/SRP implementation FIXED, evidence STILL-OPEN;
  cannonball/M6-M7/H4/penumbra-ladder FIXED (ladder+constraint; sub-arcs
  by-design-open); void discipline FIXED-in-docs/STILL-OPEN-in-JSON;
  F-C1–F-C8 → H9/H11/H12/M21/M22/M23/M24/L16. Degenerate-fixture table adopted
  (§H9). Task's "cells in wp45_combo/wp5_env" corrected: combined cells live
  in `swap_p1_parity.json:214-246` only.
- D (docs/CI/meta): N2/N3/N5/N6 traced VERIFIED; N1 doc-only, N4
  unverifiable-here; X2/FRAGILE/A7-counts/CI-L4 FIXED (L6/L11 remnants);
  skip inventory adopted (skip-clean except L13); F-D1–F-D6 → L6/L9/L10/L11/
  L12/L13.
- Lead validation: `grep`-confirmed drag arms (`vec:1187`, `spacecraft:1531`),
  B/C `−` (`:709`), NEVER-write (`:822`), filter deletion (`:1315-1318`),
  `arm="static"` hardcode (probe `:850`, test `:169`), phantom citation
  (`environment.hpp:214-216`), BUILD:410 remnant; read `environment.hpp:
  205-234`, BUILD:410/623/642, swap-test `:157-169`.

## Go / no-go for Phase 3 entry

**CONDITIONAL GO.** The 2.6 program is real: C1, H1, H2, H6, H8, Stribeck port,
M11, M14, M15, H4-wp45, M6/M7, jitter rationale, and both arm implementations
all landed in code with honest scope labels, and the seed (X1–X5) shows zero
regressions. Entry permitted for single-path work (vec/swap closed-loop fuel,
SysModel attitude) under these conditions:
1. No cross-path fuel+drag/SRP comparison, and no "arm proven" citation, until
   must-fix #1–#3 land (H9/H10/H11/H12).
2. Host regen + green behavioral re-run first (must-fix #4, #8) — this audit
   executed nothing; every FIXED above is code+ledger, not host-executed.
3. Jitter-coupled claims stay NO-GO (spike); J1 alone never cited (M19).

**Residual-risk list:** (i) discriminating arm rows host-only, unrecorded
(H9/M23/M24); (ii) cross-path under-burn divergence at order `|dc×F|`, stock
silent (H11); (iii) 7 stale SysModel-side ledgers citable at face value
(M20/M22); (iv) tautological arm gate green under any code (H10); (v) Reset
FULL-tank + declared-never-written publisher + lag carcass (M18/L7/L8);
(vi) no load averages anywhere (L9); (vii) line refs rebased to this report
(L17) — 0-2 numbers are all stale.
