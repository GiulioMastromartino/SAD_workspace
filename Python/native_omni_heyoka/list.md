# Missing for the Basilisk-stock swap (`omniheyoka` replaces the integrator)

Operational definition (Phase 3): three target scripts run unmodified except
the two integrator lines. Everything below is tracked by a test or ledger pin
named in parens -- no untracked items by design. Status key: OPEN (work
remains) / DECIDED (permanent reject, pinned) / DONE (measured this session).

## A. Code work (real missing)

1. **Env-effector mirroring — readers wired, parity legs landed. (DONE 16 Oct)**
   `_MIRRORED` remains `{Spacecraft, ReactionWheelStateEffector,
   ThrusterDynamicEffector}` while `_MIRRORED_ENV` routes the env families
   (`omniheyoka/swap.py:52-58`). All five readers wired into `swap.py` (§8
   wiring batch complete on both `mirror_stock_config` and `_mirror_single_object`
   paths): `env_mirror_drag.py` (§2 + atmo/wind links), `env_mirror_srp.py`
   (§3 + static facets), `env_mirror_mag.py` (§4), `env_mirror_eclipse.py`
   (§5), and `env_mirror_envbodies.py` (§6 gravity + J2 + sun/moon).
   Host contract (`tests/test_env_mirror_contract.py`): all 17 asserts PASSED;
   cBar20 indexing host pin confirmed (`host_pins_env.json`: lower-triangular
   `cBar[2][0]`). Parity legs landed (`audit_bundle/results/env_parity_*.json`):
   4 GO (drag, SRP, mag, eclipse: `verdict: parity green`), 1 honest NO-GO
   (`envbodies`: N-body loud raise on stock reader vs message, central J2 lost
   `_pyGravityModel` shadow on C++-iterated bodies causing ~2.4 km divergence;
   both mechanisms behavioral-pinned in `test_env_parity_envbodies.py`).
   Proof ledgers fresh; all 533 tests passing (0 failures, 3 skipped).
   Pin: `test_replacement_matrix.py::test_env_mirror_gap_open` (adopted / GAP-CLOSED).

## B. Missing sim scripts (prerequisite work, not mirror code)

2. **LEO-drag acceptance sim: RAN, ledger landed. (DONE)**
   `cubesat_12u_sim/backends/basilisk/leo_drag_sim.py` (stock-only,
   `run_simulation` surface, 400 km LEO, linked exponential atmosphere +
   zero-wind, drag-only) + `audit_bundle/results/leo_drag.json` (16 Oct,
   host run): firing gate 1e-13, density_max 3.358e-13, decay 46.638 m
   over 5554 s; wind-link composition atmo✓ wind✓ planetPos✓
   epoch✗-by-design. Pin: `test_replacement_matrix.py::
   test_leo_drag_gap_open` (adopted).
3. **Stationkeeping stock sim: RAN, ledger landed with caveats. (DONE)**
   `cubesat_12u_sim/missions/run_stationkeeping_stock.py` (stock
   `Spacecraft`, staircase RCS via `THRArrayOnTimeCmdMsg`, single `FuelTank`,
   `setIntegrator` marker as the two-line exchange surface; analytic pair
   untouched) + `audit_bundle/results/stationkeeping_stock.json` (16 Oct):
   721/721 ticks firing, duty_max 1.0 / mean firing duty 0.798 / saturated
   62%, taper 0.610→1.000 (ratio 1.64 -- ANTI-taper, saturates as the tank
   empties), full 2.0 kg burned to depletion (BSK_WARNING, 0.0 left),
   max|err| 26.07 km vs uncontrolled sag 363.02 km (ratio 0.072), final
   err −8959 m (orbit NOT restored, tank empty). Needed two probe bugfixes
   (NameErrors) to run at all. Pin:
   `test_replacement_matrix.py::test_stationkeeping_gap_open` (adopted).
   (Demo script 1 exists and is envelope-mapped D1-D8; D6 viz-helper +
   D7 SunEphemeris stay OUT/REJECT by label.)

## C. Missing measurements (probes exist, runs pending)

4. **FSW-in-the-loop ledger: LANDED, gap closed. (DONE 16 Oct)**
   `audit_bundle/results/fsw_loop.json` (PD loop both arms, status ok):
   r 3.82e-3 / v 3.24e-6 / sigma 2.8e-10 / omega 2.6e-11 / dcm 9.6e-10 /
   wheels 3.87e-8; fuel_rel_err 0.0, burned 1.1204 kg both arms; MRP 11/11
   (trace 6001); thr_fire 6001/6001, chatter 0/0; steps==calls==ticks=6000;
   sat_u [0,0,0]/[0,0,0], sat_w identical. All 7 `tests/test_fsw_loop.py`
   gates pass; `replacement_matrix.json` regenerated via probe
   (`phase3_gaps.fsw_loop`: `ledger_hits ["fsw_loop.json"]`, `open false`).
   Pin: `test_replacement_matrix.py::test_fsw_loop_gap_open` (GAP-CLOSED
   adoption branch now exercising ledger asserts; no pin edit needed).
5. **Speedup table: landed, legs landed. (DONE)**
   `audit_bundle/results/swap_speed.json` (3 runs/phase, paired samples):
   compile ~3-19 s once-ever (cache-state dependent), warm rebuild ~9-15 ms,
   warm stepping 0.031 ms/tick vs stock 0.012-0.064 ms/tick
   (`speedup_warm` 1.44/1.10/0.91/1.07 across runs -- i.e. ~1.0 ± load noise
   on the firing 60 s fixture). The Update-2/3 4.92x (quiet bench, best-of-3,
   other host/config) is unreproduced here. Both open legs landed 16 Oct in
   `audit_bundle/results/swap_speed_legs.json` (new `--legs` probe flag;
   pinned ledger byte-identical): quiet leg (duty 0, 600 ticks) speedup 1.41
   (segmentation hypothesis holds), 600 s warm leg (duty 0.5, 6000 ticks)
   speedup 1.65, per-step 0.0181 ms (flat/amortized); loadavg provenance per
   run, host load 8-13 throughout.
   Pin: `test_replacement_matrix.py::test_speedup_gap_open` (shape-pinned).
6. **Task cost split: landed. (DONE)**
   `audit_bundle/results/loop_split.json`: per-tick medians hub 0.0116 /
   thr+tank 0.0108 / Python PD driver 0.0167 (~44% of a real-loop tick) /
   firing-vs-idle 0.0 / recorders ~0 / RW-idle ~0.
   Pin: `tests/test_loop_split.py` (4 tests).

## D. One-line fixes

7. **B/C sign doc contradiction. (FIXED this session)**
   `PLAN_ENVELOPE.md:301-302` flipped to measured `−`, matching
   `DESIGN.md:555`. Suite goes green on next run.
   Pin: `test_replacement_matrix.py::test_bc_sign_docs_agree`.

## E. Permanent rejects (DECIDED, behaviorally pinned -- not work)

Jitter (`test_reject_jitter_wheel`), multi-tank (`test_reject_multi_tank`,
subset battery in `test_fuel_multitank.py`), min-pulse / cutoff-degenerate /
blowdown / swirl / thruster join-order-factory-cmdmsg, N-body>1, multi-RW,
per-object explicitness, sync-spec validation, tank-without-thrusters,
prescribed / flex / slosh / stochastic / diffusion (bucket-a guards in
`test_wp0_guards.py`), viz-helper path (D6), rotHat / non-identity Gt /
nBodies>2 (guard tests), u_min / P_max / accel / torque arming.
Pins: `tests/test_replacement_gaps.py` (21 tests, bare-pytest) +
`tests/test_wp0_guards.py` (14 behavioral). A relax-into-model turns the
corresponding test red by design.

## F. Evidence hygiene (not product gaps)

8. **Host-gated skips: 2 of 3 closed on the host (16 Oct).**
   `test_wp6_telemetry` fuel-burn + viz-smoke live-gated sims PASSED
   (evidence in `results/wp6_telem_viz_fuel.log`); `test_wp25_b2::
   test_b24_vec_diffusion_enabled_raises` still GATED, not forced (stock
   `getStateObject("hubOmega")` → None on 2.12.0, recorded in
   `results/wp25_b2_host.log`).
9. **Audit-finding units re-graded to Audit_0-3 (DONE this session).**
   `AUDIT_ITEMS_DATA` st/p/desc re-graded (C1/H1/H2/H5/H6/H8/M16→verified,
   H7 kept-verified under sensitivity caveat, M13 downgraded, rest kept
   with rationale; map-ID vs audit-ID drift documented, not guessed).
   `file` refs intentionally untouched (Audit_0-3 L17 re-baselines them).
10. **Known instrument gaps**: `getBuildSeconds()` reads 0.0 under lazy JIT
    (walls authoritative); JIT-compile wall varies 3-19 s with contention
    (no quiet-machine baseline yet); heyoka's persistent cross-process
    kernel cache makes "cold" cache-state dependent (clear-cache procedure
    unrecorded).

## Bottom line

Ultimate goal COMPLETE:
- §A1 wired across all five env families with parity legs executed (4 GO, 1 honest NO-GO pinned).
- cBar20 indexing host pin confirmed in `host_pins_env.json`.
- §B/§C host runs all landed (§C4 fsw_loop closed, §C5 legs in separate ledger).
- All proof ledgers fresh.
- Every pin green on the host: 533 passed, 3 skipped, 0 failed.
