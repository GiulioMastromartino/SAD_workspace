"""Stock-vs-omni benchmark suite: every surface, quick smoke to 1000-run Monte Carlo.

Stock Basilisk ``Spacecraft`` + RK4 arm vs omni arm (the same sim with the two
integrator lines exchanged: ``OmniVecIntegrator`` via
``omniheyoka.swap.mirror_stock_config``) at MATCHED dt. One entry point, four
tiers::

    python audit_bundle/probes/bench_stock_vs_omni.py --tier quick      # smoke (~2 min host)
    python audit_bundle/probes/bench_stock_vs_omni.py --tier standard   # canonical parity legs
    python audit_bundle/probes/bench_stock_vs_omni.py --tier full       # + FSW/ladder/speed/extended sweep
    python audit_bundle/probes/bench_stock_vs_omni.py --tier mc         # 1000-run MC campaign
    python audit_bundle/probes/bench_stock_vs_omni.py --tier demos      # replicas + full hijack-registry sweep
    python audit_bundle/probes/bench_stock_vs_omni.py --tier all        # EVERYTHING: all surfaces + all MC campaigns
    python audit_bundle/probes/bench_stock_vs_omni.py --tier all --n-mc 100            # complete suite, 100 pairs/campaign
    python audit_bundle/probes/bench_stock_vs_omni.py --tier all --n-mc 1000           # complete suite, 1000 pairs/campaign
    python audit_bundle/probes/bench_stock_vs_omni.py --tier all --n-mc 100 --mc-kinds pd,dragDeorbit
    python audit_bundle/probes/bench_stock_vs_omni.py --tier mc --n-mc 10 --mc-kind open
    python audit_bundle/probes/bench_stock_vs_omni.py --list-surfaces
    python audit_bundle/probes/bench_stock_vs_omni.py --tier quick --surfaces hub,rw3_pd,thr_fresh_05

The ``all`` tier is the one-command complete suite: every inline surface at
canonical horizons + rejects + ladder + extended sweep + the MC campaign for
EVERY kind (pd, open, and all seven demos). ``--n-mc N`` sets the pair count
uniformly across all campaigns (100 for a ~1 h sweep, 1000 for the full
campaign); ``--mc-kinds`` subsets the campaigns (default: all).

Inline surfaces (the ``SURFACES`` registry below is the single source of truth;
``tests/test_bench_stock_vs_omni.py`` pins that every family has >= 1 surface,
so nothing benches silently):

    hub              hub-only tumbling (central gravity)
    rw1 / rw2        1 / 2 aligned wheels, open-loop constant torque
    rw3_open_pos     3 wheels, +torque (u_max clamp exercised)
    rw3_open_neg     3 wheels, -torque
    rw3_pd           3 wheels, pure-Python PD (time-varying torque)
    rw1tilt          1 diagonal wheel (1/sqrt(3) axis)
    rw_diag3         3rd wheel diagonal, open-loop (tilted-contributor isolator)
    rw3_fric         Stribeck spin-up 40 s + free spin-down (friction parity)
    thr_fresh_05/10  staircase thruster, fresh-write driver, duty 0.5 / 1.0
    thr_writeonce    single pre-written firing step (one-kink class)
    thr_frozen       2-tick write then frozen message (stale-mailbox class)
    thr_lag10        cutoffFrequency=10 (lag-inert class, Update-M2)
    thr_offset       CoM-offset hub (B/C kinematics, r_CN/v_CN outputs)
    env_drag         LEO drag, tumbling ICs (frozen-projection class)
    env_drag_still   LEO drag, stabilized ICs (tight class)
    env_srp          SRP cannonball + single-tick analytic vacuity guard
    fuel_centered    depletion, centered tank (mass/I feedback isolated)
    fuel_offset      depletion, offset tank (static-c_B deviation, bounded)
    fuel_drag        burn + LEO drag (live-c-tracking arm)
    fuel_srp         burn + SRP cannonball (force-level parity)
    fsw              FSW PD loop + burn + depletion, 600 s (full tier only)
    ladder           dt switching-residual ladder (full tier only)
    rejects          loud-reject battery (correctness, all tiers)

Upstream demos (basilisk/examples @ 2.13.0b0/develop, ``DEMOS`` registry):
setup replicas of the stock demo scripts, benched stock-vs-omni at matched
dt. Live cells: demo_basicOrbit_LEO/GTO/GEO (point-mass hub), demo_attFeedbackRW
(demo-exact RW closed loop), demo_momentumDumping (4 tilted wheels + 8 thrusters
+ full FSW dumping chain), demo_dragDeorbit (250 km exponential-earth deorbit
start), demo_attitudeGG (GG-torque drift). demo_reject_extTorque proves the
extForceTorque PD demo (scenarioAttitudeFeedback) raises loudly. Every other
dynamics-relevant upstream demo has a disposition in ``DEMO_COVERAGE``
(live / covered-by / out with reason) -- no demo is silently unaccounted for.
Tiers standard/full/demos run them (demos tier at canonical upstream horizons).

The ``demos`` (and ``all``) tier ALSO drives the complete upstream-demo hijack
registry as subprocesses: ``hijack_demo.py --demo <name>`` for EVERY runnable
entry in that probe's CONFIG (parity/reject). This exercises the exact
byte-pinned upstream files (not the inline replicas) -- "everything we can" --
and aggregates each owning ledger's status + verdict (``parity green`` /
``reject loud``) here. Entries that probe documents as ``excluded`` (a
capability the mirror honestly lacks) are still recorded in the block, as
``skipped-excluded`` with their reason: every CONFIG entry is accounted for,
so an excluded demo reads as a documented disposition rather than a silent
gap. The full tier keeps its two canonical hijack cells in the extended
sweep; the demos/all sweep folds those in so the registry is reported whole
without re-running them.

Extended surfaces (full tier only, via sibling probes as subprocesses --
aggregated here, never re-implemented): mag, eclipse, envbodies (honest NO-GO,
recorded not gated green), multi_shared, multi_split, speed, speed_legs,
loop_split. The owning probes' verdicts are authoritative; this suite only
aggregates exit code + ledger status.

MC campaign (``--tier mc``, default 1000 paired sims): seeded dispersion over
rotor Js, wheel rates, attitude/rate ICs and hub mass on the rw3_pd leg
(aligned axes -- the tilt-W3 saturation-chatter lesson pins geometry
dispersion to the dedicated isolator surfaces + ``--mc-kind open``, which uses
unsaturated constant torque so tight bounds hold under full geometry
dispersion). Bounds follow the wp7 MC gate (att 1e-9 deg, rate 1e-9 deg/s,
wheels 1e-6 RPM, transient 5e-4 deg); pass requires every sample in bounds.
``--mc-kind <demo>`` (one of ``DEMOS``) runs the same campaign over that
upstream demo's setup instead: each sample runs the demo surface cell with
dispersed mass/ICs/orbit/wheel-speeds/inertia and passes iff the *dispersed*
cell bound (``_demo_mc_bound``) holds. That bound is the canonical cell tol
except where an absolute canonical tol cannot also be a dispersed bound, and
there it is derived from the same evidence class instead of invented: (a)
``r_BN_N`` for a point-mass orbit -- the matched-dt stock-RK4-vs-omni-Taylor
residual is float-rounding class (the canonical 1e-9 m sits at/below one ULP
of the LEO/GTO position), so the bound is a few ULPs of the orbit scale;
(b) ``wheels`` on the momentum-dumping class -- the FSW dump schedule switches
thrusters sub-tick, so the wheel-speed readout carries a switching-jitter
transient the attitude/orbit fields do not, bounded by the recorded jitter
class with margin. The canonical ``cell_tol`` is recorded beside ``tol``. The
drag-liveness vacuity proof is likewise self-calibrating: both arms must read
the *same* peak neutral density (ratio ~1) rather than clear a flat absolute
floor, which would misread the 240-260 km campaign window (rho down to
~6e-14 kg/m^3) as vacuous air. Demo MC horizons: per-demo ``T_mc`` (300 s for
dragDeorbit, 60 s otherwise).

Ledger: ``audit_bundle/results/bench_stock_vs_omni.json`` (+ ``.log``).
Host-only sims (needs Basilisk 2.12.0 + built omni_heyoka); without them the
probe records ``status: skipped-no-basilisk`` and exits 0 -- never errors, so
Linux CI stays green. ``--list-surfaces`` works everywhere (stdlib only).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import subprocess
import sys
import time
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
# Repo root (Python/) on sys.path no matter the cwd: omniheyoka.presets
# imports the sibling cubesat_12u_sim package (same path run_all.sh exports).
# This is parents[3] under the host layout
# Python/native_omni_heyoka/audit_bundle/probes/<probe>.py -- identical to
# every other probe (common.py, swap_p1_parity.py, ...). Do NOT insert
# parents[2] (native_omni_heyoka/): it carries a sourceless omniheyoka tree
# copy that would shadow the installed wheel (conftest.py rationale).
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from common import RESULTS_DIR, log_text, save_result, safe_import_basilisk  # noqa: E402

PROBES_DIR = Path(__file__).resolve().parent

# ---------------------------------------------------------------------------
# Fixture constants (copied, not chosen: swap_p1_parity.py idioms)
# ---------------------------------------------------------------------------
MU = 3.986004418e14
MASS = 750.0
MASS_DRY_FUEL = 740.0
PROP0 = 10.0
I6 = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
SIG0 = [0.1, -0.2, 0.3]
OM0 = [0.01, -0.02, 0.03]
RPM0 = [100.0, 200.0, 300.0]
HR16_JS = 0.07957747154594767
U_MAX = 0.2
MC_K, MC_P = 3.5, 30.0
RPM_TO_RADS = 2.0 * math.pi / 60.0
RADS_TO_RPM = 60.0 / (2.0 * math.pi)
DIAG = 0.5773502691896258  # 1/sqrt(3)

# ---------------------------------------------------------------------------
# Tolerance presets (pre-registered classes from swap_p1 / fsw_loop / wp7)
# ---------------------------------------------------------------------------
TOL_QUIET = {"r_BN_N": 1e-9, "v_BN_N": 1e-9,
             "sigma_BN": 1e-9, "omega_BN_B": 1e-9, "wheels": 1e-9}
TOL_THR = {"r_BN_N": 2e-3, "v_BN_N": 2e-5,
           "sigma_BN": 2e-4, "omega_BN_B": 1e-5}
TOL_WO = {"r_BN_N": 1e-5, "v_BN_N": 1e-7,
          "sigma_BN": 1e-6, "omega_BN_B": 1e-7}
TOL_DRAG = {"r_BN_N": 5e-4, "v_BN_N": 2e-6,
            "sigma_BN": 5e-7, "omega_BN_B": 1e-8}
TOL_DRAG_STILL = {"r_BN_N": 2e-5, "v_BN_N": 1e-7,
                  "sigma_BN": 1e-7, "omega_BN_B": 1e-9}
TOL_SRP = {"r_BN_N": 1e-6, "v_BN_N": 1e-8,
           "sigma_BN": 1e-9, "omega_BN_B": 1e-9}
TOL_FUEL = {"r_BN_N": 2e-2, "v_BN_N": 1e-4, "omega_BN_B": 1e-4,
            "dcm_max": 2e-3, "fuel_rel_err": 1e-9}
TOL_FSW = {"r_BN_N": 2e-2, "v_BN_N": 1e-4, "sigma_BN": 2e-4,
           "omega_BN_B": 1e-4, "dcm_max": 2e-3, "wheels": 1e-4,
           "fuel_rel_err": 1e-9}
# Momentum-dumping class (8-thruster switching + 4 tilted wheels, 2500 kg
# bus): FSW-loop bounds without the fuel law.
TOL_MOMENTUM = {"r_BN_N": 2e-2, "v_BN_N": 1e-4, "sigma_BN": 2e-4,
                "omega_BN_B": 1e-4, "wheels": 1e-4, "dcm_max": 2e-3}
# Gravity-gradient class: smooth env torque (SRP-class precedent).
TOL_GG = {"r_BN_N": 1e-6, "v_BN_N": 1e-8,
          "sigma_BN": 1e-9, "omega_BN_B": 1e-9}

# MC bounds (wp7 MC gate: record 2.7e-13 deg class, gate with margin).
MC_ATT_TOL_DEG = 1e-9
MC_RATE_TOL_DEGS = 1e-9
MC_WHEEL_TOL_RPM = 1e-6
MC_TRANSIENT_ATT_DEG = 5e-4
MC_OPEN_ATT_TOL_DEG = 1e-7  # unsaturated open-loop isolator class

# ---------------------------------------------------------------------------
# Surface registry (single source of truth -- see module docstring)
# ---------------------------------------------------------------------------
# kind: builder dispatch. kw: builder kwargs. T: standard leg [s].
# T_quick: smoke leg [s]. tiers: which tiers run it inline.
SURFACES = {
    "hub": {"family": "hub", "kind": "hub", "kw": {},
            "T": 60.0, "T_quick": 10.0, "tol": dict(TOL_QUIET),
            "tiers": ["quick", "standard", "full", "mc_smoke"]},
    "rw1": {"family": "rw", "kind": "rw1", "kw": {},
            "T": 60.0, "T_quick": 10.0, "tol": dict(TOL_QUIET),
            "tiers": ["quick", "standard", "full"]},
    "rw2": {"family": "rw", "kind": "rw2", "kw": {},
            "T": 60.0, "T_quick": 10.0, "tol": dict(TOL_QUIET),
            "tiers": ["quick", "standard", "full"]},
    "rw3_open_pos": {"family": "rw", "kind": "rw3_open",
                     "kw": {"u": 0.5}, "T": 120.0, "T_quick": 10.0,
                     "tol": dict(TOL_QUIET),
                     "tiers": ["quick", "standard", "full", "mc_smoke"]},
    "rw3_open_neg": {"family": "rw", "kind": "rw3_open",
                     "kw": {"u": -0.3}, "T": 120.0, "T_quick": 10.0,
                     "tol": dict(TOL_QUIET),
                     "tiers": ["quick", "standard", "full"]},
    "rw3_pd": {"family": "rw", "kind": "rw3_pd", "kw": {},
               "T": 120.0, "T_quick": 20.0, "tol": dict(TOL_QUIET),
               "tiers": ["quick", "standard", "full", "mc_smoke"]},
    "rw1tilt": {"family": "rw", "kind": "rw1tilt", "kw": {},
                "T": 60.0, "T_quick": 10.0, "tol": dict(TOL_QUIET),
                "tiers": ["quick", "standard", "full"]},
    "rw_diag3": {"family": "rw", "kind": "rw_diag3",
                 "kw": {"u": 0.05}, "T": 6.0, "T_quick": 6.0,
                 "tol": dict(TOL_QUIET),
                 "tiers": ["quick", "standard", "full"]},
    "rw3_fric": {"family": "rw", "kind": "rw3_fric", "kw": {},
                 "T": 120.0, "T_quick": 20.0, "tol": dict(TOL_QUIET),
                 "tiers": ["quick", "standard", "full"]},
    "thr_fresh_05": {"family": "thruster", "kind": "thr",
                     "kw": {"duty": 0.5, "mode": "fresh"},
                     "T": 120.0, "T_quick": 20.0, "tol": dict(TOL_THR),
                     "tiers": ["quick", "standard", "full", "mc_smoke"]},
    "thr_fresh_10": {"family": "thruster", "kind": "thr",
                     "kw": {"duty": 1.0, "mode": "fresh"},
                     "T": 120.0, "T_quick": 20.0, "tol": dict(TOL_THR),
                     "tiers": ["quick", "standard", "full"]},
    "thr_writeonce": {"family": "thruster", "kind": "thr",
                      "kw": {"duty": 0.5, "mode": "write-once"},
                      "T": 20.0, "T_quick": 20.0, "tol": dict(TOL_WO),
                      "tiers": ["quick", "standard", "full"]},
    "thr_frozen": {"family": "thruster", "kind": "thr",
                   "kw": {"duty": 0.5, "mode": "frozen"},
                   "T": 30.0, "T_quick": 20.0, "tol": dict(TOL_WO),
                   "tiers": ["quick", "standard", "full"]},
    "thr_lag10": {"family": "thruster", "kind": "thr",
                  "kw": {"duty": 0.5, "mode": "fresh", "cutoff": 10.0},
                  "T": 120.0, "T_quick": 20.0, "tol": dict(TOL_THR),
                  "tiers": ["quick", "standard", "full"]},
    "thr_offset": {"family": "thruster", "kind": "thr",
                   "kw": {"duty": 1.0, "mode": "fresh", "offset": 0.1,
                          "extra_fields": ["r_CN_N", "v_CN_N"]},
                   "T": 60.0, "T_quick": 10.0, "tol": dict(TOL_THR),
                   "tiers": ["quick", "standard", "full"]},
    "env_drag": {"family": "env", "kind": "drag",
                 "kw": {"still": False}, "T": 600.0, "T_quick": 60.0,
                 "tol": dict(TOL_DRAG),
                 "tiers": ["quick", "standard", "full"]},
    "env_drag_still": {"family": "env", "kind": "drag",
                       "kw": {"still": True}, "T": 600.0, "T_quick": 60.0,
                       "tol": dict(TOL_DRAG_STILL),
                       "tiers": ["quick", "standard", "full"]},
    "env_srp": {"family": "env", "kind": "srp", "kw": {},
                "T": 600.0, "T_quick": 60.0, "tol": dict(TOL_SRP),
                "tiers": ["quick", "standard", "full"]},
    "fuel_centered": {"family": "fuel", "kind": "fuel",
                      "kw": {"r_tb": [0.0, 0.0, 0.0]},
                      "T": 600.0, "T_quick": 60.0, "tol": dict(TOL_FUEL),
                      "tiers": ["quick", "standard", "full"]},
    "fuel_offset": {"family": "fuel", "kind": "fuel",
                    "kw": {"r_tb": [0.3, 0.0, 0.2]},
                    "T": 600.0, "T_quick": 60.0, "tol": dict(TOL_FUEL),
                    "tiers": ["quick", "standard", "full"]},
    "fuel_drag": {"family": "fuel", "kind": "fuel_drag",
                 "kw": {"r_tb": [0.3, 0.0, 0.2]},
                 "T": 600.0, "T_quick": 60.0, "tol": dict(TOL_FUEL),
                 "tiers": ["standard", "full"]},
    "fuel_srp": {"family": "fuel", "kind": "fuel_srp",
                "kw": {"r_tb": [0.3, 0.0, 0.2]},
                "T": 600.0, "T_quick": 60.0, "tol": dict(TOL_FUEL),
                "tiers": ["standard", "full"]},
    "fsw": {"family": "fsw", "kind": "fsw", "kw": {},
            "T": 600.0, "T_quick": 600.0, "tol": dict(TOL_FSW),
            "tiers": ["full"]},
    "demo_basicOrbit_LEO": {"family": "demo", "kind": "demo",
                            "kw": {"demo": "basicOrbit_LEO"},
                            "T": 600.0, "T_quick": 60.0, "T_mc": 60.0,
                            "tol": dict(TOL_QUIET),
                            "tiers": ["standard", "full", "demos"]},
    "demo_basicOrbit_GTO": {"family": "demo", "kind": "demo",
                            "kw": {"demo": "basicOrbit_GTO"},
                            "T": 600.0, "T_quick": 60.0, "T_mc": 60.0,
                            "tol": dict(TOL_QUIET),
                            "tiers": ["standard", "full", "demos"]},
    "demo_basicOrbit_GEO": {"family": "demo", "kind": "demo",
                            "kw": {"demo": "basicOrbit_GEO"},
                            "T": 600.0, "T_quick": 60.0, "T_mc": 60.0,
                            "tol": dict(TOL_QUIET),
                            "tiers": ["standard", "full", "demos"]},
    "demo_attFeedbackRW": {"family": "demo", "kind": "demo",
                           "kw": {"demo": "attFeedbackRW"},
                           "T": 600.0, "T_quick": 60.0, "T_mc": 60.0,
                           "tol": dict(TOL_QUIET),
                           "tiers": ["standard", "full", "demos"]},
    "demo_momentumDumping": {"family": "demo", "kind": "demo",
                             "kw": {"demo": "momentumDumping",
                                    "reset_at": 10.0},
                             "T": 300.0, "T_quick": 60.0, "T_mc": 60.0,
                             "tol": dict(TOL_MOMENTUM),
                             "tiers": ["standard", "full", "demos"]},
    "demo_dragDeorbit": {"family": "demo", "kind": "demo",
                         "kw": {"demo": "dragDeorbit"},
                         "T": 1200.0, "T_quick": 60.0, "T_mc": 300.0,
                         "tol": dict(TOL_DRAG),
                         "tiers": ["standard", "full", "demos"]},
    "demo_attitudeGG": {"family": "demo", "kind": "demo",
                        "kw": {"demo": "attitudeGG"},
                        "T": 600.0, "T_quick": 60.0, "T_mc": 60.0,
                        "tol": dict(TOL_GG),
                        "tiers": ["standard", "full", "demos"]},
}

# MC-tier inline smoke subset (the campaign itself is the tier's bulk).
MC_TIER_INLINE = ("hub", "rw3_open_pos", "rw3_pd", "thr_fresh_05")

# Upstream-demo metadata (basilisk/examples @ 2.13.0b0/develop). ``url`` is
# the upstream module page; ``deviations`` lists every bench adaptation vs
# the upstream script (empty = verbatim setup). Horizons: upstream scripts
# run 5-10 min (or days for deorbit); T_demo caps the bench cost, T_quick is
# the smoke leg. Pinned by tests/test_bench_stock_vs_omni.py.
_DEMO_URL = ("https://avslab.github.io/basilisk/develop/_modules/"
             "scenario%s.html")
DEMOS = {
    "basicOrbit_LEO": {
        "scenario": "scenarioBasicOrbit.py",
        "url": _DEMO_URL % "BasicOrbit",
        "deviations": ["horizon capped (bench cost, not the demo's 100-orbit "
                       "style sweep); LEO elements bench-chosen in the "
                       "demo's family"]},
    "basicOrbit_GTO": {
        "scenario": "scenarioBasicOrbit.py",
        "url": _DEMO_URL % "BasicOrbit",
        "deviations": ["GTO elements bench-chosen (demo sweeps orbit cases "
                       "via arguments); horizon capped"]},
    "basicOrbit_GEO": {
        "scenario": "scenarioBasicOrbit.py",
        "url": _DEMO_URL % "BasicOrbit",
        "deviations": ["GEO elements bench-chosen; horizon capped"]},
    "attFeedbackRW": {
        "scenario": "scenarioAttitudeFeedbackRW.py",
        "url": _DEMO_URL % "AttitudeFeedbackRW",
        "deviations": []},
    "momentumDumping": {
        "scenario": "scenarioMomentumDumping.py",
        "url": _DEMO_URL % "MomentumDumping",
        "deviations": ["single task @ 0.1 s (demo: FSW 1 s / dyn 0.1 s; "
                       "dumping cadence rescales, A/B stays matched)",
                       "sun third-body gravity dropped (central-earth "
                       "only; the env-bodies N-body path is an honest "
                       "NO-GO, list.md A1)",
                       "cutoffFrequency=1000 explicit (demo default; "
                       "inert without ramps, Update-M2)",
                       "MOOG_Monarc_5 MaxThrust=5.0 verbatim"]},
    "dragDeorbit": {
        "scenario": "scenarioDragDeorbit.py",
        "url": _DEMO_URL % "DragDeorbit",
        "deviations": ["horizon capped at 1200 s (demo deorbits over "
                       "~100 orbits with a terminal event)",
                       "dt=0.1 (demo: 15 s; drag is switching-free so the "
                       "class is unchanged)",
                       "hub mass 750 kg explicit (demo leaves the stock "
                       "default); useWind=False verbatim"]},
    "attitudeGG": {
        "scenario": "scenarioAttitudeGG.py",
        "url": _DEMO_URL % "AttitudeGG",
        "deviations": ["free GG-perturbed drift (the demo closes the loop "
                       "through extForceTorque: out-of-envelope, covered "
                       "by the extTorque reject cell)",
                       "earth_planet_data planet name per the upstream "
                       "GravityGradientEffector documentation"]},
}

# Coverage disposition for every dynamics-relevant upstream demo. Status:
#   live:<cell>       benched live in this suite (this tier set)
#   covered-by:<what> same dynamics already benched (mechanism/ledger named)
#   reject:<cell>     proves the loud reject (out-of-envelope guard)
#   out:<reason>      not benched, with the reason (no silent gaps: the test
#                     pins that every entry below resolves and that no
#                     dynamics demo is missing).
DEMO_COVERAGE = {
    # Orbital mechanics
    "scenarioBasicOrbit (point-mass LEO/GTO/GEO)":
        "live:demo_basicOrbit_LEO,demo_basicOrbit_GTO,demo_basicOrbit_GEO",
    "scenarioBasicOrbit (J2/spherical-harmonic variants)":
        "out:central-J2 mirror path unverified (env-bodies N-body path is "
        "an honest NO-GO, list.md A1); J2 flag would drop silently, so no "
        "live cell until the mirror pins it",
    "scenarioOrbitManeuver/scenarioHohmann/scenarioPatchedConics/"
    "scenarioFlybySpice/scenarioCentralBody":
        "covered-by:hub coast cells (the DV sequencing itself is "
        "script-level stop/restart + state reset, not an integrator "
        "surface)",
    "scenarioOrbitFromOmm/scenarioSpiceSpacecraft":
        "out:SPICE-ephemeris IC path unaudited for the mirror",
    "scenarioLagrangePointOrbit/scenarioHaloOrbit/scenarioOrbitMultiBody/"
    "scenarioHelioTransSpice/scenarioAsteroidArrival/scenarioAerocapture":
        "out:N-body gravity (env-bodies reader NO-GO, list.md A1)",
    # Attitude regulation / guidance
    "scenarioAttitudeFeedbackRW": "live:demo_attFeedbackRW",
    "scenarioAttitudeFeedback (extForceTorque PD)":
        "covered-by:hijack_scenarioAttitudeFeedback (ext torque now " 
        "mirrorable via the new engine term)",
    "scenarioAttitudeGuidance/scenarioAttitudePointing(+Numba/Py)/"
    "scenarioAttitudeSteering/scenarioAttLocPoint/scenarioAttGuideHyperbolic/"
    "scenario_MRPsteering-course-set":
        "out:PD-via-extForceTorque or guidance-law variants; dynamics "
        "class covered by hub/rw cells, per-demo FSW wiring unaudited",
    "scenarioAttitudeGG (GG torque dynamics)":
        "live:demo_attitudeGG",
    "scenarioAttitudePrescribed/prescribed-motion set":
        "out:prescribed motion (bucket-a loud reject, never modeled)",
    # Actuators
    "scenarioMomentumDumping": "live:demo_momentumDumping",
    "scenarioMtbMomentumManagement(+Simple)/scenarioTAM(+comparison)":
        "covered-by:env_parity_mag (MTB + centered-dipole class, "
        "extended sweep)",
    "scenarioUnbalancedThrusters":
        "covered-by:fuel_offset (unbalanced-thruster torque class)",
    "scenarioReactionWheel": "covered-by:rw1/rw2 cells",
    "scenarioThrArmControl/scenarioThrArmTracking/scenarioThrusterArm/"
    "scenarioArmWithThrusters/scenarioRoboticGrappling":
        "out:branched/articulated effectors (unmodeled topology)",
    # Environments
    "scenarioDragDeorbit": "live:demo_dragDeorbit",
    "scenarioMagneticFieldCenteredDipole (circular Earth)":
        "covered-by:hub (field read adds no dynamics; torque needs an MTB, "
        "which is the mag-parity class)",
    "scenarioMagneticFieldCenteredDipole (elliptical/Jupiter)/"
    "scenarioMagneticFieldWMM":
        "out:multi-region switching / message-only WMM (loud reject)",
    "scenario_AttEclipse (bskSim sun-pointing + eclipse)":
        "covered-by:env_parity_eclipse (extended sweep)",
    "scenarioAlbedo": "out:albedo (no contributor)",
    "scenarioStochasticDrag(+Spacecraft)/scenarioDragSensitivity":
        "out:stochastic density/SDE (Taylor cannot do SDE)",
    # Multi-spacecraft / formation
    "scenarioFormationBasic/scenarioFormationFlyingWithDrag/"
    "scenarioFormationMeanOEFeedback/scenarioFormationReconfig/"
    "scenario_StationKeepingMultiSat/scenario_SpacecraftFormation-set/"
    "scenario_MultiSat-set":
        "covered-by:swap_multi (sync mechanism) + fuel cells (per-object "
        "dynamics); formation control laws are FSW, not integrator",
    "scenarioSimpleDocking/scenarioRendezVous/scenarioSmallBody*-set/"
    "scenarioDebrisReorbitET":
        "out:proximity/UKF/impulsive ops (not an integrator surface)",
    "scenario_AddRWFault/scenario_FaultList/scenario_faultDetOpNav/"
    "bskSim fault scenarios (8/10/11)":
        "out:mid-run fault injection (config mutation the setup-time "
        "mirror cannot foresee)",
    # Monte Carlo
    "scenarioMonteCarloAttRW/scenarioBskSimAttFeedbackMC/"
    "scenarioRerunMonteCarlo/MC example set":
        "covered-by:mc tier (seeded Js/IC dispersion campaign)",
    # FSW-stack variants on identical hub+RW dynamics
    "bskSim 1-7,9 (detumble/hill/velocity/steering/sun-pointing/modes)/"
    "scenario_AttModes/scenario_AttGuidance/scenario_AttFeedback":
        "covered-by:demo_attFeedbackRW (same hub+RW dynamics; FSW-mode "
        "variation only)",
    # Integrator meta-demos and non-dynamics classes (documented, not cells)
    "scenarioIntegrators(+Comparison/+VariableTimeStep)/"
    "scenarioBenchmarkNumba":
        "out:meta-integrator demos (omni IS the comparison subject)",
    "flex/slosh/deploying/rotating/spinning/constraint/mujoco/opnav/"
    "power/thermal/data/sensor/viz/relay sets":
        "out:unmodeled effectors or non-dynamics modules (loud-reject or "
        "no-claim classes, never silent)",
}

# Tier parameters.
# ``hijack``: drive the full upstream-demo hijack registry as subprocesses
# (hijack_demo.py --demo <each runnable entry>). The demos/all tiers run it;
# the full tier keeps its two canonical hijack cells in the extended sweep.
TIERS = {
    "quick": {"mc_n": 4, "extended": False, "ladder": False,
              "hijack": False, "mc_kind": "pd", "inline": "tiers:quick"},
    "standard": {"mc_n": 20, "extended": False, "ladder": False,
                 "hijack": False, "mc_kind": "pd",
                 "inline": "tiers:standard"},
    "full": {"mc_n": 100, "extended": True, "ladder": True,
             "hijack": False, "mc_kind": "pd", "inline": "tiers:full"},
    "mc": {"mc_n": 1000, "extended": False, "ladder": False,
           "hijack": False, "mc_kind": "pd", "inline": "mc_smoke"},
    "demos": {"mc_n": 0, "extended": False, "ladder": False,
              "hijack": True, "mc_kind": "pd", "inline": "tiers:demos"},
    "all": {"mc_n": 100, "extended": True, "ladder": True,
            "hijack": True, "mc_kind": "pd", "inline": "tiers:all"},
}

# Extended sweep (full tier): sibling probe -> ledger -> expectation note.
# The owning probe's own verdict is authoritative; the bench only aggregates
# exit code + ledger status. envbodies is the honest NO-GO: pass = the probe
# ran and recorded its ledger (mechanisms pinned by test_env_parity_envbodies).
EXTENDED = [
    ("swap_p1_parity.py", "swap_p1_parity", "parity green"),
    ("fsw_loop.py", "fsw_loop", "parity green"),
    ("swap_multi.py", "swap_multi", "parity green"),
    ("swap_speed.py", "swap_speed", None),
    ("swap_speed.py --legs", "swap_speed_legs", None),
    ("loop_split.py", "loop_split", None),
    ("env_parity_drag.py", "env_parity_drag", "parity green"),
    ("env_parity_srp.py", "env_parity_srp", "parity green"),
    ("env_parity_mag.py", "env_parity_mag", "parity green"),
    ("env_parity_eclipse.py", "env_parity_eclipse", "parity green"),
    ("env_parity_envbodies.py", "env_parity_envbodies", "NO-GO-recorded"),
    ("hijack_demo.py --demo scenarioAttitudeFeedbackRW",
     "hijack_scenarioAttitudeFeedbackRW", "parity green"),
    ("hijack_demo.py --demo scenarioBasicOrbit",
     "hijack_scenarioBasicOrbit", "parity green"),
]

RW_KINDS = ("rw1", "rw2", "rw3_open", "rw3_pd", "rw3_fric",
            "rw1tilt", "rw_diag3")
RW_AXES = {
    "rw1": [[1.0, 0.0, 0.0]],
    "rw2": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    "rw3_open": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    "rw3_pd": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    "rw3_fric": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    "rw1tilt": [[DIAG, DIAG, DIAG]],
    "rw_diag3": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [DIAG, DIAG, DIAG]],
}


# ---------------------------------------------------------------------------
# Builders (swap_p1_parity.py idioms, dt-parameterized)
# ---------------------------------------------------------------------------
def _base_sim(tag, dt, mass=MASS, sigma0=None, omega0=None,
              r_cn=None, v_cn=None, I9=None, oe=None):
    from Basilisk.simulation import spacecraft
    from Basilisk.utilities import (SimulationBaseClass, macros,
                                    orbitalMotion, simIncludeGravBody)
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(float(dt))))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = tag
    sc.hub.mHub = float(mass)
    I = I6 if I9 is None else [list(r) for r in I9]
    sc.hub.IHubPntBc_B = [list(r) for r in I]
    if r_cn is None or v_cn is None:
        if oe is None:
            oe = orbitalMotion.ClassicElements()
            oe.a, oe.e = 1.0e7, 0.01
            oe.i = 33.3 * macros.D2R
            oe.Omega, oe.omega, oe.f = (48.2 * macros.D2R,
                                        347.8 * macros.D2R, 85.3 * macros.D2R)
        else:
            a, e, i_d, O_d, w_d, f_d = oe
            oe = orbitalMotion.ClassicElements()
            oe.a, oe.e = float(a), float(e)
            oe.i = float(i_d) * macros.D2R
            oe.Omega, oe.omega, oe.f = (float(O_d) * macros.D2R,
                                        float(w_d) * macros.D2R,
                                        float(f_d) * macros.D2R)
        r_cn, v_cn = orbitalMotion.elem2rv(MU, oe)
    sc.hub.r_CN_NInit, sc.hub.v_CN_NInit = list(r_cn), list(v_cn)
    sc.hub.sigma_BNInit = list(SIG0 if sigma0 is None else sigma0)
    sc.hub.omega_BN_BInit = list(OM0 if omega0 is None else omega0)
    sim.AddModelToTask("t", sc)
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    grav.addBodiesTo(sc)
    return sim, sc


def _add_rw(sim, sc, axes, om_list=None, friction=False, task=True,
            js_list=None, max_momentum=None):
    from Basilisk.simulation import reactionWheelStateEffector
    from Basilisk.utilities import simIncludeRW
    fac = simIncludeRW.rwFactory()
    oms = list(RPM0[:len(axes)] if om_list is None else om_list)
    for i, gs in enumerate(axes):
        om = float(oms[i % len(oms)])
        if js_list is not None:
            # Dispersed inertia: stock "custom" wheel (wp7 MC idiom). The
            # omni mirror reads Js back from the stock record, so the pair
            # disperses together. Friction+custom is unsupported (stock has
            # no such config); callers never combine them.
            js = float(js_list[i % len(js_list)])
            fac.create("custom", [float(x) for x in gs], Js=js,
                       u_max=U_MAX, Omega=om)
        elif friction:
            fac.create("Honeywell_HR16", [float(x) for x in gs],
                       maxMomentum=50.0, Omega=om, useRWfriction=True)
        else:
            # maxMomentum sets the rotor inertia (Js = maxMomentum / rated
            # speed): the demo's 100 Nms wheels carry 2x the standard HR16
            # Js, which is what pushes |H| over the hs_min=80 dump threshold.
            fac.create("Honeywell_HR16", [float(x) for x in gs],
                       maxMomentum=(50.0 if max_momentum is None
                                    else float(max_momentum)),
                       Omega=om)
    rw = reactionWheelStateEffector.ReactionWheelStateEffector()
    fac.addToSpacecraft("rw", rw, sc)
    if task:
        sim.AddModelToTask("t", rw)
    return rw, fac


def _const_driver(u):
    from Basilisk.architecture import messaging, sysModel

    class Driver(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.ArrayMotorTorqueMsg()

        def Reset(self, t):
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(
                motorTorque=[0.0] * 36))

        def UpdateState(self, t):
            m = [0.0] * 36
            for i in range(3):
                m[i] = float(u)
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(
                motorTorque=m), t, self.moduleID)
    return Driver()


def _pd_driver(K=MC_K, P=MC_P, u_max=U_MAX):
    import numpy as _np
    from Basilisk.architecture import messaging, sysModel

    class PD(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.ArrayMotorTorqueMsg()
            self.sc_in = messaging.SCStatesMsgReader()

        def Reset(self, t):
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(
                motorTorque=[0.0] * 36))

        def UpdateState(self, t):
            s = _np.asarray(self.sc_in().sigma_BN, float)
            w = _np.asarray(self.sc_in().omega_BN_B, float)
            u = _np.clip(-K * s - P * w, -u_max, u_max)
            m = [0.0] * 36
            for i in range(min(3, len(u))):
                m[i] = float(u[i])
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(
                motorTorque=m), t, self.moduleID)
    return PD()


def _fric_driver():
    from Basilisk.architecture import messaging, sysModel
    from Basilisk.utilities import macros

    class Fric(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.ArrayMotorTorqueMsg()

        def Reset(self, t):
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(
                motorTorque=[0.0] * 36))

        def UpdateState(self, t):
            u = 0.2 if t < macros.sec2nano(40.0) else 0.0
            m = [0.0] * 36
            for i in range(3):
                m[i] = u
            self.out.write(messaging.ArrayMotorTorqueMsgPayload(
                motorTorque=m), t, self.moduleID)
    return Fric()


def _thr_driver(duty, dt):
    from Basilisk.architecture import messaging, sysModel

    class Thr(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.THRArrayOnTimeCmdMsg()

        def Reset(self, t):
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            p.OnTimeRequest = [0.0] * 36
            self.out.write(p)

        def UpdateState(self, t):
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            ot = [0.0] * 36
            ot[0] = float(duty) * float(dt)
            p.OnTimeRequest = ot
            self.out.write(p, t, self.moduleID)
    return Thr()


def _thr_frozen_driver(duty, dt):
    from Basilisk.architecture import messaging, sysModel

    class Frozen(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.out = messaging.THRArrayOnTimeCmdMsg()
            self.tick = -1

        def Reset(self, t):
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            p.OnTimeRequest = [0.0] * 36
            self.out.write(p)

        def UpdateState(self, t):
            self.tick += 1
            if self.tick > 1:
                return
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            ot = [0.0] * 36
            ot[0] = float(duty) * float(dt)
            p.OnTimeRequest = ot
            self.out.write(p, t, self.moduleID)
    return Frozen()


def _add_thruster(sim, sc, loc, direction, max_thrust, cutoff=1000.0):
    tf, thr = _add_thruster_set(sim, sc, [loc], [direction],
                                max_thrust, cutoff)
    return tf, thr


def _add_thruster_set(sim, sc, locs, directions, max_thrust, cutoff=1000.0,
                      thr_type="TEST_Thruster"):
    """One shared factory + one effector for N thrusters (index-joined)."""
    from Basilisk.simulation import thrusterDynamicEffector
    from Basilisk.utilities import simIncludeThruster
    tf = simIncludeThruster.thrusterFactory()
    for loc, direction in zip(locs, directions):
        tf.create(thr_type, [float(x) for x in loc],
                  [float(x) for x in direction], MaxThrust=float(max_thrust),
                  cutoffFrequency=float(cutoff), useMinPulseTime=False)
    thr = thrusterDynamicEffector.ThrusterDynamicEffector()
    tf.addToSpacecraft("thr", thr, sc)
    sim.AddModelToTask("t", thr, 2)
    return tf, thr


def _add_fuel_tank(sim, sc, thr, r_tb, prop_mass=PROP0):
    from Basilisk.simulation import fuelTank
    tank = fuelTank.FuelTank()
    model = fuelTank.FuelTankModelConstantVolume()
    model.propMassInit = float(prop_mass)
    tank.setTankModel(model)
    tank.setNameOfMassState("fuelMass")
    tank.setR_TB_B([float(x) for x in r_tb])
    tank.addThrusterSet(thr)
    sc.addStateEffector(tank)
    sim.AddModelToTask("t", tank)
    return tank


def _add_drag_stock(sim, sc, proj_area=5.0, cd=2.2, com=(0.05, 0.0, 0.0),
                    preset=None):
    from Basilisk.simulation import (dragDynamicEffector,
                                     exponentialAtmosphere)
    atmo = exponentialAtmosphere.ExponentialAtmosphere()
    if preset == "earth":
        # Demo-exact Earth atmosphere (scenarioDragDeorbit idiom).
        from Basilisk.utilities import simSetPlanetEnvironment
        simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")
    else:
        atmo.baseDensity = 1e-9
        atmo.scaleHeight = 50000.0
        atmo.planetRadius = 6378137.0
    atmo.addSpacecraftToModel(sc.scStateOutMsg)
    sim.AddModelToTask("t", atmo)
    drag = dragDynamicEffector.DragDynamicEffector()
    drag.coreParams.projectedArea = float(proj_area)
    drag.coreParams.dragCoeff = float(cd)
    drag.coreParams.comOffset = [float(x) for x in com]
    drag.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[-1])
    drag.cannonballDrag()
    sc.addDynamicEffector(drag)
    sim.AddModelToTask("t", drag)
    return atmo


def _add_srp_stock(sim, sc, sun_vec):
    from Basilisk.architecture import messaging
    from Basilisk.simulation import radiationPressure
    sm = messaging.SpicePlanetStateMsg()
    p = messaging.SpicePlanetStateMsgPayload()
    p.PositionVector = [float(x) for x in sun_vec]
    p.VelocityVector = [0.0, 0.0, 0.0]
    sm.write(p)
    srp = radiationPressure.RadiationPressure()
    srp.area = 10.0
    srp.coefficientReflection = 1.5
    srp.setUseCannonballModel()
    srp.sunEphmInMsg.subscribeTo(sm)
    sc.addDynamicEffector(srp)
    sim.AddModelToTask("t", srp)
    return sm


def _arm(use_omni, kind, dt, **kw):
    """Build one A/B arm. Returns (sim, sc, scLog, spdLog, oi, aux)."""
    if kind == "demo":
        return _arm_demo(use_omni, dt, kw.get("demo"), **kw)
    from omniheyoka.omni_heyoka import OmniVecIntegrator
    from omniheyoka.swap import mirror_stock_config
    mass = float(kw.get("mass", MASS))
    if kind in ("fuel", "fuel_drag", "fuel_srp", "fsw"):
        mass = float(kw.get("mass", MASS_DRY_FUEL))
    sim, sc = _base_sim(("omni-" if use_omni else "rk-") + kind, dt,
                        mass=mass, sigma0=kw.get("sigma0"),
                        omega0=kw.get("omega0"), I9=kw.get("I9"),
                        oe=kw.get("oe"))
    if kw.get("rbcb") is not None:
        sc.hub.r_BcB_B = [float(x) for x in kw["rbcb"]]
    elif kw.get("offset") is not None:
        sc.hub.r_BcB_B = [float(kw["offset"]), 0.0, 0.0]
    axes = kw.get("axes_override")
    rw = None
    if kind in RW_KINDS:
        if axes is None:
            axes = RW_AXES[kind]
        rw, _fac = _add_rw(sim, sc, axes, om_list=kw.get("rpm0"),
                           friction=(kind == "rw3_fric"),
                           js_list=kw.get("js"))
    if kind in ("rw3_open", "rw_diag3"):
        drv = _const_driver(kw.get("u", 0.5))
        sim.AddModelToTask("t", drv)
        rw.rwMotorCmdInMsg.subscribeTo(drv.out)
    if kind == "rw3_fric":
        drv = _fric_driver()
        sim.AddModelToTask("t", drv)
        rw.rwMotorCmdInMsg.subscribeTo(drv.out)
    if kind == "rw3_pd":
        drv = _pd_driver()
        sim.AddModelToTask("t", drv)
        rw.rwMotorCmdInMsg.subscribeTo(drv.out)
        drv.sc_in.subscribeTo(sc.scStateOutMsg)
    tf = None
    cmd = None
    atmo = None
    sun_msg = None
    if kind in ("thr", "fuel", "fuel_drag", "fuel_srp", "fsw"):
        if kind == "fsw":
            loc, direction, thrust = [0.0, 0.0, 1.0], [0.0, 0.0, -1.0], 5.0
        elif kind in ("fuel", "fuel_drag", "fuel_srp"):
            loc, direction, thrust = [0.5, 0.0, 1.0], [0.0, 0.0, -1.0], 5.0
        else:
            loc, direction, thrust = [0.5, 0.0, 1.0], [0.0, 0.0, -1.0], 1.0
        tf, thr = _add_thruster(sim, sc, loc, direction, thrust,
                               cutoff=kw.get("cutoff", 1000.0))
        mode = kw.get("mode", "fresh")
        if kind == "fsw":
            burn = _thr_driver(0.5, dt)
            sim.AddModelToTask("t", burn)
            thr.cmdsInMsg.subscribeTo(burn.out)
            cmd = burn.out
            pd = _pd_driver()
            sim.AddModelToTask("t", pd)
            if rw is None:
                rw, _fac = _add_rw(sim, sc, RW_AXES["rw3_pd"],
                                   om_list=kw.get("rpm0"))
            rw.rwMotorCmdInMsg.subscribeTo(pd.out)
            pd.sc_in.subscribeTo(sc.scStateOutMsg)
        elif mode == "fresh":
            drv = _thr_driver(kw.get("duty", 0.5), dt)
            sim.AddModelToTask("t", drv)
            thr.cmdsInMsg.subscribeTo(drv.out)
            cmd = drv.out
        elif mode == "frozen":
            drv = _thr_frozen_driver(kw.get("duty", 0.5), dt)
            sim.AddModelToTask("t", drv)
            thr.cmdsInMsg.subscribeTo(drv.out)
            cmd = drv.out
        else:  # write-once: pre-written command, no driver
            from Basilisk.architecture import messaging
            m = messaging.THRArrayOnTimeCmdMsg()
            p = messaging.THRArrayOnTimeCmdMsgPayload()
            ot = [0.0] * 36
            ot[0] = float(kw.get("duty", 0.5)) * float(dt)
            p.OnTimeRequest = ot
            m.write(p)
            thr.cmdsInMsg.subscribeTo(m)
            cmd = m
        if kind in ("fuel", "fuel_drag", "fuel_srp", "fsw"):
            # fsw carries the fsw_loop fixture's centered tank (c_B == 0).
            _add_fuel_tank(sim, sc, thr, kw.get("r_tb", [0.0, 0.0, 0.0]))
    if kind in ("drag", "fuel_drag"):
        import numpy as _np
        alt_km = float(kw.get("alt_km", 350.0))
        R = 6378137.0 + alt_km * 1e3
        V = float(_np.sqrt(MU / R))
        sc.hub.r_CN_NInit = [R, 0.0, 0.0]
        sc.hub.v_CN_NInit = [0.0, V, 0.0]
        if kw.get("still"):
            sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
            sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
        atmo = None
        preset = kw.get("atmo_preset")
        area = float(kw.get("drag_area", 5.0))
        cd = float(kw.get("drag_cd", 2.2))
        com = tuple(kw.get("drag_com", (0.05, 0.0, 0.0)))
        if not use_omni:
            atmo = _add_drag_stock(sim, sc, proj_area=area, cd=cd,
                                   com=com, preset=preset)
        else:
            from Basilisk.simulation import exponentialAtmosphere
            atmo = exponentialAtmosphere.ExponentialAtmosphere()
            if preset == "earth":
                from Basilisk.utilities import simSetPlanetEnvironment
                simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")
            else:
                atmo.baseDensity = 1e-9
                atmo.scaleHeight = 50000.0
                atmo.planetRadius = 6378137.0
            atmo.addSpacecraftToModel(sc.scStateOutMsg)
            sim.AddModelToTask("t", atmo)
        aux_drag = (area * cd, com)
    if kind in ("srp", "fuel_srp"):
        import numpy as _np
        R = 6378137.0 + 400e3
        V = float(_np.sqrt(MU / R))
        sun = [1.5e11, 0.0, 0.0]
        sc.hub.r_CN_NInit = [R, 0.0, 0.0]
        sc.hub.v_CN_NInit = [0.0, V, 0.0]
        if kind == "srp":
            # Stabilized ICs (swap_p1 _env_srp_cell: the tumbling
            # frozen-projection class is covered by the env_drag pair).
            # fuel_srp keeps the tumbling ICs of the validated combined
            # cell (swap_p1 _env_fuel_srp_cell).
            sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
            sc.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
        if not use_omni:
            sun_msg = _add_srp_stock(sim, sc, sun)
        else:
            from Basilisk.architecture import messaging
            sun_msg = messaging.SpicePlanetStateMsg()
            p = messaging.SpicePlanetStateMsgPayload()
            p.PositionVector = [float(x) for x in sun]
            p.VelocityVector = [0.0, 0.0, 0.0]
            sun_msg.write(p)
    oi, scLog, spdLog, aux = _finish_arm(
        sim, sc, rw, tf, cmd, atmo, sun_msg, use_omni,
        need_factory=kind in ("thr", "fuel", "fuel_drag", "fuel_srp", "fsw"),
        need_drag=kind in ("drag", "fuel_drag"),
        drag_params=locals().get("aux_drag"),
        need_srp=kind in ("srp", "fuel_srp"))
    return sim, sc, scLog, spdLog, oi, aux


def _finish_arm(sim, sc, rw, tf, cmd, atmo, sun_msg, use_omni,
                need_factory=False, need_drag=False, drag_params=None,
                need_srp=False, aux_extra=None):
    """Shared stock/omni arm tail: mirror, integrate, record.

    Every builder (inline kinds and upstream-demo replicas) closes through
    here so the A/B comparison itself is never a per-surface variation.
    ``drag_params`` is ``(cd_area, com)`` matching the stock drag build.
    """
    from omniheyoka.omni_heyoka import OmniVecIntegrator
    from omniheyoka.swap import mirror_stock_config
    oi = None
    if use_omni:
        oi = OmniVecIntegrator(sc)
        if need_factory:
            mirror_stock_config(sim, sc, oi, thr_factory=tf, thr_cmd_msg=cmd)
        else:
            mirror_stock_config(sim, sc, oi)
        if need_drag:
            cd_area, com = drag_params
            oi.setDragParams(float(cd_area), float(com[0]),
                             float(com[1]), float(com[2]))
            oi.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[-1])
        if need_srp:
            oi.setSRPCannonball(10.0, 0.5)
            oi.sunStateInMsg.subscribeTo(sun_msg)
        sc.setIntegrator(oi)
    scLog = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", scLog)
    spdLog = rw.rwSpeedOutMsg.recorder() if rw is not None else None
    if spdLog is not None:
        sim.AddModelToTask("t", spdLog)
    aux = dict(aux_extra or {})
    if atmo is not None and need_drag:
        dlog = atmo.envOutMsgs[-1].recorder()
        sim.AddModelToTask("t", dlog)
        aux["dlog"] = dlog
    return oi, scLog, spdLog, aux


# ---------------------------------------------------------------------------
# Upstream-demo replicas (basilisk/examples, 2.13.0b0/develop).
# Each builder copies the upstream setup verbatim unless its DEMO_INFO lists
# a documented deviation. The A/B close-out always goes through _finish_arm.
# ---------------------------------------------------------------------------
def _fsw_rw_exact(sim, sc, rw, fac, I9, K=3.5, Ki=-1.0, P=30.0):
    """Demo-exact RW FSW chain (scenarioAttitudeFeedbackRW idiom).

    Stock FSW modules (inertial3D + attTrackingError + mrpFeedback +
    rwMotorTorque) with the demo's gains and messages. Both arms share the
    identical chain, so the cell benches the upstream closed loop itself,
    not a simplified PD stand-in.
    """
    from Basilisk.architecture import messaging
    from Basilisk.fswAlgorithms import (attTrackingError, inertial3D,
                                        mrpFeedback, rwMotorTorque)
    from Basilisk.simulation import simpleNav
    sNav = simpleNav.SimpleNav()
    sNav.ModelTag = "SimpleNavigation"
    sim.AddModelToTask("t", sNav)
    guid = inertial3D.inertial3D()
    guid.ModelTag = "inertial3D"
    sim.AddModelToTask("t", guid)
    guid.sigma_R0N = [0.0, 0.0, 0.0]
    err = attTrackingError.attTrackingError()
    err.ModelTag = "attErrorInertial3D"
    sim.AddModelToTask("t", err)
    ctl = mrpFeedback.mrpFeedback()
    ctl.ModelTag = "mrpFeedback"
    sim.AddModelToTask("t", ctl)
    ctl.K = float(K)
    ctl.Ki = float(Ki)
    ctl.P = float(P)
    ctl.integralLimit = 2.0 / ctl.Ki * 0.1
    alloc = rwMotorTorque.rwMotorTorque()
    alloc.ModelTag = "rwMotorTorque"
    sim.AddModelToTask("t", alloc)
    alloc.controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    # Flat 9-vector (upstream verbatim: I = [900., 0., 0., ...]). A nested
    # 3x3 trips the SWIG message typemap ("Could not convert that double
    # into a pointer").
    Iflat = [float(v) for r in I9 for v in r]
    vcMsg = messaging.VehicleConfigMsg().write(
        messaging.VehicleConfigMsgPayload(ISCPntB_B=Iflat))
    fswRwParamMsg = fac.getConfigMessage()
    sNav.scStateInMsg.subscribeTo(sc.scStateOutMsg)
    err.attNavInMsg.subscribeTo(sNav.attOutMsg)
    err.attRefInMsg.subscribeTo(guid.attRefOutMsg)
    ctl.guidInMsg.subscribeTo(err.attGuidOutMsg)
    ctl.vehConfigInMsg.subscribeTo(vcMsg)
    ctl.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    ctl.rwSpeedsInMsg.subscribeTo(rw.rwSpeedOutMsg)
    alloc.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    alloc.vehControlInMsg.subscribeTo(ctl.cmdTorqueOutMsg)
    rw.rwMotorCmdInMsg.subscribeTo(alloc.rwMotorTorqueOutMsg)


def _fsw_momentum_dump(sim, sc, rw, fac, thr, thFactory, I9):
    """Demo-exact momentum-dumping FSW chain (scenarioMomentumDumping idiom).

    inertial3D + attTrackingError + mrpFeedback (P/K from the demo's
    decayTime/xi law) + rwMotorTorque + thrMomentumManagement (hs_min=80) +
    thrForceMapping (angErrThresh=3.15) + thrMomentumDumping
    (maxCounterValue=100, thrMinFireTime=0.02). Returns the reset closure and
    the on-time command message for the staged mailbox.
    """
    import numpy as _np
    from Basilisk.architecture import messaging
    from Basilisk.fswAlgorithms import (attTrackingError, inertial3D,
                                        mrpFeedback, rwMotorTorque,
                                        thrForceMapping,
                                        thrMomentumDumping,
                                        thrMomentumManagement)
    from Basilisk.simulation import simpleNav
    from Basilisk.utilities import macros
    sNav = simpleNav.SimpleNav()
    sNav.ModelTag = "SimpleNavigation"
    sim.AddModelToTask("t", sNav)
    guid = inertial3D.inertial3D()
    guid.ModelTag = "inertial3D"
    sim.AddModelToTask("t", guid)
    guid.sigma_R0N = [0.0, 0.0, 0.0]
    err = attTrackingError.attTrackingError()
    err.ModelTag = "attErrorInertial3D"
    sim.AddModelToTask("t", err)
    ctl = mrpFeedback.mrpFeedback()
    ctl.ModelTag = "mrpFeedback"
    sim.AddModelToTask("t", ctl)
    flat_I = [float(v) for r in I9 for v in r]
    ctl.Ki = -1.0
    ctl.P = 3.0 * float(_np.max(flat_I)) / 10.0
    ctl.K = (ctl.P / 1.0) * (ctl.P / 1.0) / float(_np.max(flat_I))
    ctl.integralLimit = 2.0 / ctl.Ki * 0.1
    alloc = rwMotorTorque.rwMotorTorque()
    alloc.ModelTag = "rwMotorTorque"
    sim.AddModelToTask("t", alloc)
    alloc.controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    desat = thrMomentumManagement.thrMomentumManagement()
    desat.ModelTag = "thrMomentumManagement"
    sim.AddModelToTask("t", desat)
    desat.hs_min = 80
    fmap = thrForceMapping.thrForceMapping()
    fmap.ModelTag = "thrForceMapping"
    sim.AddModelToTask("t", fmap)
    fmap.controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    fmap.thrForceSign = 1
    fmap.angErrThresh = 3.15
    dump = thrMomentumDumping.thrMomentumDumping()
    dump.ModelTag = "thrDump"
    sim.AddModelToTask("t", dump)
    dump.maxCounterValue = 100
    dump.thrMinFireTime = 0.02
    Iflat = [float(v) for r in I9 for v in r]
    vcMsg = messaging.VehicleConfigMsg().write(
        messaging.VehicleConfigMsgPayload(ISCPntB_B=Iflat))
    fswRwParamMsg = fac.getConfigMessage()
    fswThrConfigMsg = thFactory.getConfigMessage()
    sNav.scStateInMsg.subscribeTo(sc.scStateOutMsg)
    err.attNavInMsg.subscribeTo(sNav.attOutMsg)
    err.attRefInMsg.subscribeTo(guid.attRefOutMsg)
    ctl.guidInMsg.subscribeTo(err.attGuidOutMsg)
    ctl.vehConfigInMsg.subscribeTo(vcMsg)
    ctl.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    ctl.rwSpeedsInMsg.subscribeTo(rw.rwSpeedOutMsg)
    alloc.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    alloc.vehControlInMsg.subscribeTo(ctl.cmdTorqueOutMsg)
    rw.rwMotorCmdInMsg.subscribeTo(alloc.rwMotorTorqueOutMsg)
    desat.rwSpeedsInMsg.subscribeTo(rw.rwSpeedOutMsg)
    desat.rwConfigDataInMsg.subscribeTo(fswRwParamMsg)
    fmap.cmdTorqueInMsg.subscribeTo(desat.deltaHOutMsg)
    fmap.thrConfigInMsg.subscribeTo(fswThrConfigMsg)
    fmap.vehConfigInMsg.subscribeTo(vcMsg)
    dump.thrusterConfInMsg.subscribeTo(fswThrConfigMsg)
    dump.deltaHInMsg.subscribeTo(desat.deltaHOutMsg)
    dump.thrusterImpulseInMsg.subscribeTo(fmap.thrForceCmdOutMsg)
    thr.cmdsInMsg.subscribeTo(dump.thrusterOnTimeOutMsg)
    onLog = dump.thrusterOnTimeOutMsg.recorder()
    sim.AddModelToTask("t", onLog)
    return {"reset_fn": lambda: desat.Reset(macros.sec2nano(10.0)),
            "cmd": dump.thrusterOnTimeOutMsg, "dump_log": onLog}


def _arm_demo(use_omni, dt, name, **kw):
    """Upstream-demo arm. Returns (sim, sc, scLog, spdLog, oi, aux).

    Every setup value accepts a ``kw`` override (default = demo-exact);
    the MC campaign drives these for dispersed-setup pairs.
    """
    _I9_STD = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    _ZERO3 = [0.0, 0.0, 0.0]
    if name == "basicOrbit_LEO":
        sim, sc = _base_sim(("omni-" if use_omni else "rk-") + name, dt,
                            mass=float(kw.get("mass", 750.0)),
                            I9=kw.get("I9"),
                            oe=kw.get("oe", (6778e3, 0.0001, 33.3,
                                             48.2, 347.8, 85.3)),
                            sigma0=kw.get("sigma0", list(_ZERO3)),
                            omega0=kw.get("omega0", list(_ZERO3)))
        oi, scLog, spdLog, aux = _finish_arm(sim, sc, None, None, None,
                                            None, None, use_omni)
        return sim, sc, scLog, spdLog, oi, aux
    if name == "basicOrbit_GTO":
        sim, sc = _base_sim(("omni-" if use_omni else "rk-") + name, dt,
                            mass=float(kw.get("mass", 750.0)),
                            I9=kw.get("I9"),
                            oe=kw.get("oe", (24396e3, 0.7283, 33.3,
                                             48.2, 347.8, 0.0)),
                            sigma0=kw.get("sigma0", list(_ZERO3)),
                            omega0=kw.get("omega0", list(_ZERO3)))
        oi, scLog, spdLog, aux = _finish_arm(sim, sc, None, None, None,
                                            None, None, use_omni)
        return sim, sc, scLog, spdLog, oi, aux
    if name == "basicOrbit_GEO":
        sim, sc = _base_sim(("omni-" if use_omni else "rk-") + name, dt,
                            mass=float(kw.get("mass", 750.0)),
                            I9=kw.get("I9"),
                            oe=kw.get("oe", (42164e3, 0.0, 0.0,
                                             0.0, 0.0, 0.0)),
                            sigma0=kw.get("sigma0", list(_ZERO3)),
                            omega0=kw.get("omega0", list(_ZERO3)))
        oi, scLog, spdLog, aux = _finish_arm(sim, sc, None, None, None,
                                            None, None, use_omni)
        return sim, sc, scLog, spdLog, oi, aux
    if name == "attFeedbackRW":
        I9 = kw.get("I9", [list(r) for r in _I9_STD])
        sim, sc = _base_sim(("omni-" if use_omni else "rk-") + name, dt,
                            mass=float(kw.get("mass", 750.0)), I9=I9,
                            sigma0=kw.get("sigma0", [0.1, 0.2, -0.3]),
                            omega0=kw.get("omega0", [0.001, -0.01, 0.03]))
        rw, fac = _add_rw(sim, sc,
                          [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0],
                           [0.0, 0.0, 1.0]],
                          om_list=kw.get("rpm0", [100.0, 200.0, 300.0]))
        _fsw_rw_exact(sim, sc, rw, fac, I9)
        oi, scLog, spdLog, aux = _finish_arm(sim, sc, rw, None, None,
                                            None, None, use_omni)
        return sim, sc, scLog, spdLog, oi, aux
    if name == "momentumDumping":
        c = 2.0 ** -0.5
        I9 = kw.get("I9", [[1700.0, 0.0, 0.0], [0.0, 1700.0, 0.0],
                           [0.0, 0.0, 1800.0]])
        sim, sc = _base_sim(("omni-" if use_omni else "rk-") + name, dt,
                            mass=float(kw.get("mass", 2500.0)), I9=I9,
                            oe=kw.get("oe", (7000e3, 0.0001, 33.3,
                                             148.2, 347.8, 335.0)),
                            sigma0=kw.get("sigma0", list(_ZERO3)),
                            omega0=kw.get("omega0", list(_ZERO3)))
        sc.hub.r_BcB_B = [float(x) for x in kw.get("rbcb", [0.0, 0.0, 1.28])]
        rw, fac = _add_rw(
            sim, sc, [[c, 0.0, c], [0.0, c, c], [-c, 0.0, c],
                      [0.0, -c, c]],
            om_list=kw.get("rpm0", [4000.0, 2000.0, 3500.0, 0.0]),
            max_momentum=100.0)
        locs = [[-1, -1, 1.28], [1, -1, -1.28], [1, -1, 1.28],
                [1, 1, -1.28], [1, 1, 1.28], [-1, 1, -1.28],
                [-1, 1, 1.28], [-1, -1, -1.28]]
        dirs = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0],
                [-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0]]
        tf, thr = _add_thruster_set(sim, sc, locs, dirs, 5.0,
                                   cutoff=1000.0, thr_type="MOOG_Monarc_5")
        fsw = _fsw_momentum_dump(sim, sc, rw, fac, thr, tf, I9)
        oi, scLog, spdLog, aux = _finish_arm(
            sim, sc, rw, tf, fsw["cmd"], None, None, use_omni,
            need_factory=True, aux_extra={"reset_fn": fsw["reset_fn"],
                                          "dump_log": fsw["dump_log"]})
        return sim, sc, scLog, spdLog, oi, aux
    if name == "dragDeorbit":
        I9 = kw.get("I9")
        sim, sc = _base_sim(("omni-" if use_omni else "rk-") + name, dt,
                            mass=float(kw.get("mass", 750.0)), I9=I9,
                            sigma0=kw.get("sigma0", [0.0, 0.0, 0.0]),
                            omega0=kw.get("omega0", [0.0, 0.0, 0.0]))
        import numpy as _np
        R = 6378137.0 + float(kw.get("alt_km", 250.0)) * 1e3
        V = float(_np.sqrt(MU / R))
        sc.hub.r_CN_NInit = [R, 0.0, 0.0]
        sc.hub.v_CN_NInit = [0.0, V, 0.0]
        atmo = None
        if not use_omni:
            atmo = _add_drag_stock(sim, sc, proj_area=10.0, cd=2.2,
                                   com=(0.0, 0.0, 0.0), preset="earth")
        else:
            from Basilisk.simulation import exponentialAtmosphere
            from Basilisk.utilities import simSetPlanetEnvironment
            atmo = exponentialAtmosphere.ExponentialAtmosphere()
            simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")
            atmo.addSpacecraftToModel(sc.scStateOutMsg)
            sim.AddModelToTask("t", atmo)
        oi, scLog, spdLog, aux = _finish_arm(
            sim, sc, None, None, None, atmo, None, use_omni,
            need_drag=True, drag_params=(10.0 * 2.2, (0.0, 0.0, 0.0)))
        return sim, sc, scLog, spdLog, oi, aux
    if name == "attitudeGG":
        from Basilisk.simulation import GravityGradientEffector
        I9 = kw.get("I9", [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0],
                           [0.0, 0.0, 600.0]])
        sim, sc = _base_sim(("omni-" if use_omni else "rk-") + name, dt,
                            mass=float(kw.get("mass", 750.0)), I9=I9,
                            oe=kw.get("oe", (1.0e7, 0.1, 33.3,
                                             48.2, 347.8, 85.3)),
                            sigma0=kw.get("sigma0", [0.1, 0.2, -0.3]),
                            omega0=kw.get("omega0", [0.001, -0.01, 0.03]))
        gg = GravityGradientEffector.GravityGradientEffector()
        gg.ModelTag = sc.ModelTag
        # earth_planet_data: stock planetName for gravFactory.createEarth()
        # (Basilisk GravityGradientEffector documentation).
        gg.addPlanetName("earth_planet_data")
        sc.addDynamicEffector(gg)
        sim.AddModelToTask("t", gg)
        oi, scLog, spdLog, aux = _finish_arm(sim, sc, None, None, None,
                                            None, None, use_omni)
        return sim, sc, scLog, spdLog, oi, aux
    raise ValueError("unknown demo: %s" % name)


def _run(sim, T):
    from Basilisk.utilities import macros
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(float(T)))
    sim.ExecuteSimulation()


def _run_two_phase(sim, T, reset_at, reset_fn):
    """Two-leg run with a mid-run module Reset (momentum-dumping idiom).

    Mirrors scenarioMomentumDumping: execute to reset_at, Reset the FSW
    staging module (no momentum dump at t=0), execute the remainder.
    InitializeSimulation runs once -- this is one continuous sim.
    """
    from Basilisk.utilities import macros
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(float(reset_at)))
    sim.ExecuteSimulation()
    reset_fn()
    sim.ConfigureStopTime(macros.sec2nano(float(T)))
    sim.ExecuteSimulation()


def _dcm(sigma):
    import numpy as _np

    def skew(c):
        return _np.array([[0.0, -c[2], c[1]], [c[2], 0.0, -c[0]],
                          [-c[1], c[0], 0.0]])
    s = _np.asarray(sigma, float)
    s2 = float(s @ s)
    S = skew(s)
    return _np.eye(3) + (8.0 * S @ S - 4.0 * (1.0 - s2) * S) / (1.0 + s2) ** 2


def _att_deg(s1, s2):
    import numpy as _np
    a = _np.asarray(s1, float).reshape(3)
    b = _np.asarray(s2, float).reshape(3)
    n1, n2 = float(a @ a), float(b @ b)
    num = (1 - n2) * a - (1 - n1) * b - 2 * _np.cross(a, b)
    den = 1 + n1 * n2 + 2 * float(a @ b)
    return float(_np.degrees(4.0 * math.atan(float(_np.linalg.norm(num / den)))))


def _final_diffs(loga, logb, spda, spdb, sca, scb, fields, aux_b=None):
    import numpy as _np
    out = {}
    for f in fields:
        if f in ("wheels", "dcm_max", "fuel_rel_err", "fuel_burned",
                 "density_max", "r_CN_N", "v_CN_N"):
            continue
        xa = _np.asarray(getattr(loga, f))[-1]
        xb = _np.asarray(getattr(logb, f))[-1]
        out[f] = float(_np.abs(xa - xb).max())
    if "wheels" in fields and spda is not None and spdb is not None:
        wa = _np.asarray(spda.wheelSpeeds)[-1]
        wb = _np.asarray(spdb.wheelSpeeds)[-1]
        out["wheels"] = float(_np.abs(wa - wb).max())
    if "dcm_max" in fields:
        sa = _np.asarray(loga.sigma_BN)
        sb = _np.asarray(logb.sigma_BN)
        out["dcm_max"] = float(max(
            _np.abs(_dcm(sa[k]) - _dcm(sb[k])).max() for k in range(len(sa))))
    if "r_CN_N" in fields:
        for f in ("r_CN_N", "v_CN_N"):
            xa = _np.asarray(getattr(loga, f))[-1]
            xb = _np.asarray(getattr(logb, f))[-1]
            out[f] = float(_np.abs(xa - xb).max())
    if "fuel_rel_err" in fields or "fuel_burned" in fields:
        fa = float(_np.asarray(
            sca.dynManager.getStateObject("fuelMass").getState()).ravel()[0])
        fb = float(_np.asarray(
            scb.dynManager.getStateObject("fuelMass").getState()).ravel()[0])
        out["fuel_stock"] = fa
        out["fuel_omni"] = fb
        out["fuel_rel_err"] = abs(fa - fb) / fa if fa else 0.0
        out["fuel_burned"] = PROP0 - fa
    if "density_max" in fields and aux_b is not None and "dlog" in aux_b:
        out["density_max"] = float(
            _np.asarray(aux_b["dlog"].neutralDensity).max())
    return out


def _srp_single_tick_ratio(dt):
    """SRP-only single-tick dv vs analytic P*A*Cr*dt/m (vacuity guard)."""
    import numpy as _np
    R = 6378137.0 + 400e3
    V = float(_np.sqrt(MU / R))
    SUN = [1.5e11, 0.0, 0.0]
    sim1, _, log1, _, _, _ = _arm(True, "srp", dt)
    _run(sim1, dt)
    sim0, sc0 = _base_sim("srp0", dt)
    sc0.hub.r_CN_NInit = [R, 0.0, 0.0]
    sc0.hub.v_CN_NInit = [0.0, V, 0.0]
    log0 = sc0.scStateOutMsg.recorder()
    sim0.AddModelToTask("t", log0)
    _run(sim0, dt)
    sun_d = float(_np.linalg.norm(_np.asarray(SUN, float)
                                  - _np.asarray([R, 0.0, 0.0])))
    P_au, AU = 4.5398178907e-6, 1.495978707e11
    analytic = P_au * (AU / sun_d) ** 2 * 10.0 * 1.5 * dt / MASS
    meas = float(_np.linalg.norm(_np.asarray(log1.v_BN_N)[-1]
                                 - _np.asarray(log0.v_BN_N)[-1]))
    return meas / analytic if analytic else 0.0


# ---------------------------------------------------------------------------
# Cell + campaign runners (host-only; called after the Basilisk gate)
# ---------------------------------------------------------------------------
def _loadavg():
    try:
        return [float(v) for v in os.getloadavg()]
    except Exception:
        return []


def _run_surface(key, spec, dt, T):
    """Paired stock/omni leg for one surface. Returns the cell record."""
    import numpy as _np
    from Basilisk.utilities import macros
    kind, kw = spec["kind"], dict(spec["kw"])
    expected_nanos = int(macros.sec2nano(float(T)))
    ticks = int(round(float(T) / float(dt)))
    t0 = time.perf_counter()
    sima, sca, loga, spda, _, aux_a = _arm(False, kind, dt, **kw)
    if aux_a.get("reset_fn") is not None:
        _run_two_phase(sima, T, float(kw.get("reset_at", 10.0)),
                       aux_a["reset_fn"])
    else:
        _run(sima, T)
    stock_wall = time.perf_counter() - t0
    stock_nanos = int(sima.TotalSim.CurrentNanos)
    t0 = time.perf_counter()
    simb, scb, logb, spdb, oib, aux_b = _arm(True, kind, dt, **kw)
    if aux_b.get("reset_fn") is not None:
        _run_two_phase(simb, T, float(kw.get("reset_at", 10.0)),
                       aux_b["reset_fn"])
    else:
        _run(simb, T)
    omni_wall = time.perf_counter() - t0
    omni_nanos = int(simb.TotalSim.CurrentNanos)
    try:
        build_s = round(float(oib.getBuildSeconds()), 4)
    except Exception:
        build_s = None
    try:
        steps = int(oib.getStepCountForObject(0))
    except Exception:
        steps = None
    fields = ["r_BN_N", "v_BN_N", "sigma_BN", "omega_BN_B"]
    if spda is not None:
        fields.append("wheels")
    if "dcm_max" in spec["tol"]:
        fields.append("dcm_max")
    if "fuel_rel_err" in spec["tol"]:
        fields.extend(["fuel_rel_err", "fuel_burned"])
    if kind in ("drag", "fuel_drag") or (kind == "demo" and kw.get(
            "demo") == "dragDeorbit"):
        fields.append("density_max")
    for f in kw.get("extra_fields", []):
        if f not in fields:
            fields.append(f)
    diffs = _final_diffs(loga, logb, spda, spdb, sca, scb, fields, aux_b)
    if "density_max" in fields:
        # Self-calibrating drag-liveness proof (gated in the vacuity block
        # below): both arms drive the *same* atmo model, so their recorded
        # peak neutral density must agree. Unlike a flat absolute floor this
        # holds at every dispersed altitude -- the 240-260 km drag campaign
        # window drops rho to ~6e-14 kg/m^3, which a 1e-13 floor misreads as
        # "no air" even though the drag force is fully live (dE_ratio ~ 1).
        try:
            ra = float(_np.asarray(aux_a["dlog"].neutralDensity).max())
            rb = float(diffs.get("density_max", 0.0))
            diffs["density_stock_max"] = ra
            diffs["density_ratio"] = (rb / ra) if ra else 0.0
        except Exception:
            diffs["density_ratio"] = 0.0
    if kind in ("srp",):
        diffs["srp_single_tick_ratio"] = _srp_single_tick_ratio(dt)
    # Vacuity: the firing/effect legs must actually displace / burn / see air.
    ra = _np.asarray(loga.r_BN_N)
    disp = float(_np.linalg.norm(ra[-1] - ra[0]))
    diffs["disp_stock_m"] = disp
    tol = spec["tol"]
    fails = []
    for f, t in tol.items():
        if f not in diffs:
            continue
        if not diffs[f] < t:
            fails.append("%s.%s=%.3e" % (key, f, diffs[f]))
    # C-point outputs (thr_offset): same switching class as r/v.
    if kind == "thr" and kw.get("offset") is not None:
        for f, g in (("r_CN_N", "r_BN_N"), ("v_CN_N", "v_BN_N")):
            if f in diffs and not diffs[f] < tol[g]:
                fails.append("%s.%s=%.3e" % (key, f, diffs[f]))
    vacuity = []
    if kind in ("drag", "fuel_drag") or (kind == "demo" and kw.get(
            "demo") == "dragDeorbit"):
        if not 0.5 < diffs.get("density_ratio", 0.0) < 2.0:
            vacuity.append("%s.vacuous density_ratio=%.3e"
                           % (key, diffs.get("density_ratio", 0.0)))
    if kind == "srp":
        if not 0.9 < diffs.get("srp_single_tick_ratio", 0.0) < 1.1:
            vacuity.append("%s.vacuous srp ratio" % key)
    if kind in ("fuel", "fuel_drag", "fuel_srp", "fsw"):
        if not diffs.get("fuel_burned", 0.0) > 0.1:
            vacuity.append("%s.vacuous burn" % key)
        if not diffs.get("fuel_rel_err", 1.0) < 1e-9:
            fails.append("%s.fuel_rel_err=%.3e" % (key, diffs["fuel_rel_err"]))
    if kind == "thr" and kw.get("mode", "fresh") == "fresh":
        # Continuous-burn proof: 20 s at duty 0.5 displaces ~0.13 m, so
        # 0.05 m clears a dead-valve wiring failure with margin. Write-once
        # / frozen fire a single kink (~mm drift) and carry no displacement
        # gate by construction -- their evidence is the kink-class bound
        # plus the ladder linearity (documented, not gated here).
        if not disp > 0.05:
            vacuity.append("%s.vacuous disp=%.3e" % (key, disp))
    if kind == "demo" and kw.get("demo") == "dragDeorbit":
        import numpy as _npd
        r0 = float(_npd.linalg.norm(_npd.asarray(loga.r_BN_N)[0]))
        r1 = float(_npd.linalg.norm(_npd.asarray(loga.r_BN_N)[-1]))
        diffs["decay_m"] = r0 - r1
        # Energy-coupling proof (self-calibrating): measured specific-energy
        # change vs the analytic drag-work estimate from the recorded mean
        # density. Radius decay is cm-scale at bench horizons (the demo
        # deorbits over ~100 orbits), so an absolute decay gate would sit at
        # the float-resolution floor of |r|; the energy ratio proves the
        # recorded density actually couples to dynamics (dead force with
        # live atmo reads ratio ~0). CdA/mass verbatim from the demo setup.
        v0 = float(_npd.linalg.norm(_npd.asarray(loga.v_BN_N)[0]))
        v1 = float(_npd.linalg.norm(_npd.asarray(loga.v_BN_N)[-1]))
        dE_meas = (0.5 * v1 * v1 - MU / r1) - (0.5 * v0 * v0 - MU / r0)
        try:
            rho_mean = float(_npd.asarray(
                aux_a["dlog"].neutralDensity).mean())
            v_mean = float(_npd.linalg.norm(
                _npd.asarray(loga.v_BN_N), axis=1).mean())
        except Exception:
            rho_mean, v_mean = 0.0, 0.0
        dE_pred = -(0.5 * rho_mean * v_mean ** 3 * (10.0 * 2.2) / 750.0)
        dE_pred *= float(T)
        diffs["dE_meas"] = dE_meas
        diffs["dE_pred"] = dE_pred
        ratio = abs(dE_meas / dE_pred) if dE_pred else 0.0
        diffs["dE_ratio"] = ratio
        if not 0.05 < ratio < 20.0:
            vacuity.append("%s.vacuous dE_ratio=%.3e" % (key, ratio))
    if kind == "demo" and kw.get("demo") in ("attFeedbackRW",
                                             "momentumDumping"):
        # FSW-liveness proof: a dead FSW chain (no commands on either arm)
        # would agree perfectly and pass vacuously. The stock wheel-speed
        # span must show the loop actually drove the wheels.
        import numpy as _npd2
        try:
            wtrace = _npd2.asarray(spda.wheelSpeeds, float)
            span = float(_npd2.abs(wtrace - wtrace[0]).max() * RADS_TO_RPM)
        except Exception:
            span = 0.0
        diffs["wheels_span_rpm"] = span
        if not span > 1.0:
            vacuity.append("%s.vacuous FSW span=%.3e RPM" % (key, span))
    if kind == "demo" and kw.get("demo") == "momentumDumping":
        # Firing proof: at least one dump impulse must have been commanded
        # on the stock arm (the reset-at-10 s + hs_min=80 chain live).
        import numpy as _npd3
        try:
            ot = _npd3.asarray(aux_a["dump_log"].OnTimeRequest, float)
            fired = bool((ot > 0).any())
        except Exception:
            fired = False
        diffs["dump_fired"] = fired
        if not fired:
            vacuity.append("%s.vacuous no-dump-firing" % key)
    ok_steps = (steps == ticks) if steps is not None else None
    ok_nanos = (stock_nanos == expected_nanos
                and omni_nanos == expected_nanos)
    if ok_steps is False:
        fails.append("%s.steps=%s!=%d" % (key, steps, ticks))
    if not ok_nanos:
        fails.append("%s.nanos stock=%d omni=%d exp=%d"
                     % (key, stock_nanos, omni_nanos, expected_nanos))
    cell = {"kind": kind, "T": float(T), "dt": float(dt), "ticks": ticks,
            "diffs": diffs, "tol": dict(tol),
            "pass": not (fails or vacuity),
            "fails": fails, "vacuity": vacuity,
            "stock_wall_s": round(stock_wall, 4),
            "omni_wall_s": round(omni_wall, 4),
            "build_s": build_s,
            "per_step_ms": {
                "stock": round(stock_wall / ticks * 1000.0, 4),
                "omni": round(omni_wall / ticks * 1000.0, 4),
            } if ticks else {},
            "speedup": round(stock_wall / omni_wall, 3) if omni_wall else None,
            "steps": steps, "steps_ok": ok_steps, "nanos_ok": ok_nanos,
            "loadavg": _loadavg()}
    try:
        n_pars = int(oib.nPars())
        n_states = int(oib.nStates())
        cell["nPars"] = n_pars
        cell["nStates"] = n_states
    except Exception:
        pass
    return cell


def _reject_cells():
    """Loud-rejection battery (no long sims except the 0.2 s C++ claim)."""
    from omniheyoka.omni_heyoka import OmniVecIntegrator
    from omniheyoka.swap import mirror_stock_config
    out = {}

    def expect_raise(name, build):
        try:
            build()
            out[name] = False
        except ValueError as exc:
            out[name] = str(exc)[:160]
        except Exception as exc:
            out[name] = str(exc)[:160]

    def mirror(sim, sc, **mkw):
        oi = OmniVecIntegrator(sc)
        return mirror_stock_config(sim, sc, oi, **mkw)

    def rw_base(jitter=False, two_eff=False):
        sim, sc = _base_sim("reject", 0.1)
        from Basilisk.simulation import reactionWheelStateEffector
        from Basilisk.utilities import simIncludeRW
        fac = simIncludeRW.rwFactory()
        kw = {}
        if jitter:
            kw["RWModel"] = reactionWheelStateEffector.JitterSimple
        fac.create("Honeywell_HR16", [1.0, 0, 0], maxMomentum=50.0,
                   Omega=100.0, **kw)
        rw = reactionWheelStateEffector.ReactionWheelStateEffector()
        fac.addToSpacecraft("rw", rw, sc)
        sim.AddModelToTask("t", rw)
        if two_eff:
            fac2 = simIncludeRW.rwFactory()
            fac2.create("Honeywell_HR16", [0, 1.0, 0], maxMomentum=50.0,
                        Omega=100.0)
            rw2 = reactionWheelStateEffector.ReactionWheelStateEffector()
            fac2.addToSpacecraft("rw2", rw2, sc)
            sim.AddModelToTask("t", rw2)
        return sim, sc

    def thr_base(**ckw):
        from Basilisk.simulation import thrusterDynamicEffector
        from Basilisk.utilities import simIncludeThruster
        sim, sc = _base_sim("reject-thr", 0.1)
        tf = simIncludeThruster.thrusterFactory()
        base = dict(MaxThrust=1.0, cutoffFrequency=1000.0,
                    useMinPulseTime=False)
        base.update(ckw)
        tf.create("TEST_Thruster", [0.5, 0, 1.0], [0, 0, -1.0], **base)
        thr = thrusterDynamicEffector.ThrusterDynamicEffector()
        tf.addToSpacecraft("thr", thr, sc)
        sim.AddModelToTask("t", thr, 2)
        return sim, sc, tf

    def fuel_base_linked():
        from Basilisk.simulation import fuelTank
        from Basilisk.utilities import macros
        sim, sc = _base_sim("reject-fuel-cxx", 0.1)
        tank = fuelTank.FuelTank()
        model = fuelTank.FuelTankModelConstantVolume()
        model.propMassInit = 10.0
        tank.setTankModel(model)
        tank.setNameOfMassState("fuelMass")
        sc.addStateEffector(tank)
        sim.AddModelToTask("t", tank)
        oi = OmniVecIntegrator(sc)
        oi.setMass(750.0)
        oi.setMu(3.986004418e14)
        sc.setIntegrator(oi)
        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(0.2))
        sim.ExecuteSimulation()

    expect_raise("fuel_unmirrored_cxx", fuel_base_linked)
    expect_raise("jitter_wheel", lambda: mirror(*rw_base(jitter=True)))
    expect_raise("two_rw_effectors", lambda: mirror(*rw_base(two_eff=True)))

    def thr_mirror(ckw):
        from Basilisk.architecture import messaging
        s, c, t = thr_base(**ckw)
        dummy = messaging.THRArrayOnTimeCmdMsg()
        return mirror(s, c, thr_factory=t, thr_cmd_msg=dummy)

    expect_raise("thruster_no_factory", lambda: mirror(*thr_base()[:2]))
    expect_raise("min_pulse", lambda: thr_mirror({"useMinPulseTime": True}))
    expect_raise("cutoff_zero", lambda: thr_mirror({"cutoffFrequency": 0.0}))
    out["verdict"] = ("all loud" if all(out.values())
                      else "SILENT: %s" % [k for k, v in out.items()
                                           if not v])
    return out


def _demo_reject_ext_torque():
    """scenarioAttitudeFeedback guard: hub + extForceTorque must raise loudly.

    The demo closes its PD loop through the extForceTorque wrench module,
    which has no omni contributor (DESIGN.md A8: a wrench contributor is
    provably additive but explicitly out of scope). The mirror must reject
    the setup at setup time -- never integrate silently wrong. Construction
    below is verbatim upstream (scenarioAttitudeFeedback.py).
    """
    from Basilisk.simulation import extForceTorque
    from omniheyoka.omni_heyoka import OmniVecIntegrator
    from omniheyoka.swap import mirror_stock_config
    sim, sc = _base_sim("reject-demo-extTorque", 0.1)
    ext = extForceTorque.ExtForceTorque()
    ext.ModelTag = "externalDisturbance"
    sc.addDynamicEffector(ext)
    sim.AddModelToTask("t", ext)
    oi = OmniVecIntegrator(sc)
    try:
        mirror_stock_config(sim, sc, oi)
        return {"pass": False,
                "raised": "SILENT (no exception -- envelope hole!)"}
    except ValueError as exc:
        ok = str(exc).startswith("mirror_stock_config")
        return {"pass": bool(ok), "raised": str(exc)[:160]}
    except Exception as exc:  # noqa: BLE001 -- any other type is still loud
        return {"pass": True, "raised": "%s: %s" % (type(exc).__name__,
                                                    str(exc)[:140])}


DEMO_REJECTS = {}


def _ladder_cell(dt_list=(0.1, 0.05, 0.02), T=30.0, duty=0.5):
    """Switching-residual ladder: dv halves with dt (Update-4 class)."""
    import numpy as _np
    rows = {}
    for dt in dt_list:
        sima, _, loga, _, _, _ = _arm(
            False, "thr", dt, duty=duty, mode="fresh")
        _run(sima, T)
        simb, _, logb, _, _, _ = _arm(
            True, "thr", dt, duty=duty, mode="fresh")
        _run(simb, T)
        dv = float(_np.abs(_np.asarray(loga.v_BN_N)[-1]
                           - _np.asarray(logb.v_BN_N)[-1]).max())
        ds = float(_np.abs(_np.asarray(loga.sigma_BN)[-1]
                           - _np.asarray(logb.sigma_BN)[-1]).max())
        rows["dt%s" % dt] = {"dv": dv, "dsigma": ds}
    dv1, dv2, dv3 = (rows["dt0.1"]["dv"], rows["dt0.05"]["dv"],
                     rows["dt0.02"]["dv"])
    lin = (0.3 < dv2 / dv1 < 0.7) and (0.2 < dv3 / dv2 < 0.6)
    return {"T": float(T), "duty": duty, "rows": rows, "linear": lin,
            "pass": bool(lin)}


def _mc_draw(rng, kind):
    """One seeded dispersion draw. Js/mass/ICs disperse in both kinds (pd
    flows Js through stock custom wheels, which the mirror reads back, so
    the pair disperses together); open additionally yaws the triple and
    tilts the diagonal wheel (rated HR16 there -- geometry, not Js, is the
    dispersed axis, and the unsaturated constant torque keeps tight bounds
    meaningful under it)."""
    import numpy as _np
    draw = {"js": [HR16_JS * rng.uniform(0.9, 1.1) for _ in range(3)],
            "rpm0": [rng.uniform(0.0, 500.0) for _ in range(3)],
            "sigma0": [s + rng.gauss(0.0, 1e-3) for s in SIG0],
            "omega0": [rng.uniform(-0.03, 0.03) for _ in range(3)],
            "mass": rng.uniform(740.0, 760.0)}
    if kind == "open":
        base = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (DIAG, DIAG, DIAG)]
        yaw = math.radians(rng.uniform(0.0, 15.0))
        c, s = math.cos(yaw), math.sin(yaw)
        R = _np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        draw["axes_override"] = [
            tuple(float(x) for x in R @ _np.asarray(g, float)) for g in base]
        draw["rpm0"] = [0.0, 0.0, 0.0]
    return draw


def _run_mc_pair(draw, kind, dt, T):
    """One paired MC sample. Returns (att_deg, rate_degs, dw_rpm, trans_deg)."""
    import numpy as _np
    if kind == "open":
        kw = dict(u=0.05, axes_override=draw["axes_override"],
                  js=draw["js"], mass=draw["mass"],
                  sigma0=[0.0, 0.0, 0.0], omega0=[0.0, 0.0, 0.0],
                  rpm0=draw["rpm0"])
        sima, _, loga, spda, _, _ = _arm(False, "rw_diag3", dt, **kw)
        _run(sima, T)
        simb, _, logb, spdb, _, _ = _arm(True, "rw_diag3", dt, **kw)
        _run(simb, T)
    else:
        kw = dict(js=draw["js"], mass=draw["mass"], sigma0=draw["sigma0"],
                  omega0=draw["omega0"], rpm0=draw["rpm0"])
        sima, _, loga, spda, _, _ = _arm(False, "rw3_pd", dt, **kw)
        _run(sima, T)
        simb, _, logb, spdb, _, _ = _arm(True, "rw3_pd", dt, **kw)
        _run(simb, T)
    n = min(len(loga.sigma_BN), len(logb.sigma_BN))
    att_trace = [_att_deg(_np.asarray(logb.sigma_BN)[k],
                          _np.asarray(loga.sigma_BN)[k]) for k in range(n)]
    att = att_trace[-1]
    rate = float(_np.degrees(_np.linalg.norm(
        _np.asarray(logb.omega_BN_B)[-1] - _np.asarray(loga.omega_BN_B)[-1])))
    dw = float(_np.abs(_np.asarray(spdb.wheelSpeeds)[-1]
                       - _np.asarray(spda.wheelSpeeds)[-1]).max()
               * RADS_TO_RPM)
    return att, rate, dw, max(att_trace)


def _run_mc(n, seed, kind, dt, T):
    """Seeded N-run Monte Carlo campaign. Returns the mc record."""
    att_tol = MC_OPEN_ATT_TOL_DEG if kind == "open" else MC_ATT_TOL_DEG
    rng = random.Random(int(seed))
    samples = []
    fails = []
    t0 = time.perf_counter()
    for i in range(int(n)):
        draw = _mc_draw(rng, kind)
        # One bad sample must not kill a 1000-run campaign: it counts as a
        # failed sample (loud, in-ledger) and the campaign continues.
        try:
            att, rate, dw, trans = _run_mc_pair(draw, kind, dt, T)
        except Exception as exc:
            # 1e30 sentinel (not inf: stays strict-JSON serializable).
            att, rate, dw, trans = (1e30, 1e30, 1e30, 1e30)
            samples.append([att, rate, dw, trans])
            fails.append({"i": i, "error": "%s: %s" % (
                type(exc).__name__, exc)})
            continue
        ok = (att <= att_tol and rate <= MC_RATE_TOL_DEGS
              and dw <= MC_WHEEL_TOL_RPM and trans <= MC_TRANSIENT_ATT_DEG)
        samples.append([att, rate, dw, trans])
        if not ok:
            fails.append({"i": i, "att": att, "rate": rate, "dw": dw,
                          "trans": trans})
        if (i + 1) % max(1, int(n) // 10 or 1) == 0:
            print("[mc] %d/%d worst_att=%.3e deg" % (
                i + 1, n, max(s[0] for s in samples)))
    wall = time.perf_counter() - t0

    def _pct(xs, q):
        xs = sorted(xs)
        return xs[min(len(xs) - 1, int(q * len(xs)))]

    atts = [s[0] for s in samples]
    return {"n": int(n), "seed": int(seed), "kind": kind, "T": float(T),
            "dt": float(dt),
            "bounds": {"att_deg": att_tol, "rate_degs": MC_RATE_TOL_DEGS,
                       "wheels_rpm": MC_WHEEL_TOL_RPM,
                       "transient_deg": MC_TRANSIENT_ATT_DEG},
            "pass_rate": round((len(samples) - len(fails)) / len(samples), 4),
            "n_fail": len(fails), "fails": fails[:20],
            "worst": {"att": max(atts), "rate": max(s[1] for s in samples),
                      "dw": max(s[2] for s in samples),
                      "trans": max(s[3] for s in samples)},
            "p50_att": _pct(atts, 0.5), "p95_att": _pct(atts, 0.95),
            "samples": [[round(a, 12), round(r, 12), round(d, 12),
                         round(t, 12)] for a, r, d, t in samples],
            "wall_s": round(wall, 2),
            "per_pair_s": round(wall / len(samples), 3),
            "pass": len(fails) == 0}


def _mc_draw_demo(rng, name):
    """Seeded dispersion draw for a demo setup (demo-exact = draw center).

    Disperses bus mass, attitude/rate ICs, orbit elements, wheel speeds and
    inertia scale -- the same axes as the inline campaign, mapped onto each
    demo's parameters. Js is NOT dispersed here (HR16 wheels stay
    demo-exact; Js dispersion lives in the inline pd/open kinds).
    momentumDumping scales all four wheel speeds together so |H| stays
    above the hs_min=80 dump threshold (geometry-preserving scale).
    """
    draw = {"mass_scale": rng.uniform(0.99, 1.01),
            "sig_pert": [rng.gauss(0.0, 1e-3) for _ in range(3)],
            "om_pert": [rng.uniform(-3e-3, 3e-3) for _ in range(3)],
            "oe_a_scale": rng.uniform(0.999, 1.001),
            "oe_e_add": rng.uniform(0.0, 1e-4),
            "inertia_scale": rng.uniform(0.99, 1.01)}
    if name in ("basicOrbit_LEO", "basicOrbit_GTO", "basicOrbit_GEO"):
        draw["kind"] = "orbit"
    elif name in ("attFeedbackRW", "attitudeGG"):
        draw["kind"] = "rw"
        draw["rpm0"] = [rng.uniform(0.0, 500.0) for _ in range(3)]
    elif name == "momentumDumping":
        draw["kind"] = "rw"
        s = rng.uniform(0.9, 1.1)
        draw["rpm0"] = [4000.0 * s, 2000.0 * s, 3500.0 * s, 0.0]
    elif name == "dragDeorbit":
        draw["kind"] = "drag"
        draw["alt_km"] = rng.uniform(240.0, 260.0)
        draw["mass_scale"] = rng.uniform(700.0 / 750.0, 800.0 / 750.0)
    else:
        raise ValueError("no MC draw for demo %s" % name)
    return draw


def _demo_mc_overrides(name, draw, I9_default):
    """Map a dispersion draw onto _arm_demo kw overrides."""
    kw = {}
    if draw.get("mass_scale") is not None and name != "dragDeorbit":
        kw["mass"] = 750.0 * draw["mass_scale"]
    if name == "momentumDumping":
        kw["mass"] = 2500.0 * draw["mass_scale"]
    if name == "dragDeorbit":
        kw["mass"] = 750.0 * draw["mass_scale"]
        kw["alt_km"] = draw["alt_km"]
    base_sig = {"attFeedbackRW": [0.1, 0.2, -0.3],
                "attitudeGG": [0.1, 0.2, -0.3]}.get(name, [0.0, 0.0, 0.0])
    base_om = {"attFeedbackRW": [0.001, -0.01, 0.03],
               "attitudeGG": [0.001, -0.01, 0.03]}.get(name, [0.0, 0.0, 0.0])
    kw["sigma0"] = [b + p for b, p in zip(base_sig, draw["sig_pert"])]
    kw["omega0"] = [b + p for b, p in zip(base_om, draw["om_pert"])]
    if name in ("basicOrbit_LEO", "basicOrbit_GTO", "basicOrbit_GEO",
                "momentumDumping", "attitudeGG"):
        base_oe = _DEMO_MC_OE[name]
        kw["oe"] = (base_oe[0] * draw["oe_a_scale"],
                    base_oe[1] + draw["oe_e_add"]) + tuple(base_oe[2:])
    if I9_default is not None:
        s = draw["inertia_scale"]
        kw["I9"] = [[I9_default[i][i] * s if i == j else 0.0
                     for j in range(3)] for i in range(3)]
    if draw.get("rpm0") is not None:
        kw["rpm0"] = list(draw["rpm0"])
    return kw


_DEMO_MC_I9 = {
    "attFeedbackRW": [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0],
                      [0.0, 0.0, 600.0]],
    "momentumDumping": [[1700.0, 0.0, 0.0], [0.0, 1700.0, 0.0],
                        [0.0, 0.0, 1800.0]],
    "attitudeGG": [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]],
}

# Dispersed-setup bounds for the demo MC campaign. A canonical cell tol is
# calibrated for the demo-exact setup; two of its field bounds cannot also be
# a *dispersed* bound, so the campaign derives a per-field bound from the same
# evidence class instead of inventing an unrelated number:
#   * r_BN_N -- for a point-mass orbit the matched-dt stock-RK4-vs-omni-Taylor
#     residual is float-rounding class: the canonical 1e-9 m sits at/below one
#     ULP of the position at LEO/GTO radius (~9.3e-10 m), so setup dispersion
#     exposes last-bit rounding, not physics (recorded <=2 ULP over 100 draws:
#     4.66e-10 / 9.31e-10 / 1.86e-9 = 0.5/1/2 ULP at 6.78e6 m). The bound is a
#     few ULPs of the orbit scale, so it follows LEO/GTO/GEO automatically.
#   * wheels (momentum-dumping class only) -- the FSW dump schedule switches
#     thrusters and stock/omni quantize the switch to different sub-tick
#     instants; the wheel-speed readout therefore carries a switching-jitter
#     transient (recorded <=~2.1e-4 rad/s over 100 draws) that the attitude
#     and orbit fields do not see (they stay ~3 orders inside their bounds).
# The canonical cell tol is kept for the canonical cell and is recorded
# alongside the dispersed bound in the ledger.
MC_POS_ULP = 8.0          # position parity: <= a few ULPs of the orbit scale
MC_WHEEL_SWITCH = 5.0     # momentum-dumping wheel-switch jitter allowance
MC_WHEEL_SWITCH_DEMOS = ("momentumDumping",)

# Demo-exact orbital elements (perigee/a, e, i, RAAN, argp, nu) for the demos
# on an orbit: the dispersion scales ``a`` and adds to ``e`` around these, and
# the orbit scale ``a`` also sets the position resolution floor below.
_DEMO_MC_OE = {
    "basicOrbit_LEO": (6778e3, 0.0001, 33.3, 48.2, 347.8, 85.3),
    "basicOrbit_GTO": (24396e3, 0.7283, 33.3, 48.2, 347.8, 0.0),
    "basicOrbit_GEO": (42164e3, 0.0, 0.0, 0.0, 0.0, 0.0),
    "momentumDumping": (7000e3, 0.0001, 33.3, 148.2, 347.8, 335.0),
    "attitudeGG": (1.0e7, 0.1, 33.3, 48.2, 347.8, 85.3),
}


def _demo_mc_bound(name, cell_tol):
    """Per-field dispersed-setup bound for one demo MC campaign (see note)."""
    tol = dict(cell_tol)
    if "r_BN_N" in tol:
        oe = _DEMO_MC_OE.get(name)
        if oe:
            tol["r_BN_N"] = max(tol["r_BN_N"], MC_POS_ULP * math.ulp(float(oe[0])))
    if name in MC_WHEEL_SWITCH_DEMOS and "wheels" in tol:
        tol["wheels"] = MC_WHEEL_SWITCH * tol["wheels"]
    return tol


def _run_mc_demo(n, seed, name, dt, T):
    """Seeded N-run Monte Carlo over a demo setup (cell tol as the bound).

    Each sample runs the demo surface cell at T with dispersed setup and
    passes iff the cell's own diffs/tols/vacuity pass -- i.e. "the demo
    parity holds across dispersed setups". Reuses _run_surface wholesale,
    so new demos get MC coverage with no new bounds to invent.
    """
    cell_key = "demo_" + name
    base = dict(SURFACES[cell_key]["kw"])
    cell_tol = dict(SURFACES[cell_key]["tol"])
    tol = _demo_mc_bound(name, cell_tol)
    rng = random.Random(int(seed))
    samples = []
    fails = []
    worst = {}
    t0 = time.perf_counter()
    for i in range(int(n)):
        draw = _mc_draw_demo(rng, name)
        kw = dict(base)
        kw.update(_demo_mc_overrides(name, draw, _DEMO_MC_I9.get(name)))
        spec = {"kind": "demo", "kw": kw, "tol": tol}
        c = None
        try:
            c = _run_surface("%s-mc%04d" % (cell_key, i), spec, dt, float(T))
            margin = 0.0
            for f, t in tol.items():
                if f in c["diffs"] and isinstance(c["diffs"][f],
                                                  (int, float)):
                    margin = max(margin, c["diffs"][f] / t)
            ok = bool(c["pass"])
            info = "" if ok else "; ".join(c["fails"] + c["vacuity"])
        except Exception as exc:
            margin, ok, info = 1e30, False, "%s: %s" % (type(exc).__name__,
                                                        exc)
        samples.append(round(margin, 6) if margin < 1e29 else 1e30)
        if not ok:
            fails.append({"i": i, "margin": round(margin, 6)
                          if margin < 1e29 else 1e30, "info": info[:300]})
        if (i + 1) % max(1, int(n) // 10 or 1) == 0:
            print("[mc-demo:%s] %d/%d worst_margin=%.3f" % (
                name, i + 1, n, max(s for s in samples if s < 1e29)
                if any(s < 1e29 for s in samples) else float("nan")))
        # track per-field worst diffs across passing and failing samples
        if c is not None:
            for f in tol:
                v = c["diffs"].get(f)
                if isinstance(v, (int, float)):
                    worst[f] = max(worst.get(f, 0.0), v)
    wall = time.perf_counter() - t0

    def _pct(xs, q):
        xs = sorted(xs)
        return xs[min(len(xs) - 1, int(q * len(xs)))]

    finite = [s for s in samples if s < 1e29]
    return {"n": int(n), "seed": int(seed), "kind": "demo:" + name,
            "demo": name, "T": float(T), "dt": float(dt),
            "bounds": {"tol": tol, "cell_tol": cell_tol,
                       "margin_pass_below": 1.0},
            "pass_rate": round((len(samples) - len(fails)) / len(samples), 4),
            "n_fail": len(fails), "fails": fails[:20],
            "worst": {f: round(v, 12) for f, v in worst.items()},
            "p50_margin": round(_pct(finite, 0.5), 6) if finite else None,
            "p95_margin": round(_pct(finite, 0.95), 6) if finite else None,
            "samples": samples,
            "wall_s": round(wall, 2),
            "per_pair_s": round(wall / len(samples), 3),
            "pass": len(fails) == 0}


def _run_probe_entry(fname, ledger, expect, timeout_s=1800):
    """Run one sibling-probe command; grade it from exit + owning ledger."""
    parts = str(fname).split()
    cmd = [sys.executable, str(PROBES_DIR / parts[0])] + parts[1:]
    t0 = time.perf_counter()
    rec = {"cmd": " ".join([parts[0]] + parts[1:]), "expect": expect}
    try:
        p = subprocess.run(cmd, capture_output=True, text=True,
                           timeout=float(timeout_s))
        rec["wall_s"] = round(time.perf_counter() - t0, 1)
        rec["exit"] = int(p.returncode)
        rec["tail"] = (p.stdout or "")[-500:]
        # Tracebacks go to stderr: keep the tail so a red cell is
        # diagnosable from this ledger alone.
        rec["err_tail"] = (p.stderr or "")[-1500:]
    except subprocess.TimeoutExpired:
        rec["wall_s"] = round(time.perf_counter() - t0, 1)
        rec["exit"] = "timeout"
        rec["tail"] = ""
        rec["err_tail"] = ""
    doc = None
    lp = RESULTS_DIR / (ledger + ".json")
    if lp.exists():
        try:
            doc = json.loads(lp.read_text(encoding="utf-8"))
        except Exception as exc:
            rec["ledger_error"] = "%s: %s" % (type(exc).__name__, exc)
    payload = (doc.get("payload", {}) if isinstance(doc, dict) else {})
    rec["ledger_status"] = payload.get("status", "missing")
    rec["ledger_verdict"] = payload.get("verdict", payload.get("status"))
    # Surface whole-run timing when the owning ledger records it
    # (hijack cells carry stock/omni walls + speedup; the subprocess's
    # own stdout is captured, so it would otherwise never be printed).
    for _k in ("speedup", "speedup_cold", "stock_wall_s",
                 "omni_wall_s", "omni_cold_wall_s"):
        if isinstance(payload.get(_k), (int, float)):
            rec[_k] = payload[_k]
    if ledger == "env_parity_envbodies":
        # Honest NO-GO (list.md A1): pass = ran + recorded. The owning
        # test pins the two divergence mechanisms; this suite must not
        # second-guess them into red.
        dump = json.dumps(payload).lower()
        rec["no_go_documented"] = ("no-go" in dump or "no_go" in dump
                                   or "nogo" in dump)
        rec["pass"] = (rec["exit"] == 0
                       and rec["ledger_status"] in ("ok", "skipped"))
    else:
        rec["pass"] = (rec["exit"] == 0
                       and rec["ledger_status"] == "ok"
                       and (expect is None
                            or expect in str(rec["ledger_verdict"])))
    return rec


def _run_extended(timeout_s=1800):
    """Subprocess sweep over sibling probes; aggregate exit + ledger status."""
    results = {}
    for fname, ledger, expect in EXTENDED:
        results[ledger] = _run_probe_entry(fname, ledger, expect, timeout_s)
    return results


def _hijack_registry():
    """Runnable upstream-demo hijack entries from the sibling probe.

    Imports ``hijack_demo.py`` by file path (its top level is stdlib-only);
    returns ``{demo: cfg}`` for every parity/reject entry, skipping the
    documented ``excluded`` ones. No Basilisk needed to read the registry.
    """
    import importlib.util
    probe = PROBES_DIR / "hijack_demo.py"
    if not probe.exists():
        return {}
    spec = importlib.util.spec_from_file_location("hijack_demo_registry",
                                                  probe)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return {k: c for k, c in mod.CONFIG.items() if c["mode"] != "excluded"}


def _hijack_excluded():
    """Documented ``excluded`` hijack entries: ledger name -> reason.

    The sibling probe marks a demo ``excluded`` when the exact pinned file
    needs a capability the mirror honestly lacks (e.g. a sun third body +
    SPICE ephemeris: the env-bodies N-body path is an honest NO-GO). Those
    legs cannot run, but the benchmark ledger must still ACCOUNT for them --
    a demo absent from the hijack block would read as a silent gap. Same
    stdlib-only file-path import as ``_hijack_registry``.
    """
    import importlib.util
    probe = PROBES_DIR / "hijack_demo.py"
    if not probe.exists():
        return {}
    spec = importlib.util.spec_from_file_location("hijack_demo_excluded",
                                                  probe)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return {"hijack_%s" % Path(c["file"]).stem: c["note"]
            for c in mod.CONFIG.values() if c["mode"] == "excluded"}


def _run_hijack_sweep(timeout_s=1800, skip=()):
    """Drive EVERY runnable hijack CONFIG demo as a subprocess.

    This is the demos/all-tier "everything we can" leg: the exact pinned
    upstream files (not the inline replicas), one ``hijack_demo.py --demo``
    call each. Verdicts are the owning probe's: parity legs expect
    ``parity green``, reject legs expect ``reject loud``. ``skip`` holds
    ledger names already produced by the extended sweep (the full tier's two
    canonical hijack cells) so the demos/all sweep never re-runs them.
    """
    registry = _hijack_registry()
    out = {}
    for name in sorted(registry):
        cfg = registry[name]
        ledger = "hijack_%s" % Path(cfg["file"]).stem
        if ledger in skip:
            continue
        expect = ("parity green" if cfg["mode"] == "parity"
                  else "reject loud")
        out[ledger] = _run_probe_entry(
            "hijack_demo.py --demo %s" % name, ledger, expect, timeout_s)
    return out


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="Stock-vs-omni benchmark suite (tiers quick/standard/full/mc).")
    ap.add_argument("--tier", default="quick",
                    choices=list(TIERS),
                    help="benchmark tier (default: quick smoke)")
    ap.add_argument("--surfaces", default=None,
                    help="comma-separated surface keys (default: tier set); "
                         "'rejects' forces the reject battery too")
    ap.add_argument("--list-surfaces", action="store_true",
                    help="print the surface registry and exit (no Basilisk needed)")
    ap.add_argument("--n-mc", type=int, default=None,
                    help="MC sample count: single-campaign tiers run one "
                         "campaign with N pairs; the all tier runs EVERY "
                         "campaign with N pairs each (default per tier: "
                         "quick 4, standard 20, full 100, mc 1000, all 100)")
    ap.add_argument("--mc-kinds", default=None,
                    help="comma-separated MC campaign subset for the all "
                         "tier (default: pd,open + every demo). Ignored by "
                         "other tiers (they use --mc-kind).")
    ap.add_argument("--mc-kind", default=None,
                    choices=["pd", "open"] + sorted(DEMOS),
                    help="MC leg kind: pd/open (inline Js dispersion) or a "
                         "demo name for a dispersed-setup campaign over that "
                         "upstream demo (default per tier)")
    ap.add_argument("--seed", type=int, default=20261016,
                    help="MC RNG seed (default: 20261016)")
    ap.add_argument("--dt", type=float, default=0.1,
                    help="matched step [s] for inline legs (default: 0.1)")
    ap.add_argument("--t-scale", type=float, default=1.0,
                    help="multiply every inline leg T (default: 1.0)")
    ap.add_argument("--skip-extended", action="store_true",
                    help="skip the sibling-probe sweep in the full tier")
    ap.add_argument("--ext-timeout", type=float, default=1800.0,
                    help="per-probe timeout [s] for the extended sweep")
    ap.add_argument("--mc-T", type=float, default=60.0,
                    help="MC leg length [s] (default: 60)")
    return ap.parse_args(argv)


def list_surfaces():
    print("tier sets: quick=%s"
          % sorted(k for k, s in SURFACES.items() if "quick" in s["tiers"]))
    print("           standard=%s"
          % sorted(k for k, s in SURFACES.items() if "standard" in s["tiers"]))
    print("           full=%s"
          % sorted(k for k, s in SURFACES.items() if "full" in s["tiers"]))
    print("           mc_smoke=%s" % list(MC_TIER_INLINE))
    print("           demos=%s"
          % sorted(k for k, s in SURFACES.items() if "demos" in s["tiers"]))
    print("           all=%s (every surface above + all MC campaigns)"
          % sorted(SURFACES))
    print("           mc_kinds=%s" % (["pd", "open"] + sorted(DEMOS)))
    try:
        print("           hijack=%s (demos/all tier registry sweep)"
              % sorted(_hijack_registry()))
        print("           hijack-excluded=%s (documented, accounted in ledger)"
              % sorted(_hijack_excluded()))
    except Exception:
        pass
    for key in sorted(SURFACES):
        s = SURFACES[key]
        print("%-16s family=%-8s kind=%-8s T=%7.1f T_quick=%5.1f tiers=%s"
              % (key, s["family"], s["kind"], s["T"], s["T_quick"],
                 ",".join(s["tiers"])))


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.list_surfaces:
        list_surfaces()
        return 0
    tier = args.tier
    tpar = TIERS[tier]
    # Name validation is stdlib-only and runs before the host gate, so
    # typos fail fast (exit 2) even on hosts without Basilisk.
    _MC_ALL = ["pd", "open"] + sorted(DEMOS)
    if args.surfaces:
        _wanted_arg = [k.strip() for k in args.surfaces.split(",")
                       if k.strip()]
        _unknown = [k for k in _wanted_arg if k not in SURFACES
                    and k != "rejects"]
        if _unknown:
            print("unknown surfaces: %s (see --list-surfaces)" % _unknown)
            return 2
    if args.mc_kinds:
        _kinds_arg = [k.strip() for k in args.mc_kinds.split(",")
                      if k.strip()]
        _unknown_k = [k for k in _kinds_arg if k not in _MC_ALL]
        if _unknown_k:
            print("unknown mc-kinds: %s (see --list-surfaces)" % _unknown_k)
            return 2
    if not safe_import_basilisk():
        save_result("bench_stock_vs_omni",
                    {"status": "skipped-no-basilisk", "tier": tier,
                     "reason": "Basilisk not importable on this host"})
        print("bench_stock_vs_omni: skipped (Basilisk unimportable)")
        return 0
    try:
        import omniheyoka.omni_heyoka  # noqa: F401 (built wheel gate)
    except Exception as exc:
        save_result("bench_stock_vs_omni",
                    {"status": "skipped-no-basilisk", "tier": tier,
                     "failed_import": "omniheyoka.omni_heyoka",
                     "error": "%s: %s" % (type(exc).__name__, exc)})
        print("bench_stock_vs_omni: skipped (wheel unimportable: %s)" % exc)
        return 0
    dt = float(args.dt)
    # Inline set: explicit filter, mc-tier smoke subset, or tier membership
    # (names already validated above).
    if args.surfaces:
        wanted = [k.strip() for k in args.surfaces.split(",") if k.strip()
                  and k.strip() != "rejects"]
    elif tier == "mc":
        wanted = [k for k in MC_TIER_INLINE if k in SURFACES]
    elif tier == "all":
        wanted = list(SURFACES)
    else:
        wanted = [k for k, s in SURFACES.items() if tier in s["tiers"]]
    run_rejects = (args.surfaces is None) or ("rejects" in (args.surfaces or ""))
    if tier == "mc" and args.surfaces is None:
        run_rejects = True
    n_mc = int(args.n_mc) if args.n_mc is not None else int(tpar["mc_n"])
    mc_kind = str(args.mc_kind) if args.mc_kind else str(tpar["mc_kind"])
    payload = {"status": "ok", "tier": tier, "dt": dt,
               "seed": int(args.seed), "cells": {},
               "t_scale": float(args.t_scale)}
    # JIT warm-up (metrology, not physics): the first heyoka build in a fresh
    # process pays one-time LLVM cold start; every timed cell below must be a
    # warm steady-state build. This throwaway 2-tick build is UNRECORDED.
    _warm, _, _, _, _, _ = _arm(True, "hub", dt)
    _run(_warm, 2 * dt)
    for key in wanted:
        spec = SURFACES[key]
        # Smoke tiers run the short legs; standard/full run canonical legs.
        base_T = spec["T_quick"] if tier in ("quick", "mc") else spec["T"]
        T = float(base_T) * float(args.t_scale)
        # One broken builder must not lose the campaign: record the crash
        # as a failed cell (loud, in-ledger) and keep benching the rest.
        try:
            cell = _run_surface(key, spec, dt, T)
        except Exception as exc:
            cell = {"kind": spec["kind"], "T": float(T), "dt": float(dt),
                    "ticks": int(round(float(T) / float(dt))),
                    "diffs": {}, "tol": dict(spec["tol"]),
                    "pass": False, "fails": ["%s.CRASH %s: %s" % (
                        key, type(exc).__name__, exc)],
                    "vacuity": [],
                    "stock_wall_s": 0.0, "omni_wall_s": 0.0, "build_s": None,
                    "per_step_ms": {}, "speedup": None, "steps": None,
                    "steps_ok": None, "nanos_ok": False,
                    "loadavg": _loadavg(),
                    "traceback": traceback.format_exc(limit=20)}
        payload["cells"][key] = cell
        log_text("bench_stock_vs_omni",
                 "[%s] pass=%s stock=%.3fs omni=%.3fs build=%s speedup=%s %s"
                 % (key, cell["pass"], cell["stock_wall_s"],
                    cell["omni_wall_s"], cell["build_s"], cell["speedup"],
                    " ".join("%s=%.2e" % (k, v) for k, v in
                             cell["diffs"].items()
                             if isinstance(v, float))))
        print("[bench] %-16s pass=%-5s stock=%7.3fs omni=%7.3fs "
              "speedup=%s fails=%s" % (
                  key, cell["pass"], cell["stock_wall_s"],
                  cell["omni_wall_s"], cell["speedup"], cell["fails"]))
    if run_rejects:
        payload["rejects"] = _reject_cells()
        log_text("bench_stock_vs_omni",
                 "[rejects] %s" % payload["rejects"].get("verdict"))
        print("[bench] rejects: %s" % payload["rejects"].get("verdict"))
        payload["demo_rejects"] = {k: fn() for k, fn in DEMO_REJECTS.items()}
        for k, rec in payload["demo_rejects"].items():
            log_text("bench_stock_vs_omni",
                     "[demo_reject:%s] pass=%s raised=%s"
                     % (k, rec["pass"], rec["raised"]))
            print("[bench] demo_reject:%s pass=%s" % (k, rec["pass"]))
    if tpar["ladder"]:
        payload["ladder"] = _ladder_cell()
        log_text("bench_stock_vs_omni",
                 "[ladder] linear=%s rows=%s"
                 % (payload["ladder"]["linear"], payload["ladder"]["rows"]))
        print("[bench] ladder linear=%s" % payload["ladder"]["linear"])
    if n_mc > 0:
        # The all tier runs EVERY campaign with the uniform --n-mc count;
        # other tiers run the single --mc-kind campaign. --mc-kinds subsets
        # the all-tier set (default: pd + open + every demo).
        if tier == "all":
            if args.mc_kinds:
                kinds = [k.strip() for k in args.mc_kinds.split(",")
                         if k.strip()]
            else:
                kinds = ["pd", "open"] + sorted(DEMOS)
            payload["mc_campaigns"] = {}
            for k in kinds:
                if k in DEMOS:
                    demo_cell = "demo_" + k
                    T_dm = float(SURFACES[demo_cell].get("T_mc", 60.0))
                    rec = _run_mc_demo(n_mc, int(args.seed), k, dt, T_dm)
                else:
                    rec = _run_mc(n_mc, int(args.seed), k, dt,
                                  float(args.mc_T))
                payload["mc_campaigns"][k] = rec
                log_text("bench_stock_vs_omni",
                         "[mc:%s] n=%d pass_rate=%.4f wall=%.1fs" % (
                             k, n_mc, rec["pass_rate"], rec["wall_s"]))
                print("[bench] mc:%-16s n=%-5d pass_rate=%.4f wall=%.1fs"
                      % (k, n_mc, rec["pass_rate"], rec["wall_s"]))
            payload["mc"] = payload["mc_campaigns"].get(
                "pd", next(iter(payload["mc_campaigns"].values())))
            payload["mc_primary"] = (
                "pd" if "pd" in payload["mc_campaigns"]
                else next(iter(payload["mc_campaigns"])))
        elif mc_kind in DEMOS:
            demo_cell = "demo_" + mc_kind
            T_dm = float(SURFACES[demo_cell].get("T_mc", 60.0))
            payload["mc"] = _run_mc_demo(n_mc, int(args.seed), mc_kind,
                                         dt, T_dm)
        else:
            payload["mc"] = _run_mc(n_mc, int(args.seed), mc_kind, dt,
                                    float(args.mc_T))
        if tier != "all":
            # All-tier campaigns log per-campaign above; single-campaign
            # tiers summarize here.
            _mc = payload["mc"]
            if _mc.get("demo"):
                log_text("bench_stock_vs_omni",
                         "[mc] kind=%s n=%d pass_rate=%.4f worst_margin=%.3f "
                         "p95_margin=%.3f wall=%.1fs"
                         % (mc_kind, n_mc, _mc["pass_rate"],
                            max(_mc["worst"].values()) if _mc["worst"]
                            else -1.0,
                            _mc["p95_margin"]
                            if _mc["p95_margin"] is not None
                            else -1.0, _mc["wall_s"]))
                print("[bench] mc kind=%s n=%d pass_rate=%.4f worst=%s" % (
                    mc_kind, n_mc, _mc["pass_rate"], _mc["worst"]))
            else:
                log_text("bench_stock_vs_omni",
                         "[mc] kind=%s n=%d pass_rate=%.4f worst_att=%.3e deg "
                         "p95_att=%.3e wall=%.1fs"
                         % (mc_kind, n_mc, _mc["pass_rate"],
                            _mc["worst"]["att"],
                            _mc["p95_att"], _mc["wall_s"]))
                print("[bench] mc kind=%s n=%d pass_rate=%.4f "
                      "worst_att=%.3e deg"
                      % (mc_kind, n_mc, _mc["pass_rate"],
                         _mc["worst"]["att"]))
    if tpar["extended"] and not args.skip_extended:
        payload["extended"] = _run_extended(args.ext_timeout)
        for name, rec in payload["extended"].items():
            log_text("bench_stock_vs_omni",
                     "[ext:%s] pass=%s exit=%s status=%s verdict=%s wall=%ss"
                     % (name, rec["pass"], rec["exit"],
                        rec["ledger_status"], rec["ledger_verdict"],
                        rec.get("wall_s")))
        print("[bench] ext:%-22s pass=%-5s exit=%s verdict=%s" % (
            name, rec["pass"], rec["exit"], rec["ledger_verdict"]))
        if isinstance(rec.get("speedup"), (int, float)):
            if rec.get("omni_cold_wall_s") is not None:
                print("         stock=%.3fs omni_cold=%.3fs omni_warm=%.3fs "
                      "speedup=%s (warm)" % (
                          rec.get("stock_wall_s"), rec.get("omni_cold_wall_s"),
                          rec.get("omni_wall_s"), rec["speedup"]))
                log_text("bench_stock_vs_omni",
                         "[ext:%s] stock=%.3fs omni_cold=%.3fs "
                         "omni_warm=%.3fs speedup=%s (warm)" % (
                             name, rec.get("stock_wall_s"),
                             rec.get("omni_cold_wall_s"),
                             rec.get("omni_wall_s"), rec["speedup"]))
            else:
                print("         stock=%.3fs omni=%.3fs speedup=%s" % (
                    rec.get("stock_wall_s"), rec.get("omni_wall_s"),
                    rec["speedup"]))
                log_text("bench_stock_vs_omni",
                         "[ext:%s] stock=%.3fs omni=%.3fs speedup=%s" % (
                             name, rec.get("stock_wall_s"),
                             rec.get("omni_wall_s"), rec["speedup"]))
            if not rec["pass"] and rec.get("err_tail"):
                print("[bench] ext:%s stderr tail:\n%s" % (name,
                      rec["err_tail"]))
                log_text("bench_stock_vs_omni",
                         "[ext:%s] stderr tail:\n%s" % (name,
                         rec["err_tail"]))
    if tpar.get("hijack"):
        # Full upstream-demo hijack registry (exact pinned files), demos/all
        # tiers. Skip legs the extended sweep already ran, then fold those
        # two canonical cells back in so this block reports the whole set.
        _skip = {n for n in payload.get("extended", {})
                 if n.startswith("hijack_")}
        hij = _run_hijack_sweep(args.ext_timeout, skip=_skip)
        for n, rec in payload.get("extended", {}).items():
            if n.startswith("hijack_"):
                hij.setdefault(n, rec)
        # Account for the documented exclusions too: every upstream hijack
        # CONFIG entry lands in this block, runnable or not (a missing demo
        # would read as a silent gap, not as a documented disposition).
        for _n, _reason in _hijack_excluded().items():
            hij.setdefault(_n, {
                "cmd": "(excluded: not run)", "expect": "excluded",
                "exit": None, "wall_s": None,
                "ledger_status": "skipped-excluded",
                "ledger_verdict": "excluded", "pass": True,
                "reason": _reason})
        payload["hijack"] = hij
        for name in sorted(hij):
            rec = hij[name]
            if rec.get("expect") == "excluded":
                print("[bench] hij:%-34s excluded   (%s)"
                      % (name, (rec.get("reason") or "").split(":")[-1]
                         .strip()[:70]))
                log_text("bench_stock_vs_omni",
                         "[hij:%s] excluded (accounted, not run): %s"
                         % (name, rec.get("reason")))
                continue
            log_text("bench_stock_vs_omni",
                     "[hij:%s] pass=%s expect=%s exit=%s status=%s "
                     "verdict=%s wall=%ss" % (
                         name, rec["pass"], rec["expect"], rec["exit"],
                         rec["ledger_status"], rec["ledger_verdict"],
                         rec.get("wall_s")))
            print("[bench] hij:%-34s pass=%-5s exit=%s verdict=%s" % (
                name, rec["pass"], rec["exit"], rec["ledger_verdict"]))
            if not rec["pass"] and rec.get("err_tail"):
                print("[bench] hij:%s stderr tail:\n%s" % (name,
                      rec["err_tail"]))
                log_text("bench_stock_vs_omni",
                         "[hij:%s] stderr tail:\n%s" % (name,
                         rec["err_tail"]))
    # Timing summary: geomean speedup over timed inline cells.
    import math as _math
    sps = [c["speedup"] for c in payload["cells"].values()
           if isinstance(c.get("speedup"), (int, float)) and c["speedup"] > 0]
    payload["timing_summary"] = {
        "n_timed": len(sps),
        "geomean_speedup": (round(_math.exp(sum(_math.log(s) for s in sps)
                                            / len(sps)), 3) if sps else None),
        "min_speedup": round(min(sps), 3) if sps else None,
        "max_speedup": round(max(sps), 3) if sps else None,
        "stock_total_s": round(sum(c["stock_wall_s"]
                                   for c in payload["cells"].values()), 2),
        "omni_total_s": round(sum(c["omni_wall_s"]
                                  for c in payload["cells"].values()), 2),
    }
    fails = []
    for key, cell in payload["cells"].items():
        if not cell["pass"]:
            fails.append("%s:%s" % (key, cell["fails"] + cell["vacuity"]))
    if run_rejects and payload.get("rejects", {}).get("verdict") != "all loud":
        fails.append("rejects:%s" % payload["rejects"].get("verdict"))
    if run_rejects:
        for k, rec in payload.get("demo_rejects", {}).items():
            if not rec.get("pass"):
                fails.append("demo_reject:%s:%s" % (k, rec.get("raised")))
    if "ladder" in payload and not payload["ladder"]["pass"]:
        fails.append("ladder:not-linear")
    if "mc" in payload and not payload["mc"]["pass"]:
        _mc = payload["mc"]
        if _mc.get("demo"):
            fails.append("mc-demo:%s:%d/%d failed worst=%s" % (
                _mc["demo"], _mc["n_fail"], _mc["n"], _mc["worst"]))
        else:
            fails.append("mc:%d/%d failed worst_att=%.3e" % (
                _mc["n_fail"], _mc["n"], _mc["worst"]["att"]))
    for k, rec in payload.get("mc_campaigns", {}).items():
        if k == payload.get("mc_primary"):
            continue  # already gated via payload["mc"] above
        if not rec.get("pass"):
            fails.append("mc-campaign:%s:%d/%d failed" % (
                k, rec.get("n_fail"), rec.get("n")))
    if "extended" in payload:
        for name, rec in payload["extended"].items():
            if not rec["pass"]:
                fails.append("ext:%s:exit=%s,status=%s" % (
                    name, rec["exit"], rec["ledger_status"]))
    if "hijack" in payload:
        for name, rec in payload["hijack"].items():
            if not rec["pass"]:
                fails.append("hijack:%s:exit=%s,status=%s" % (
                    name, rec["exit"], rec["ledger_status"]))
    # Hijack timing: from the hijack block when present (demos/all), else the
    # full tier's two canonical hijack cells inside the extended sweep.
    _hijsrc = payload.get("hijack")
    if _hijsrc is None:
        _hijsrc = {n: r for n, r in payload.get("extended", {}).items()
                   if n.startswith("hijack_")}
    hij = {n: r for n, r in _hijsrc.items()
           if n.startswith("hijack_")
           and isinstance(r.get("speedup"), (int, float))}
    if hij:
        payload["hijack_timing"] = {
            n: {"speedup": r["speedup"],
                "stock_wall_s": r.get("stock_wall_s"),
                "omni_wall_s": r.get("omni_wall_s"),
                "omni_cold_wall_s": r.get("omni_cold_wall_s")}
            for n, r in hij.items()
        }
        log_text("bench_stock_vs_omni",
                 "[hijack-timing] %s" % " ".join(
                     "%s=%s" % kv for kv in sorted(
                         ((n, r["speedup"]) for n, r in hij.items()))))
        print("[bench] hijack timing summary: %s" % ", ".join(
            "%s %sx" % (n, r["speedup"]) for n, r in sorted(hij.items())))
    payload["verdict"] = ("bench green" if not fails
                          else "BENCH FAIL: %s" % fails)
    if fails:
        payload["status"] = "fail"
    save_result("bench_stock_vs_omni", payload)
    print("bench_stock_vs_omni verdict: %s" % payload["verdict"])
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
