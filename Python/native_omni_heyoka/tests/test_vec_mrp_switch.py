"""WP25-B2.5 vec MRP switch count (Phase-3 FSW dependency).

Stock owns the shadow-set switch (postIntegration); the vec integrator only
COUNTS crossings per tick (|sigma|^2 > 1 post-propagation, same threshold as
the SysModel path), exposed via ``OmniVecIntegrator::getMRPSwitchCount()``.
The swap path "publishes" by per-tick polling (helper poller below -- the
scatter-hook vs helper-publish decision, probe-first: scatter-hook counting
+ helper-side trace collection).

Gate (PLAN_PHASE25.md WP25-B2.5): leg-total equality +-1 with the tick trace
RECORDED (``audit_bundle/results/vec_mrp_switch_trace.json``). Tick-exact
equality is NOT required -- saturation-edge timing jitter can legitimately
flip individual ticks; flag it if observed, do not force it.

Host-only sims via importorskip: skipped (no failure) without Basilisk or
the built wheel.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
TRACE_JSON = REPO / "audit_bundle" / "results" / "vec_mrp_switch_trace.json"

MU = 3.986004418e14
MASS = 750.0
IXX, IYY, IZZ = 900.0, 800.0, 600.0
DT = 0.1
T_LEG = 30.0  # 0.6 rad/s tumble: crossings near theta = 180, 540 deg
OMEGA_TUMBLE = (0.6, 0.0, 0.0)


def _stock_tumble_sim(tag, use_omni):
    """Hub-only stock tumble sim; vec arm mirrors + swaps the integrator."""
    pytest.importorskip("numpy")
    pytest.importorskip("Basilisk.architecture.messaging")
    oh = pytest.importorskip("omniheyoka.omni_heyoka")
    from Basilisk.simulation import spacecraft
    from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                    simIncludeGravBody)

    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(DT)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = tag
    sc.hub.mHub = MASS
    sc.hub.IHubPntBc_B = [[IXX, 0.0, 0.0], [0.0, IYY, 0.0], [0.0, 0.0, IZZ]]
    oe = orbitalMotion.ClassicElements()
    oe.a, oe.e = 1.0e7, 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega, oe.omega, oe.f = (48.2 * macros.D2R, 347.8 * macros.D2R,
                                85.3 * macros.D2R)
    r0, v0 = orbitalMotion.elem2rv(MU, oe)
    sc.hub.r_CN_NInit, sc.hub.v_CN_NInit = list(r0), list(v0)
    sc.hub.sigma_BNInit = [0.0, 0.0, 0.0]
    sc.hub.omega_BN_BInit = list(OMEGA_TUMBLE)
    sim.AddModelToTask("t", sc)
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    grav.addBodiesTo(sc)

    oi = None
    poller = None
    if use_omni:
        from omniheyoka.omni_heyoka import OmniVecIntegrator
        from omniheyoka.swap import mirror_stock_config

        oi = OmniVecIntegrator(sc)
        mirror_stock_config(sim, sc, oi)
        sc.setIntegrator(oi)
        if not callable(getattr(oi, "getMRPSwitchCount", None)):
            pytest.skip("WP25-B2.5 C++ spec not yet applied")
        poller = _SwitchPoller(oi)
        sim.AddModelToTask("t", poller)
    sc_log = sc.scStateOutMsg.recorder()
    sim.AddModelToTask("t", sc_log)
    return sim, sc, sc_log, macros, oi, poller


def _SwitchPoller(oi):
    """Per-tick poller model: appends oi.getMRPSwitchCount() to .trace."""
    from Basilisk.architecture import sysModel

    class Poller(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self._oi = oi
            self.trace = []

        def Reset(self, t):
            self.trace = []

        def UpdateState(self, t):
            self.trace.append(int(self._oi.getMRPSwitchCount()))

    return Poller()


def _run_leg(use_omni):
    import numpy as np

    tag = "vec-switch" if use_omni else "rk-switch"
    sim, sc, sc_log, macros, oi, poller = _stock_tumble_sim(tag, use_omni)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(T_LEG))
    sim.ExecuteSimulation()
    stock_trace = [int(v) for v in
                   np.asarray(sc_log.MRPSwitchCount).ravel().tolist()]
    vec_trace = list(poller.trace) if poller is not None else None
    return stock_trace, vec_trace


def test_vec_mrp_switch_leg_total_pm1_with_trace():
    """B2.5 gate: vec leg-total == stock leg-total +-1; tick trace recorded."""
    stock_trace, _ = _run_leg(use_omni=False)
    _, vec_trace = _run_leg(use_omni=True)
    # Re-run shape: stock arm total + vec arm (trace, total).
    stock_total = int(stock_trace[-1]) if stock_trace else 0
    assert stock_total >= 2, (
        f"tumble leg never switched (total={stock_total}) -- vacuous gate")
    assert vec_trace is not None and len(vec_trace) == len(stock_trace), (
        len(vec_trace or []), len(stock_trace))
    vec_total = int(vec_trace[-1])
    payload = {
        "dt": DT,
        "t_leg": T_LEG,
        "omega_BN_BInit": list(OMEGA_TUMBLE),
        "stock_trace": stock_trace,
        "vec_trace": vec_trace,
        "stock_total": stock_total,
        "vec_total": vec_total,
        "gate": "abs(vec-stock) <= 1",
    }
    TRACE_JSON.write_text(json.dumps(payload, indent=2))
    assert abs(vec_total - stock_total) <= 1, (vec_total, stock_total)
    # Tick-exact equality is NOT forced (saturation-edge timing jitter may
    # flip individual ticks); record-and-flag only.
    mism = [i for i, (a, b) in enumerate(zip(stock_trace, vec_trace)) if a != b]
    if mism:
        print(f"\n[mrp-switch] {len(mism)} tick mismatches (allowed): {mism[:10]}")
