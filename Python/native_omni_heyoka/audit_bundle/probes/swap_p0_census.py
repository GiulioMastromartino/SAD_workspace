"""Phase-0 probe 2 (swap): stock registered-state census + scatter round-trip.

Builds stock sims (hub-only, hub+3 BalancedWheels RW, hub+fuelTank),
initializes, and dumps every registered state {name, rows, cols, values}
via DynParamManager.getStateObject. Then round-trips each state through
setState(same-values) and asserts bit-exact readback (the gather/scatter
contract OmniVecIntegrator will reuse).

Run: ../../env/bin/python audit_bundle/probes/swap_p0_census.py
Writes: audit_bundle/results/swap_p0_census.json
Host-only (needs Basilisk macOS build); writes status=skipped otherwise.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import log_text, save_result, safe_import_basilisk  # noqa: E402

CANDIDATE_NAMES = [
    # hub (Spacecraft/hubEffector registration)
    "hubPosition", "hubVelocity", "hubSigma", "hubOmega",
    # reaction wheels (reactionWheelStateEffector)
    "reactionWheelOmegas", "reactionWheelThetas",
    # fuel tank (fuelTank state effector; default name is numbered, e.g.
    # fuelTankMass2 — the probe sets an explicit name, both recorded)
    "fuelMass", "fuelTankMass2",
]


def _build_hub_only():
    from Basilisk.simulation import spacecraft
    from Basilisk.utilities import SimulationBaseClass, macros
    sim = SimulationBaseClass.SimBaseClass()
    sim.SetProgressBar(False)
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(0.1)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = "swap-p0-hub"
    sc.hub.mHub = 750.0
    sc.hub.IHubPntBc_B = [[900., 0., 0.], [0., 800., 0.], [0., 0., 600.]]
    sc.hub.r_CN_NInit = [1.0e7, 0.0, 0.0]
    sc.hub.v_CN_NInit = [0.0, 7500.0, 0.0]
    sc.hub.sigma_BNInit = [0.1, -0.2, 0.3]
    sc.hub.omega_BN_BInit = [0.01, -0.02, 0.03]
    sim.AddModelToTask("t", sc)
    return sim, sc


def _build_hub_rw():
    from Basilisk.simulation import reactionWheelStateEffector
    from Basilisk.utilities import simIncludeRW
    sim, sc = _build_hub_only()
    sc.ModelTag = "swap-p0-rw"
    rw_factory = simIncludeRW.rwFactory()
    for gs in ([1., 0., 0.], [0., 1., 0.], [0., 0., 1.]):
        rw_factory.create("Honeywell_HR16", list(gs),
                          maxMomentum=50.0, Omega=100.0)
    rw_eff = reactionWheelStateEffector.ReactionWheelStateEffector()
    rw_factory.addToSpacecraft("rw", rw_eff, sc)
    sim.AddModelToTask("t", rw_eff)  # proven pattern (wp1_sat): Reset/
    # UpdateState (ReadInputs/ConfigureRWRequests) run from the task
    return sim, sc


def _build_hub_fuel():
    from Basilisk.simulation import fuelTank
    sim, sc = _build_hub_only()
    sc.ModelTag = "swap-p0-fuel"
    model = fuelTank.FuelTankModelConstantVolume()
    model.propMassInit = 10.0
    tank = fuelTank.FuelTank()
    tank.setTankModel(model)
    tank.setNameOfMassState("fuelMass")
    sc.addStateEffector(tank)
    sim.AddModelToTask("t", tank)
    return sim, sc


def _census(sc):
    import numpy as np
    rows = []
    for name in CANDIDATE_NAMES:
        so = sc.dynManager.getStateObject(name)
        if so is None:
            continue
        val = np.asarray(so.getState(), dtype=float)
        rows.append({"name": name,
                     "rows": int(so.getRowSize()),
                     "cols": int(so.getColumnSize()),
                     "values": val.ravel().tolist()})
    return rows


def _roundtrip(sc, rows):
    """setState(same values) then read back; max abs delta must be 0."""
    import numpy as np
    worst = 0.0
    for r in rows:
        so = sc.dynManager.getStateObject(r["name"])
        v0 = np.asarray(so.getState(), dtype=float)
        so.setState(v0)
        v1 = np.asarray(so.getState(), dtype=float)
        worst = max(worst, float(np.abs(v1 - v0).max(initial=0.0)))
    return worst


def main() -> int:
    if not safe_import_basilisk():
        save_result("swap_p0_census",
                    {"status": "skipped", "reason": "Basilisk not importable"})
        return 0
    import numpy as np  # noqa: F401
    payload: dict = {"status": "ok", "configs": {}}
    for tag, builder in (("hub_only", _build_hub_only),
                         ("hub_3rw", _build_hub_rw),
                         ("hub_fuel", _build_hub_fuel)):
        sim, sc = builder()
        sim.InitializeSimulation()
        rows = _census(sc)
        worst = _roundtrip(sc, rows)
        total = int(sum(r["rows"] * r["cols"] for r in rows))
        payload["configs"][tag] = {
            "states": rows,
            "n_states_total": total,
            "roundtrip_max_abs_delta": worst,
            "roundtrip_bitexact": bool(worst == 0.0),
        }
        log_text("swap_p0_census",
                 f"[{tag}] n={total} roundtrip_delta={worst:.3e} "
                 f"names={[r['name'] for r in rows]}")
        print(f"[{tag}] states={total} "
              f"names={[r['name'] for r in rows]} "
              f"roundtrip_delta={worst:.1e}")
    bad = [t for t, c in payload["configs"].items()
           if not c["roundtrip_bitexact"]]
    payload["verdict"] = ("census+roundtrip green"
                          if not bad else f"ROUNDTRIP FAIL: {bad}")
    if bad:
        payload["status"] = "fail"
    save_result("swap_p0_census", payload)
    return 0 if not bad else 1


if __name__ == "__main__":
    raise SystemExit(main())
