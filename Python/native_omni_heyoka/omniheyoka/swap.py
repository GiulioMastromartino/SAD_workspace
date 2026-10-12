"""Stock-to-omni mirror helper for the integrator swap (Phase 1).

Division of labour (load-bearing, follows the SDK boundary):
- THIS helper sees stock OBJECTS via the sim task lists: hub record, wheel
  gs/Js/limits, central mu, and REJECTS anything unmirrorable. Neither C++
  (no ``spacecraft.h`` in bsk_sdk) nor naive attribute walks (``sc.states``
  / ``sc.dynEffectors`` are opaque SWIG vectors) can enumerate attached
  effectors -- the task model lists are the one complete, typed census.
- C++ sees stock STATES by name (DynParamManager): census + airtight claim
  check + gather/propagate/scatter. It cross-checks the mirrored wheel
  count against ``reactionWheelOmegas`` rows.
- Per-tick RW torques are read back from the stock wheel-log messages
  (``rwOutMsgs``), published from the same ``u_current`` stock integrates:
  timing-identical under every task order. Thruster duties flow through a
  staged on-time mailbox with sub-step arcs (staircase only).

Swap recipe (RW + thruster + central-gravity envelope)::

    from omniheyoka.omni_heyoka import OmniVecIntegrator
    from omniheyoka.swap import mirror_stock_config
    oi = OmniVecIntegrator(sc)
    mirror_stock_config(sim, sc, oi, thr_factory=tf, thr_cmd_msg=drv.out)
    sc.setIntegrator(oi)

Everything unmirrorable raises :class:`ValueError` before the sim runs --
never a silent divergence.
"""

from __future__ import annotations

from typing import Any

__all__ = ["mirror_stock_config", "task_effectors"]

# Task models that are NOT dynamics: anything outside Basilisk.simulation
# (FSW, recorders, drivers) is none of our business. Inside it, only these
# classes are mirrorable; every other *simulation* module (drag, SRP,
# fuel, ...) raises.
_MIRRORED = {"Spacecraft", "ReactionWheelStateEffector",
             "ThrusterDynamicEffector"}


def task_effectors(sim: Any) -> list:
    """All task models in every task, in task order: [(taskName, model)]."""
    out = []
    for task in list(sim.TaskList):
        for model in list(task.TaskModels):
            out.append((str(task.Name), model))
    return out


def _hub_record(sc: Any) -> dict:
    h = sc.hub
    I = [[float(h.IHubPntBc_B[i][j]) for j in range(3)] for i in range(3)]
    return {
        "mass": float(h.mHub),
        "Ixx": I[0][0], "Iyy": I[1][1], "Izz": I[2][2],
        "Ixy": I[0][1], "Ixz": I[0][2], "Iyz": I[1][2],
        "rBcB": _as_vec3(h.r_BcB_B),
    }


def _central_mu(sc: Any) -> float:
    """Central-body mu from the stock gravity field; single-body enforced."""
    bodies = list(sc.gravField.gravBodies)
    centrals = [b for b in bodies if bool(getattr(b, "isCentralBody", False))]
    if len(bodies) != 1 or len(centrals) != 1:
        raise ValueError(
            f"mirror_stock_config: stage 1 needs exactly one central body, "
            f"found {len(bodies)} bodies ({len(centrals)} central). "
            f"N-body gravity is Phase-2/env work.")
    return float(centrals[0].mu)


def _as_vec3(v: Any) -> list:
    import numpy as np
    return [float(x) for x in np.asarray(v, dtype=float).ravel()[:3]]


def _bases(model: Any) -> set:
    return {c.__name__ for c in type(model).__mro__}


def mirror_stock_config(sim: Any, sc: Any, oi: Any,
                        thr_factory: Any = None,
                        thr_cmd_msg: Any = None) -> dict:
    """Push hub/RW/thruster/gravity config from stock into ``oi``.

    ``thr_factory`` (simIncludeThruster factory) is required when a
    ``ThrusterDynamicEffector`` task model is found: geometry comes from its
    ``THRArrayConfigMsg``, and ``useMinPulseTime`` + per-thruster
    cutoff/blowdown/swirl are gated to the staircase envelope (Update-4:
    min-pulse off, cutoff pinned high, no lag/blowdown/swirl/ramps).
    ``thr_cmd_msg`` is the driver's ``THRArrayOnTimeCmdMsg``, subscribed to
    ``oi.thrOnTimeInMsg``.

    Returns the mirrored record for ledgers/tests.
    """
    rw_models = []
    thr_models = []
    use_gg = False
    for task_name, model in task_effectors(sim):
        cls = type(model).__name__
        if cls in _MIRRORED:
            if cls == "ReactionWheelStateEffector":
                rw_models.append(model)
            elif cls == "ThrusterDynamicEffector":
                thr_models.append(model)
            continue
        if cls == "GravityGradientEffector":
            use_gg = True
            continue
        # Anything else that IS dynamics (SWIG MRO carries the
        # DynamicEffector/StateEffector bases) is unmirrorable and raises.
        # Pure field/model modules (atmosphere, ephemeris, ...) are not
        # dynamics and pass through.
        bases = _bases(model)
        if "DynamicEffector" in bases or "StateEffector" in bases:
            raise ValueError(
                f"mirror_stock_config: task '{task_name}' holds "
                f"{cls}; envelope is hub + wheels + staircase "
                f"thrusters + central gravity + explicit omni env "
                f"(stock env effectors: use omni env setters instead).")
    oi.setUseGravityGradient(use_gg)
    if len(rw_models) > 1:
        raise ValueError(
            f"mirror_stock_config: {len(rw_models)} RW effectors; stock "
            f"registration order is ambiguous -- use one effector.")

    hub = _hub_record(sc)
    oi.setMass(hub["mass"])
    oi.setInertia(hub["Ixx"], hub["Iyy"], hub["Izz"],
                  hub["Ixy"], hub["Ixz"], hub["Iyz"])
    oi.setCoMOffset(*hub["rBcB"])
    mu = _central_mu(sc)
    oi.setMu(mu)

    n_wheels = 0
    for eff in rw_models:
        wheels = list(eff.ReactionWheelData)
        logs = list(eff.rwOutMsgs)
        if len(logs) < len(wheels):
            raise ValueError(
                "mirror_stock_config: RW effector has fewer rwOutMsgs than "
                "wheels; cannot link u_current readback.")
        for k, w in enumerate(wheels):
            model = str(getattr(w, "RWModel", 0))
            if model not in ("0", "RWModels.BalancedWheels", "BalancedWheels"):
                raise ValueError(
                    f"mirror_stock_config: wheel {n_wheels} RWModel={model}; "
                    f"stage 1 supports BalancedWheels only (jitter: Phase 2).")
            if (float(getattr(w, "fCoulomb", 0.0)) != 0.0
                    or float(getattr(w, "fStatic", 0.0)) != 0.0
                    or float(getattr(w, "cViscous", 0.0)) != 0.0
                    or float(getattr(w, "betaStatic", -1.0)) > 0.0):
                raise ValueError(
                    f"mirror_stock_config: wheel {n_wheels} has friction "
                    f"coefficients set; friction mirroring is Phase 2.")
            gs = _as_vec3(w.gsHat_B)
            oi.addWheel(gs[0], gs[1], gs[2], float(w.Js))
            # u_current readback: timing-identical by construction (see header).
            oi.linkWheelLog(n_wheels, logs[k])
            n_wheels += 1

    n_thrusters = _mirror_thrusters(thr_models, thr_factory, thr_cmd_msg, oi)

    return {"mass": hub["mass"], "mu": mu, "use_gg": use_gg,
            "n_wheels": n_wheels, "n_thrusters": n_thrusters,
            "rBcB": hub["rBcB"]}


def _mirror_thrusters(thr_models: list, thr_factory: Any,
                      thr_cmd_msg: Any, oi: Any) -> int:
    """Mirror thruster geometry + gate the staircase envelope. Returns count."""
    if not thr_models:
        return 0
    if len(thr_models) > 1:
        raise ValueError(
            f"mirror_stock_config: {len(thr_models)} thruster effectors; "
            f"stock registration order is ambiguous -- use one effector.")
    if thr_factory is None:
        raise ValueError(
            "mirror_stock_config: thruster effector found but no "
            "thr_factory given; its THRSimConfig list carries MaxThrust.")
    eff = thr_models[0]
    # Geometry join: factory THRSimConfig values carry MaxThrust (absent
    # from thrusterData); effector thrusterData carries the live loc/dir.
    # Joined by INDEX with an exact loc/dir cross-check (both append in
    # create/add order; any deviation raises). NOTE: `.label` reads abort
    # (SWIG string issue observed) -- never touch labels.
    cfgs = list(getattr(thr_factory, "thrusterList", {}).values())
    data = list(getattr(eff, "thrusterData", []))
    if not cfgs or len(cfgs) != len(data):
        raise ValueError(
            "mirror_stock_config: factory configs "
            f"({len(cfgs)}) vs effector thrusters ({len(data)}) mismatch; "
            "geometry join needs the creating factory.")
    n = 0
    for cfg, d in zip(cfgs, data):
        loc = _as_vec3(d.thrLoc_B)
        th = _as_vec3(d.thrDir_B)
        if loc != _as_vec3(cfg.thrLoc_B) or th != _as_vec3(cfg.thrDir_B):
            raise ValueError(
                "mirror_stock_config: factory/effector order mismatch; "
                "cannot join geometry safely.")
        if float(getattr(cfg, "MinOnTime", 0.0)) > 0.0:
            raise ValueError(
                "mirror_stock_config: MinOnTime="
                f"{getattr(cfg, 'MinOnTime', '?')} > 0; stage 2 covers the "
                "staircase envelope only (create(useMinPulseTime=False)).")
        if float(getattr(d, "cutoffFrequency", 1000.0)) < 500.0:
            raise ValueError(
                "mirror_stock_config: cutoffFrequency "
                f"{getattr(d, 'cutoffFrequency', '?')} < 500 rad/s; stage 2 "
                "has no lag model (pin cutoffFrequency=1000).")
        for attr in ("thrBlowDownCoeff", "ispBlowDownCoeff"):
            try:
                if len(list(getattr(d, attr) or [])) > 0:
                    raise ValueError(
                        f"mirror_stock_config: {attr} set; blowdown is "
                        "out of the staircase envelope (M6).")
            except TypeError:
                pass
        if float(getattr(cfg, "MaxSwirlTorque", 0.0)) != 0.0:
            raise ValueError(
                "mirror_stock_config: MaxSwirlTorque != 0; swirl is out of "
                "the staircase envelope.")
        oi.addThruster(th[0], th[1], th[2], loc[0], loc[1], loc[2],
                       float(cfg.MaxThrust))
        n += 1
    if thr_cmd_msg is None:
        raise ValueError(
            "mirror_stock_config: thrusters mirrored but no thr_cmd_msg "
            "given; pass the driver's THRArrayOnTimeCmdMsg.")
    oi.thrOnTimeInMsg.subscribeTo(thr_cmd_msg)
    # Pre-init command snapshot (Reset-equivalent staging for tick 0):
    # a pre-written command (write-once path) fires tick 0; an unwritten
    # message reads zeros (fresh fill path, idles tick 0). Snapshot NOW
    # (mirror time = pre-init state); a resetting driver + static command
    # is contradictory input (documented).
    from Basilisk.architecture.messaging import THRArrayOnTimeCmdMsgReader
    _snap_reader = THRArrayOnTimeCmdMsgReader()
    _snap_reader.subscribeTo(thr_cmd_msg)
    try:
        _snap = [float(v) for v in list(_snap_reader().OnTimeRequest)]
    except Exception:
        _snap = [0.0] * 36
    oi.setInitialThrusterCmd(_snap)
    return n
