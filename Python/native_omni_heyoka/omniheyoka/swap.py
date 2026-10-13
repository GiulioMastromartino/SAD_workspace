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

__all__ = ["mirror_stock_config", "mirror_sync_configs", "task_effectors"]

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
                        thr_cmd_msg: Any = None,
                        object_index: int = 0,
                        rw_models: Any = None,
                        thr_models: Any = None,
                        fuel_models: Any = None,
                        use_gg: Any = None) -> dict:
    """Push hub/RW/thruster/gravity config from stock into ``oi``.

    ``thr_factory`` (simIncludeThruster factory) is required when a
    ``ThrusterDynamicEffector`` task model is found: geometry comes from its
    ``THRArrayConfigMsg``, and ``useMinPulseTime`` + per-thruster
    cutoff/blowdown/swirl are gated to the staircase envelope (Update-4:
    min-pulse off, cutoff pinned high, no lag/blowdown/swirl/ramps).
    ``thr_cmd_msg`` is the driver's ``THRArrayOnTimeCmdMsg``, subscribed to
    ``oi.thrOnTimeInMsg``.

    WP25-A4 multi-spacecraft: ``object_index`` selects the integrator's
    per-object config slot (C++ auto-grows it; that slot alone is marked
    mirrored+dirty, so object B never silently inherits object A's graph).
    ``object_index == 0`` with all explicit lists ``None`` is the legacy
    single-object path (task-list scan, byte-identical behavior). Any other
    combination is the per-object path and REQUIRES explicit
    ``rw_models``/``thr_models``/``fuel_models`` (pass ``[]`` when the
    object has none) plus explicit ``use_gg``: the task-list scan cannot
    attribute effectors to objects. The C++ per-object claim check
    (wheel-count, fuel-name, unknown-state rejects against THAT object's
    registered states) closes the attribution gap. Use
    :func:`mirror_sync_configs` for the full synced set. Returns the
    mirrored record for ledgers/tests.
    """
    legacy = (object_index == 0 and rw_models is None and thr_models is None
              and fuel_models is None and use_gg is None)
    if not legacy:
        return _mirror_single_object(sim, sc, oi, object_index,
                                     rw_models, thr_models, fuel_models,
                                     use_gg, thr_factory, thr_cmd_msg)
    rw_models = []
    thr_models = []
    fuel_models = []
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
        if cls == "FuelTank":
            fuel_models.append(model)
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

    n_wheels, fric_on = _mirror_wheels(rw_models, oi)

    n_thrusters, thr_isps = _mirror_thrusters(thr_models, thr_factory,
                                             thr_cmd_msg, oi)
    n_fuel = _mirror_fuel(fuel_models, oi, thr_isps)

    return {"mass": hub["mass"], "mu": mu, "use_gg": use_gg,
            "n_wheels": n_wheels, "n_thrusters": n_thrusters,
            "n_fuel": n_fuel, "friction_on": fric_on, "rBcB": hub["rBcB"],
            "object_index": 0}


def _reject_unmirrorable_tasks(sim: Any) -> None:
    """Global envelope scan for the per-object path: any dynamics task model
    outside the mirrorable set raises (same message as the legacy scan).
    RW/thruster/fuel/GravityGradient collections are NOT used here -- the
    per-object path trusts its explicit model lists (the C++ per-object
    claim check closes the attribution gap: wheel-count, fuel-name, and
    unknown-state rejects run against THAT object's registered states)."""
    for task_name, model in task_effectors(sim):
        cls = type(model).__name__
        if cls in _MIRRORED:
            continue
        if cls in ("GravityGradientEffector", "FuelTank"):
            continue
        bases = _bases(model)
        if "DynamicEffector" in bases or "StateEffector" in bases:
            raise ValueError(
                f"mirror_stock_config: task '{task_name}' holds "
                f"{cls}; envelope is hub + wheels + staircase "
                f"thrusters + central gravity + explicit omni env "
                f"(stock env effectors: use omni env setters instead).")


def _mirror_wheels(rw_models: list, oi: Any,
                   object_index: Any = None) -> tuple:
    """Mirror wheel records + u_current log links + Stribeck sets.

    ``object_index=None`` is the legacy slot-0 surface (identical calls);
    otherwise the per-object setters run for that slot alone (only it is
    marked mirrored+dirty). Returns (n_wheels, friction_on).
    """
    if len(rw_models) > 1:
        raise ValueError(
            f"mirror_stock_config: {len(rw_models)} RW effectors; stock "
            f"registration order is ambiguous -- use one effector.")
    n_wheels = 0
    fric_on = False
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
                    f"only BalancedWheels is mirrored (jitter: loud reject).")
            gs = _as_vec3(w.gsHat_B)
            if object_index is None:
                oi.addWheel(gs[0], gs[1], gs[2], float(w.Js))
                # u_current readback: timing-identical by construction.
                oi.linkWheelLog(n_wheels, logs[k])
            else:
                oi.addWheelForObject(object_index, gs[0], gs[1], gs[2],
                                     float(w.Js))
                oi.linkWheelLogForObject(object_index, n_wheels, logs[k])
            # Phase-2 friction: mirror the live Stribeck set when any
            # coefficient is active (factory-off zeroes fCoulomb only, so an
            # all-zero set means OFF and needs no records).
            fC = float(getattr(w, "fCoulomb", 0.0))
            fS = float(getattr(w, "fStatic", 0.0))
            cV = float(getattr(w, "cViscous", 0.0))
            bS = float(getattr(w, "betaStatic", -1.0))
            oL = float(getattr(w, "omegaLimitCycle", 0.0001))
            if fC != 0.0 or fS != 0.0 or cV != 0.0 or bS > 0.0:
                if object_index is None:
                    oi.setFrictionEnabled(True)
                    oi.setFrictionParams(n_wheels, fC, fS, bS, cV, oL)
                else:
                    oi.setFrictionEnabledForObject(object_index, True)
                    oi.setFrictionParamsForObject(object_index, n_wheels,
                                                  fC, fS, bS, cV, oL)
                fric_on = True
            n_wheels += 1
    return n_wheels, fric_on


def _mirror_single_object(sim: Any, sc: Any, oi: Any, idx: int,
                          rw_models: Any, thr_models: Any,
                          fuel_models: Any, use_gg: Any,
                          thr_factory: Any, thr_cmd_msg: Any) -> dict:
    """Per-object mirror (WP25-A4): explicit model lists into slot ``idx``.

    The task-list scan cannot attribute effectors to objects, so every list
    must be explicit (``[]`` when the object has none) and ``use_gg`` must
    be an explicit bool. Only slot ``idx`` is marked mirrored+dirty; the
    C++ build requires every dynPtr slot mirrored, so a forgotten object
    fails loudly instead of inheriting another object's graph.
    """
    if rw_models is None or thr_models is None or fuel_models is None:
        raise ValueError(
            f"mirror_stock_config: object {idx} needs explicit "
            f"rw_models/thr_models/fuel_models lists (pass [] when the "
            f"object has none); the task-list scan cannot attribute "
            f"effectors to objects, and object B must never silently "
            f"inherit object A's graph.")
    if use_gg is None:
        raise ValueError(
            f"mirror_stock_config: object {idx} needs explicit use_gg "
            f"(True/False); gravity-gradient task models cannot be "
            f"attributed to objects by the scan.")
    _reject_unmirrorable_tasks(sim)
    oi.setUseGravityGradientForObject(idx, bool(use_gg))

    hub = _hub_record(sc)
    oi.setMassForObject(idx, hub["mass"])
    oi.setInertiaForObject(idx, hub["Ixx"], hub["Iyy"], hub["Izz"],
                           hub["Ixy"], hub["Ixz"], hub["Iyz"])
    oi.setCoMOffsetForObject(idx, *hub["rBcB"])
    mu = _central_mu(sc)
    oi.setMuForObject(idx, mu)

    n_wheels, fric_on = _mirror_wheels(rw_models, oi, idx)
    n_thrusters, thr_isps = _mirror_thrusters(thr_models, thr_factory,
                                              thr_cmd_msg, oi, idx)
    n_fuel = _mirror_fuel(fuel_models, oi, thr_isps, idx)

    return {"mass": hub["mass"], "mu": mu, "use_gg": bool(use_gg),
            "n_wheels": n_wheels, "n_thrusters": n_thrusters,
            "n_fuel": n_fuel, "friction_on": fric_on, "rBcB": hub["rBcB"],
            "object_index": idx}


def mirror_sync_configs(sim: Any, oi: Any, specs: list) -> list:
    """Mirror EVERY object of a synced set into ONE integrator (WP25-A4).

    ``specs`` is one dict per object, in dynPtrs order (leader first):
    ``{"sc", "rw_models", "thr_models", "thr_factory", "thr_cmd_msg",
    "fuel_models", "use_gg"}``. All of ``rw_models``/``thr_models``/
    ``fuel_models``/``use_gg`` must be explicit (no scan attribution).
    Recipe after this call::

        scA.setIntegrator(oi)
        scA.syncDynamicsIntegration(scB)  # B follows A (upstream
        # dynamicObject.cpp: pushes B into A's integrator, marks B synced)

    Returns the per-object mirrored records.
    """
    if not specs:
        raise ValueError("mirror_sync_configs: need >= 1 object spec.")
    out = []
    for idx, spec in enumerate(specs):
        for key in ("rw_models", "thr_models", "fuel_models", "use_gg"):
            if spec.get(key) is None:
                raise ValueError(
                    f"mirror_sync_configs: object {idx} spec needs explicit "
                    f"'{key}' (object B must never silently inherit object "
                    f"A's graph).")
        out.append(mirror_stock_config(
            sim, spec["sc"], oi, object_index=idx,
            thr_factory=spec.get("thr_factory"),
            thr_cmd_msg=spec.get("thr_cmd_msg"),
            rw_models=spec["rw_models"], thr_models=spec["thr_models"],
            fuel_models=spec["fuel_models"], use_gg=spec["use_gg"]))
    return out


def _mirror_thrusters(thr_models: list, thr_factory: Any,
                      thr_cmd_msg: Any, oi: Any,
                      object_index: Any = None) -> tuple:
    """Mirror thruster geometry + gate the staircase envelope.

    Returns (count, steadyIsp-per-thruster) for the fuel mdot law.
    ``object_index=None`` is the legacy slot-0 surface; otherwise the
    per-object setters/subscribes run for that slot alone.
    """
    if not thr_models:
        return 0, []
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
    isps = []
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
        # Cutoff is INERT without ramps (measured bit-exact 10 vs 1000
        # over fire + shutoff); accepted unconditionally, never modeled.
        # Ramps stay rejected, and cutoff must be modeled WITH them if
        # ramps ever land (M2 composition). cutoff <= 0 still rejected
        # (degenerate stock behavior unknown and unmeasured).
        _cf = float(getattr(d, "cutoffFrequency", 1000.0))
        if not _cf > 0.0:
            raise ValueError(
                "mirror_stock_config: cutoffFrequency "
                f"{getattr(d, 'cutoffFrequency', '?')} <= 0; degenerate "
                "stock behavior unknown.")
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
        if object_index is None:
            oi.addThruster(th[0], th[1], th[2], loc[0], loc[1], loc[2],
                           float(cfg.MaxThrust))
        else:
            oi.addThrusterForObject(object_index, th[0], th[1], th[2],
                                    loc[0], loc[1], loc[2],
                                    float(cfg.MaxThrust))
        isps.append(float(cfg.steadyIsp))
        n += 1
    if thr_cmd_msg is None:
        raise ValueError(
            "mirror_stock_config: thrusters mirrored but no thr_cmd_msg "
            "given; pass the driver's THRArrayOnTimeCmdMsg.")
    if object_index is None:
        oi.thrOnTimeInMsg.subscribeTo(thr_cmd_msg)
    else:
        oi.subscribeThrCmdForObject(object_index, thr_cmd_msg)
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
    if object_index is None:
        oi.setInitialThrusterCmd(_snap)
    else:
        oi.setInitialThrusterCmdForObject(object_index, _snap)
    return n, isps


def _mirror_fuel(fuel_models: list, oi: Any, thr_isps: list,
                 object_index: Any = None) -> int:
    """Mirror a single fuel tank (Phase-2 depletion, v1 scope).

    Reads the stock registered state name, tank center, and leak rate;
    thruster Isps (mirrored order) feed the mdot law. Guards: more than one
    tank raises (per-tank thruster subsets unmapped); a tank with NO
    mirrored thrusters raises (nothing drains it -- likely a missed
    addThrusterSet, fail loud rather than bookkeep zeros).
    Mass/I_C feedback + live aggregate c (per-tick; see swap fuel gate).
    ``object_index=None`` is the legacy slot-0 surface; otherwise the
    per-object setters run for that slot alone.
    """
    if not fuel_models:
        return 0
    if len(fuel_models) > 1:
        raise ValueError(
            f"mirror_stock_config: {len(fuel_models)} fuel tanks; v1 "
            f"mirrors exactly one (per-tank thruster subsets unmapped).")
    tank = fuel_models[0]
    if not thr_isps:
        raise ValueError(
            "mirror_stock_config: fuel tank with no mirrored thrusters; "
            "nothing drains it (missed addThrusterSet or thruster-less sim).")
    name = str(tank.getNameOfMassState())
    rtb = _as_vec3(tank.getR_TB_B())
    leak = float(tank.getFuelLeakRate())
    if object_index is None:
        oi.setFuelStateName(name)
        oi.setFuelTank(rtb[0], rtb[1], rtb[2], leak)
        oi.setThrusterIsps([float(v) for v in thr_isps])
    else:
        oi.setFuelStateNameForObject(object_index, name)
        oi.setFuelTankForObject(object_index, rtb[0], rtb[1], rtb[2], leak)
        oi.setThrusterIspsForObject(object_index,
                                    [float(v) for v in thr_isps])
    return 1
