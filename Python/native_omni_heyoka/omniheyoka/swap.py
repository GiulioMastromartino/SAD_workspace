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
# (FSW, recorders, drivers) is none of our business. Inside it, the hub +
# wheels + staircase-thruster classes below are mirrorable; every other
# dynamics module is either routed to an env reader (_MIRRORED_ENV) or
# raises.
_MIRRORED = {"Spacecraft", "ReactionWheelStateEffector",
             "ThrusterDynamicEffector"}

# Env-effector classes routed to the PLAN_ENV_MIRROR readers (list.md §A1
# wiring batch) instead of the generic dynamics reject below. _MIRRORED
# itself stays exactly hub + wheels + thrusters (the probe parses that
# literal, and staged suites pin it); this ADJACENT set is the rest of the
# mirrorable surface. Every member is consumed by exactly one _mirror_*
# family slice: routed classes never hit the generic "use omni env
# setters instead" reject. Members that stay unmirrorable (facet drag,
# message-driven SRP, non-cannonball selections, ...) raise their
# reader's specific reject instead -- still loud, never silent (§1.2).
_MIRRORED_ENV = {"DragDynamicEffector", "FacetDragDynamicEffector",
                 "RadiationPressure", "FacetSRPDynamicEffector",
                 "FacetedSRPEffector", "FacetedSpacecraftProjectedArea",
                 "MtbEffector", "GravityEffector",
                 "ExtForceTorque", "ThrusterStateEffector"}

# Census slices: task-model class names routed per family (PLAN §7.1:
# dispatch on type(model).__name__, never string-typed surfaces).
# Field modules (atmosphere, wind, eclipse, magnetic-field, ephemeris)
# are plain SysModels, not dynamics: they pass through the task
# untouched (§1.1) and are only COLLECTED here so the readers can wire
# their out-messages.
_DRAG_FAMILY = {"DragDynamicEffector", "FacetDragDynamicEffector"}
_SRP_FAMILY = {"RadiationPressure", "FacetSRPDynamicEffector",
               "FacetedSRPEffector", "FacetedSpacecraftProjectedArea"}
_MTB_FAMILY = {"MtbEffector"}
_MAGFIELD_FAMILY = {"MagneticFieldCenteredDipole"}
_ECLIPSE_FAMILY = {"Eclipse"}
_GRAV_FAMILY = {"GravityEffector"}
# PLAN §6.3 stay-in-task providers: recorded, never removed/reconfigured.
_EPHEM_FAMILY = {"SpiceInterface", "PlanetEphemeris",
                 "EphemerisConverter"}
_ATMO_FAMILY = {"ExponentialAtmosphere", "MsisAtmosphere",
                "TabularAtmosphere"}
_WIND_FAMILY = {"ZeroWindModel"}

# Per-object env lists that MUST be explicit (WP25-A4 rule): omitting one
# could silently drop dynamics no C++ claim check can see (env leaves no
# named states). Atmo/wind/ephemeris slices are ledger-only (their links
# are recovered exactly from the stock readers), so None means [] there.
_REQUIRED_ENV_KEYS = (("drag_models", "drag"),
                      ("srp_models", "SRP"),
                      ("eclipse_models", "eclipse"),
                      ("mtb_models", "MTB"),
                      ("magfield_models", "magnetic-field"),
                      ("grav_models", "gravity"))

_STAGED_CACHE: dict = {}

# §7.3 census note, carried in every env record: mirrored env EFFECTORS
# are routed out of the dynamics census into omni setters, but their
# task entries are NOT removed (same as the RW/thruster/fuel precedent
# -- the swap never mutates task lists). The stock object stays
# attached to ``sc`` yet no longer drives propagation (``sc``
# detachment): its per-tick force still computes stock-side but feeds
# nothing omni-side. Field modules STAY fully live: only their
# out-message links are forwarded, never removed or reconfigured.
_TASK_NOTE = ("routed: stock task entry retained (never mutated); "
              "effector dynamics superseded by omni setters (sc "
              "detachment); field-module links forwarded only")


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
    """Central-body mu from the stock gravity field.

    Exactly one CENTRAL body is required; non-central (ephemeris-driven)
    bodies are legal since the env-bodies wiring (PLAN_ENV_MIRROR §6.1,
    list.md §A1) and mirror via addEnvironmentBody, not here.
    """
    bodies = list(sc.gravField.gravBodies)
    centrals = [b for b in bodies if bool(getattr(b, "isCentralBody", False))]
    if len(centrals) != 1:
        raise ValueError(
            f"mirror_stock_config: stage 1 needs exactly one central body, "
            f"found {len(bodies)} bodies ({len(centrals)} central). "
            f"Extra bodies mirror via addEnvironmentBody (env reader).")
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
                        use_gg: Any = None,
                        drag_models: Any = None,
                        atmo_models: Any = None,
                        wind_models: Any = None,
                        srp_models: Any = None,
                        eclipse_models: Any = None,
                        mtb_models: Any = None,
                        magfield_models: Any = None,
                        grav_models: Any = None,
                        ephemeris_models: Any = None,
                        srp_facet_specs: Any = None,
                        srp_mode: Any = None,
                        ext_torque_msg: Any = None,
                        ext_force_body_msg: Any = None,
                        ext_force_inertial_msg: Any = None,
                        ext_models: Any = None) -> dict:
    """Push hub/RW/thruster/gravity config from stock into ``oi``.

    ``thr_factory`` (simIncludeThruster factory) is required when a
    ``ThrusterDynamicEffector`` task model is found: geometry comes from its
    ``THRArrayConfigMsg``, and ``useMinPulseTime`` + per-thruster
    cutoff/blowdown/swirl are gated to the staircase envelope (Update-4:
    min-pulse off, cutoff pinned high, no lag/blowdown/swirl/ramps).
    ``thr_cmd_msg`` is the driver's ``THRArrayOnTimeCmdMsg``, subscribed to
    ``oi.thrOnTimeInMsg``.

    Env wiring (list.md §A1, PLAN_ENV_MIRROR §8): the task census is
    sliced per family and each slice delegates to its staged reader's
    public ``mirror_*_models`` surface (drag §2, SRP §3, mag/MTB §4,
    eclipse §5, env-bodies §6). Mirrored env effectors leave the
    dynamics census for omni setters (their task entries are NOT
    removed -- same precedent as wheels/thrusters/fuel); field modules
    (atmosphere, wind, eclipse, magnetic-field, ephemeris) stay in task
    and only gain forwarded out-message links. ``srp_facet_specs`` /
    ``srp_mode`` are the staged SRP reader's caller-passed geometry and
    mode selection (the stock C++ keeps both private); ``None`` is the
    stock-default cannonball path.

    WP25-A4 multi-spacecraft: ``object_index`` selects the integrator's
    per-object config slot (C++ auto-grows it; that slot alone is marked
    mirrored+dirty, so object B never silently inherits object A's graph).
    ``object_index == 0`` with all explicit lists ``None`` is the legacy
    single-object path (task-list scan, byte-identical behavior). Any other
    combination is the per-object path and REQUIRES explicit
    ``rw_models``/``thr_models``/``fuel_models`` (pass ``[]`` when the
    object has none) plus explicit ``use_gg``: the task-list scan cannot
    attribute effectors to objects. The env dynamics lists
    (``drag_models``, ``srp_models``, ``eclipse_models``,
    ``mtb_models``, ``magfield_models``, ``grav_models``) are likewise
    required explicit (env leaves no named states for the C++ per-object
    claim check to verify); the ledger-only field slices
    (``atmo_models``, ``wind_models``, ``ephemeris_models``) default to
    ``[]``. The C++ per-object claim check (wheel-count, fuel-name,
    unknown-state rejects against THAT object's registered states)
    closes the attribution gap. Use
    :func:`mirror_sync_configs` for the full synced set. Returns the
    mirrored record for ledgers/tests.
    """
    legacy = (object_index == 0 and rw_models is None and thr_models is None
              and fuel_models is None and use_gg is None
              and drag_models is None and atmo_models is None
              and wind_models is None and srp_models is None
              and eclipse_models is None and mtb_models is None
              and magfield_models is None and grav_models is None
              and ephemeris_models is None and srp_facet_specs is None
              and srp_mode is None)
    if not legacy:
        return _mirror_single_object(sim, sc, oi, object_index,
                                     rw_models, thr_models, fuel_models,
                                     use_gg, thr_factory, thr_cmd_msg,
                                     drag_models, atmo_models, wind_models,
                                     srp_models, eclipse_models, mtb_models,
                                     magfield_models, grav_models,
                                     ephemeris_models, srp_facet_specs,
                                     srp_mode)
    rw_models = []
    thr_models = []
    thr_state_models = []
    fuel_models = []
    drag_models = []
    atmo_models = []
    wind_models = []
    srp_models = []
    eclipse_models = []
    mtb_models = []
    magfield_models = []
    grav_models = []
    ephemeris_models = []
    ext_models = []
    use_gg = False
    for task_name, model in task_effectors(sim):
        cls = type(model).__name__
        if cls in _MIRRORED:
            if cls == "ReactionWheelStateEffector":
                rw_models.append(model)
            elif cls == "ThrusterDynamicEffector":
                thr_models.append(model)
            elif cls == "ThrusterStateEffector":
                thr_state_models.append(model)
            continue
        if cls == "GravityGradientEffector":
            use_gg = True
            continue
        if cls == "FuelTank":
            fuel_models.append(model)
            continue
        if cls == "ExtForceTorque":
            ext_models.append(model)
            continue
        if cls == "ThrusterStateEffector":
            thr_state_models.append(model)
            continue
        # Env families route to their _mirror_* readers (never the
        # generic reject): effectors mirror via omni setters, field
        # modules pass through and only lend their out-messages.
        if cls in _DRAG_FAMILY:
            drag_models.append(model)
            continue
        if cls in _SRP_FAMILY:
            srp_models.append(model)
            continue
        if cls in _MTB_FAMILY:
            mtb_models.append(model)
            continue
        if cls in _MAGFIELD_FAMILY:
            magfield_models.append(model)
            continue
        if cls in _ECLIPSE_FAMILY:
            eclipse_models.append(model)
            continue
        if cls in _GRAV_FAMILY:
            grav_models.append(model)
            continue
        if cls == "ExtForceTorque":
            ext_models.append(model)
            continue
        if cls in _EPHEM_FAMILY:
            ephemeris_models.append(model)
            continue
        # Anything else that IS dynamics (SWIG MRO carries the
        # DynamicEffector/StateEffector bases) is unmirrorable and raises.
        # Pure field/model modules (atmosphere, wind, ...) are not
        # dynamics and pass through (collected below for the link ledger).
        bases = _bases(model)
        if "DynamicEffector" in bases or "StateEffector" in bases:
            raise ValueError(
                f"mirror_stock_config: task '{task_name}' holds "
                f"{cls}; envelope is hub + wheels + staircase "
                f"thrusters + central gravity + explicit omni env "
                f"(stock env effectors: use omni env setters instead).")
        if cls in _ATMO_FAMILY or "AtmosphereBase" in bases:
            atmo_models.append(model)
        elif cls in _WIND_FAMILY or "WindBase" in bases:
            wind_models.append(model)
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
    n_thr_state = _mirror_thruster_state(thr_state_models, thr_factory,
                                         thr_cmd_msg, oi, None)
    n_thrusters = n_thrusters + n_thr_state

    n_fuel = _mirror_fuel(fuel_models, oi, thr_isps)

    env = _mirror_env_all(sc, oi, None, drag_models, atmo_models,
                          wind_models, srp_models, eclipse_models,
                          mtb_models, magfield_models, grav_models,
                          ephemeris_models, srp_facet_specs, srp_mode)
    ext = _mirror_ext_force_torque(ext_models, oi, None,
                                    ext_torque_msg, ext_force_body_msg,
                                    ext_force_inertial_msg)

    return {"mass": hub["mass"], "mu": mu, "use_gg": use_gg,
            "n_wheels": n_wheels, "n_thrusters": n_thrusters,
            "n_fuel": n_fuel, "friction_on": fric_on, "rBcB": hub["rBcB"],
            "object_index": 0,
            "n_drag": env["drag"]["n_drag"],
            "n_extra_bodies": env["envbodies"]["n_extra_bodies"],
            "j2": env["envbodies"]["j2"]["j2"],
            "env": env,
            "ext": ext}


def _reject_unmirrorable_tasks(sim: Any) -> None:
    """Global envelope scan for the per-object path: any dynamics task model
    outside the mirrorable set raises (same message as the legacy scan).
    RW/thruster/fuel/GravityGradient collections are NOT used here -- the
    per-object path trusts its explicit model lists (the C++ per-object
    claim check closes the attribution gap: wheel-count, fuel-name, and
    unknown-state rejects run against THAT object's registered states).
    Env-effector classes in _MIRRORED_ENV pass: they are routed to their
    _mirror_* reader (which raises its own specific reject when the
    selection is unmirrorable)."""
    for task_name, model in task_effectors(sim):
        cls = type(model).__name__
        if cls in _MIRRORED:
            continue
        if cls in ("GravityGradientEffector", "FuelTank"):
            continue
        if cls in _MIRRORED_ENV:
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
                          thr_factory: Any, thr_cmd_msg: Any,
                          drag_models: Any = None,
                          atmo_models: Any = None,
                          wind_models: Any = None,
                          srp_models: Any = None,
                          eclipse_models: Any = None,
                          mtb_models: Any = None,
                          magfield_models: Any = None,
                          grav_models: Any = None,
                          ephemeris_models: Any = None,
                          srp_facet_specs: Any = None,
                          srp_mode: Any = None,
                          ext_models: Any = None,
                          ext_torque_msg: Any = None,
                          ext_force_body_msg: Any = None,
                          ext_force_inertial_msg: Any = None) -> dict:
    """Per-object mirror (WP25-A4): explicit model lists into slot ``idx``.

    The task-list scan cannot attribute effectors to objects, so every list
    must be explicit (``[]`` when the object has none) and ``use_gg`` must
    be an explicit bool. Only slot ``idx`` is marked mirrored+dirty; the
    C++ build requires every dynPtr slot mirrored, so a forgotten object
    fails loudly instead of inheriting another object's graph. The env
    dynamics lists (drag/SRP/eclipse/MTB/magfield/gravity) follow the
    same rule -- env leaves no named states, so no C++ check can close a
    forgotten env list, and a task census that cannot be attributed to
    objects is a CALLER OBLIGATION (pass each object's own slices).
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
    for key in ("drag_models", "srp_models", "eclipse_models",
                "mtb_models", "magfield_models", "grav_models"):
        if locals().get(key) is None:
            raise ValueError(
                "mirror_stock_config: object %d needs explicit %s (pass [] "
                "when the object has none); env dynamics leave no named "
                "states for the C++ per-object claim check, so a missing "
                "list would drop dynamics silently." % (idx, key))
    if atmo_models is None:
        atmo_models = []
    if wind_models is None:
        wind_models = []
    if ephemeris_models is None:
        ephemeris_models = []
    if ext_models is None:
        ext_models = []
    if thr_state_models is None:
        thr_state_models = []
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
    n_thr_state = _mirror_thruster_state(thr_state_models, thr_factory,
                                         thr_cmd_msg, oi, idx)
    n_thrusters = n_thrusters + n_thr_state
    n_fuel = _mirror_fuel(fuel_models, oi, thr_isps, idx)

    env = _mirror_env_all(sc, oi, idx, drag_models, atmo_models,
                          wind_models, srp_models, eclipse_models,
                          mtb_models, magfield_models, grav_models,
                          ephemeris_models, srp_facet_specs, srp_mode)
    ext = _mirror_ext_force_torque(ext_models if ext_models is not None else [],
                                    oi, idx, ext_torque_msg,
                                    ext_force_body_msg, ext_force_inertial_msg)

    return {"mass": hub["mass"], "mu": mu, "use_gg": bool(use_gg),
            "n_wheels": n_wheels, "n_thrusters": n_thrusters,
            "n_fuel": n_fuel, "friction_on": fric_on, "rBcB": hub["rBcB"],
            "object_index": idx,
            "n_drag": env["drag"]["n_drag"],
            "n_extra_bodies": env["envbodies"]["n_extra_bodies"],
            "j2": env["envbodies"]["j2"]["j2"],
            "env": env}


def mirror_sync_configs(sim: Any, oi: Any, specs: list) -> list:
    """Mirror EVERY object of a synced set into ONE integrator (WP25-A4).

    ``specs`` is one dict per object, in dynPtrs order (leader first):
    ``{"sc", "rw_models", "thr_models", "thr_factory", "thr_cmd_msg",
    "fuel_models", "use_gg"}``. All of ``rw_models``/``thr_models``/
    ``fuel_models``/``use_gg`` must be explicit (no scan attribution).
    The env dynamics lists (``drag_models``, ``srp_models``,
    ``eclipse_models``, ``mtb_models``, ``magfield_models``,
    ``grav_models``) follow the same rule, except that a spec written
    before the env wiring may omit them: a missing key defaults to ``[]``
    only when the task census holds no model of that family, and raises
    otherwise (a forgotten env list would drop dynamics no C++ check can
    see). The ledger-only field slices (``atmo_models``,
    ``wind_models``, ``ephemeris_models``) and the SRP caller-passed
    ``srp_facet_specs``/``srp_mode`` always default when absent.
    Recipe after this call::

        scA.setIntegrator(oi)
        scA.syncDynamicsIntegration(scB)  # B follows A (upstream
        # dynamicObject.cpp: pushes B into A's integrator, marks B synced)

    Returns the per-object mirrored records.
    """
    if not specs:
        raise ValueError("mirror_sync_configs: need >= 1 object spec.")
    census = _census_env_families(sim)
    out = []
    for idx, spec in enumerate(specs):
        for key in ("rw_models", "thr_models", "fuel_models", "use_gg"):
            if spec.get(key) is None:
                raise ValueError(
                    f"mirror_sync_configs: object {idx} spec needs explicit "
                    f"'{key}' (object B must never silently inherit object "
                    f"A's graph).")
        env_lists = {}
        for key, fam in _REQUIRED_ENV_KEYS:
            if spec.get(key) is None:
                if fam in census:
                    raise ValueError(
                        f"mirror_sync_configs: object {idx} spec needs "
                        f"explicit '{key}' (the task census holds {fam} "
                        f"models; object B must never silently inherit "
                        f"object A's graph).")
                env_lists[key] = []
            else:
                env_lists[key] = spec.get(key)
        out.append(mirror_stock_config(
            sim, spec["sc"], oi, object_index=idx,
            thr_factory=spec.get("thr_factory"),
            thr_cmd_msg=spec.get("thr_cmd_msg"),
            rw_models=spec["rw_models"], thr_models=spec["thr_models"],
            fuel_models=spec["fuel_models"], use_gg=spec["use_gg"],
            drag_models=env_lists["drag_models"],
            atmo_models=spec.get("atmo_models") or [],
            wind_models=spec.get("wind_models") or [],
            srp_models=env_lists["srp_models"],
            eclipse_models=env_lists["eclipse_models"],
            mtb_models=env_lists["mtb_models"],
            magfield_models=env_lists["magfield_models"],
            grav_models=env_lists["grav_models"],
            ephemeris_models=spec.get("ephemeris_models") or [],
            srp_facet_specs=spec.get("srp_facet_specs"),
            srp_mode=spec.get("srp_mode"),
            ext_models=spec.get("ext_models"),
            ext_torque_msg=spec.get("ext_torque_msg"),
            ext_force_body_msg=spec.get("ext_force_body_msg"),
            ext_force_inertial_msg=spec.get("ext_force_inertial_msg")))
    return out


def _census_env_families(sim: Any) -> set:
    """Env dynamics families present in the task census (sync tripwire).

    Returns subset of {"drag", "SRP", "eclipse", "MTB", "magnetic-field",
    "gravity"}. Unreadable census (sim None in unit tests) yields the
    empty set, i.e. all-missing env keys default to [].
    """
    try:
        entries = task_effectors(sim)
    except Exception:
        return set()
    out = set()
    for _, model in entries:
        cls = type(model).__name__
        if cls in _DRAG_FAMILY:
            out.add("drag")
        elif cls in _SRP_FAMILY:
            out.add("SRP")
        elif cls in _ECLIPSE_FAMILY:
            out.add("eclipse")
        elif cls in _MTB_FAMILY:
            out.add("MTB")
        elif cls in _MAGFIELD_FAMILY:
            out.add("magnetic-field")
        elif cls in _GRAV_FAMILY:
            out.add("gravity")
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


def _staged_reader(module: str) -> Any:
    """Load a staged env-mirror reader (``omniheyoka/env_mirror_*.py``).

    File-path import (importlib, stdlib only): importing the
    ``omniheyoka`` package would run ``omniheyoka/__init__`` (numpy +
    sibling sim packages), which bare-pytest hosts must never need --
    same discipline as the staged suites, which load their reader from
    its file path. Loaded once and cached.
    """
    cached = _STAGED_CACHE.get(module)
    if cached is not None:
        return cached
    import importlib.util
    from pathlib import Path
    path = Path(__file__).with_name(module + ".py")
    spec = importlib.util.spec_from_file_location(
        "swap_staged_" + module, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    _STAGED_CACHE[module] = mod
    return mod


def _reader_source(reader: Any) -> Any:
    """Recover the writer Message behind a linked stock reader functor.

    Uses the wrapper's own retention accessor ``_get_source_owner``
    (the mechanism the staged drag reader forwards links with);
    ``None`` when the source cannot be recovered.
    """
    getter = getattr(reader, "_get_source_owner", None)
    if not callable(getter):
        return None
    try:
        return getter()
    except Exception:
        return None


def _reader_linked(reader: Any) -> Any:
    """Best-effort linkage probe: True / False / None (undeterminable).

    Informational for OPTIONAL links; REQUIRED links treat anything but
    ``True`` as unlinked (a dropped link is a divergence, never a
    default -- PLAN_ENV_MIRROR §1.5).
    """
    if reader is None:
        return None
    fn = getattr(reader, "isLinked", None)
    if not callable(fn):
        return None
    try:
        return bool(fn())
    except Exception:
        return None


def _slot0_subscribe(oi: Any, member: str, src: Any, hpp_line: int) -> str:
    """Subscribe ``src`` into a slot-0 omni message member (``hpp`` line).

    The header carries no bare ``subscribeSun`` / ``subscribeEclipse``
    free functions -- slot 0 subscribes through the public ReadFunctor
    members (``oi.sunStateInMsg``, ``hpp:242``; ``oi.eclipseInMsg``,
    ``hpp:241``) with ``subscribeTo`` (precedent ``swap.py`` thruster
    command link).
    """
    functor = getattr(oi, member, None)
    if functor is None or not hasattr(functor, "subscribeTo"):
        raise ValueError(
            f"mirror_stock_config: oi has no {member} member; cannot "
            f"wire the link (expected vec_integrator.hpp:{hpp_line}).")
    functor.subscribeTo(src)
    return member + ".subscribeTo"


def _forobject_subscribe(oi: Any, twin: str, obj: Any, src: Any,
                         hpp_line: int) -> str:
    """Subscribe ``src`` through a per-object twin (``hpp`` line)."""
    subscribe = getattr(oi, twin, None)
    if not callable(subscribe):
        raise ValueError(
            f"mirror_stock_config: oi has no {twin}; cannot wire the "
            f"link for object {obj} (expected vec_integrator.hpp:"
            f"{hpp_line}).")
    subscribe(obj, src)
    return twin


def _mirror_drag(drag_models: Any, atmo_models: Any, wind_models: Any,
                 oi: Any, object_index: Any) -> dict:
    """Mirror the drag family (PLAN_ENV_MIRROR §2).

    Cannonball ``DragDynamicEffector`` -> ``oi.setDragParams`` (or the
    ``...ForObject`` twin) + the REQUIRED atmo link + the OPTIONAL wind
    link (unlinked = inertial fallback, §1.5). Facet drag and
    non-cannonball selections raise the reader's specific reject.
    Atmosphere/wind FIELD models stay in task (§1.1): their slices are
    ledger counts only -- the staged reader recovers the links exactly
    from the stock readers, never guessed from the census.
    """
    reader = _staged_reader("env_mirror_drag")
    rec = dict(reader.mirror_drag_models(drag_models, oi, object_index))
    rec["n_atmo_models"] = len(list(atmo_models or []))
    rec["n_wind_models"] = len(list(wind_models or []))
    rec["task_entry"] = _TASK_NOTE
    return rec


# Stock sun / eclipse reader attributes per SRP class (wrapper-verified:
# radiationPressure.py:637-638, facetSRPDynamicEffector.py:653-654).
_SRP_SUN_ATTR = {"RadiationPressure": "sunEphmInMsg",
                 "FacetSRPDynamicEffector": "sunInMsg"}
_SRP_ECLIPSE_ATTR = {"RadiationPressure": "sunEclipseInMsg",
                     "FacetSRPDynamicEffector": "sunEclipseInMsg"}


def _mirror_srp(srp_models: Any, eclipse_models: Any, oi: Any,
                object_index: Any, facet_specs: Any = None,
                mode: Any = None) -> dict:
    """Mirror the SRP family (PLAN_ENV_MIRROR §3).

    Cannonball ``RadiationPressure`` -> ``oi.setSRPCannonball`` (with
    ``cr = C_R - 1``, ``hpp:112-116``); static
    ``FacetSRPDynamicEffector`` facets -> ``oi.addSRPFacet`` each. Both
    setters are applied by the staged delegation; the sun link
    (REQUIRED, omni bskErrors on SRP-on + unlinked sun,
    ``cpp:1199-1205``) is resolved here from the stock readers and
    subscribed (slot-0 ``oi.sunStateInMsg``, ``hpp:242``, else the
    ``subscribeSunForObject`` twin, ``hpp:213``). The eclipse link
    (OPTIONAL, unlinked = full sun, ``cpp:1220-1222``) is wired by
    :func:`_mirror_eclipse_link` when census Eclipse models exist, and
    forwarded directly from the recovered stock reader otherwise.

    Gates (all loud, before any setter fires): more than one
    cannonball model (the omni slot holds ONE cannonball record --
    ``setSRPCannonball`` overwrites, ``cpp:282-291``, while stock sums
    both effectors); an unlinked or source-unrecoverable REQUIRED sun
    reader; an Eclipse census with registered spacecraft while no SRP
    model links its eclipse reader (the omni eclipse slot is global to
    SRP -- wiring it would shadow stock's full-sun SRP); a linked SRP
    eclipse reader with neither a recoverable source nor a census
    Eclipse model. Faceted-CPU/lookup selections, articulated facets,
    nonzero rotHat and message-driven faceted paths raise the staged
    reader's specific rejects.
    """
    models = list(srp_models or [])
    eclipses = list(eclipse_models or [])
    n_cannon = sum(1 for m in models
                   if type(m).__name__ == "RadiationPressure")
    if n_cannon > 1:
        raise ValueError(
            f"mirror_stock_config: {n_cannon} RadiationPressure task "
            f"models; the omni slot holds ONE cannonball record "
            f"(setSRPCannonball overwrites, "
            f"src/omni_vec_integrator.cpp:282-291) while stock sums "
            f"both effectors -- use one cannonball model "
            f"(PLAN_ENV_MIRROR §3.1).")
    # Gate pass (read-only): resolve every REQUIRED sun source and the
    # eclipse coherence BEFORE the staged delegation fires setters.
    sun_srcs = []
    eclipse_wanted = []
    for m in models:
        cls = type(m).__name__
        if cls in _SRP_SUN_ATTR:
            attr = _SRP_SUN_ATTR[cls]
            reader = getattr(m, attr, None)
            if _reader_linked(reader) is not True:
                raise ValueError(
                    f"mirror_stock_config: {cls}.{attr} unlinked; the "
                    f"sun link is REQUIRED (stock and omni both bskError "
                    f"on SRP-on + unlinked sun, "
                    f"src/omni_vec_integrator.cpp:1199-1205, §1.5).")
            src = _reader_source(reader)
            if src is None:
                raise ValueError(
                    f"mirror_stock_config: {cls}.{attr} is linked but "
                    f"its source message is unrecoverable; the REQUIRED "
                    f"sun link cannot be dropped silently.")
            sun_srcs.append(src)
        if cls in _SRP_ECLIPSE_ATTR:
            reader = getattr(m, _SRP_ECLIPSE_ATTR[cls], None)
            if _reader_linked(reader) is True:
                eclipse_wanted.append((cls, _reader_source(reader)))
    eclipse_outs = _eclipse_out_count(eclipses)
    eclipse_direct: list = []
    if eclipse_wanted and not eclipses:
        # No census Eclipse to wire later: forward each recovered stock
        # source directly (exactly one subscribe per wanted link).
        for cls, src in eclipse_wanted:
            if src is None:
                raise ValueError(
                    f"mirror_stock_config: {cls} eclipse reader is "
                    f"linked but its source is unrecoverable and no "
                    f"Eclipse task model exists to wire instead; "
                    f"dropping it would run full sun against stock "
                    f"shadows.")
            eclipse_direct.append(src)
    elif models and not eclipse_wanted and eclipse_outs:
        raise ValueError(
            "mirror_stock_config: Eclipse task model(s) with registered "
            "spacecraft present, but no SRP model links its eclipse "
            "reader; the omni eclipse slot is global to SRP -- wiring it "
            "would shadow stock's full-sun SRP. Link the stock "
            "sunEclipseInMsg or remove the Eclipse model (§5).")
    staged = _staged_reader("env_mirror_srp")
    rec = dict(staged.mirror_srp_models(models, oi, object_index,
                                        facet_specs, mode))
    if object_index is None:
        for src in sun_srcs:
            _slot0_subscribe(oi, "sunStateInMsg", src, 242)
        for src in eclipse_direct:
            _slot0_subscribe(oi, "eclipseInMsg", src, 241)
        sun_link = "sunStateInMsg.subscribeTo" if sun_srcs else None
        eclipse_link = ("eclipseInMsg.subscribeTo" if eclipse_direct
                        else "deferred-to-eclipse-models")
    else:
        for src in sun_srcs:
            _forobject_subscribe(oi, "subscribeSunForObject",
                                 object_index, src, 213)
        for src in eclipse_direct:
            _forobject_subscribe(oi, "subscribeEclipseForObject",
                                 object_index, src, 211)
        sun_link = "subscribeSunForObject" if sun_srcs else None
        eclipse_link = ("subscribeEclipseForObject" if eclipse_direct
                        else "deferred-to-eclipse-models")
    rec["sun_link"] = sun_link
    rec["n_sun_sources"] = len(sun_srcs)
    rec["eclipse_link"] = eclipse_link
    rec["task_entry"] = _TASK_NOTE
    return rec


def _eclipse_out_count(eclipses: Any) -> Any:
    """Registered-spacecraft out-message count, or None if unknown.

    Guarded read only (``eclipseOutMsgs``, wrapper eclipse.py:377):
    ``None`` (unreadable vector) skips the SRP/eclipse coherence gate
    and lets :func:`_mirror_eclipse_link` raise its own specific error.
    """
    total = 0
    for e in list(eclipses or []):
        try:
            outs = getattr(e, "eclipseOutMsgs", None)
            total += len(list(outs)) if outs is not None else 0
        except Exception:
            return None
    return total


def _mirror_mag(mtb_models: Any, magfield_models: Any, oi: Any,
                object_index: Any) -> dict:
    """Mirror the magnetic/MTB family (PLAN_ENV_MIRROR §4).

    Identity-Gt single-bar ``MtbEffector`` with a latched dipole command
    -> ``oi.setMagneticMoment`` + ``oi.setMaxMtbDipole`` (or the
    ``...ForObject`` twins); ``MagneticFieldCenteredDipole`` g10/g11/h11
    -> ``oi.setMagneticDipole``; the field ``envOutMsgs[0]`` feeds the
    omni mag link (REQUIRED, ``hpp:244`` / ``hpp:217``). An MTB slice
    with no field model rejects BEFORE the staged delegation fires, so
    a rejected mirror leaves ``oi`` untouched; ``MagneticFieldWMM``
    stays unmirrorable (message-only, no model surface -- the staged
    reader pins that reject, §4.2).
    """
    mtb = list(mtb_models or [])
    fields = list(magfield_models or [])
    if mtb and not fields:
        raise ValueError(
            "mirror_stock_config: MtbEffector with no "
            "MagneticFieldCenteredDipole in the census; the mag link is "
            "REQUIRED (mag on + unlinked = bskError, "
            "src/omni_vec_integrator.cpp:1255-1258, PLAN §1.5). "
            "MagneticFieldWMM stays unmirrorable (message-only, no "
            "model surface; staged SUPPORTED_CLASSES pins it, §4.2).")
    staged = _staged_reader("env_mirror_mag")
    rec = dict(staged.mirror_mag_models(mtb + fields, oi, object_index))
    rec["task_entry"] = _TASK_NOTE
    return rec


def _mirror_eclipse_link(eclipse_models: Any, oi: Any,
                         object_index: Any) -> dict:
    """Wire the eclipse field link (PLAN_ENV_MIRROR §5).

    The ``Eclipse`` module is a FIELD module (plain SysModel): it stays
    in task and only its out-message is forwarded (single registered
    spacecraft -> ``eclipseOutMsgs[0]``; order-match beyond that).
    Absent/empty stays legal-unlinked (factor 1.0, §1.5).
    """
    staged = _staged_reader("env_mirror_eclipse")
    rec = dict(staged.mirror_eclipse_models(eclipse_models, oi,
                                             object_index))
    rec["task_entry"] = ("stays-in-task: the Eclipse module is a field "
                         "module (plain SysModel, PLAN §1.1); only its "
                         "out-message is forwarded, never removed.")
    return rec


def _gravity_census_bodies(holder: Any, what: str) -> list:
    """``gravBodies`` holder -> ``[(is_central, mu)]`` (numeric reads).

    A missing/unreadable surface or a non-numeric mu is a loud reject
    (PLAN §1.3) -- the sc/task consistency gate below must not guess.
    """
    try:
        bodies = list(holder.gravBodies)
    except Exception as exc:
        raise ValueError(
            f"mirror_stock_config: {what}.gravBodies unreadable "
            f"({exc!r}); the env-bodies census needs it (§6.1).") from exc
    out = []
    for b in bodies:
        try:
            central = bool(b.isCentralBody)
        except Exception as exc:
            raise ValueError(
                f"mirror_stock_config: GravBodyData.isCentralBody "
                f"unreadable ({exc!r}).") from exc
        try:
            mu = float(b.mu)
        except Exception as exc:
            raise ValueError(
                f"mirror_stock_config: GravBodyData.mu unreadable "
                f"({exc!r}).") from exc
        out.append((central, mu))
    return out


def _mirror_env_bodies(sc: Any, grav_models: Any, ephemeris_models: Any,
                       oi: Any, object_index: Any) -> dict:
    """Mirror the gravity / J2 / ephemeris family (PLAN_ENV_MIRROR §6).

    Each non-central body (stock ``gravBodies`` order) ->
    ``oi.addEnvironmentBody`` with the first extra body on the sun link
    and the second on the moon link (§1.6 index rule); the central body
    stays on the existing central-mu rule and only lends its J2
    (``maxDeg == 2`` -> ``oi.setJ2``, applied once by the staged
    delegation, validated by :func:`_mirror_j2`). Ephemeris providers
    stay in task (§6.3, ledger-only). The sc/task consistency gate runs
    BEFORE the staged delegation: stock must feel exactly the bodies
    the task model carries, else extras would freeze or invent tides
    silently. A task ``GravityEffector`` with no sc extras still
    delegates (the staged reader gates linkage/model/count itself).
    """
    grav = list(grav_models or [])
    ephem = list(ephemeris_models or [])
    sc_bodies = _gravity_census_bodies(sc.gravField, "sc.gravField")
    sc_extra = sorted(mu for central, mu in sc_bodies if not central)
    sc_central = sorted(mu for central, mu in sc_bodies if central)
    staged = _staged_reader("env_mirror_envbodies")
    if not grav:
        if sc_extra:
            raise ValueError(
                f"mirror_stock_config: sc feels {len(sc_extra)} "
                f"non-central bodies but no GravityEffector task model "
                f"exists; their per-tick ephemeris (REQUIRED, §6.1) has "
                f"no mirror -- add the GravityEffector to the task.")
        rec = dict(staged.mirror_envbodies_models(ephem, oi,
                                                  object_index))
        rec["j2_check"] = _mirror_j2(rec["j2"], oi, object_index)
        rec["task_entry"] = _TASK_NOTE
        return rec
    if len(grav) == 1:
        task_bodies = _gravity_census_bodies(grav[0], "GravityEffector")
        task_extra = sorted(mu for c, mu in task_bodies if not c)
        task_central = sorted(mu for c, mu in task_bodies if c)
        if task_extra != sc_extra or task_central != sc_central:
            raise ValueError(
                "mirror_stock_config: task GravityEffector bodies "
                f"{task_central + task_extra} != sc gravity bodies "
                f"{sc_central + sc_extra}; the staged reader mirrors "
                f"the task model, so a mismatch would freeze or invent "
                f"tides silently (one census, §6.1).")
    rec = dict(staged.mirror_envbodies_models(grav + ephem, oi,
                                              object_index))
    rec["j2_check"] = _mirror_j2(rec["j2"], oi, object_index)
    rec["task_entry"] = _TASK_NOTE
    return rec


def _mirror_j2(j2_record: Any, oi: Any, object_index: Any) -> dict:
    """Validate + ledger the central-body J2 leg (PLAN_ENV_MIRROR §6.2).

    The staged env-bodies delegation is the SINGLE setter-application
    point (its gates-before-setters pass fires ``oi.setJ2`` exactly
    once when ``maxDeg == 2``); this leg validates the resulting record
    and confirms the ``oi`` surface, so a drift between the recorded J2
    and the integrator surface fails loudly instead of ledgering
    fiction. ``J2 = -sqrt(5) * Cbar20`` (staged rule); negative or
    non-finite values never reach this leg (staged rejects first).
    """
    if not isinstance(j2_record, dict) or "j2" not in j2_record:
        raise ValueError(
            "mirror_stock_config: env-bodies J2 record malformed "
            f"({j2_record!r}); the J2 leg needs the staged record.")
    value = j2_record["j2"]
    if value is not None:
        import math
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(
                f"mirror_stock_config: J2={value!r} is not finite "
                f"non-negative; omni setJ2 bskErrors on it "
                f"(src/omni_vec_integrator.cpp:236-237).")
        name = "setJ2" if object_index is None else "setJ2ForObject"
        if not callable(getattr(oi, name, None)):
            raise ValueError(
                f"mirror_stock_config: oi has no {name}; cannot confirm "
                f"the J2 setter surface (vec_integrator.hpp:107/174).")
    return {"j2": value, "setter": j2_record.get("setter"),
            "args": j2_record.get("args"), "validated": True,
            "object_index": object_index}


def _mirror_ext_force_torque(ext_models: Any, oi: Any, object_index: Any,
                            ext_torque_msg: Any = None,
                            ext_force_body_msg: Any = None,
                            ext_force_inertial_msg: Any = None) -> dict:
    """Mirror stock ExtForceTorque (external force/torque channel).

    Constant fields are always mirrored. Linked messages are forwarded when
    the caller supplies the corresponding message handle (the stock functor
    alone does not expose its source); an unforwarded-but-linked stock
    message is a loud reject, never a silent zero.
    """
    models = list(ext_models or [])
    if not models:
        return {"n_ext": 0, "constant": False, "msgs": []}
    if len(models) > 1:
        raise ValueError(
            "mirror_stock_config: %d ExtForceTorque effectors; stock "
            "registration order is ambiguous -- use one effector."
            % len(models))
    eff = models[0]

    def _vec(attr):
        v = getattr(eff, attr, None)
        if v is None:
            return [0.0, 0.0, 0.0]
        try:
            return [float(x) for x in _as_vec3(v)]
        except Exception:
            try:
                return [float(x) for x in v]
            except Exception:
                return [0.0, 0.0, 0.0]

    fB = _vec("extForce_B")
    tB = _vec("extTorquePntB_B")
    fN = _vec("extForce_N")

    # Constant registration: always register the channel when any constant
    # is nonzero OR a message is being forwarded.
    has_const = any(x != 0.0 for x in (fB + tB + fN))
    msgs_to_wire = []
    # Detect linked stock messages.
    def _linked(name):
        try:
            fn = getattr(eff, name, None)
            return bool(fn is not None and fn.isLinked())
        except Exception:
            return False
    linked_torque = _linked("cmdTorqueInMsg")
    linked_force_body = _linked("cmdForceBodyInMsg")
    linked_force_inertial = _linked("cmdForceInertialInMsg")

    if linked_torque and ext_torque_msg is None:
        raise ValueError(
            "mirror_stock_config: ExtForceTorque.cmdTorqueInMsg is linked "
            "but no ext_torque_msg handle was supplied; the omni cannot "
            "subscribe it (pass ext_torque_msg=...).")
    if linked_force_body and ext_force_body_msg is None:
        raise ValueError(
            "mirror_stock_config: ExtForceTorque.cmdForceBodyInMsg is linked "
            "but no ext_force_body_msg handle was supplied.")
    if linked_force_inertial and ext_force_inertial_msg is None:
        raise ValueError(
            "mirror_stock_config: ExtForceTorque.cmdForceInertialInMsg is "
            "linked but no ext_force_inertial_msg handle was supplied.")

    if has_const or linked_torque or linked_force_body or linked_force_inertial:
        if object_index is None:
            oi.setExtForceTorque(fB[0], fB[1], fB[2], tB[0], tB[1], tB[2])
            if any(x != 0.0 for x in fN):
                oi.setExtForceInertial(fN[0], fN[1], fN[2])
        else:
            oi.setExtForceTorqueForObject(object_index, fB[0], fB[1], fB[2],
                                          tB[0], tB[1], tB[2])
            if any(x != 0.0 for x in fN):
                oi.setExtForceInertialForObject(object_index, fN[0], fN[1],
                                                fN[2])

    # Forward linked messages.
    if linked_torque and ext_torque_msg is not None:
        if object_index is None:
            oi.extTorqueInMsg.subscribeTo(ext_torque_msg)
        else:
            oi.subscribeExtTorqueForObject(object_index, ext_torque_msg)
        msgs_to_wire.append("extTorqueInMsg")
    if linked_force_body and ext_force_body_msg is not None:
        if object_index is None:
            oi.extForceBodyInMsg.subscribeTo(ext_force_body_msg)
        else:
            oi.subscribeExtForceBodyForObject(object_index, ext_force_body_msg)
        msgs_to_wire.append("extForceBodyInMsg")
    if linked_force_inertial and ext_force_inertial_msg is not None:
        if object_index is None:
            oi.extForceInertialInMsg.subscribeTo(ext_force_inertial_msg)
        else:
            oi.subscribeExtForceInertialForObject(object_index,
                                                 ext_force_inertial_msg)
        msgs_to_wire.append("extForceInertialInMsg")

    return {"n_ext": 1, "constant": has_const, "msgs": msgs_to_wire}


def _mirror_thruster_state(thr_state_models: Any, thr_factory: Any,
                           thr_cmd_msg: Any, oi: Any,
                           object_index: Any) -> int:
    """Mirror a stock ThrusterStateEffector as an IDLE dynamic thruster.

    The kappa (thrustFactor) lag is NOT modeled (Feature 2 pending); this
    reader is honest only when the command is provably zero over the
    mirrored horizon. It raises loudly otherwise. Geometry is joined exactly
    like the dynamic effector (same factory THRSimConfig + effector
    thrusterData).
    """
    models = list(thr_state_models or [])
    if not models:
        return 0
    if len(models) > 1:
        raise ValueError(
            "mirror_stock_config: %d ThrusterStateEffector; ambiguous."
            % len(models))
    eff = models[0]
    # Idleness gate.
    if thr_cmd_msg is None:
        raise ValueError(
            "mirror_stock_config: ThrusterStateEffector requires thr_cmd_msg "
            "to prove idleness over the horizon.")
    try:
        linked = bool(thr_cmd_msg.isLinked() if hasattr(thr_cmd_msg, "isLinked")
                      else thr_cmd_msg.functor.isLinked())
    except Exception:
        linked = False
    if not linked:
        raise ValueError(
            "mirror_stock_config: ThrusterStateEffector thr_cmd_msg unlinked; "
            "cannot prove idleness.")
    try:
        payload = thr_cmd_msg() if callable(thr_cmd_msg) else None
    except Exception:
        payload = None
    if payload is not None:
        try:
            reqs = list(payload.OnTimeRequest)
        except Exception:
            try:
                reqs = list(payload.onTimeRequest)
            except Exception:
                reqs = []
        if any(float(x) != 0.0 for x in reqs):
            raise ValueError(
                "mirror_stock_config: ThrusterStateEffector has a nonzero "
                "OnTimeRequest; the kappa lag is not modeled (Feature 2 "
                "pending) -- refusing to mirror a firing effector.")
    # Read cutoff from the factory config (state effector's tau = 1/cutoff).
    cfgs = list(getattr(thr_factory, "thrusterList", {}).values())
    cutoff = 10.0
    if cfgs:
        try:
            cutoff = float(getattr(cfgs[0], "cutoffFrequency", 10.0))
        except Exception:
            cutoff = 10.0
    if cutoff <= 0.0:
        cutoff = 10.0
    # Delegate geometry to the dynamic reader.
    n, _ = _mirror_thrusters(models, thr_factory, thr_cmd_msg, oi,
                             object_index)
    # Enable the kappa state (lag) on the omni side.
    if object_index is None:
        oi.setThrusterLag(True, cutoff)
    else:
        oi.setThrusterLagForObject(object_index, True, cutoff)
    return n


def _mirror_env_all(sc: Any, oi: Any, object_index: Any,
                    drag_models: Any, atmo_models: Any, wind_models: Any,
                    srp_models: Any, eclipse_models: Any, mtb_models: Any,
                    magfield_models: Any, grav_models: Any,
                    ephemeris_models: Any, srp_facet_specs: Any = None,
                    srp_mode: Any = None) -> dict:
    """Run every env family mirror for one object slot (PLAN §8).

    ``object_index=None`` is the legacy slot-0 surface; an int selects
    the per-object ``...ForObject`` twins for that slot alone (links
    never mark a slot mirrored, ``hpp:202-204``). Order: drag, SRP
    (validates the shared eclipse slot before the eclipse link wires),
    mag, eclipse, env-bodies. Cross-family atomicity follows the
    existing swap pattern (each family gates before its own setters;
    an early family applies before a later one rejects -- same as the
    wheels-before-thrusters order).
    """
    drag = _mirror_drag(drag_models, atmo_models, wind_models, oi,
                        object_index)
    srp = _mirror_srp(srp_models, eclipse_models, oi, object_index,
                      srp_facet_specs, srp_mode)
    mag = _mirror_mag(mtb_models, magfield_models, oi, object_index)
    eclipse = _mirror_eclipse_link(eclipse_models, oi, object_index)
    envbodies = _mirror_env_bodies(sc, grav_models, ephemeris_models,
                                   oi, object_index)
    return {"drag": drag, "srp": srp, "mag": mag, "eclipse": eclipse,
            "envbodies": envbodies, "object_index": object_index}
