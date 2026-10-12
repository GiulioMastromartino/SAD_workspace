"""Direct-VizInterface wiring for OmniSpacecraft (WP6, PLAN Rev 1).

Why this file exists
--------------------
The ``vizSupport.enableVisualization`` helper path is **type-bound and
unhijackable** for omni (measured T2, BSK 2.12.0):

* it does ``isinstance(sc, Spacecraft)`` — ``OmniSpacecraft`` is a
  ``SysModel`` subclass, not a ``Spacecraft``, so the helper rejects it;
* it harvests ``gravField`` / gravity bodies off the stock ``Spacecraft``
  object, which omni does not carry.

The supported path is configuring ``VizInterface`` **directly**, the way
``test_dataFileToViz`` does: create the module, subscribe its input ports
to the message(s) omni already publishes, add it to the task. This module
is that helper. It deliberately has **no import of the helper package**
(the helper import lives only in the negative-control probe, not here),
so importing this file can never pull the type-bound path back in.

What it wires (plural-first, version-tolerant)
----------------------------------------------
* ``scStateInMsgs`` (plural vector, ``pushBack``) with fallback to
  ``scStateInMsg`` (singular, ``subscribeTo``). The plural name is the
  BSK 2.12.0 contract; the singular fallback keeps older builds working.
* ``thrInMsgs`` (plural vector) — one entry per ``thrusterOutMsgs[i]``
  (WP3 paragraph owns that publisher; before WP3 lands pass ``[]``).
* ``rwInMsgs`` (plural vector) — one entry per ``rwOutMsgs[i]``
  (WP6 paragraph owns that publisher; before WP6 lands pass ``[]``).

Port discovery is by attribute dump (``dir(viz)``), not by hard-coded
assumption: :func:`configure_direct_viz` probes ``scStateInMsgs`` then
``scStateInMsg``, and ``thrInMsgs`` / ``rwInMsgs`` independently, so a
build that renames one port still wires the others. Timestamps
(``CurrentSimNanos`` + ``moduleID``) are carried by the messages
themselves; this helper records nothing.

Smoke scope
-----------
``configure_direct_viz`` only builds and subscribes (no sim run) so it is
safe to call on any host. A full ``ExecuteSimulation`` with a live
``VizInterface`` segfaults headless on the audit host — the smoke runner
:func:`viz_smoke` therefore only executes when
``WP6_RUN_VIZ_SIM=1`` is set, exactly like the T2 probe gate; otherwise it
initializes (when asked) or returns the un-run ``viz`` object with a
``skipped`` record. Same pattern as the fuel-burn gate
``WP6_RUN_FUEL_SIM=1`` (T3 static-only default).

Fuel / DV bookkeeping helpers in this file are **caller-side, non-parity**
(see :func:`fuel_mass_step` docstring for the 3 divergences) and exist so
the host test suite can pin the contract without a sim.
"""

from __future__ import annotations

import os
from typing import Any, Sequence

__all__ = [
    "THR_REAL_FIELDS_9",
    "RW_REAL_FIELDS_23",
    "SC_REAL_FIELDS_12",
    "FUEL_REAL_FIELDS_3",
    "SWIG_EXTRAS",
    "G0",
    "fuel_mass_step",
    "dv_accumulate_step",
    "collect_omni_msgs",
    "describe_viz_ports",
    "configure_direct_viz",
    "viz_smoke",
]

#: Stock THROutputMsgPayload real fields (9/9, T1). SWIG adds this/thisown.
THR_REAL_FIELDS_9: tuple[str, ...] = (
    "thrustFactor",
    "thrustForce",
    "thrustForce_B",
    "thrustTorquePntB_B",
    "thrustBlowDownFactor",
    "ispBlowDownFactor",
    "maxThrust",
    "thrusterLocation",
    "thrusterDirection",
)

#: Stock RWConfigLogMsgPayload real fields (23, P-WP4.1 / T1).
RW_REAL_FIELDS_23: tuple[str, ...] = (
    "rWB_B",
    "gsHat_B",
    "w2Hat0_B",
    "w3Hat0_B",
    "mass",
    "theta",
    "Omega",
    "Js",
    "Jt",
    "Jg",
    "U_s",
    "U_d",
    "d",
    "J13",
    "u_current",
    "frictionTorque",
    "u_max",
    "u_min",
    "u_f",
    "Omega_max",
    "P_max",
    "linearFrictionRatio",
    "RWModel",
)

#: Stock SCStatesMsgPayload real fields (12 + this/thisown = 14 static, T1).
SC_REAL_FIELDS_12: tuple[str, ...] = (
    "MRPSwitchCount",
    "TotalAccumDVBdy",
    "TotalAccumDV_BN_B",
    "TotalAccumDV_CN_N",
    "nonConservativeAccelpntB_B",
    "omegaDot_BN_B",
    "omega_BN_B",
    "r_BN_N",
    "r_CN_N",
    "sigma_BN",
    "v_BN_N",
    "v_CN_N",
)

#: Stock FuelTankMsgPayload real fields (3 + this/thisown, T3 static census).
FUEL_REAL_FIELDS_3: tuple[str, ...] = (
    "fuelMass",
    "fuelMassDot",
    "maxFuelMass",
)

#: SWIG wrapper extras present on every payload dir() — never part of parity.
SWIG_EXTRAS: frozenset[str] = frozenset({"this", "thisown"})

#: Standard gravity for fuel bookkeeping [m/s^2].
G0: float = 9.80665


def real_fields(static_names: Sequence[str]) -> list[str]:
    """Strip SWIG extras (``this``/``thisown``) from a payload dir() list."""
    return [n for n in static_names if n not in SWIG_EXTRAS]


# ---------------------------------------------------------------------------
# Caller-side bookkeeping (pure Python, host-testable, NON-PARITY labeled)
# ---------------------------------------------------------------------------

def fuel_mass_step(
    fuel_mass: float,
    achieved_thrust_N: float,
    dt: float,
    isp_s: float,
    g0: float = G0,
) -> tuple[float, float]:
    """One tick of caller-side fuel bookkeeping.

    ``mdot = -F_achieved / (isp * g0)``; ``mass += mdot * dt`` (floored at 0).

    CRITICAL: ``F_achieved`` is the **staircase-ACHIEVED** thrust
    (``thrustFactor * maxThrust`` summed over thrusters), NOT the commanded
    thrust. Using the commanded value re-opens measured gap #1
    (commanded 1.0 Ns vs achieved ~0.2 Ns class, T3 divergence note; the
    staircase ``{0, 1/6, 5/6, 1}`` fires less impulse than requested and
    fuel must follow what fired).

    3 LABELED divergences vs stock ``FuelTank`` (bookkeeping, not parity):

    1. commanded != achieved — closed here by taking the achieved factor,
       but only to first order (substep arc phasing still differs);
    2. no blowdown / ISP scaling — ``thrustBlowDownFactor`` /
       ``ispBlowDownFactor`` are echoed as 1.0 until WP3 lands;
    3. NO mass feedback — omni ``mass`` stays fixed; stock depletes
       ``m / c_B / I`` per substep (cf. upstream PR #1488). Never wire
       this back into the dynamics without a WP4 aggregate-mass path.

    Returns ``(new_mass, mdot)`` with ``mdot <= 0``.
    """
    f = float(achieved_thrust_N)
    d = float(dt)
    isp = float(isp_s)
    if not (d > 0.0):
        raise ValueError(f"dt must be positive, got {dt!r}")
    if not (isp > 0.0):
        raise ValueError(f"isp must be positive, got {isp_s!r}")
    if f < 0.0:
        f = 0.0
    mdot = -f / (isp * float(g0))
    new_mass = float(fuel_mass) + mdot * d
    if new_mass < 0.0:
        # Clamp: recompute mdot so mass lands exactly at 0 (empty tank).
        mdot = -float(fuel_mass) / d if d > 0 else 0.0
        new_mass = 0.0
    return new_mass, mdot


def dv_accumulate_step(
    accum: Sequence[float],
    force_N: Sequence[float],
    mass_kg: float,
    dt: float,
) -> list[float]:
    """One tick of per-tick impulse bookkeeping: ``accum += F * dt / mass``.

    LABELED non-parity: stock integrates DV per **substep** inside the
    effectors (thruster arcs, continuous RW/mag/drag/SRP); omni accumulates
    the tick-average impulse per **tick** in ``writeMsgs``. Values agree to
    O(dt) on smooth burns and diverge across staircase edges — that is the
    documented contract, not a bug. All three SC DV slots
    (``TotalAccumDV_CN_N`` / ``TotalAccumDV_BN_B`` / ``TotalAccumDVBdy``)
    advance by the same Δv magnitude in their respective frames at zero
    attitude; the frame split is applied by the caller in C++.
    """
    import numpy as np

    a = np.asarray(accum, dtype=float).reshape(3)
    f = np.asarray(force_N, dtype=float).reshape(3)
    m = float(mass_kg)
    d = float(dt)
    if not (m > 0.0):
        raise ValueError(f"mass must be positive, got {mass_kg!r}")
    if not (d > 0.0):
        raise ValueError(f"dt must be positive, got {dt!r}")
    return (a + f * (d / m)).tolist()


# ---------------------------------------------------------------------------
# Message collection (works before/after WP3+WP6 land)
# ---------------------------------------------------------------------------

def collect_omni_msgs(omni_sc: Any) -> dict[str, Any]:
    """Collect subscribable omni outputs without assuming WP3/WP6 landed.

    Returns ``{"scState": msg-or-None, "thr": [...], "rw": [...]}`` where
    each entry is a Basilisk ``Message`` object (or empty/None when the
    publisher does not exist yet). Never raises: missing attributes yield
    empty slots so the viz helper degrades to scState-only wiring.
    """
    out: dict[str, Any] = {"scState": None, "thr": [], "rw": []}
    get = getattr
    sc = get(omni_sc, "scStateOutMsg", None)
    out["scState"] = sc
    # THR publisher (WP3 owns): accept vector `thrusterOutMsgs` or list.
    for attr in ("thrusterOutMsgs", "thrOutMsgs", "thrusterOutMsg"):
        try:
            cand = get(omni_sc, attr, None)
        except Exception:
            continue
        if cand is None:
            continue
        try:
            out["thr"] = list(cand)
        except TypeError:
            out["thr"] = [cand]
        break
    # RW publisher (WP6 owns): accept vector `rwOutMsgs` or per-wheel getter.
    try:
        cand = get(omni_sc, "rwOutMsgs", None)
    except Exception:
        cand = None
    if cand is not None:
        try:
            out["rw"] = list(cand)
        except TypeError:
            out["rw"] = [cand]
    else:
        # Fallback: `rwLogOutMsg(i)` / `getRwOutMsg(i)` accessor + nWheels().
        getter = get(omni_sc, "rwLogOutMsg", get(omni_sc, "getRwOutMsg", None))
        nWheels = None
        for meth in ("nWheels", "getNumRW", "numRW"):
            try:
                fn = get(omni_sc, meth, None)
                if callable(fn):
                    nWheels = int(fn())
                    break
            except Exception:
                continue
        if callable(getter) and nWheels:
            msgs = []
            for i in range(nWheels):
                try:
                    msgs.append(getter(i))
                except Exception:
                    break
            out["rw"] = msgs
    return out


def describe_viz_ports(viz: Any) -> dict[str, Any]:
    """Attribute-dump census of a ``VizInterface`` instance (no sim, no run).

    Returns ``{"has": {name: bool}, "sample": [...]}`` over the port names
    this helper knows (``scStateInMsgs`` plural-first, singular fallback,
    ``thrInMsgs``, ``rwInMsgs``). Used by the host test to pin the
    plural-``scStateInMsgs`` contract without executing a sim.
    """
    names: set[str] = set()
    try:
        names = {a for a in dir(viz) if not a.startswith("_")}
    except Exception:
        names = set()
    keys = ("scStateInMsgs", "scStateInMsg", "thrInMsgs", "rwInMsgs")
    sample = sorted(
        a
        for a in names
        if ("scState" in a or "thr" in a.lower() or "rw" in a.lower() or "Msg" in a)
    )[:20]
    return {"has": {k: (k in names) for k in keys}, "sample": sample}


def _subscribe_single(port: Any, msg: Any) -> str:
    """Subscribe one message to a singular port; returns how it worked."""
    port.subscribeTo(msg)
    return "subscribeTo"


def _push_vector(vec: Any, msg: Any) -> str:
    """Append one message to a plural vector port; returns how it worked."""
    push = getattr(vec, "pushBack", None)
    if callable(push):
        push(msg)
        return "pushBack"
    # Some builds expose the vector as a Python list.
    if isinstance(vec, list):
        vec.append(msg)
        return "list.append"
    # Last resort: `append`.
    append = getattr(vec, "append", None)
    if callable(append):
        append(msg)
        return "append"
    raise AttributeError("plural port has neither pushBack nor append")


def configure_direct_viz(
    sim: Any,
    omni_sc: Any,
    thr_out_msgs: Sequence[Any] | None = None,
    rw_out_msgs: Sequence[Any] | None = None,
    model_tag: str = "viz",
    spacecraft_name: str = "omniSat",
) -> Any:
    """Create a ``VizInterface`` wired directly to omni messages.

    Parameters
    ----------
    sim
        The ``SimBaseClass`` (only used by the caller to
        ``AddModelToTask`` afterwards — this function does NOT add the
        task itself, mirroring ``test_dataFileToViz`` where task
        placement is the caller's choice).
    omni_sc
        The ``OmniSpacecraft`` (or anything exposing ``scStateOutMsg`` /
        ``rwOutMsgs`` / ``thrusterOutMsgs`` — see :func:`collect_omni_msgs`).
    thr_out_msgs, rw_out_msgs
        Explicit per-effector message lists. ``None`` (default) collects
        them from ``omni_sc``; pass ``[]`` to wire spacecraft-only.
    model_tag, spacecraft_name
        Cosmetic tags forwarded to the ``VizInterface``
        (``spacecraftName`` when the attribute exists).

    Wiring order (each step independent, version-tolerant):

    1. ``scStateInMsgs.pushBack(scState)`` (plural, BSK 2.12.0); fallback
       ``scStateInMsg.subscribeTo(scState)`` (singular, older builds).
    2. ``thrInMsgs`` — one ``pushBack`` per THR output msg (skipped when
       the list is empty, e.g. before WP3 lands; the demo passes no RW
       list either — RW viz is already demo-out-of-scope).
    3. ``rwInMsgs`` — one ``pushBack`` per RW log msg (skipped when empty).

    Returns the configured ``viz`` object. Raises ``RuntimeError`` with an
    actionable message when no spacecraft port exists (instead of failing
    later inside ``InitializeSimulation``). Never imports the helper
    package.

    Smoke docstring (host gate)
    ---------------------------
    This function performs **zero simulation**: safe on Linux/macOS/CI.
    To run the live loop on the macOS host::

        WP6_RUN_VIZ_SIM=1 python -m pytest \\
            native_omni_heyoka/tests/test_wp6_telemetry.py -k viz_smoke -q

    Headless ``ExecuteSimulation`` with viz segfaults on some hosts (T2
    probe record); keep it gated until bisected. ``sim`` may be ``None``
    when the caller only wants the subscription record (host import test).
    """
    # Basilisk imports are function-local so `import omniheyoka.viz_direct`
    # works on hosts without Basilisk (Linux CI) — the host test pins this.
    from Basilisk.simulation import vizInterface

    collected = collect_omni_msgs(omni_sc)
    sc_msg = collected["scState"]
    thr_msgs = list(thr_out_msgs) if thr_out_msgs is not None else list(collected["thr"])
    rw_msgs = list(rw_out_msgs) if rw_out_msgs is not None else list(collected["rw"])

    viz = vizInterface.VizInterface()
    try:
        viz.ModelTag = model_tag
    except Exception:
        pass
    try:
        if hasattr(viz, "spacecraftName"):
            viz.spacecraftName = spacecraft_name
    except Exception:
        pass

    how: dict[str, str] = {}
    # 1. spacecraft state: plural-first, singular fallback.
    if sc_msg is None:
        raise RuntimeError(
            "configure_direct_viz: omni object exposes no scStateOutMsg; "
            "cannot wire VizInterface (collect_omni_msgs found nothing)."
        )
    wired_sc = False
    if hasattr(viz, "scStateInMsgs"):
        try:
            _push_vector(getattr(viz, "scStateInMsgs"), sc_msg)
            how["scState"] = "scStateInMsgs.pushBack"
            wired_sc = True
        except Exception:
            wired_sc = False
    if not wired_sc and hasattr(viz, "scStateInMsg"):
        _subscribe_single(getattr(viz, "scStateInMsg"), sc_msg)
        how["scState"] = "scStateInMsg.subscribeTo"
        wired_sc = True
    if not wired_sc:
        # Attr-dump fallback: any attribute with scState in the name.
        try:
            cands = [a for a in dir(viz) if "scState" in a and not a.startswith("_")]
        except Exception:
            cands = []
        raise RuntimeError(
            "configure_direct_viz: VizInterface exposes neither scStateInMsgs "
            f"nor scStateInMsg (candidates visible: {cands}). BSK version drift? "
            "Record dir(viz) via describe_viz_ports and update the helper."
        )
    # 2/3. effector vectors (each optional, each independent).
    for key, attr, msgs in (("thr", "thrInMsgs", thr_msgs), ("rw", "rwInMsgs", rw_msgs)):
        if not msgs:
            how[key] = "skipped(empty)"
            continue
        if not hasattr(viz, attr):
            how[key] = f"skipped(no-{attr})"
            continue
        try:
            vec = getattr(viz, attr)
            for m in msgs:
                _push_vector(vec, m)
            how[key] = f"{attr}.pushBack x{len(msgs)}"
        except Exception as exc:
            how[key] = f"failed({type(exc).__name__}: {exc})"
    # Stash the record on the object for the gated smoke test to assert.
    try:
        viz._omni_wire_how = how  # type: ignore[attr-defined]
    except Exception:
        pass
    return viz


def viz_smoke(
    sim: Any,
    viz: Any,
    dt: float = 0.1,
    nticks: int = 2,
    task_name: str = "vizTask",
    run_sim: bool | None = None,
) -> dict[str, Any]:
    """Gated live smoke: add ``viz`` to ``sim`` and optionally execute.

    Default (``run_sim=None``) executes ONLY when
    ``WP6_RUN_VIZ_SIM=1`` is set — otherwise returns
    ``{"status": "skipped", ...}`` after attempting ``InitializeSimulation``
    alone (the T2 static path: subscription + census, no headless run).
    Pass ``run_sim=True`` to force the run (host bisection only).

    Never raises: failures are returned as ``{"status": "guarded-fail"}``.
    """
    import traceback

    if run_sim is None:
        run_sim = os.environ.get("WP6_RUN_VIZ_SIM", "0") == "1"
    rec: dict[str, Any] = {
        "run_sim": bool(run_sim),
        "wire_how": getattr(viz, "_omni_wire_how", "unknown"),
        "ports": None,
    }
    try:
        rec["ports"] = describe_viz_ports(viz)
    except Exception:
        pass
    if sim is None:
        rec["status"] = "skipped(no-sim)"
        return rec
    try:
        # Task placement is intentionally explicit (caller-visible).
        try:
            proc = sim.CreateNewProcess("vizProc")
            proc.addTask(sim.CreateNewTask(task_name, int(float(dt) * 1e9)))
            sim.AddModelToTask(task_name, viz)
        except Exception:
            # Caller may have already placed viz on their own task; fall
            # through to init/run regardless.
            try:
                sim.AddModelToTask(task_name, viz)
            except Exception:
                pass
        if not run_sim:
            try:
                sim.InitializeSimulation()
                rec["init"] = "ok"
            except Exception as exc:
                rec["init"] = f"guarded-fail: {type(exc).__name__}: {exc}"
            rec["status"] = "skipped(WP6_RUN_VIZ_SIM!=1)"
            return rec
        sim.InitializeSimulation()
        sim.ConfigureStopTime(int(float(dt) * int(nticks) * 1e9))
        sim.ExecuteSimulation()
        rec["status"] = "ok(full-sim)"
        return rec
    except Exception as exc:
        rec["status"] = "guarded-fail"
        rec["error"] = f"{type(exc).__name__}: {exc}"
        rec["trace"] = traceback.format_exc(limit=3)
        return rec
