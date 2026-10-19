"""STAGED env-mirror reader: stock DragDynamicEffector (cannonball) -> omni drag.

STAGED -- NOT WIRED INTO ``mirror_stock_config`` (``omniheyoka/swap.py``).
This module lands alone per the env-mirror batch discipline: no existing
file is modified, and ``tests/test_replacement_matrix.py::
test_env_mirror_gap_open`` (which scans ``swap.py`` text only) stays green
until the wiring batch lands with its registry update (PLAN_ENV_MIRROR §8,
list.md §A1).  The reader is exercised host-free by
``tests/test_env_mirror_drag.py`` until then.

PLAN_ENV_MIRROR.md §2.1 -- cannonball drag mirror.  Every stock name below
was VERIFIED by grep against the installed SWIG wrapper
(``Basilisk/simulation/dragDynamicEffector.py``, cited ``wrapper:line``) and
the upstream Basilisk v2.12.0 sources (the wheel pinned in
``pyproject.toml``; cited ``upstream cpp:line`` / ``upstream h``); every omni
setter was verified against ``include/omni/vec_integrator.hpp`` (cited
``hpp:line``); C++ consumption rules against
``src/omni_vec_integrator.cpp`` (cited ``cpp:line``).

Wrapper surface (``wrapper:line`` -> use)::

    coreParams                     wrapper:643  DragBaseData record
      projectedArea                wrapper:599  [m^2]
      dragCoeff                    wrapper:600  [-]
      comOffset                    wrapper:601  [m] CP offset from c
    atmoDensInMsg                  wrapper:644  REQUIRED link (§1.5)
    windVelInMsg                   wrapper:645  OPTIONAL link (§1.5)
    modelType                      wrapper:646  std::string (upstream h);
                                              default AND only branch spelling
                                              "cannonball" (upstream cpp:29,
                                              cpp:182-183)
    densityCorrectionStateName     wrapper:649  std::string, default ""
                                              (upstream h; upstream cpp:106-109
                                              resolves the correction state)
    densityCorrection              wrapper:650  StateData* -- NEVER READ

Reader law (PLAN §2.1): ``CdA = dragCoeff * projectedArea`` and
``com = comOffset`` verbatim -> ``oi.setDragParams(CdA, cx, cy, cz)``
(``hpp:108``) or ``oi.setDragParamsForObject(obj, ...)`` (``hpp:175-176``,
object_index given).  The arm is NOT pre-shifted: the C++ drag arm is
``cLive - comOffset`` (``cpp:1187-1191``), so the recorded offset shifts
C++-side with the live aggregate c.

Links:
- atmo is REQUIRED: stock bskErrors when drag runs without a density source
  (upstream cpp:60-62) and omni does the same (``cpp:1163-1167``); an
  unlinked -- or linked-but-unrecoverable -- ``atmoDensInMsg`` rejects
  loudly here.  The stock reader's writer Message is recovered through the
  wrapper's own retention accessor ``_get_source_owner``
  (``AtmoPropsMsgPayload.py:401-412``; the accessor the wrapper's
  ``__call__`` uses to pin its source) and forwarded to the omni side.
- wind is OPTIONAL: an explicitly-linked ``windVelInMsg`` is forwarded the
  same way; unlinked stays unlinked = inertial-velocity fallback
  (``cpp:1170-1174``, §1.5) and is LABELED as such in the record.  A linked
  wind whose source cannot be recovered rejects loudly (dropping it is the
  documented ~6% LEO divergence, PLAN §2.4 / wp5_env ledger).

Subscription surface: there is NO bare ``oi.subscribeAtmo`` /
``oi.subscribeWind`` free function in the header (verified by reading
``vec_integrator.hpp`` in full).  Slot 0 subscribes through the public
ReadFunctor members ``oi.atmoDensInMsg`` / ``oi.windVelInMsg``
(``hpp:239-240``; ``.subscribeTo`` precedent ``swap.py:419`` and
``audit_bundle/probes/swap_p1_parity.py:467``); per-object slots use the
twins ``oi.subscribeAtmoForObject`` (``hpp:209``) and
``oi.subscribeWindForObject`` (``hpp:210``).

Loud rejects (``mirror_stock_config: ...`` message style, PLAN §1.2):
non-cannonball ``modelType`` (any other spelling runs NO drag model in
stock, upstream cpp:182-183 -- silent divergence, never allowed); a
configured ``densityCorrectionStateName`` (omni pins the 0.0 correction
literal, ``cpp:1190-1191``, with no correction setter); missing density
source; unrecoverable subscribed source; any class outside
``SUPPORTED_CLASSES`` (FacetDragDynamicEffector is the named family,
PLAN §2.2); more than one drag effector (order ambiguity, swap.py
convention).

SDK traps honoured (PLAN §7): ModelTag / planet-name / label string
surfaces are never touched (§7.1); the two string reads this reader needs
(``modelType``, ``densityCorrectionStateName``) are guarded, their
spellings pinned from the matching upstream v2.12.0 tag, and any read
failure REJECTS instead of guessing.  The live ``densityCorrection`` value
is never branched on (fresh-object reads are process-state-dependent
-- see ``tests/test_env_mirror_contract.py::
test_drag_correction_default_zero`` docstring); the gate is the state-name
LINKAGE (empty name = no correction, upstream cpp:106-109).  Nothing is
attached to the stock objects (§7.2); a plain record is returned.

HOST-TO-PIN (unverifiable in this container -- no Python/Basilisk runtime):
(1) the runtime ``modelType`` cannonball spelling (upstream-pinned
"cannonball"; the contract test only hasattr-pins the surface);
(2) that ``_get_source_owner()`` returns the writer Message for a normal
link -- it is the forwarding mechanism for BOTH required links.
"""

from __future__ import annotations

from typing import Any

__all__ = ["SUPPORTED_CLASSES", "mirror_drag_models"]

# Cannonball drag only (PLAN_ENV_MIRROR §2.1).  The envelope
# mirror_stock_config raises for stock env effectors ("use omni env setters
# instead", swap.py:146-150); this reader is the DragDynamicEffector leg of
# that setters surface.  Other drag families stay out: FacetDragDynamicEffector
# (§2.2, per-facet law structurally invisible to omni) and the MuJoCo
# CannonballDrag (§2.5, plain SysModel, passes through untouched).
SUPPORTED_CLASSES = {"DragDynamicEffector"}

# modelType spelling that selects cannonballDrag(): the constructor default
# AND the only branch in computeForceTorque (upstream v2.12.0
# dragDynamicEffector.cpp:29, :182-183).  HOST-TO-PIN at runtime.  Any other
# spelling makes stock run NO drag model at all (force/torque stay zero) --
# a silent divergence, so it rejects here instead.
_CANNONBALL_MODEL_TYPE = "cannonball"


def _as_vec3(v: Any) -> list:
    """Vec3 -> ``[float, float, float]`` (Eigen proxies via numpy, plain
    sequences via indexing; swap.py ``_as_vec3`` pattern)."""
    try:
        import numpy as np
        return [float(x) for x in np.asarray(v, dtype=float).ravel()[:3]]
    except ImportError:
        pass
    try:
        return [float(v[i]) for i in range(3)]
    except Exception as exc:
        raise ValueError(
            f"mirror_stock_config: DragDynamicEffector: cannot read a "
            f"3-vector from {type(v).__name__} without numpy ({exc!r})."
        ) from exc


def _guarded_read(model: Any, attr: str, what: str) -> Any:
    """Guarded attribute read.  A surface the reader cannot see is a LOUD
    reject, never a silent pass (PLAN §1.3)."""
    try:
        return getattr(model, attr)
    except Exception as exc:
        raise ValueError(
            f"mirror_stock_config: DragDynamicEffector: {what} unreadable "
            f"({exc!r}); the mirror gates need this surface and will not "
            f"guess.") from exc


def _num(holder: Any, attr: str, what: str) -> float:
    """Guarded numeric read of a coreParams member (PLAN §7.1: numerics may
    be read with skip-on-fail; here a failure rejects loudly)."""
    try:
        return float(getattr(holder, attr))
    except Exception as exc:
        raise ValueError(
            f"mirror_stock_config: DragDynamicEffector: coreParams."
            f"{attr} ({what}) unreadable ({exc!r}).") from exc


def _reader_linked(reader: Any) -> bool:
    """ReadFunctor link state via ``isLinked()`` (wrapper surface:
    AtmoPropsMsgPayload.py:350 / WindMsgPayload.py:350; stock's own
    validateConfiguration uses it, upstream cpp:60-62).  Unreadable/missing
    counts as UNLINKED so the caller rejects."""
    if reader is None:
        return False
    fn = getattr(reader, "isLinked", None)
    if not callable(fn):
        return False
    try:
        return bool(fn())
    except Exception:
        return False


def _linked_source(reader: Any) -> Any:
    """Recover the writer Message a subscription points at through the
    wrapper's own retention accessor ``_get_source_owner``
    (AtmoPropsMsgPayload.py:401-412 / WindMsgPayload.py:388-399 -- the
    accessor the wrapper's ``__call__`` uses to pin its source).  Returns
    None when the source cannot be recovered; callers reject loudly when the
    reader IS linked (a dropped link is the inertial-velocity divergence)."""
    getter = getattr(reader, "_get_source_owner", None)
    if not callable(getter):
        return None
    try:
        return getter()
    except Exception:
        return None


def mirror_drag_models(models: Any, oi: Any,
                       object_index: Any = None) -> dict:
    """Mirror stock cannonball drag effectors into ``oi`` (PLAN §2.1).

    ``models``: the drag-family task models found by the caller's census
    (swap.py task-scan convention: a plain list of stock objects).  At most
    one is mirrorable -- the omni slot carries a single drag record, so two
    effectors are an order ambiguity (same convention as the RW/thruster/
    fuel mirrors).  ``object_index=None`` is the legacy slot-0 surface
    (``oi.setDragParams`` + member subscribes); an integer selects the
    per-object twins (``...ForObject``; an explicit 0 is allowed and
    semantically identical, ``hpp:89-93``).

    Returns the mirrored record for ledgers/tests.  Every reject is a
    ValueError with the ``mirror_stock_config: ...`` convention (PLAN §1.2),
    raised before any setter fires for the config gates.
    """
    model_list = list(models or [])
    if not model_list:
        return {"n_drag": 0, "object_index": object_index}
    if len(model_list) > 1:
        raise ValueError(
            f"mirror_stock_config: {len(model_list)} drag effectors; stock "
            f"registration order is ambiguous -- use one effector.")
    model = model_list[0]
    cls = type(model).__name__

    # --- class gate (envelope wording, swap.py:146-150) ---------------------
    if cls not in SUPPORTED_CLASSES:
        if cls == "FacetDragDynamicEffector":
            reason = ("per-facet drag law is structurally invisible to omni "
                      "(no per-facet drag setter; setDragParams is "
                      "cannonball-only, PLAN_ENV_MIRROR §2.2)")
        else:
            reason = (f"{cls}: outside SUPPORTED_CLASSES (DragDynamicEffector "
                      f"only, PLAN_ENV_MIRROR §2.1)")
        raise ValueError(
            f"mirror_stock_config: task list holds {cls}; envelope is hub + "
            f"wheels + staircase thrusters + central gravity + explicit omni "
            f"env (stock env effectors: use omni env setters instead); "
            f"{reason}.")

    # --- shape gate (wrapper-verified surface) ------------------------------
    core = getattr(model, "coreParams", None)
    if core is None:
        raise ValueError(
            "mirror_stock_config: DragDynamicEffector: coreParams missing; "
            "projectedArea/dragCoeff/comOffset (dragDynamicEffector.py:643, "
            "DragBaseData :599-601) are the mirrored surface.")

    # --- mode gate: cannonball only -----------------------------------------
    model_type = _guarded_read(model, "modelType", "modelType")
    if (model_type is None
            or str(model_type).strip() != _CANNONBALL_MODEL_TYPE):
        raise ValueError(
            f"mirror_stock_config: DragDynamicEffector: non-cannonball "
            f"modelType {model_type!r}; the omni drag surface (setDragParams, "
            f"vec_integrator.hpp:108) is cannonball-only and stock runs NO "
            f"drag model for any other spelling (upstream v2.12.0 "
            f"dragDynamicEffector.cpp:182-183) -- host must pin the spelling.")

    # --- correction-LINKAGE gate (never a live densityCorrection read) ------
    corr_name = _guarded_read(model, "densityCorrectionStateName",
                              "densityCorrectionStateName")
    if corr_name is not None and str(corr_name).strip() != "":
        raise ValueError(
            f"mirror_stock_config: DragDynamicEffector: densityCorrection "
            f"state {corr_name!r} configured; omni pins the 0.0 correction "
            f"literal with no correction setter "
            f"(src/omni_vec_integrator.cpp:1190-1191) and the live "
            f"densityCorrection value is never read (contract-test policy: "
            f"fresh-object reads are process-state-dependent) -- reject.")

    # --- required atmo link (stock bskError, cpp:1163-1167) -----------------
    atmo_reader = getattr(model, "atmoDensInMsg", None)
    if not _reader_linked(atmo_reader):
        raise ValueError(
            "mirror_stock_config: DragDynamicEffector: atmoDensInMsg "
            "unlinked; drag without a density source bskErrors in stock "
            "(upstream v2.12.0 dragDynamicEffector.cpp:60-62) and in omni "
            "(src/omni_vec_integrator.cpp:1163-1167) -- subscribe the "
            "atmosphere model's envOutMsgs[i] before mirroring.")
    atmo_src = _linked_source(atmo_reader)
    if atmo_src is None:
        raise ValueError(
            "mirror_stock_config: DragDynamicEffector: atmoDensInMsg is "
            "linked but its source message could not be recovered "
            "(_get_source_owner, AtmoPropsMsgPayload.py:401-412); the "
            "density link is REQUIRED and cannot be dropped silently.")

    # --- params: CdA = dragCoeff * projectedArea, com verbatim --------------
    area = _num(core, "projectedArea", "projectedArea")
    cd = _num(core, "dragCoeff", "dragCoeff")
    try:
        com_raw = core.comOffset
    except Exception as exc:
        raise ValueError(
            f"mirror_stock_config: DragDynamicEffector: coreParams.comOffset "
            f"unreadable ({exc!r}).") from exc
    com = _as_vec3(com_raw)  # verbatim; the C++ arm is cLive - comOffset
    cda = cd * area

    if object_index is None:
        oi.setDragParams(cda, com[0], com[1], com[2])
        oi.atmoDensInMsg.subscribeTo(atmo_src)
    else:
        oi.setDragParamsForObject(object_index, cda, com[0], com[1], com[2])
        oi.subscribeAtmoForObject(object_index, atmo_src)

    # --- optional wind link: forward or labeled inertial fallback -----------
    wind_reader = getattr(model, "windVelInMsg", None)
    wind = "inertial-fallback"
    if _reader_linked(wind_reader):
        wind_src = _linked_source(wind_reader)
        if wind_src is None:
            raise ValueError(
                "mirror_stock_config: DragDynamicEffector: windVelInMsg is "
                "linked but its source message could not be recovered "
                "(_get_source_owner, WindMsgPayload.py:388-399); dropping a "
                "linked wind would be the inertial-velocity divergence "
                "(~6% LEO, PLAN §1.5/§2.4) -- reject.")
        if object_index is None:
            oi.windVelInMsg.subscribeTo(wind_src)
        else:
            oi.subscribeWindForObject(object_index, wind_src)
        wind = "forwarded"

    return {"n_drag": 1, "class": cls, "CdA": cda, "projectedArea": area,
            "dragCoeff": cd, "comOffset": com, "atmo": "linked",
            "wind": wind, "object_index": object_index}
