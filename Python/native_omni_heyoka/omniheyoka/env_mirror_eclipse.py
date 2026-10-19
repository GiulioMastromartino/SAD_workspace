"""STAGED env-mirror reader: stock Eclipse field model -> omni eclipse link.

STAGED -- NOT WIRED INTO ``mirror_stock_config`` (``omniheyoka/swap.py``).
This module lands alone per the env-mirror batch discipline: no existing
file is modified, and ``tests/test_replacement_matrix.py::
test_env_mirror_gap_open`` (which scans ``swap.py`` text only) stays green
until the wiring batch lands with its registry update (PLAN_ENV_MIRROR §8,
list.md §A1).  The reader is exercised host-free by
``tests/test_env_mirror_eclipse.py`` until then.

PLAN_ENV_MIRROR.md §5 -- eclipse mirror.  Every stock name below was
VERIFIED by grep against the installed SWIG wrapper
(``Basilisk/simulation/eclipse.py``, cited ``wrapper:line``) and the
shared-object symbol table of the installed wheel
(``Basilisk/simulation/_eclipse.so``, cited ``so:symbol``); every omni
setter/link was verified against ``include/omni/vec_integrator.hpp``
(cited ``hpp:line``); C++ consumption rules against
``src/omni_vec_integrator.cpp`` (cited ``cpp:line``).

Wrapper surface (``wrapper:line`` -> use)::

    class Eclipse              wrapper:352   plain SysModel -- §1.1: the
                                 module STAYS in the task, only its
                                 out-message is wired
    addSpacecraftToModel()     wrapper:369   ONE call per spacecraft; it
                                 appends one entry to positionInMsgs AND
                                 one to eclipseOutMsgs, in call order
    addPlanetToModel()         wrapper:372   planet entry (ledger count)
    sunInMsg                   wrapper:374   sun entry (ledger)
    planetInMsgs               wrapper:375   planet entries (ledger count)
    positionInMsgs             wrapper:376   sc-registration vector, the
                                 call-order cross-check
    eclipseOutMsgs             wrapper:377   THE MIRRORED SURFACE (§7.4
                                 plural vector<Message<EclipseMsgPayload>*>)
    rEqCustom                  wrapper:379   double (ledger)

Message types verified from the wheel symbol table (``so:symbol``):
``addSpacecraftToModel`` takes ``Message<SCStatesMsgPayload>*``
(``Eclipse_addSpacecraftToModel``), ``addPlanetToModel`` takes
``Message<SpicePlanetStateMsgPayload>*``
(``Eclipse_addPlanetToModel``), ``sunInMsg`` / ``planetInMsgs`` are
SpicePlanetState readers, ``positionInMsgs`` is an SCStatesMsgPayload
reader vector, and ``eclipseOutMsgs`` is a vector of message pointers.

Payload (``wrapper:line``)::

    class EclipseMsgPayload          wrapper:383
    shadowFactor                    wrapper:386  ONE field only

    ``illuminationFactor`` is a deprecated C alias, never read: the
    wrapper exposes exactly ``EclipseMsgPayload_shadowFactor_get/_set``
    (verified: no ``EclipseMsgPayload_illuminationFactor*`` symbol in the
    installed ``_eclipse.so``) and ``include/omni/environment.hpp:60-61``
    says "shadowFactor ... illuminationFactor is a deprecated C #define
    alias, never read".

Subscription surface (``hpp``): the header has NO bare
``oi.subscribeEclipse`` -- the only symbol matching ``subscribeEclipse``
anywhere under ``include/`` + ``src/`` is
``subscribeEclipseForObject`` (``hpp:211-212``,
``cpp:476-482``).  So this reader picks the slot-0 surface the same way
the drag module does for ``atmoDensInMsg`` (``hpp:239``):
- ``object_index=None`` -> the public ReadFunctor member
  ``oi.eclipseInMsg`` (``hpp:241``) with ``.subscribeTo(src)``
  (precedent ``swap.py:419`` ``oi.thrOnTimeInMsg.subscribeTo(...)``);
- ``object_index`` int -> the ``...ForObject`` twin
  ``oi.subscribeEclipseForObject(obj, src)`` (``hpp:211-212``).

Reader law (PLAN §5): the Eclipse module is a FIELD module (plain
``SysModel``, wrapper:352) and PASSES THROUGH the task untouched
(§1.1); the reader only ADDS the eclipse link from its out-messages,
exactly like the atmo/mag links of the sibling readers.  Stock
registration order is the attribution key: ``addSpacecraftToModel``
appends one out-message per spacecraft (wrapper:369), so
``eclipseOutMsgs[i]`` is the eclipse out-message of the i-th registered
spacecraft.  One registered spacecraft -> ``eclipseOutMsgs[0]`` ->
``oi.eclipseInMsg`` (slot-0) / ``subscribeEclipseForObject`` (any int).
Several registered spacecraft -> ORDER-MATCH ``eclipseOutMsgs[obj]``;
the caller must mirror object ``obj`` in the same order it registered
the spacecraft (the ``swap.py`` per-object convention, ``swap.py:256``
-- the task-list scan cannot attribute field modules to objects, §7.3).

UNLINKED STAYS LEGAL.  An absent or never-registered Eclipse model, or a
model with an empty ``eclipseOutMsgs``, is NOT an error: the omni eclipse
slot simply stays unlinked and reads 1.0 (full sun) --
``src/omni_vec_integrator.cpp:1220-1222``
``double shadow = 1.0; if (M.eclipse->isLinked()) shadow =
(*M.eclipse)().shadowFactor;`` and
``include/omni/environment.hpp:59-62`` "eclipse unlinked -> silent-ON
1.0 (full sun ...)".  Stock agrees (PLAN §1.5 table, ``cpp:1220-1222``),
so the reader RECORDS the unlinked-legal state (no subscribe, no
reject).  A version where the omni side would silently run full sun
while stock ran the eclipse is impossible here: the ONLY way stock
computes a shadowFactor is through this same out-message.

Loud rejects (``mirror_stock_config: ...`` message style, PLAN §1.2):
any class outside ``SUPPORTED_CLASSES`` (the eclipse family is one class
-- unknowns are other field laws, §1.3); more than one Eclipse model
(two out-message vectors would compete for one omni link); several
registered spacecraft on the legacy slot-0 path (attribution
ambiguous); an ``object_index`` with no registered spacecraft (or one
that is not a usable int); a registration whose out-vector and
position-vector disagree (call order no longer inferable); a
missing/null out-message entry; and an ``oi`` that is missing or lacks
the verified subscribe surface.

SDK traps honoured (PLAN §7): no string-typed surface is ever read
(§7.1) -- the reader touches only numerics (``rEqCustom``, guarded,
informational) and message vectors; nothing is attached to the stock
objects (§7.2); out-messages are read through the plural
``eclipseOutMsgs``, never a singular (``wrapper:377``, §7.4).  Eclipse
needs a sun and a planet entry to compute anything -- stock bskErrors
otherwise (``wp5_env.py:466`` "eclipse no sc/planet -> bskError") --
so the reader RECORDS ``sun_linked`` / ``planet_msgs`` /
``position_msgs`` in the ledger instead of gating on them: they are
stock-runtime requirements of a field module that passes through
untouched (§1.1), not omni config surface.
"""

from __future__ import annotations

from typing import Any

__all__ = ["SUPPORTED_CLASSES", "PAYLOAD_FIELD",
           "mirror_eclipse_models"]

# PLAN §5: the stock eclipse family is exactly one class.  Its payload is
# one scalar field, so there is no per-law variant to accept or reject.
SUPPORTED_CLASSES = {"Eclipse"}

# EclipseMsgPayload's ONE field (wrapper eclipse.py:386; sole symbols in
# _eclipse.so).  ``illuminationFactor`` is the deprecated C alias and is
# never read (environment.hpp:60-61).
PAYLOAD_FIELD = "shadowFactor"

# Unlinked eclipse link reads this, on BOTH paths (cpp:1220-1222,
# environment.hpp:59-62).  The value is legal, not an error, so it is
# RECORDED rather than rejected.
UNLINKED_SHADOW_FACTOR = 1.0

# Attribution note carried in the record (swap.py per-object convention).
_ORDER_MATCH = ("eclipseOutMsgs[i] in addSpacecraftToModel order "
                "(wrapper eclipse.py:369 appends one out-message per "
                "spacecraft; mirror object i in the same order it "
                "registered that spacecraft)")

_ENVELOPE = ("envelope is hub + wheels + staircase thrusters + central "
             "gravity + explicit omni env (stock env effectors: use omni "
             "env setters instead)")

_SLOT0_LINK = "eclipseInMsg.subscribeTo"
_FOROBJ_LINK = "subscribeEclipseForObject"
_UNLINKED = "unlinked"


def mirror_eclipse_models(models: Any, oi: Any,
                          object_index: Any = None) -> dict:
    """Mirror stock Eclipse field models into ``oi`` (PLAN §5).

    ``models``: the eclipse-family task models found by the caller's
    census (swap.py task-scan convention: a plain list of stock
    objects).  At most one is mirrorable -- one omni eclipse slot
    carries one scalar link, so two Eclipse modules are an attribution
    ambiguity (same convention as the RW/thruster/fuel mirrors).
    ``object_index=None`` is the legacy slot-0 surface
    (``oi.eclipseInMsg.subscribeTo``, ``hpp:241``); an integer selects
    the per-object twin (``oi.subscribeEclipseForObject``, ``hpp:211``;
    an explicit 0 is allowed and semantically identical to slot 0).

    Returns the record for ledgers/tests: counts, the registration
    census, the wired link name (``eclipse_link``), the forwarded source
    message (``eclipse_src``) and the legal-unlinked state
    (``unlinked_legal`` / ``shadow_factor_default``).  Every reject is a
    ValueError with the ``mirror_stock_config: ...`` convention (PLAN
    §1.2), raised before any subscribe fires.
    """
    model_list = list(models or [])
    if not model_list:
        return _record(object_index, 0)
    if len(model_list) > 1:
        raise ValueError(
            f"mirror_stock_config: {len(model_list)} Eclipse task models; "
            f"stock registration order is ambiguous -- use one module "
            f"(one omni eclipse slot, PLAN_ENV_MIRROR §5).")
    model = model_list[0]
    cls = type(model).__name__

    # --- class gate (envelope wording, swap.py:146-150) --------------------
    if cls not in SUPPORTED_CLASSES:
        raise ValueError(
            f"mirror_stock_config: task list holds {cls}; {_ENVELOPE}; "
            f"{cls}: outside SUPPORTED_CLASSES (Eclipse only, "
            f"PLAN_ENV_MIRROR §5 -- the stock eclipse family is one class, "
            f"so anything else is an unknown field law).")

    rec = _record(object_index, 1, cls=cls)
    _ledger_fields(rec, model)

    # --- registration census (the mirrored surface) ------------------------
    outs = _msg_vector(model, "eclipseOutMsgs", required=True)
    ins = _msg_vector(model, "positionInMsgs", required=False)
    rec["spacecraft_registered"] = len(outs)
    rec["position_msgs"] = None if ins is None else len(ins)
    if ins is not None and len(ins) != len(outs):
        raise ValueError(
            f"mirror_stock_config: Eclipse: positionInMsgs has {len(ins)} "
            f"entries but eclipseOutMsgs has {len(outs)}; "
            f"addSpacecraftToModel (wrapper eclipse.py:369) appends one "
            f"entry to BOTH in call order, so the registration order is "
            f"no longer inferable -- refuse a guessed attribution.")

    # --- UNLINKED STAYS LEGAL (factor 1.0, no subscribe, no reject) -------
    if not outs:
        return rec

    if object_index is None:
        if len(outs) > 1:
            raise ValueError(
                f"mirror_stock_config: Eclipse: {len(outs)} registered "
                f"spacecraft on the legacy slot-0 path; registration "
                f"order is ambiguous -- eclipseOutMsgs[i] belongs to the "
                f"i-th addSpacecraftToModel registered spacecraft "
                f"(wrapper eclipse.py:369/:377), so which one is slot 0 "
                f"is not inferable; mirror each object with its "
                f"object_index instead.")
        idx = 0
    else:
        obj = _as_index(object_index)
        if obj >= len(outs):
            raise ValueError(
                f"mirror_stock_config: Eclipse: object_index {obj} has no "
                f"registered spacecraft (eclipseOutMsgs has "
                f"{len(outs)} entries, addSpacecraftToModel order, "
                f"wrapper eclipse.py:369); register the spacecraft or "
                f"mirror an existing object_index -- never drop the link "
                f"silently (it would run full sun).")
        idx = obj

    src = outs[idx]
    if src is None:
        raise ValueError(
            f"mirror_stock_config: Eclipse: eclipseOutMsgs[{idx}] is a "
            f"null message entry; the eclipse link (shadowFactor) cannot "
            f"be forwarded from it (wrapper eclipse.py:377).")

    rec["eclipse_link"] = _wire(oi, src, object_index)
    rec["eclipse_src"] = src
    rec["eclipse_out_index"] = idx
    rec["linked"] = True
    return rec


def _record(object_index: Any, n_eclipse: int, cls: str = None) -> dict:
    """Zero/unlinked-shaped record: legal, and no link fired."""
    return {"n_eclipse": n_eclipse, "class": cls,
            "object_index": object_index,
            "spacecraft_registered": 0,
            "position_msgs": None, "planet_msgs": None,
            "sun_linked": None, "rEqCustom": None,
            "payload_field": PAYLOAD_FIELD,
            "shadow_factor_default": UNLINKED_SHADOW_FACTOR,
            "unlinked_legal": True, "linked": False,
            "eclipse_link": _UNLINKED, "eclipse_src": None,
            "eclipse_out_index": None,
            "order_match": _ORDER_MATCH}


def _ledger_fields(rec: dict, model: Any) -> None:
    """Record the stock field-module shape (never gate on it).

    ``planetInMsgs`` / ``sunInMsg`` / ``rEqCustom`` are stock-internal
    config: the module passes through untouched (§1.1), and its own
    runtime requirements (a sun and a planet entry) bskError on the
    stock side (``wp5_env.py:466``), which the ledger can flag later.
    Every read is guarded and informational -- a surface that cannot be
    read becomes ``None`` and never rejects (PLAN §7.1 skip-on-fail for
    numerics/messages; nothing here is physics the omni graph consumes).
    """
    planets = _msg_vector(model, "planetInMsgs", required=False)
    rec["planet_msgs"] = None if planets is None else len(planets)
    rec["sun_linked"] = _linked(_attr(model, "sunInMsg"))
    rec["rEqCustom"] = _num_or_none(model, "rEqCustom")


def _wire(oi: Any, src: Any, object_index: Any) -> str:
    """Subscribe ``src`` into ``oi``; return the link name.

    Slot 0 has no ``subscribeEclipse`` method -- verified absent from
    ``include/omni/vec_integrator.hpp`` (the only ``subscribeEclipse``
    symbol under ``include/`` + ``src/`` is ``subscribeEclipseForObject``,
    ``hpp:211-212`` / ``cpp:476-482``).  The legacy surface is therefore
    the ``eclipseInMsg`` ReadFunctor member (``hpp:241``,
    ``cpp:576`` ``M.eclipse = &this->eclipseInMsg;``) with
    ``subscribeTo`` -- the same pattern as the drag module's
    ``atmoDensInMsg`` (``hpp:239``) and ``swap.py:419``.  Never branches
    to a plural/singular out-message name (§7.4).
    """
    if oi is None:
        raise ValueError(
            "mirror_stock_config: mirror_eclipse_models needs an omni "
            "integrator (oi) to subscribe the eclipse out-message; the "
            "UNLINKED-legal path needs none (PLAN §5 / §1.5).")
    if object_index is None:
        functor = _attr(oi, "eclipseInMsg")
        if functor is None or not hasattr(functor, "subscribeTo"):
            raise ValueError(
                "mirror_stock_config: oi has no eclipseInMsg ReadFunctor "
                "member; the slot-0 eclipse link needs it (expected "
                "include/omni/vec_integrator.hpp:241 -- there is no bare "
                "oi.subscribeEclipse, only subscribeEclipseForObject at "
                "hpp:211).")
        functor.subscribeTo(src)
        return _SLOT0_LINK
    subscribe = _attr(oi, "subscribeEclipseForObject")
    if not callable(subscribe):
        raise ValueError(
            "mirror_stock_config: oi has no subscribeEclipseForObject; "
            "cannot wire the eclipse link (expected "
            "include/omni/vec_integrator.hpp:211-212).")
    subscribe(object_index, src)
    return _FOROBJ_LINK


def _attr(holder: Any, name: str):
    """Attribute read that keeps a raising getter from escaping: an
    unreadable (or missing) surface returns ``None`` so the caller
    rejects loudly instead of guessing."""
    try:
        return getattr(holder, name)
    except Exception:
        return None


def _msg_vector(model: Any, attr: str, required: bool):
    """Read a stock out/registration message vector (plural surface,
    §7.4).  ``required=True`` is the mirrored surface: a missing,
    unreadable, ``None`` or non-sequence value REJECTS (the shadowFactor
    link cannot be dropped silently).  ``required=False`` is the ledger
    cross-check: any of those returns ``None`` (informational)."""
    try:
        raw = getattr(model, attr)
    except Exception as exc:
        if required:
            raise ValueError(
                f"mirror_stock_config: Eclipse: {attr} unreadable "
                f"({exc!r}); the mirrored surface (wrapper "
                f"eclipse.py:377) is needed to forward the eclipse "
                f"link.") from exc
        return None
    if raw is None:
        if required:
            raise ValueError(
                f"mirror_stock_config: Eclipse: {attr} is None; the "
                f"registration vector (wrapper eclipse.py:377) is "
                f"required to forward the eclipse link.")
        return None
    try:
        return list(raw)
    except Exception as exc:
        if required:
            raise ValueError(
                f"mirror_stock_config: Eclipse: {attr} is not a message "
                f"vector ({exc!r}); expected the wrapper surface "
                f"eclipse.py:377.") from exc
        return None


def _as_index(object_index: Any) -> int:
    """Per-object slot index, guarded (never a string, PLAN §7.1)."""
    try:
        idx = int(object_index)
    except Exception as exc:
        raise ValueError(
            "mirror_stock_config: object_index must be None (legacy "
            f"slot 0) or an int; got {object_index!r} ({exc!r}).") \
            from exc
    if idx < 0:
        raise ValueError(
            f"mirror_stock_config: object_index {idx} is negative; "
            f"omni slots are 0-based (swap.py per-object convention).")
    return idx


def _linked(msg: Any):
    """Best-effort linkage probe via ``isLinked()`` (wrapper surface;
    the stock ReadFunctor's own convention).  Informational only --
    returns None when undeterminable."""
    if msg is None:
        return None
    fn = _attr(msg, "isLinked")
    if callable(fn):
        try:
            return bool(fn())
        except Exception:
            return None
    return None


def _num_or_none(holder: Any, attr: str):
    """Guarded numeric read (PLAN §7.1 allows skip-on-fail for
    numerics); informational, never a gate."""
    value = _attr(holder, attr)
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None
