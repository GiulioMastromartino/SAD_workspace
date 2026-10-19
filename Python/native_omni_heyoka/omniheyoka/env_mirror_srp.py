"""Stock SRP env-effector mirroring (PLAN_ENV_MIRROR §3) -- STAGED.

STAGED LANDING: this is a standalone reader. It is NOT imported by
``omniheyoka/swap.py`` and nothing in the swap envelope changed
(``swap.py:39-40`` ``_MIRRORED`` still rejects every stock env
effector), so ``tests/test_replacement_matrix.py::
test_env_mirror_gap_open`` stays green until the wiring batch lands this
family. No message is subscribed here: the sun/eclipse link NEEDS are
recorded in the returned ``link_needs`` list for the wiring batch.

Everything below was verified statically against the Basilisk 2.12
wrapper files and the omni headers/sources; nothing was executed (this
container has no Python/compiler/Basilisk runtime -- the host owns first
execution).

Stock surface (``Basilisk/simulation/*.py``):
* ``RadiationPressure`` (``radiationPressure.py:598``): ``area`` :635
  and ``coefficientReflection`` :636 are plain doubles (safe value
  reads, unlike state-backed fields); ``sunEphmInMsg`` :637 (REQUIRED --
  C++ errors on SRP-on + unlinked sun, ``src/omni_vec_integrator.cpp:
  1199-1205``); ``sunEclipseInMsg`` :638 (OPTIONAL -- unlinked = full
  sun, factor 1.0, ``src/omni_vec_integrator.cpp:1220-1222``); lookup
  surface ``lookupForce_B`` :639 / ``lookupTorque_B`` :640 /
  ``lookupSHat_B`` :641. Mode is SELECTED by
  ``setUseCannonballModel`` :621 (the stock default per the module tech
  memo) or ``setUseFacetedCPUModel`` :624, and the C++ ``srpModel`` enum
  is PRIVATE with no wrapper getter: faceted-CPU mode is therefore gated
  by the caller-passed ``mode`` plus the readable lookup entries (the
  same "host must pin the selection" discipline as the drag
  ``modelType`` gate, PLAN §2.1).
* ``FacetSRPDynamicEffector`` (``facetSRPDynamicEffector.py:613``):
  ``setNumFacets`` :633, ``setNumArticulatedFacets`` :636,
  ``getNumFacets`` :639, ``getNumArticulatedFacets`` :642,
  ``addFacet(area, dcm_F0B, nHat_F, rotHat_F, r_CopB_B, diffuseCoeff,
  specularCoeff)`` :645, ``addArticulatedFacet`` :648; links
  ``sunInMsg`` :653 (REQUIRED), ``sunEclipseInMsg`` :654 (optional).
  The facet geometry lives in the PRIVATE ``scGeometry`` member
  (``FacetedSRPSpacecraftGeometryData``): the wrapper exposes its seven
  lists only on the standalone proxy (``facetSRPDynamicEffector.py:
  599-605``), NOT on the effector proxy (verified: no
  ``FacetSRPDynamicEffector_facetAreaList_get`` symbol in
  ``_facetSRPDynamicEffector.so``; the C++ header keeps
  ``scGeometry`` private). Per-facet specs therefore arrive either from
  an object that exposes the lists (guarded read -- fakes, future
  builds) or from the caller-passed ``facet_specs`` captured at
  ``addFacet`` time, exactly the stock-fuel/thruster staging pattern
  (``swap.py:441-476``, geometry join ``swap.py:361-367``).

Omni targets (``include/omni/vec_integrator.hpp``):
* ``setSRPCannonball(area, cr)`` :112 (slot 0) /
  ``setSRPCannonballForObject(obj, area, cr)`` :181. ``cr`` is the
  SPECULAR FRACTION r in [0,1]; stock ``coefficientReflection`` is the
  TOTAL multiplier C_R in [1,2], so ``cr = C_R - 1`` (:113-116; range
  check ``src/omni_vec_integrator.cpp:285-288``).
* ``addSRPFacet(area, nx,ny,nz, copx,copy,copz, c_diffuse,
  c_specular)`` :109-111 / ``addSRPFacetForObject`` :177-180; the normal
  is normalized C++-side (``src/omni_vec_integrator.cpp:265-272``) and
  ``r_CopB_B`` is passed VERBATIM (the arm shifts with the live
  aggregate c C++-side, PLAN §1.6).
* Sun/eclipse LINKS: the omni surface has NO slot-0 ``subscribeSun`` /
  ``subscribeEclipse`` -- only ``subscribeSunForObject`` :213 /
  ``subscribeEclipseForObject`` :211 plus the slot-0 message members
  ``sunStateInMsg`` :242 / ``eclipseInMsg`` :241 with ``subscribeTo``
  (precedent ``swap.py:419``). Those calls also need the sun
  OUT-message, and a stock effector's reader functor does not expose its
  source in Python, so this staged reader records the need and the
  wiring batch resolves the source (PLAN §5 eclipse module / §6.3
  ephemeris providers).

Rejects are loud (``ValueError``, ``mirror_stock_config:`` wording per
PLAN §1.2) and pre-sim: the message-driven faceted path
(``FacetedSRPEffector`` + ``FacetedSpacecraftProjectedArea``), the
faceted-CPU/lookup ``RadiationPressure`` models, nonzero
``rotHat``-equivalents, articulated/time-varying facets, and any facet
count/list disagreement.
"""

from __future__ import annotations

import math
from typing import Any

__all__ = ["SUPPORTED_CLASSES", "mirror_srp_models"]

SUPPORTED_CLASSES = {"RadiationPressure", "FacetSRPDynamicEffector"}

_ENVELOPE = ("envelope is hub + wheels + staircase thrusters + central "
             "gravity + explicit omni env (stock env effectors: use omni "
             "env setters instead)")

_REJECT_CLASSES = {"FacetedSRPEffector", "FacetedSpacecraftProjectedArea"}

_MODE_ALIASES = {"cannonball": "cannonball", "srp_cannonball_model":
                 "cannonball"}
_FACETED_MODE_SPELLINGS = ("faceted_cpu", "srp_faceted_cpu_model",
                           "faceted", "cpu", "lookup", "table",
                           "table_lookup")

_LOOKUP_ATTRS = ("lookupForce_B", "lookupTorque_B", "lookupSHat_B")

_FACET_LIST_ATTRS = (
    "facetAreaList", "facetDcm_F0BList", "facetNHat_FList",
    "facetRotHat_FList", "facetR_CopB_BList", "facetDiffuseCoeffList",
    "facetSpecularCoeffList")

_FACET_SPEC_KEYS = ("area", "dcm_F0B", "nHat_F", "rotHat_F", "r_CopB_B",
                    "diffuseCoeff", "specularCoeff")

_IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

_LINK_NOTE = ("resolve the OUT-message source in the wiring batch "
              "(PLAN_ENV_MIRROR §5 eclipse module / §6.3 ephemeris "
              "providers): a stock reader functor does not expose its "
              "source in Python, and the verified omni surface has no "
              "slot-0 subscribeSun/subscribeEclipse (only the "
              "...ForObject twins and the slot-0 message members)")

_LINK_TARGETS = {
    "sun": {"slot0_msg": "sunStateInMsg",
            "for_object": "subscribeSunForObject",
            "slot0_subscribe": "subscribeSun"},
    "eclipse": {"slot0_msg": "eclipseInMsg",
                "for_object": "subscribeEclipseForObject",
                "slot0_subscribe": "subscribeEclipse"},
}


def _reject(cls: str, reason: str) -> ValueError:
    return ValueError(
        f"mirror_stock_config: {cls} is not mirrorable; {_ENVELOPE}; "
        f"{cls}: {reason}.")


def _model_list(models: Any) -> list:
    if models is None:
        return []
    if isinstance(models, (list, tuple)):
        return list(models)
    return [models]


def _call_int(fn: Any):
    if not callable(fn):
        return None
    try:
        return int(fn())
    except Exception:
        return None


def _read_float(model: Any, attr: str):
    try:
        value = getattr(model, attr)
    except Exception:
        return None
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _nonempty(value: Any) -> bool:
    """True when a stock lookup vector holds entries. ``None`` and empty
    vectors are no entries; a value that cannot be sized at all counts as
    present (fail loud, never silently mirror a lookup table)."""
    if value is None:
        return False
    try:
        return len(value) > 0
    except TypeError:
        pass
    try:
        return len(list(value)) > 0
    except TypeError:
        return True


def _linked(msg: Any):
    """Best-effort linkage probe (precedent: SimulationBaseClass.py
    ``_messageIsLinked``). Informational only -- never branched on for
    gating (fresh-object VALUE reads are process-state-dependent, PLAN
    §2.1). Returns None when undeterminable."""
    if msg is None:
        return None
    fn = getattr(msg, "isLinked", None)
    if callable(fn):
        try:
            return bool(fn())
        except Exception:
            pass
    header = getattr(msg, "header", None)
    if header is not None:
        try:
            return bool(header.isLinked)
        except Exception:
            pass
    return None


def _mode_key(cls: str, mode: Any) -> str:
    if mode is None:
        return "cannonball"
    key = str(mode).strip().lower().replace("-", "_").replace(" ", "_")
    if key in _MODE_ALIASES:
        return _MODE_ALIASES[key]
    if key in _FACETED_MODE_SPELLINGS:
        raise _reject(cls, f"SRP mode={mode!r} is the faceted-CPU/lookup "
                           "model; the CPU facet law and lookup tables are "
                           "structural state omni cannot see (PLAN_ENV_MIRROR "
                           "§3.1)")
    raise _reject(cls, f"SRP mode={mode!r} unrecognized; pass None (stock "
                       "default cannonball) or 'cannonball' "
                       "(PLAN_ENV_MIRROR §3.1)")


def _spec_dict(spec: Any, i: int) -> dict:
    if isinstance(spec, dict):
        return spec
    try:
        seq = list(spec)
    except TypeError:
        raise ValueError(
            f"mirror_stock_config: facet spec {i} is not a mapping or a "
            f"{len(_FACET_SPEC_KEYS)}-field addFacet record: {spec!r}")
    if len(seq) != len(_FACET_SPEC_KEYS):
        raise ValueError(
            f"mirror_stock_config: facet spec {i} needs the "
            f"{len(_FACET_SPEC_KEYS)} addFacet fields "
            f"({', '.join(_FACET_SPEC_KEYS)}), got {len(seq)}")
    return dict(zip(_FACET_SPEC_KEYS, seq))


def _as_spec_list(facet_specs: Any) -> list:
    if facet_specs is None:
        return []
    if isinstance(facet_specs, dict):
        facet_specs = [facet_specs]
    return [_spec_dict(spec, i) for i, spec in enumerate(facet_specs)]


def _spec_float(spec: dict, key: str, i: int) -> float:
    if key not in spec or spec[key] is None:
        raise ValueError(
            f"mirror_stock_config: facet spec {i} is missing {key}")
    try:
        return float(spec[key])
    except (TypeError, ValueError):
        raise ValueError(
            f"mirror_stock_config: facet spec {i} {key} is not numeric: "
            f"{spec[key]!r}")


def _spec_vec3(spec: dict, key: str, i: int, default=None) -> list:
    value = spec.get(key)
    if value is None or (hasattr(value, "__len__") and len(value) == 0):
        if default is None:
            raise ValueError(
                f"mirror_stock_config: facet spec {i} is missing {key}")
        return [float(x) for x in default]
    try:
        seq = list(value)
    except TypeError:
        raise ValueError(
            f"mirror_stock_config: facet spec {i} {key} is not a 3-vector: "
            f"{value!r}")
    if len(seq) != 3:
        raise ValueError(
            f"mirror_stock_config: facet spec {i} {key} needs 3 components, "
            f"got {len(seq)}")
    return [float(x) for x in seq]


def _spec_mat3(spec: dict, key: str, i: int) -> list:
    value = spec.get(key)
    if value is None:
        return [list(row) for row in _IDENTITY]
    try:
        rows = [list(row) for row in value]
    except TypeError:
        raise ValueError(
            f"mirror_stock_config: facet spec {i} {key} is not a 3x3 dcm: "
            f"{value!r}")
    if len(rows) != 3 or any(len(row) != 3 for row in rows):
        raise ValueError(
            f"mirror_stock_config: facet spec {i} {key} must be 3x3, got "
            f"{value!r}")
    return [[float(x) for x in row] for row in rows]


def _dcm_t_vec(dcm: list, v: list) -> list:
    """``nHat_B = dcm_F0B^T . nHat_F`` (PLAN_ENV_MIRROR §3.2), pure
    floats (no numpy needed for a 3x3)."""
    return [sum(dcm[c][r] * v[c] for c in range(3)) for r in range(3)]


def _record_link(links: list, cls: str, model: Any, kind: str, attr: str,
                 required: bool, oi: Any) -> None:
    src = getattr(model, attr, None)
    target = _LINK_TARGETS[kind]
    links.append({
        "kind": kind, "class": cls, "attr": attr, "src": src,
        "linked": _linked(src), "required": required,
        "omni": {"slot0": f"oi.{target['slot0_msg']}.subscribeTo(src)",
                 "for_object": f"oi.{target['for_object']}"
                               "(object_index, src)",
                 "slot0_subscribe_exists": callable(
                     getattr(oi, target["slot0_subscribe"], None))},
        "note": _LINK_NOTE})


def _reject_articulated(model: Any, cls: str) -> None:
    n = _call_int(getattr(model, "getNumArticulatedFacets", None))
    if n is None:
        try:
            n = int(getattr(model, "numArticulatedFacets", 0) or 0)
        except (TypeError, ValueError):
            n = 0
    if n > 0:
        raise _reject(cls, f"articulated facets configured "
                           f"(getNumArticulatedFacets()={n}); "
                           "articulated/time-varying facets are unmirrorable "
                           "(omni addSRPFacet is static-only, PLAN_ENV_MIRROR "
                           "§3.2)")


def _geometry_from_object(model: Any, cls: str):
    present = [a for a in _FACET_LIST_ATTRS if hasattr(model, a)]
    if not present:
        return None
    missing = [a for a in _FACET_LIST_ATTRS if a not in present]
    if missing:
        raise _reject(cls, f"facet geometry partially exposed (missing "
                           f"{', '.join(missing)}); refuse a partial read "
                           "(PLAN_ENV_MIRROR §3.2)")
    lists = {}
    for a in _FACET_LIST_ATTRS:
        try:
            lists[a] = list(getattr(model, a))
        except TypeError:
            raise _reject(cls, f"facet list {a} is not readable as a vector; "
                               "refuse a partial read (PLAN_ENV_MIRROR §3.2)")
    lengths = {a: len(v) for a, v in lists.items()}
    if len(set(lengths.values())) != 1:
        raise _reject(cls, f"facet list lengths disagree {lengths}; the "
                           "geometry lists must all match "
                           "(PLAN_ENV_MIRROR §3.2)")
    n = lengths[_FACET_LIST_ATTRS[0]]
    return [{"area": lists["facetAreaList"][i],
             "dcm_F0B": lists["facetDcm_F0BList"][i],
             "nHat_F": lists["facetNHat_FList"][i],
             "rotHat_F": lists["facetRotHat_FList"][i],
             "r_CopB_B": lists["facetR_CopB_BList"][i],
             "diffuseCoeff": lists["facetDiffuseCoeffList"][i],
             "specularCoeff": lists["facetSpecularCoeffList"][i]}
            for i in range(n)]


def _facet_records(model: Any, facet_specs: Any, n_stock) -> list:
    cls = type(model).__name__
    from_object = _geometry_from_object(model, cls)
    if from_object is not None:
        specs = from_object
        if facet_specs is not None and len(_as_spec_list(facet_specs)) != len(specs):
            raise _reject(cls, "facet_specs disagrees with the "
                               "object-exposed facet lists; pass one source "
                               "of truth (PLAN_ENV_MIRROR §3.2)")
    elif facet_specs is not None:
        specs = _as_spec_list(facet_specs)
    else:
        raise _reject(cls, "facet geometry is not exposed by the stock object "
                           "(BSK 2.12 keeps scGeometry private); pass "
                           "facet_specs captured at addFacet time "
                           "(PLAN_ENV_MIRROR §3.2)")
    if n_stock is not None and n_stock != len(specs):
        raise _reject(cls, f"facet counts must agree: getNumFacets()="
                           f"{n_stock} vs {len(specs)} facet records "
                           "(PLAN_ENV_MIRROR §3.2)")
    return specs


def _mirror_cannonball(model: Any, oi: Any, object_index: Any, mode: Any,
                       links: list, records: list) -> None:
    cls = type(model).__name__
    _mode_key(cls, mode)
    for attr in _LOOKUP_ATTRS:
        if _nonempty(getattr(model, attr, None)):
            raise _reject(cls, "lookup-table SRP entries present "
                               f"({', '.join(_LOOKUP_ATTRS)}); table lookup "
                               "is structural (PLAN_ENV_MIRROR §3.1)")
    area = _read_float(model, "area")
    c_r = _read_float(model, "coefficientReflection")
    if area is None or c_r is None:
        raise _reject(cls, "cannot read area/coefficientReflection; the "
                           "cannonball mirror needs both values "
                           "(PLAN_ENV_MIRROR §3.1)")
    if not math.isfinite(area) or not area > 0.0:
        raise ValueError(
            f"mirror_stock_config: {cls} area={area} must be finite positive "
            "(omni setSRPCannonball rejects it: "
            "src/omni_vec_integrator.cpp:283-284).")
    cr = c_r - 1.0
    if not (0.0 <= cr <= 1.0):
        raise _reject(cls, f"coefficientReflection={c_r} maps to cr={cr} "
                           "outside the specular fraction [0,1]; cr is the "
                           "SPECULAR FRACTION r in [0,1] and stock C_R is the "
                           "TOTAL multiplier in [1,2], so cr = C_R - 1 "
                           "(PLAN_ENV_MIRROR §3.1)")
    if object_index is None:
        oi.setSRPCannonball(area, cr)
        setter, args = "setSRPCannonball", [area, cr]
    else:
        oi.setSRPCannonballForObject(object_index, area, cr)
        setter, args = "setSRPCannonballForObject", [object_index, area, cr]
    records.append({"class": cls, "kind": "cannonball", "setter": setter,
                    "args": args, "area": area,
                    "coefficientReflection": c_r, "cr": cr})
    _record_link(links, cls, model, "sun", "sunEphmInMsg", True, oi)
    _record_link(links, cls, model, "eclipse", "sunEclipseInMsg", False, oi)


def _mirror_facets(model: Any, oi: Any, object_index: Any,
                   facet_specs: Any, links: list, records: list) -> None:
    cls = type(model).__name__
    _reject_articulated(model, cls)
    n_stock = _call_int(getattr(model, "getNumFacets", None))
    specs = _facet_records(model, facet_specs, n_stock)
    if object_index is None:
        setter = "addSRPFacet"
    else:
        setter = "addSRPFacetForObject"
    facets = []
    for i, spec in enumerate(specs):
        area = _spec_float(spec, "area", i)
        if not math.isfinite(area) or not area > 0.0:
            raise ValueError(
                f"mirror_stock_config: {cls} facet {i} area={area} must be "
                "finite positive (omni addSRPFacet rejects it: "
                "src/omni_vec_integrator.cpp:266-267).")
        rot = _spec_vec3(spec, "rotHat_F", i, default=(0.0, 0.0, 0.0))
        if any(c != 0.0 for c in rot):
            raise _reject(cls, f"facet {i} rotHat_F={rot} is nonzero; rotHat "
                               "stays REJECT -- omni addSRPFacet has no "
                               "rotHat surface (binding constraint, "
                               "PLAN_ENV_MIRROR §3.2)")
        n_hat_b = _dcm_t_vec(_spec_mat3(spec, "dcm_F0B", i),
                             _spec_vec3(spec, "nHat_F", i))
        if math.sqrt(sum(c * c for c in n_hat_b)) <= 0.0:
            raise ValueError(
                f"mirror_stock_config: {cls} facet {i} normal nHat_B="
                f"{n_hat_b} has zero norm; omni addSRPFacet rejects it "
                "(src/omni_vec_integrator.cpp:268-269).")
        cop = _spec_vec3(spec, "r_CopB_B", i)
        cd = _spec_float(spec, "diffuseCoeff", i)
        cs = _spec_float(spec, "specularCoeff", i)
        if object_index is None:
            oi.addSRPFacet(area, n_hat_b[0], n_hat_b[1], n_hat_b[2],
                           cop[0], cop[1], cop[2], cd, cs)
            args = [area, n_hat_b[0], n_hat_b[1], n_hat_b[2],
                    cop[0], cop[1], cop[2], cd, cs]
        else:
            oi.addSRPFacetForObject(object_index, area, n_hat_b[0],
                                    n_hat_b[1], n_hat_b[2],
                                    cop[0], cop[1], cop[2], cd, cs)
            args = [object_index, area, n_hat_b[0], n_hat_b[1], n_hat_b[2],
                    cop[0], cop[1], cop[2], cd, cs]
        facets.append({"area": area, "nHat_B": n_hat_b, "r_CopB_B": cop,
                       "cd": cd, "cs": cs, "setter": setter, "args": args})
    records.append({"class": cls, "kind": "static_facets",
                    "n_facets": len(specs), "facets": facets,
                    "setter": setter})
    _record_link(links, cls, model, "sun", "sunInMsg", True, oi)
    _record_link(links, cls, model, "eclipse", "sunEclipseInMsg", False, oi)


def mirror_srp_models(models: Any, oi: Any, object_index: Any = None,
                      facet_specs: Any = None, mode: Any = None) -> dict:
    """Mirror the stock SRP effectors of ONE object into ``oi``.

    ``models`` is the object's stock SRP model list (task-scan census,
    PLAN §7.3): ``RadiationPressure`` (cannonball only) mirrors via
    ``oi.setSRPCannonball(area, coefficientReflection - 1)`` and
    ``FacetSRPDynamicEffector`` (static facets only) loops per facet
    into ``oi.addSRPFacet(area, nHat_B..., r_CopB_B..., cd, cs)`` with
    ``nHat_B = dcm_F0B^T . nHat_F``. ``object_index=None`` uses the
    legacy slot-0 surface; an int routes to the ``...ForObject`` twins
    (PLAN §1.4), mirroring the ``swap.py`` per-object pattern.

    ``mode`` declares the ``RadiationPressure`` selection (``None`` /
    ``"cannonball"`` = the stock default; anything faceted/lookup
    rejects: the C++ mode enum is private and unreadable). For
    ``FacetSRPDynamicEffector``, per-facet specs come from the object
    when it exposes the geometry lists, else from ``facet_specs`` -- a
    list of mappings (or 7-field ``addFacet`` records) with keys
    ``area``, ``dcm_F0B`` (``None`` = identity), ``nHat_F``,
    ``rotHat_F``, ``r_CopB_B``, ``diffuseCoeff``, ``specularCoeff``
    captured at ``addFacet`` time, because BSK 2.12 keeps the effector's
    ``scGeometry`` private.

    STAGED: no message is subscribed. The sun link is REQUIRED
    (C++ errors on SRP-on + unlinked sun) and the eclipse link optional
    (unlinked = full sun); both are returned in ``link_needs`` with the
    stock source reader, its attr name, a best-effort linkage flag, and
    the verified omni target call for the wiring batch.

    Returns the mirrored record: ``{"staged", "n_models",
    "object_index", "srp", "link_needs"}``.
    """
    if oi is None:
        raise ValueError(
            "mirror_stock_config: mirror_srp_models needs an omni integrator "
            "(oi); stage the SRP records into a real OmniVecIntegrator.")
    models = _model_list(models)
    faceted = [m for m in models
               if type(m).__name__ == "FacetSRPDynamicEffector"]
    if len(faceted) > 1 and facet_specs is not None:
        raise _reject("FacetSRPDynamicEffector",
                      f"facet_specs given with {len(faceted)} faceted "
                      "effectors; per-effector attribution is ambiguous "
                      "(PLAN_ENV_MIRROR §3.2)")
    links, records = [], []
    for model in models:
        cls = type(model).__name__
        if cls == "RadiationPressure":
            _mirror_cannonball(model, oi, object_index, mode, links,
                               records)
        elif cls == "FacetSRPDynamicEffector":
            _mirror_facets(model, oi, object_index, facet_specs, links,
                           records)
        elif cls in _REJECT_CLASSES:
            raise _reject(cls, "message-driven facet feed "
                               "(facetElementBodyInMsgs / "
                               "facetProjectedAreaInMsgs) plus "
                               "projected-area occlusion are structural "
                               "state omni cannot see (PLAN_ENV_MIRROR "
                               "§3.3)")
        else:
            raise _reject(cls, "not a mirrorable SRP model (supported: "
                               "RadiationPressure cannonball, "
                               "FacetSRPDynamicEffector static facets); the "
                               "SRP law is unknown (PLAN_ENV_MIRROR §3)")
    return {"staged": True, "n_models": len(models),
            "object_index": object_index, "srp": records,
            "link_needs": links}
