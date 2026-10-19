"""STAGED env-mirror reader: stock GravityEffector / GravBodyData -> omni
``addEnvironmentBody`` + sun/moon subscribes, and central-body J2 ->
``setJ2`` (PLAN_ENV_MIRROR.md §6).

STAGED reader: file-path-loaded by ``swap._staged_reader`` (never a
package import -- bare-pytest hosts must not need the sim stack) and
wired into ``swap._mirror_env_all`` (``_mirror_env_bodies`` +
``_mirror_j2``; adoption pinned by
``tests/test_env_mirror_envbodies.py::test_envbodies_wired_adoption``).
``swap.py:63-72`` central-mu rule and ``swap.py:39-40`` ``_MIRRORED``
stay untouched. Exercised host-free by
``tests/test_env_mirror_envbodies.py``; host parity lives in
``audit_bundle`` (``env_parity_envbodies``).

Host-free by construction (stdlib-only module imports: ``math`` +
``typing``; numpy is never needed -- every read is a length-3 vector or a
scalar). Every stock surface below was verified by grep against the
installed SWIG wrapper files (cited ``wrapper:line``; the 2.12.0 wheel
ships them flat under ``Basilisk/simulation/<name>.py``), every omni
setter against ``include/omni/vec_integrator.hpp`` (cited ``hpp:line``),
and the C++ consumption rules against ``src/omni_vec_integrator.cpp``
(cited ``cpp:line``) and ``src/omni_spacecraft.cpp``.

Stock surface (``wrapper:line`` -> use)::

    GravityEffector              gravityEffector.py:816
      setGravBodies              gravityEffector.py:835
      addGravBody                gravityEffector.py:838
      gravBodies                 gravityEffector.py:840  (GravBodyVector)
      centralBody                gravityEffector.py:841  (read-only)
    GravBodyData                 gravityEffector.py:701
      isCentralBody              gravityEffector.py:724  bool
      gravityModel               gravityEffector.py:725  + override :746-757
      mu                         gravityEffector.py:726  double
      radEquator                 gravityEffector.py:727  double
      planetBodyInMsg            gravityEffector.py:732  reader functor
      planetName/displayName/
        modelDictionaryKey       gravityEffector.py:729-731  STRINGS,
                                      NEVER READ (§7.1)
      localPlanet                gravityEffector.py:733  NEVER READ (§7.1)
    SphericalHarmonicsGravityModel  sphericalHarmonicsGravityModel.py:109
      radEquator :124, muBody :125, maxDeg :126,
      cBar :127, sBar :128, loadFromFile(file, maxDeg) :130
    PointMassGravityModel           pointMassGravityModel.py:52 (muBody :64)
    PolyhedralGravityModel          polyhedralGravityModel.py:52

Reader law (PLAN §6.1 / §6.2):
* The CENTRAL body STAYS in stock (``swap.py:63-72`` central-mu rule is
  unchanged -- this reader never calls ``setMu`` and never re-registers
  the central body).
* Each NON-central body, in stock ``gravBodies`` order: ``mu`` ->
  ``oi.addEnvironmentBody(mu, ox, oy, oz)`` (``hpp:106``) or
  ``oi.addEnvironmentBodyForObject(obj, mu, ...)`` (``hpp:172-173``); its
  ``planetBodyInMsg`` link target -> the FIRST extra body wires
  ``subscribeSun`` and the SECOND wires ``subscribeMoon`` (§1.6 index
  rule, ``cpp:1147-1161``; identical in ``omni_spacecraft.cpp:1497-1505``).
  The subscribe target is the WRITER Message recovered from the linked
  stock reader through the wrapper's own retention accessor
  ``_get_source_owner`` -- the drag reader's ``_linked_source``
  (``env_mirror_drag.py:173-186``) and the swap handoff's
  ``_reader_source`` (``swap.py:760-773``) discipline, never the reader
  functor itself (``subscribeTo`` on a reader raises on host); a linked
  reader with an unrecoverable source REJECTS loudly.
* Static ``(ox, oy, oz)``: a pre-written message snapshot when available,
  else recorded zeros (bench/fixture path, ``cpp:1145-1146``).
* J2 (§6.2): only on the CENTRAL body and only when its ``gravityModel``
  is a ``SphericalHarmonicsGravityModel`` with ``maxDeg == 2``; then
  ``J2 = -sqrt(5) * Cbar20`` -> ``oi.setJ2`` (``hpp:107``) /
  ``oi.setJ2ForObject`` (``hpp:174``). ``PointMassGravityModel`` -> no
  action.

Call order (documented, not arbitrary): the J2 setter fires FIRST (it
belongs to the central body), then each extra body's
``addEnvironmentBody`` immediately followed by its own sun/moon
subscribe, in stock order. All gates -- class, body count, central count,
per-body mu / linkage / gravity law, J2 model / degree / coefficient --
run over the whole census BEFORE any setter, so a rejected mirror leaves
``oi`` untouched (PLAN §1.2).

Subscription surface (verified by reading ``vec_integrator.hpp`` in full):
there is NO bare ``oi.subscribeSun`` / ``oi.subscribeMoon`` free function.
Slot 0 subscribes through the public ReadFunctor members
``oi.sunStateInMsg`` (``hpp:242``) / ``oi.moonStateInMsg`` (``hpp:243``)
with ``.subscribeTo(src)`` (precedent ``swap.py:419``); per-object slots
use the twins ``oi.subscribeSunForObject`` (``hpp:213-214``) and
``oi.subscribeMoonForObject`` (``hpp:215-216``).

Gates (all loud ``ValueError``, ``mirror_stock_config:`` wording, PLAN
§1.2):
* more than 2 total non-central bodies (binding constraint:
  ``omni_spacecraft.cpp:919`` ``nBodies>2[bucket-(a): no per-index msg
  vector]`` -- a third body is unrepresentable in the C++ slot model);
* a non-central body with an UNLINKED or MISSING ``planetBodyInMsg``
  (frozen-for-run offsets fatal: Sun tide 1.4e-6 m/s^2,
  DESIGN.md:536-538; the per-tick ZOH measurement is
  ``wp5_env.py:232-259``);
* a LINKED ``planetBodyInMsg`` whose writer Message is unrecoverable
  (``_get_source_owner`` missing/failed -- subscribing the reader
  functor raises on host, dropping the link freezes offsets);
* a central or non-central body whose gravity law is UNREADABLE (the
  concrete model lives in the ``_pyGravityModel`` Python shadow,
  ``gravityEffector.py:737-748``, and C++-vector-iterated bodies lost
  it: ``addGravBody``/``setGravBodies`` copy into the C++ vector while
  the raw ``_gravityModel`` member is an erased base pointer -- an
  unreadable law is rejected, never assumed point-mass, because the
  silent no-action ran omni without J2 while stock felt SH);
* ``maxDeg > 2`` on the central spherical-harmonics model (higher-degree
  leakage -- omni mirrors the degree-2 zonal term only);
* any non-point-mass gravity law on a NON-central body (no per-body J2 /
  shape surface exists: ``addEnvironmentBody`` carries mu + offset only);
* ``PolyhedralGravityModel`` (no omni surface at all);
* mu not finite/positive, a bad offset vector, a non-triangular ``cBar``
  layout (HOST-TO-PIN), a negative J2, an unreadable surface, an
  ambiguous census (multiple GravityEffectors / multiple central bodies),
  and a missing omni member/twin.

SDK traps honoured (PLAN §7): no string surface is ever read (§7.1 --
dispatch is ``type(x).__name__`` only and the reads are numeric/message
surfaces); nothing is attached to stock objects (§7.2 -- a plain dict
record is returned); ``gravBodies`` is iterated as a SWIG vector (§7.3);
ephemeris providers STAY IN TASK per §6.3 (recorded in the ledger,
never removed, never reconfigured).

HOST-TO-PIN (unverifiable in this container -- no Python/Basilisk
runtime here): the runtime SWIG ``cBar`` shape. The stock loader writes a
lower-triangular list (``gravityEffector.py`` ``loadGravFromFileToList``
returns "triangular degree/order lists where each row has length
degree + 1", so ``Cbar20 == cBar[2][0]``), but whether the wrapper hands
that back as a list-of-lists, a flat Eigen vector, or a numpy array is a
host fact. The reader therefore accepts only the documented triangular
``cBar[2][0]`` reading and REJECTS anything else loudly (never guesses an
index); ``env_mirror_envbodies._cbar20`` is the single pin point.
"""

from __future__ import annotations

import math
from typing import Any

__all__ = ["SUPPORTED_CLASSES", "STAY_IN_TASK_CLASSES",
           "MAX_EXTRA_BODIES", "mirror_envbodies_models"]

# The only stock class this reader mirrors (PLAN §6.1). NBodyGravity.py
# (the newer Basilisk N-body module) is OUT of scope: it is a different
# API and the envelope census never routes it here.
SUPPORTED_CLASSES = {"GravityEffector"}

# PLAN §6.3: ephemeris providers STAY IN TASK. They are plain SysModels
# (not DynamicEffector/StateEffector) so the swap census passes them
# through untouched (swap.py:140-143); the reader only records their
# presence and forwards the already-linked planetBodyInMsg object of
# §6.1 -- no planet-name resolution, no parameter mirroring.
STAY_IN_TASK_CLASSES = {"SpiceInterface", "PlanetEphemeris",
                        "EphemerisConverter"}

# Binding C++ ceiling on extra bodies: omni_spacecraft.cpp:919 raises
# "nBodies>2[bucket-(a): no per-index msg vector]". Two sun/moon slots
# exist (cpp:1147-1161); a third has no per-index message vector.
MAX_EXTRA_BODIES = 2

# PLAN §6.2: only the fully-normalized degree-2 zonal term is mirrored.
_J2_SQRT5 = math.sqrt(5.0)
_J2_TARGET_DEG = 2

# PLAN §1.2 envelope stem (swap.py:146-150 wording).
_ENVELOPE = ("envelope is hub + wheels + staircase thrusters + central "
             "gravity + explicit omni env (stock env effectors: use omni "
             "env setters instead)")


def _cls_of(model: Any) -> str:
    """Class-name key only (PLAN §7.1: never MRO walks, never strings)."""
    return type(model).__name__


def _reject(cls: str, reason: str) -> ValueError:
    """Loud reject in the established envelope wording (PLAN §1.2)."""
    return ValueError(
        f"mirror_stock_config: {cls} is not mirrorable; {_ENVELOPE}; "
        f"{cls}: {reason}.")


def _model_list(models: Any) -> list:
    """Census slice -> plain list (None-tolerant)."""
    if models is None:
        return []
    if isinstance(models, (list, tuple)):
        return list(models)
    return [models]


def _linked(msg: Any) -> bool:
    """Linkage gate via ``isLinked()`` (blessed pattern, PLAN §2.1).

    An unreadable/missing probe counts as UNLINKED so the caller rejects
    -- a dropped link is the frozen-offset divergence, never a default.
    """
    if msg is None:
        return False
    fn = getattr(msg, "isLinked", None)
    if not callable(fn):
        return False
    try:
        return bool(fn())
    except Exception:
        return False


def _num(holder: Any, attr: str, cls: str, what: str) -> float:
    """Guarded numeric read; a failure is a LOUD reject (never a guess)."""
    try:
        value = getattr(holder, attr)
    except Exception as exc:
        raise _reject(cls, f"{what} unreadable ({exc!r}); the mirror gates "
                           f"need this surface and will not guess") from exc
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise _reject(cls, f"{what}={value!r} is not numeric "
                           f"({type(value).__name__})") from exc


def _vec3(payload: Any, attr: str, cls: str) -> list:
    """Read a 3-vector Attribute off a payload wrapper (SpicePlanetState
    ``PositionVector``, gravityEffector.py:861). Eigen proxies index fine;
    plain sequences index fine; anything else rejects."""
    try:
        raw = getattr(payload, attr)
    except Exception as exc:
        raise _reject(cls, f"payload {attr} unreadable ({exc!r})") from exc
    try:
        return [float(raw[i]) for i in range(3)]
    except Exception as exc:
        raise _reject(cls, f"payload {attr} is not a 3-vector: {raw!r} "
                           f"({exc!r})") from exc


def _classify(body: Any, cls: str) -> dict:
    """GravBodyData -> ``{central, mu}`` classification record.

    ``isCentralBody`` (:724) and ``mu`` (:726) are the mirrored numerics.
    A missing/unreadable surface rejects loudly (PLAN §1.3); a mu that
    omni would bskError on is rejected here instead (finite positive,
    ``src/omni_vec_integrator.cpp:223-224``).
    """
    try:
        is_central = bool(body.isCentralBody)
    except Exception as exc:
        raise _reject(cls, f"GravBodyData.isCentralBody unreadable "
                           f"({exc!r})") from exc
    mu = _num(body, "mu", cls, "GravBodyData.mu")
    if not math.isfinite(mu) or not mu > 0.0:
        raise _reject(cls, f"GravBodyData mu={mu!r} must be finite positive "
                           f"(omni addEnvironmentBody bskErrors on it: "
                           f"src/omni_vec_integrator.cpp:223-224)")
    return {"central": is_central, "mu": mu}


def _grav_model_name(body: Any, cls: str) -> Any:
    """GravBodyData.gravityModel's concrete class name, or None.

    The wrapper override keeps the concrete Python type
    (gravityEffector.py:737-748: ``_pyGravityModel``), so a plain
    ``type(...).__name__`` is safe and never reads a string. None means
    the law is UNREADABLE, never "default point-mass": every live
    ``GravBodyData`` carries a point-mass shadow from ``__init__``
    (gravityEffector.py:708-709), so a None read is a body that lost
    its Python-side state across the C++ copy (``addGravBody`` /
    ``setGravBodies`` copy into the C++ GravBodyVector while the raw
    ``_gravityModel`` member is an erased base ``GravityModel``
    pointer -- the wrapper's own comment). Callers REJECT on None: a
    genuine default still reads ``PointMassGravityModel``; the erased
    case is unknowable and assuming point-mass ran omni without J2
    while stock felt SH (``env_parity_envbodies`` j2 cell).
    """
    try:
        model = getattr(body, "gravityModel", None)
    except Exception:
        return None
    if model is None:
        return None
    try:
        return type(model).__name__
    except Exception:
        return None


def _grav_bodies(eff: Any, cls: str) -> list:
    """``GravityEffector.gravBodies`` (gravityEffector.py:840) as a list.

    The SWIG ``GravBodyVector`` proxy iterates directly. A missing or
    unreadable surface rejects loudly (PLAN §1.3) rather than silently
    mirroring zero bodies.
    """
    try:
        raw = eff.gravBodies
    except Exception as exc:
        raise _reject(cls, f"gravBodies unreadable ({exc!r})") from exc
    if raw is None:
        raise _reject(cls, "gravBodies is None; the census needs the "
                           "GravBodyData list (gravityEffector.py:840)")
    try:
        return list(raw)
    except Exception as exc:
        raise _reject(cls, f"gravBodies is not iterable as a vector "
                           f"({exc!r})") from exc


def _offset_from_message(reader: Any, cls: str) -> tuple:
    """Static offset from a PRE-WRITTEN message snapshot (PLAN §6.1).

    Reads the reader functor payload once at mirror time (mirror-time
    snapshot pattern, env_mirror_mag.py ``_payload``) and takes
    ``PositionVector`` (gravityEffector.py:861). A not-yet-written or
    unreadable message falls back to recorded zeros -- that is the
    documented bench/fixture path (``cpp:1145-1146``: "else the recorded
    static/initial offset"), NOT a silent divergence, because a linked
    message is refreshed per tick in C++ (``cpp:1147-1161``).
    """
    fn = getattr(reader, "__call__", None)
    if not callable(fn):
        return [0.0, 0.0, 0.0], "recorded-zeros: reader not callable"
    try:
        payload = fn()
    except Exception:
        return [0.0, 0.0, 0.0], "recorded-zeros: message not pre-written"
    return _vec3(payload, "PositionVector", cls), "message-snapshot"


def _linked_source(reader: Any, cls: str) -> Any:
    """Recover the writer Message behind a linked stock reader functor.

    Same discipline as the drag reader's ``_linked_source``
    (``env_mirror_drag.py:173-186``) and the swap handoff's
    ``_reader_source`` (``swap.py:760-773``): the wrapper's own
    retention accessor ``_get_source_owner``
    (``AtmoPropsMsgPayload.py:401-412``; the accessor the wrapper's
    ``__call__`` uses to pin its source). Returns None when the source
    cannot be recovered; callers reject loudly on a linked reader with
    no source (BSK ``subscribeTo`` needs the writer Message -- handing
    it the reader functor raises on host -- and dropping a REQUIRED
    link freezes offsets for the run).
    """
    getter = getattr(reader, "_get_source_owner", None)
    if not callable(getter):
        return None
    try:
        return getter()
    except Exception:
        return None


def _body_record(body: Any, slot: int, cls: str) -> dict:
    """Full per-extra-body record, gates included, NO setter fired.

    ``slot`` is the index among the NON-central bodies in stock order and
    decides the sun/moon wiring (0 -> sun, 1 -> moon, §1.6 index rule).
    """
    info = _classify(body, cls)
    name = _grav_model_name(body, cls)
    if name is None:
        raise _reject(
            cls, "non-central-body gravityModel unreadable (the "
            "concrete law lives in the _pyGravityModel shadow, "
            "gravityEffector.py:737-748, lost across the C++ copy); "
            "an unknowable law is rejected, never assumed "
            "point-mass (PLAN §1.2)")
    if name != "PointMassGravityModel":
        raise _reject(
            cls, f"non-central body carries gravityModel {name!r}; omni "
            f"addEnvironmentBody takes (mu, ox, oy, oz) only "
            f"(vec_integrator.hpp:106) -- non-central J2/shape has no "
            f"surface (PLAN §6.2)")

    reader = getattr(body, "planetBodyInMsg", None)
    if reader is None:
        raise _reject(cls, "non-central body has no planetBodyInMsg "
                           "(gravityEffector.py:732); the per-tick ephem "
                           "link is REQUIRED -- frozen offsets are fatal "
                           "(DESIGN.md:536-538)")
    if not _linked(reader):
        raise _reject(
            cls,
            "non-central body has UNLINKED planetBodyInMsg; offsets "
            "frozen for the run are fatal (Sun tide ~1.4e-6 m/s^2, "
            "DESIGN.md:536-538; per-tick ZOH refresh measured at "
            "wp5_env.py:232-259) -- omni needs the per-tick ephemeris "
            "message (cpp:1150-1157)")

    offset, off_src = _offset_from_message(reader, cls)
    writer = _linked_source(reader, cls)
    if writer is None:
        raise _reject(
            cls, "non-central body has a LINKED planetBodyInMsg "
            "whose writer Message is unrecoverable "
            "(_get_source_owner missing/failed; "
            "env_mirror_drag.py:173-186, swap.py:760-773); BSK "
            "subscribeTo needs the writer Message, never the "
            "reader functor (subscribing the reader raises on "
            "host: env_parity_envbodies nbody cell), and dropping "
            "the link freezes offsets for the run "
            "(DESIGN.md:536-538)")
    if slot == 0:
        slot0_attr, twin = "sunStateInMsg", "subscribeSunForObject"
        kind, hpp_slot0, hpp_twin = "sun", 242, 213
    elif slot == 1:
        slot0_attr, twin = "moonStateInMsg", "subscribeMoonForObject"
        kind, hpp_slot0, hpp_twin = "moon", 243, 215
    else:  # pragma: no cover - the MAX_EXTRA_BODIES gate runs first
        raise _reject(cls, f"extra-body slot {slot} exceeds the two omni "
                           f"sun/moon message slots (cpp:1150-1157)")
    return {"mu": info["mu"], "offset": offset,
            "offset_source": off_src, "central": False,
            "link": {"slot": slot, "kind": kind, "src": writer,
                      "reader": reader, "slot0_attr": slot0_attr,
                      "twin": twin, "hpp_slot0": hpp_slot0,
                      "hpp_twin": hpp_twin}}


def _apply_body(oi: Any, object_index: Any, rec: dict) -> None:
    """Fire ONE extra body's ``addEnvironmentBody`` + its sun/moon link.

    Called only after every gate in ``_body_record`` passed, so a reject
    anywhere in the census leaves ``oi`` untouched. Links never mark a
    slot mirrored (``hpp:202-204``).
    """
    mu, off = rec["mu"], rec["offset"]
    if object_index is None:
        oi.addEnvironmentBody(mu, off[0], off[1], off[2])
        rec["setter"] = "oi.addEnvironmentBody(mu, ox, oy, oz)"
        rec["args"] = [mu, off[0], off[1], off[2]]
    else:
        oi.addEnvironmentBodyForObject(object_index, mu, off[0], off[1],
                                       off[2])
        rec["setter"] = "oi.addEnvironmentBodyForObject(obj, mu, ox, oy, oz)"
        rec["args"] = [object_index, mu, off[0], off[1], off[2]]

    link = rec["link"]
    if object_index is None:
        functor = getattr(oi, link["slot0_attr"], None)
        if functor is None or not hasattr(functor, "subscribeTo"):
            raise ValueError(
                f"mirror_stock_config: oi has no {link['slot0_attr']} "
                f"member; cannot wire the {link['kind']} link (expected "
                f"vec_integrator.hpp:{link['hpp_slot0']}).")
        functor.subscribeTo(link["src"])
        link["wired"] = f"oi.{link['slot0_attr']}.subscribeTo(src)"
    else:
        subscribe = getattr(oi, link["twin"], None)
        if not callable(subscribe):
            raise ValueError(
                f"mirror_stock_config: oi has no {link['twin']}; cannot "
                f"wire the {link['kind']} link for object {object_index} "
                f"(expected vec_integrator.hpp:{link['hpp_twin']}).")
        subscribe(object_index, link["src"])
        link["wired"] = f"oi.{link['twin']}(object_index, src)"


def _cbar20(model: Any, cls: str) -> float:
    """``Cbar20`` from the cBar triangular layout.

    ``loadGravFromFileToList`` returns "triangular degree/order lists
    where each row has length degree + 1" (gravityEffector.py
    ``loadGravFromFile`` docstring), so ``Cbar20 == cBar[2][0]``. The
    runtime SWIG container shape is HOST-TO-PIN (list-of-lists vs flat
    Eigen vector vs numpy array), so only the documented triangular
    reading is accepted and anything else REJECTS -- never a guessed
    index. This function is the single pin point.
    """
    try:
        raw = model.cBar
    except Exception as exc:
        raise _reject(cls, f"SphericalHarmonicsGravityModel.cBar "
                           f"unreadable ({exc!r})") from exc
    try:
        rows = list(raw)
        row2 = list(rows[2])
        return float(row2[0])
    except Exception as exc:
        raise _reject(
            cls, f"cBar is not the documented lower-triangular "
            f"degree/order layout (cBar[2][0]); got {raw!r} ({exc!r}) -- "
            f"the runtime cBar shape is HOST-TO-PIN, never guess an index "
            f"(gravityEffector.py loadGravFromFile docstring)") from exc


def _central_j2(body: Any, cls: str) -> dict:
    """J2 record for the CENTRAL body (PLAN §6.2).

    ``PointMassGravityModel`` -> no action. ``SphericalHarmonicsGravity
    Model`` requires ``maxDeg == 2`` (``maxDeg > 2`` = higher-degree
    leakage -> reject) and yields ``J2 = -sqrt(5) * Cbar20``
    (``wp5_env.py:262-292``; the ``cBar[2][0]`` triangular indexing is
    HOST-TO-PIN -- see the module docstring). Any other gravity model
    (``PolyhedralGravityModel`` has no omni surface at all) rejects.
    """
    name = _grav_model_name(body, cls)
    if name is None:
        raise _reject(
            cls, "central-body gravityModel unreadable (the concrete "
            "law lives in the _pyGravityModel shadow, "
            "gravityEffector.py:737-748, lost across the C++ copy); "
            "an unknowable law is rejected, never assumed "
            "point-mass -- the silent no-action ran omni without "
            "J2 while stock felt SH (env_parity_envbodies j2 cell, "
            "PLAN §1.2)")
    if name == "PointMassGravityModel":
        return {"j2": None, "model": name,
                "note": "default point-mass law -> no action "
                        "(pointMassGravityModel.py:52)"}
    if name == "PolyhedralGravityModel":
        raise _reject(cls, "PolyhedralGravityModel has no omni surface "
                           "(no shape/J2 setter; PLAN §6.2 mirrors only the "
                           "degree-2 zonal term)")
    if name != "SphericalHarmonicsGravityModel":
        raise _reject(cls, f"unknown gravityModel {name!r}; only "
                           f"PointMassGravityModel (no action) and "
                           f"SphericalHarmonicsGravityModel (J2, maxDeg==2) "
                           f"are mirrorable (PLAN §6.2)")

    model = body.gravityModel
    try:
        max_deg = int(model.maxDeg)
    except Exception as exc:
        raise _reject(cls, f"SphericalHarmonicsGravityModel.maxDeg "
                           f"unreadable ({exc!r})") from exc
    if max_deg > _J2_TARGET_DEG:
        raise _reject(
            cls, f"maxDeg={max_deg} > {_J2_TARGET_DEG}: higher-degree "
            f"leakage -- omni setJ2 carries the degree-2 zonal term only "
            f"(PLAN §6.2; maxDeg==2 pin, wp5_env.py:262-292)")
    if max_deg < _J2_TARGET_DEG:
        return {"j2": None, "model": name, "maxDeg": max_deg,
                "note": f"maxDeg={max_deg} < 2: no Cbar20 term exists, so "
                        f"there is no J2 to mirror"}

    cbar20 = _cbar20(model, cls)
    j2 = -_J2_SQRT5 * cbar20
    if not math.isfinite(j2) or j2 < 0.0:
        raise _reject(cls, f"J2={j2!r} from Cbar20={cbar20!r} is not finite "
                           f"non-negative; omni setJ2 bskErrors on it "
                           f"(src/omni_vec_integrator.cpp:236-237)")
    return {"j2": j2, "model": name, "maxDeg": max_deg, "Cbar20": cbar20,
            "rule": "J2 = -sqrt(5) * Cbar20 (wp5_env.py:262-292)"}


def mirror_envbodies_models(models: Any, oi: Any,
                            object_index: Any = None) -> dict:
    """Mirror the stock gravity task models into ``oi`` (PLAN §6).

    ``models`` is the gravity slice of the task-model census (plain list
    of stock ``GravityEffector`` objects, swap.py task-scan convention).
    ``object_index=None`` stages the legacy slot-0 surface
    (``oi.addEnvironmentBody`` + ``oi.sunStateInMsg`` /
    ``oi.moonStateInMsg`` members); an integer selects the ``...ForObject``
    twins for that slot (PLAN §1.4; an explicit 0 is the per-object path,
    WP25-A4). Message links never mark a slot mirrored (``hpp:202-204``).

    The CENTRAL body is left to the existing central-mu rule
    (``swap.py:63-72`` -> ``oi.setMu``): this reader never calls
    ``setMu`` and never re-registers it, it only reads the central body
    to build the J2 record (§6.2).

    Returns the ledger record: ``n_grav``, ``n_bodies``,
    ``n_central`` / ``n_noncentral``, ``n_extra_bodies`` (the
    ``addEnvironmentBody`` count, ceiling ``MAX_EXTRA_BODIES``), the
    per-body ``bodies`` records (mu + static offset + link wiring, central
    body first), the ``j2`` record, ``j2_note``, the ``stays_in_task``
    notes for the ephemeris providers (§6.3), and ``object_index``.

    Every reject is a loud ``ValueError`` with the ``mirror_stock_config:``
    convention (PLAN §1.2), raised BEFORE any setter fires: the gates run
    over the whole census first, so a rejected mirror leaves ``oi``
    untouched.
    """
    if oi is None:
        raise ValueError(
            "mirror_stock_config: mirror_envbodies_models needs an omni "
            "integrator (oi); stage the env-body records into a real "
            "OmniVecIntegrator.")
    model_list = _model_list(models)

    grav = [m for m in model_list if _cls_of(m) == "GravityEffector"]
    stays = [m for m in model_list if _cls_of(m) in STAY_IN_TASK_CLASSES]
    unknown = [m for m in model_list
               if _cls_of(m) != "GravityEffector"
               and _cls_of(m) not in STAY_IN_TASK_CLASSES]

    if len(grav) > 1:
        raise _reject("GravityEffector",
                      f"{len(grav)} GravityEffector task models; stock "
                      f"registration order is ambiguous -- use one "
                      f"effector per spacecraft (swap.py convention).")
    if unknown:
        raise _reject(_cls_of(unknown[0]),
                      "not a gravity/ephemeris model; the env-bodies "
                      "reader takes GravityEffector only (PLAN §6.1; "
                      "ephemeris providers stay in task, §6.3)")

    stays_notes = [{"class": _cls_of(m), "action": "stays-in-task",
                    "note": "PLAN §6.3: plain SysModel, passes through "
                            "untouched (swap.py:140-143); no removal, no "
                            "params mirrored. The reader only forwards the "
                            "already-linked planetBodyInMsg object of the "
                            "§6.1 non-central body."}
                   for m in stays]

    n_grav = len(grav)
    n_bodies = n_central = n_noncentral = 0
    central_body = None
    extra_records: list = []
    central_j2: Any = {"j2": None, "note": "no central body in census"}

    # --- gate pass: whole census, loud, before any setter (PLAN §1.2) ------
    if n_grav == 1:
        cls = _cls_of(grav[0])
        bodies = _grav_bodies(grav[0], cls)
        n_bodies = len(bodies)
        infos = [(b, _classify(b, cls)) for b in bodies]
        centrals = [b for b, i in infos if i["central"]]
        noncentrals = [b for b, i in infos if not i["central"]]
        n_central, n_noncentral = len(centrals), len(noncentrals)

        if n_noncentral > MAX_EXTRA_BODIES:
            raise _reject(
                cls, f"{n_noncentral} non-central bodies > "
                f"{MAX_EXTRA_BODIES}: omni has two sun/moon message slots "
                f"only (cpp:1147-1161) and bskErrors bucket-(a) on more "
                f"(omni_spacecraft.cpp:919 nBodies>2[bucket-(a): no "
                f"per-index msg vector]) -- binding constraint")
        if n_central > 1:
            raise _reject(cls, f"{n_central} central bodies; the swap "
                               f"envelope is one central body "
                               f"(swap.py:63-72)")
        if centrals:
            central_body = centrals[0]
            central_j2 = _central_j2(central_body, cls)
        extra_records = [_body_record(b, slot, cls)
                         for slot, b in enumerate(noncentrals)]

    # --- apply pass: every gate passed, setters are safe now ---------------
    j2 = central_j2.get("j2")
    if j2 is not None:
        if object_index is None:
            oi.setJ2(j2)
            central_j2["setter"] = "oi.setJ2(J2)"
            central_j2["args"] = [j2]
        else:
            oi.setJ2ForObject(object_index, j2)
            central_j2["setter"] = "oi.setJ2ForObject(obj, J2)"
            central_j2["args"] = [object_index, j2]

    body_records: list = []
    if central_body is not None:
        body_records.append({
            "mu": _num(central_body, "mu", cls, "GravBodyData.mu"),
            "offset": None,
            "offset_source": "n/a: central body stays in stock "
                             "(swap.py:63-72)",
            "central": True,
            "setter": "none: central mu is mirrored by the existing "
                      "central-mu rule (swap.py:63-72)",
            "j2": central_j2})
    for rec in extra_records:
        _apply_body(oi, object_index, rec)
        body_records.append(rec)

    return {
        "n_grav": n_grav,
        "n_bodies": n_bodies,
        "n_central": n_central,
        "n_noncentral": n_noncentral,
        "n_extra_bodies": len(extra_records),
        "bodies": body_records,
        "j2": central_j2,
        "j2_note": (central_j2.get("note")
                    or f"model={central_j2.get('model')}, "
                       f"maxDeg={central_j2.get('maxDeg')}"),
        "stays_in_task": stays_notes,
        "object_index": object_index,
    }
