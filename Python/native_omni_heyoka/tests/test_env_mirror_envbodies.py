"""Host-free behavioral pins for omniheyoka/env_mirror_envbodies.py (PLAN §6).

STAGED-reader pins: exact omni setter args (``addEnvironmentBody(mu, ox,
oy, oz)`` + ``sunStateInMsg``/``moonStateInMsg`` slot-0 subscribes,
``setJ2(-sqrt(5)*Cbar20)``), the sun/moon index rule, the two binding
rejects (>2 extra bodies, unlinked ``planetBodyInMsg``), the J2 gates
(``maxDeg == 2`` only, central-only), the point-mass no-action path, the
``§6.3`` ephemeris stay-in-task record, ``object_index=None`` vs int
routing, and the STAGED discipline (no swap.py wiring).

Bare pytest, stdlib only, NO Basilisk, NO numpy. The reader dispatches
on ``type(model).__name__`` and reads numeric/message surfaces only
(PLAN §7.1), so fakes (classes built with ``type()`` carrying EXACTLY the
wrapper-verified attr names) stand in for stock objects. The module is
loaded from its file path -- importing the ``omniheyoka`` package would
need the sibling sim package (the ``test_replacement_gaps.py``
precedent for swap.py). Receiver names verified against
``include/omni/vec_integrator.hpp`` (:106-107, :172-174, :213-216,
:242-243).

Call order pinned by these pins: ``setJ2`` (central) fires first, then
each extra body's ``addEnvironmentBody`` immediately followed by its own
sun/moon subscribe, in stock ``gravBodies`` order.

If a reject is ever relaxed into a model, the corresponding test goes
red -- that red is the signal to move the cell to the host parity suite
(``tests/test_env_mirror_contract.py`` + firing legs), not to delete it.
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parents[1]
SRC = REPO / "omniheyoka" / "env_mirror_envbodies.py"
SWAP_SRC = REPO / "omniheyoka" / "swap.py"


def _load():
    spec = importlib.util.spec_from_file_location(
        "env_mirror_envbodies_under_test", SRC)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


eb = _load()


# --------------------------------------------------------------------------
# Fakes: class-name keyed (the reader's only dispatch surface, PLAN §7.1)
# --------------------------------------------------------------------------

class _Reader:
    """Stock ReadFunctor fake: linkage probe + optional pre-written payload
    (mirror-time snapshot pattern, env_mirror_mag.py ``_Functor``) + the
    writer Message behind ``_get_source_owner`` (the fix: omni
    subscribes the WRITER, never this reader). ``writer="auto"`` mints
    a unique stand-in for a linked reader; ``writer=None`` is the
    linked-but-unrecoverable host case (loud reject)."""

    def __init__(self, linked=False, payload=None, raises=False,
                 writer="auto"):
        self.linked = linked
        self.payload = payload
        self.raises = raises
        self.writer = object() if writer == "auto" else writer

    def isLinked(self):
        return self.linked

    def _get_source_owner(self):
        return self.writer

    def __call__(self):
        if self.raises:
            raise RuntimeError("message never written")
        if self.payload is None:
            raise RuntimeError("no payload")
        return self.payload


def _position(pv):
    """SpicePlanetStateMsgPayload fake (PositionVector :861)."""
    return SimpleNamespace(PositionVector=list(pv))


def _fake(cls_name, **attrs):
    obj = type(cls_name, (), {})()
    for k, v in attrs.items():
        setattr(obj, k, v)
    return obj


def _grav_body(is_central=False, mu=1.32712440018e20, link=None,
               model="point-mass"):
    """GravBodyData fake (gravityEffector.py:701; isCentralBody :724,
    mu :726, planetBodyInMsg :732, gravityModel :725).

    ``model="point-mass"`` is the host ``__init__`` default
    (gravityEffector.py:708-709: every live body carries a point-mass
    shadow); ``model=None`` leaves the attribute unset (reads as None,
    the shadow-lost shape, which the reader rejects)."""
    body = _fake("GravBodyData", isCentralBody=is_central, mu=mu,
                 planetBodyInMsg=link if link is not None else _Reader())
    if model == "point-mass":
        body.gravityModel = _point_mass()
    elif model is not None:
        body.gravityModel = model
    return body


def _shadow_lost_body(is_central=False, mu=1.32712440018e20, link=None):
    """Host-proxy shape: a C++-vector-iterated body whose ``gravityModel``
    read raises ``AttributeError`` (the ``_pyGravityModel`` shadow died
    in the ``addGravBody`` copy), exactly like the host
    (``env_parity_envbodies`` ``census_shadow`` pins ``[False]``)."""

    def _lost(self):
        raise AttributeError(
            "'GravBodyData' object has no attribute '_pyGravityModel'")

    cls = type("GravBodyData", (), {"gravityModel": property(_lost)})
    body = cls()
    body.isCentralBody = is_central
    body.mu = mu
    body.planetBodyInMsg = link if link is not None else _Reader()
    return body


def _point_mass():
    return _fake("PointMassGravityModel")


def _spherical(max_deg=2, cbar="default"):
    """SphericalHarmonicsGravityModel fake (maxDeg :126, cBar :127).

    The Earth fixture is the measured pair (wp5_env.py:265-269):
    Cbar20 = -4.8417e-4 <-> J2 = 1.08262668e-3.
    """
    if cbar == "default":
        cbar = [[0.0], [0.0, 0.0], [-4.8417e-4, 0.0, 0.0]]
    elif cbar == "unreadable":
        def _abort(self):
            raise RuntimeError("swig cBar read abort")
        holder = type("SphericalHarmonicsGravityModel", (), {})()
        holder.cBar = property(_abort)
        holder.maxDeg = max_deg
        return holder
    return _fake("SphericalHarmonicsGravityModel", maxDeg=max_deg,
                 cBar=cbar)


def _grav_effector(bodies):
    """GravityEffector fake (gravBodies :840, SWIG-vector iterable)."""
    return _fake("GravityEffector", gravBodies=list(bodies))


def _spice(cls_name="SpiceInterface"):
    return _fake(cls_name)


class _OiStub:
    """Records oi.* calls. The surface is exactly the header's
    (``vec_integrator.hpp:106-107, :172-174, :213-216, :242-243``); only the
    documented slot-0 message members exist as subscribe targets -- any
    other attribute access fails loudly."""

    def __init__(self):
        self.calls = []
        self.sunStateInMsg = SimpleNamespace(
            subscribeTo=lambda src: self.calls.append(
                ("sunStateInMsg.subscribeTo", src)))
        self.moonStateInMsg = SimpleNamespace(
            subscribeTo=lambda src: self.calls.append(
                ("moonStateInMsg.subscribeTo", src)))

    def addEnvironmentBody(self, *a):
        self.calls.append(("addEnvironmentBody",) + tuple(a))

    def addEnvironmentBodyForObject(self, *a):
        self.calls.append(("addEnvironmentBodyForObject",) + tuple(a))

    def setJ2(self, *a):
        self.calls.append(("setJ2",) + tuple(a))

    def setJ2ForObject(self, *a):
        self.calls.append(("setJ2ForObject",) + tuple(a))

    def subscribeSunForObject(self, *a):
        self.calls.append(("subscribeSunForObject",) + tuple(a))

    def subscribeMoonForObject(self, *a):
        self.calls.append(("subscribeMoonForObject",) + tuple(a))


class _BlockStub(_OiStub):
    """Oi stub with named surfaces hidden (missing-surface reject paths)."""

    def __init__(self, block=()):
        super().__init__()
        self._block = set(block)

    def __getattribute__(self, name):
        if name != "_block" and name in object.__getattribute__(
                self, "_block"):
            raise AttributeError(f"{name} blocked in this stub")
        return object.__getattribute__(self, name)


def _names(calls):
    return [c[0] for c in calls]


def _earth_j2(cbar20=-4.8417e-4):
    return -math.sqrt(5.0) * cbar20


# --------------------------------------------------------------------------
# Positive read paths (exact setter args, host-free)
# --------------------------------------------------------------------------

def test_supported_classes_exact():
    assert eb.SUPPORTED_CLASSES == {"GravityEffector"}
    assert eb.STAY_IN_TASK_CLASSES == {"SpiceInterface", "PlanetEphemeris",
                                       "EphemerisConverter"}
    assert eb.MAX_EXTRA_BODIES == 2


def test_single_extra_body_mu_and_sun_wiring():
    """PLAN §6.1: one non-central body -> addEnvironmentBody(mu, ox, oy, oz)
    and the FIRST extra body wires the SUN slot (cpp:1150-1153)."""
    oi = _OiStub()
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986004418e14),
        _grav_body(mu=1.32712440018e20,
                   link=_Reader(linked=True,
                                payload=_position([1.496e11, 0.0, 0.0]))),
    ])
    rec = eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == [
        ("addEnvironmentBody", 1.32712440018e20, 1.496e11, 0.0, 0.0),
        ("sunStateInMsg.subscribeTo", rec["bodies"][1]["link"]["src"]),
    ]
    assert not any(n.startswith("setJ2") for n in _names(oi.calls))
    assert rec["n_grav"] == 1 and rec["n_bodies"] == 2
    assert rec["n_central"] == 1 and rec["n_noncentral"] == 1
    assert rec["n_extra_bodies"] == 1
    assert rec["object_index"] is None
    extra = rec["bodies"][1]
    assert extra["mu"] == pytest.approx(1.32712440018e20)
    assert extra["offset_source"] == "message-snapshot"
    assert extra["link"]["slot"] == 0 and extra["link"]["kind"] == "sun"
    assert extra["setter"] == "oi.addEnvironmentBody(mu, ox, oy, oz)"
    assert extra["args"] == [1.32712440018e20, 1.496e11, 0.0, 0.0]


def test_two_extra_bodies_sun_then_moon_index_rule():
    """PLAN §1.6 index rule: extra body 0 -> sun, extra body 1 -> moon
    (cpp:1147-1161; omni_spacecraft.cpp:1497-1505)."""
    sun_src, moon_src = None, None
    oi = _OiStub()
    sun_link = _Reader(linked=True,
                       payload=_position([1.496e11, 0, 0]))
    moon_link = _Reader(linked=True,
                        payload=_position([3.844e8, 0, 0]))
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986004418e14),
        _grav_body(mu=1.32712440018e20, link=sun_link),
        _grav_body(mu=4.9028e12, link=moon_link),
    ])
    rec = eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == [
        ("addEnvironmentBody", 1.32712440018e20, 1.496e11, 0.0, 0.0),
        ("sunStateInMsg.subscribeTo", sun_link.writer),
        ("addEnvironmentBody", 4.9028e12, 3.844e8, 0.0, 0.0),
        ("moonStateInMsg.subscribeTo", moon_link.writer),
    ]
    assert rec["n_extra_bodies"] == 2 and rec["n_noncentral"] == 2
    assert [b["link"]["kind"] for b in rec["bodies"][1:]] == ["sun", "moon"]


def test_central_body_passthrough_no_add_environment_body():
    """The CENTRAL body stays: swap.py:63-72 central-mu rule is unchanged,
    so this reader never calls addEnvironmentBody for it (only the extras)."""
    oi = _OiStub()
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986004418e14,
                   link=_Reader(linked=True)),
    ])
    rec = eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []
    assert rec["n_central"] == 1 and rec["n_noncentral"] == 0
    assert rec["n_extra_bodies"] == 0
    assert rec["bodies"][0]["central"] is True
    assert "swap.py:63-72" in rec["bodies"][0]["setter"]
    assert "n/a: central body stays in stock" in \
        rec["bodies"][0]["offset_source"]
    assert rec["j2"]["j2"] is None
    assert "no central body" not in rec["j2"]["note"]


def test_central_body_with_unlinked_message_is_not_gated():
    """An UNLINKED central planetBodyInMsg is legal: the central body is
    not mirrored here at all (only NON-central links are load-bearing)."""
    oi = _OiStub()
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986004418e14,
                   link=_Reader(linked=False)),
        _grav_body(mu=1.3e20,
                   link=_Reader(linked=True,
                                payload=_position([1.0, 2.0, 3.0]))),
    ])
    rec = eb.mirror_envbodies_models([eff], oi)
    assert rec["n_extra_bodies"] == 1
    assert oi.calls[0] == ("addEnvironmentBody", 1.3e20, 1.0, 2.0, 3.0)


def test_static_offset_from_pre_written_message_snapshot():
    """A pre-written planetBodyInMsg payload gives the static offset
    (PLAN §6.1); the arm is taken VERBATIM from PositionVector."""
    oi = _OiStub()
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _grav_body(mu=1.3e20,
                   link=_Reader(linked=True,
                                payload=_position([-1.496e11, 5e9, -2.5e9]))),
    ])
    rec = eb.mirror_envbodies_models([eff], oi)
    assert oi.calls[0] == ("addEnvironmentBody", 1.3e20, -1.496e11, 5e9,
                           -2.5e9)
    assert rec["bodies"][1]["offset"] == [-1.496e11, 5e9, -2.5e9]
    assert rec["bodies"][1]["offset_source"] == "message-snapshot"


def test_recorded_zeros_fallback_when_not_pre_written():
    """Unwritten (or unreadable) linked message -> recorded zeros: the
    documented bench/fixture path (cpp:1145-1146), never a silent
    divergence because a LINKED message is refreshed per tick
    (cpp:1150-1157)."""
    for reader in (_Reader(linked=True, raises=True),
                   _Reader(linked=True, payload=None)):
        oi = _OiStub()
        eff = _grav_effector([
            _grav_body(is_central=True, mu=3.986e14),
            _grav_body(mu=1.3e20, link=reader),
        ])
        rec = eb.mirror_envbodies_models([eff], oi)
        assert oi.calls[0] == ("addEnvironmentBody", 1.3e20, 0.0, 0.0, 0.0)
        assert rec["bodies"][1]["offset"] == [0.0, 0.0, 0.0]
        assert rec["bodies"][1]["offset_source"].startswith(
            "recorded-zeros:")


def test_no_gravity_models_is_a_noop_record():
    oi = _OiStub()
    rec = eb.mirror_envbodies_models([], oi)
    assert oi.calls == []
    assert rec["n_grav"] == 0 and rec["n_bodies"] == 0
    assert rec["n_central"] == 0 and rec["n_noncentral"] == 0
    assert rec["n_extra_bodies"] == 0 and rec["bodies"] == []
    assert rec["j2"]["j2"] is None and rec["stays_in_task"] == []
    assert rec["object_index"] is None


def test_none_models_treated_as_empty():
    oi = _OiStub()
    assert eb.mirror_envbodies_models(None, oi)["n_grav"] == 0
    assert oi.calls == []


# --------------------------------------------------------------------------
# J2 (PLAN §6.2)
# --------------------------------------------------------------------------

def test_j2_happy_path_max_deg_two():
    """maxDeg == 2 -> J2 = -sqrt(5) * Cbar20 (wp5_env.py:262-292) -> setJ2.
    Earth fixture: Cbar20 = -4.8417e-4 -> J2 ~ 1.0826e-3."""
    oi = _OiStub()
    eff = _grav_effector([_grav_body(is_central=True, mu=3.986004418e14)])
    eff.gravBodies[0].gravityModel = _spherical(max_deg=2)
    rec = eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == [("setJ2", _earth_j2())]
    assert rec["j2"]["j2"] == pytest.approx(_earth_j2())
    assert rec["j2"]["Cbar20"] == pytest.approx(-4.8417e-4)
    assert rec["j2"]["maxDeg"] == 2
    assert rec["j2"]["model"] == "SphericalHarmonicsGravityModel"
    assert rec["j2"]["args"] == [_earth_j2()]
    assert rec["j2_note"] == "model=SphericalHarmonicsGravityModel, maxDeg=2"
    assert "-sqrt(5)" in rec["j2"]["rule"]


def test_j2_value_matches_earth_reference():
    """The measured Earth pair (wp5_env.py:265-269): J2_EARTH =
    1.08262668e-3 round-trips through -sqrt(5)*Cbar20."""
    oi = _OiStub()
    j2_earth = 1.08262668e-3
    cbar20 = -j2_earth / math.sqrt(5.0)
    eff = _grav_effector([_grav_body(is_central=True, mu=3.986004418e14)])
    eff.gravBodies[0].gravityModel = _spherical(
        max_deg=2, cbar=[[0.0], [0.0, 0.0], [cbar20, 0.0, 0.0]])
    eb.mirror_envbodies_models([eff], oi)
    assert oi.calls[0][1] == pytest.approx(j2_earth, rel=1e-12)


def test_j2_is_exactly_minus_sqrt5_times_cbar20():
    """Algebra pin on an arbitrary Cbar20 (not the Earth fixture)."""
    oi = _OiStub()
    cbar20 = -0.0021357334
    eff = _grav_effector([_grav_body(is_central=True, mu=3.986e14)])
    eff.gravBodies[0].gravityModel = _spherical(
        max_deg=2, cbar=[[0.0], [0.0, 0.0], [cbar20, 0.0, 0.0]])
    eb.mirror_envbodies_models([eff], oi)
    assert oi.calls[0][1] == pytest.approx(-math.sqrt(5.0) * cbar20, rel=0,
                                           abs=1e-18)


def test_j2_ignores_higher_order_coefficients():
    """Only Cbar20 enters; the C21/C22 slots of the same row are inert."""
    oi = _OiStub()
    eff = _grav_effector([_grav_body(is_central=True, mu=3.986e14)])
    eff.gravBodies[0].gravityModel = _spherical(
        max_deg=2, cbar=[[0.0], [0.1, 0.2], [-4.8417e-4, 0.3, 0.4]])
    eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == [("setJ2", _earth_j2())]


def test_point_mass_central_no_action():
    """Default law (pointMassGravityModel.py:52) -> no J2, no setter."""
    oi = _OiStub()
    eff = _grav_effector([_grav_body(is_central=True, mu=3.986e14)])
    eff.gravBodies[0].gravityModel = _point_mass()
    rec = eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []
    assert rec["j2"]["j2"] is None
    assert rec["j2"]["model"] == "PointMassGravityModel"
    assert rec["j2_note"].startswith("default point-mass law")


def test_link_subscribes_writer_not_reader():
    """The (a) fix: the omni slot subscribes the WRITER Message recovered
    through ``_get_source_owner`` (drag ``_linked_source`` / swap
    ``_reader_source`` discipline) -- handing it the stock reader
    functor raises on host (``env_parity_envbodies`` nbody cell). The
    reader itself stays in the record for the static-offset snapshot."""
    oi = _OiStub()
    link = _Reader(linked=True, payload=_position([1.0, 2.0, 3.0]))
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _grav_body(mu=1.3e20, link=link),
    ])
    rec = eb.mirror_envbodies_models([eff], oi)
    assert oi.calls[1] == ("sunStateInMsg.subscribeTo", link.writer)
    entry = rec["bodies"][1]["link"]
    assert entry["src"] is link.writer
    assert entry["src"] is not link
    assert entry["reader"] is link
    assert rec["bodies"][1]["offset"] == [1.0, 2.0, 3.0]


def test_link_subscribes_writer_forobject_twins():
    """Same writer discipline through the per-object twins."""
    oi = _OiStub()
    link = _Reader(linked=True, payload=_position([1.0, 0.0, 0.0]))
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _grav_body(mu=1.3e20, link=link),
    ])
    rec = eb.mirror_envbodies_models([eff], oi, object_index=2)
    assert oi.calls[1] == ("subscribeSunForObject", 2, link.writer)
    assert rec["bodies"][1]["link"]["src"] is link.writer


def test_reject_linked_but_sourceless_link():
    """A LINKED reader with an unrecoverable writer (``_get_source_owner``
    -> None) REJECTS: subscribing the reader raises on host and dropping
    the REQUIRED link freezes offsets (DESIGN.md:536-538)."""
    oi = _OiStub()
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _grav_body(mu=1.3e20,
                   link=_Reader(linked=True,
                                payload=_position([1.0, 0, 0]),
                                writer=None)),
    ])
    with pytest.raises(ValueError, match="unrecoverable"):
        eb.mirror_envbodies_models([eff], oi)
    with pytest.raises(ValueError, match="_get_source_owner"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


def test_reject_link_source_accessor_missing():
    """A linked reader with no ``_get_source_owner`` accessor at all is
    equally unrecoverable (loud reject, never subscribe the reader)."""
    oi = _OiStub()
    reader = SimpleNamespace(isLinked=lambda: True,
                             __call__=lambda: _position([1.0, 0, 0]))
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _grav_body(mu=1.3e20, link=reader),
    ])
    with pytest.raises(ValueError, match="unrecoverable"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


def test_reject_link_source_accessor_raises():
    """A ``_get_source_owner`` that blows up counts as unrecoverable."""
    oi = _OiStub()

    class _BadReader(_Reader):
        def _get_source_owner(self):
            raise RuntimeError("retention lost")

    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _grav_body(mu=1.3e20,
                   link=_BadReader(linked=True,
                                   payload=_position([1.0, 0, 0]))),
    ])
    with pytest.raises(ValueError, match="unrecoverable"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


def test_reject_shadow_lost_central_body():
    """The (b) silence removed: a central body whose gravity law reads as
    None (host-proxy shape -- the ``_pyGravityModel`` shadow died in the
    ``addGravBody`` copy) REJECTS instead of recording a point-mass
    no-action while stock may feel SH (``env_parity_envbodies`` j2)."""
    oi = _OiStub()
    eff = _grav_effector([_shadow_lost_body(is_central=True, mu=3.986e14)])
    with pytest.raises(ValueError, match="unreadable"):
        eb.mirror_envbodies_models([eff], oi)
    with pytest.raises(ValueError, match="_pyGravityModel"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


def test_reject_shadow_lost_noncentral_body():
    """Same unknowable-law discipline for an extra body (mu + offset only
    is sound solely for a VISIBLE point-mass law)."""
    oi = _OiStub()
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _shadow_lost_body(mu=1.3e20,
                          link=_Reader(linked=True,
                                       payload=_position([1.0, 0, 0]))),
    ])
    with pytest.raises(ValueError, match="unreadable"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


def test_j2_and_extra_body_together():
    """J2 (central) + N-body (non-central) in one call: setJ2 fires first
    (central config), then the extra body's addEnvironmentBody + its sun
    subscribe, in stock order."""
    oi = _OiStub()
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986004418e14),
        _grav_body(mu=1.3e20,
                   link=_Reader(linked=True,
                                payload=_position([1.0, 0.0, 0.0]))),
    ])
    eff.gravBodies[0].gravityModel = _spherical(max_deg=2)
    rec = eb.mirror_envbodies_models([eff], oi)
    assert _names(oi.calls) == ["setJ2", "addEnvironmentBody",
                               "sunStateInMsg.subscribeTo"]
    assert oi.calls[0][1] == pytest.approx(_earth_j2())
    assert rec["j2"]["j2"] == pytest.approx(_earth_j2())
    assert rec["n_extra_bodies"] == 1


# --------------------------------------------------------------------------
# Ephemeris providers: §6.3 stay-in-task record
# --------------------------------------------------------------------------

@pytest.mark.parametrize("cls_name", ["SpiceInterface", "PlanetEphemeris",
                                      "EphemerisConverter"])
def test_ephemeris_providers_stay_in_task(cls_name):
    """PLAN §6.3: no removal, no params mirrored, presence recorded."""
    oi = _OiStub()
    rec = eb.mirror_envbodies_models([_spice(cls_name)], oi)
    assert oi.calls == []
    assert rec["n_grav"] == 0
    assert len(rec["stays_in_task"]) == 1
    note = rec["stays_in_task"][0]
    assert note["class"] == cls_name and note["action"] == "stays-in-task"
    assert "no removal" in note["note"]
    assert "swap.py:140-143" in note["note"]


def test_ephemeris_providers_recorded_alongside_gravity():
    oi = _OiStub()
    eff = _grav_effector([_grav_body(is_central=True, mu=3.986e14)])
    rec = eb.mirror_envbodies_models(
        [_spice("SpiceInterface"), eff, _spice("EphemerisConverter")], oi)
    assert rec["n_grav"] == 1
    assert [n["class"] for n in rec["stays_in_task"]] == [
        "SpiceInterface", "EphemerisConverter"]


def test_ephemeris_provider_string_surface_never_touched():
    """PLAN §7.1: the ephemeris providers carry string surfaces
    (addPlanetNames / setPlanetNames) that must never be read -- a fake
    named ``SpiceInterface`` that aborts on EVERY attribute read still
    records a clean stay-in-task entry (only ``type(x).__name__`` is
    touched)."""
    oi = _OiStub()

    def _abort(self, name):
        raise RuntimeError(f"string read abort on {name}")

    trap = type("SpiceInterface", (), {"__getattr__": _abort})()
    rec = eb.mirror_envbodies_models([trap], oi)
    assert rec["stays_in_task"][0]["class"] == "SpiceInterface"
    assert rec["stays_in_task"][0]["action"] == "stays-in-task"
    assert oi.calls == []


# --------------------------------------------------------------------------
# Loud rejects (each raises ValueError pre-mirror; no setter ran first)
# --------------------------------------------------------------------------

def test_reject_more_than_two_extra_bodies():
    """BINDING: >2 extra bodies is unrepresentable (omni has two sun/moon
    message slots; omni_spacecraft.cpp:919 nBodies>2[bucket-(a)])."""
    oi = _OiStub()
    bodies = [_grav_body(is_central=True, mu=3.986e14)]
    for k in range(3):
        bodies.append(_grav_body(
            mu=1.3e20 + k,
            link=_Reader(linked=True,
                         payload=_position([1.0 + k, 0.0, 0.0]))))
    eff = _grav_effector(bodies)
    with pytest.raises(ValueError, match="nBodies>2"):
        eb.mirror_envbodies_models([eff], oi)
    with pytest.raises(ValueError, match=r"bucket-\(a\)"):
        eb.mirror_envbodies_models([eff], oi)
    with pytest.raises(ValueError, match="binding constraint"):
        eb.mirror_envbodies_models([eff], oi)
    with pytest.raises(ValueError, match="mirror_stock_config:"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


def test_two_extra_bodies_is_accepted_not_rejected():
    """The complement of the gate above: exactly 2 extras is the ceiling,
    not an overflow."""
    oi = _OiStub()
    bodies = [_grav_body(is_central=True, mu=3.986e14)]
    for k in range(2):
        bodies.append(_grav_body(
            mu=1.3e20 + k,
            link=_Reader(linked=True,
                         payload=_position([1.0 + k, 0.0, 0.0]))))
    eb.mirror_envbodies_models([_grav_effector(bodies)], oi)
    assert len([c for c in oi.calls
                if c[0] == "addEnvironmentBody"]) == 2


def test_reject_unlinked_noncentral_planet_body_in_msg():
    """LOAD-BEARING: a non-central body with an unlinked planetBodyInMsg
    would freeze its offset for the run (Sun tide ~1.4e-6 m/s^2,
    DESIGN.md:536-538; ZOH measured wp5_env.py:232-259)."""
    oi = _OiStub()
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _grav_body(mu=1.3e20, link=_Reader(linked=False)),
    ])
    with pytest.raises(ValueError, match="UNLINKED planetBodyInMsg"):
        eb.mirror_envbodies_models([eff], oi)
    with pytest.raises(ValueError, match="1.4e-6"):
        eb.mirror_envbodies_models([eff], oi)
    with pytest.raises(ValueError, match="wp5_env.py:232-259"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


def test_reject_unlinked_second_body_leaves_oi_untouched():
    """Gate ordering pin: the census gate pass runs BEFORE any setter, so a
    bad second body leaves the first body's addEnvironmentBody unmade."""
    oi = _OiStub()
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _grav_body(mu=1.3e20,
                   link=_Reader(linked=True,
                                payload=_position([1.0, 0, 0]))),
        _grav_body(mu=4.9e12, link=_Reader(linked=False)),
    ])
    with pytest.raises(ValueError, match="UNLINKED planetBodyInMsg"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


def test_reject_missing_noncentral_planet_body_in_msg():
    oi = _OiStub()
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _fake("GravBodyData", isCentralBody=False, mu=1.3e20, gravityModel=_point_mass()),
    ])
    with pytest.raises(ValueError, match="no planetBodyInMsg"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


@pytest.mark.parametrize("max_deg", [3, 4, 10, 100])
def test_reject_max_deg_above_two(max_deg):
    """maxDeg > 2 = higher-degree leakage (PLAN §6.2: omni setJ2 carries
    the degree-2 zonal term only)."""
    oi = _OiStub()
    eff = _grav_effector([_grav_body(is_central=True, mu=3.986e14)])
    eff.gravBodies[0].gravityModel = _spherical(max_deg=max_deg)
    with pytest.raises(ValueError, match="higher-degree leakage"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


def test_max_deg_one_has_no_cbar20_term():
    """maxDeg < 2 -> nothing to mirror (no C20 term exists in the cBar
    layout); recorded as a no-action note, not a reject."""
    oi = _OiStub()
    eff = _grav_effector([_grav_body(is_central=True, mu=3.986e14)])
    eff.gravBodies[0].gravityModel = _spherical(
        max_deg=1, cbar=[[0.0], [0.0, 0.0]])
    rec = eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []
    assert rec["j2"]["j2"] is None
    assert "no Cbar20 term" in rec["j2"]["note"]


def test_reject_noncentral_j2():
    """J2 has no per-body surface (addEnvironmentBody takes mu + offset
    only, hpp:106) -> non-central spherical harmonics REJECT."""
    oi = _OiStub()
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _grav_body(mu=1.3e20,
                   link=_Reader(linked=True,
                                payload=_position([1.0, 0, 0]))),
    ])
    eff.gravBodies[1].gravityModel = _spherical(max_deg=2)
    with pytest.raises(ValueError, match="non-central J2/shape has no"):
        eb.mirror_envbodies_models([eff], oi)
    with pytest.raises(ValueError, match="vec_integrator.hpp:106"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


def test_reject_noncentral_polyhedral():
    """A non-central shape law is equally unrepresentable."""
    oi = _OiStub()
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _grav_body(mu=1.3e20,
                   link=_Reader(linked=True,
                                payload=_position([1.0, 0, 0]))),
    ])
    eff.gravBodies[1].gravityModel = _fake("PolyhedralGravityModel")
    with pytest.raises(ValueError, match="non-central J2/shape has no"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


def test_reject_polyhedral_gravity_model_central():
    oi = _OiStub()
    eff = _grav_effector([_grav_body(is_central=True, mu=3.986e14)])
    eff.gravBodies[0].gravityModel = _fake("PolyhedralGravityModel")
    with pytest.raises(ValueError, match="PolyhedralGravityModel"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


def test_reject_unknown_gravity_model():
    oi = _OiStub()
    eff = _grav_effector([_grav_body(is_central=True, mu=3.986e14)])
    eff.gravBodies[0].gravityModel = _fake("SomeNewGravityModel")
    with pytest.raises(ValueError, match="unknown gravityModel"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


def test_reject_negative_j2_from_cbar20_sign():
    """A positive Cbar20 maps to a negative J2, which setJ2 bskErrors on
    (src/omni_vec_integrator.cpp:236-237) -> loud reject, no setter."""
    oi = _OiStub()
    eff = _grav_effector([_grav_body(is_central=True, mu=3.986e14)])
    eff.gravBodies[0].gravityModel = _spherical(
        max_deg=2, cbar=[[0.0], [0.0, 0.0], [4.8417e-4, 0.0, 0.0]])
    with pytest.raises(ValueError, match="finite non-negative"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


def test_reject_non_triangular_cbar_layout():
    """HOST-TO-PIN: only the documented lower-triangular cBar[2][0] read
    is accepted; a flat/other layout rejects instead of guessing."""
    oi = _OiStub()
    eff = _grav_effector([_grav_body(is_central=True, mu=3.986e14)])
    eff.gravBodies[0].gravityModel = _spherical(
        max_deg=2, cbar=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -4.8e-4, 0.0, 0.0])
    with pytest.raises(ValueError, match="lower-triangular"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


def test_reject_unreadable_cbar():
    oi = _OiStub()
    eff = _grav_effector([_grav_body(is_central=True, mu=3.986e14)])
    eff.gravBodies[0].gravityModel = _spherical(max_deg=2,
                                                cbar="unreadable")
    with pytest.raises(ValueError, match="cBar"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


def test_reject_unreadable_max_deg():
    oi = _OiStub()
    eff = _grav_effector([_grav_body(is_central=True, mu=3.986e14)])
    eff.gravBodies[0].gravityModel = _spherical(max_deg=None)
    with pytest.raises(ValueError, match="maxDeg"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


@pytest.mark.parametrize("mu", [0.0, -1.0, float("nan"), float("inf")])
def test_reject_nonpositive_or_nonfinite_mu(mu):
    """Both the central and the non-central mu gates fire pre-setter."""
    for central_mu in (mu, 3.986e14):
        oi = _OiStub()
        eff = _grav_effector([
            _grav_body(is_central=True, mu=central_mu),
            _grav_body(mu=mu if central_mu == 3.986e14 else 1.3e20,
                       link=_Reader(linked=True,
                                    payload=_position([1.0, 0, 0]))),
        ])
        with pytest.raises(ValueError, match="finite positive"):
            eb.mirror_envbodies_models([eff], oi)
        assert oi.calls == []


def test_reject_two_central_bodies():
    oi = _OiStub()
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _grav_body(is_central=True, mu=1.3e20),
    ])
    with pytest.raises(ValueError, match="central bodies"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


def test_reject_multiple_gravity_effectors():
    oi = _OiStub()
    effs = [_grav_effector([_grav_body(is_central=True, mu=3.986e14)]),
            _grav_effector([_grav_body(is_central=True, mu=3.986e14)])]
    with pytest.raises(ValueError, match="registration order is ambiguous"):
        eb.mirror_envbodies_models(effs, oi)
    assert oi.calls == []


def test_reject_unknown_census_class():
    oi = _OiStub()
    with pytest.raises(ValueError, match="not a gravity/ephemeris model"):
        eb.mirror_envbodies_models([_fake("SomeOtherEffector")], oi)
    assert oi.calls == []


def test_reject_grav_bodies_unreadable():
    oi = _OiStub()
    eff = _fake("GravityEffector")
    eff.gravBodies = None
    with pytest.raises(ValueError, match="gravBodies is None"):
        eb.mirror_envbodies_models([eff], oi)
    assert oi.calls == []


def test_reject_missing_oi():
    with pytest.raises(ValueError, match="needs an omni integrator"):
        eb.mirror_envbodies_models(
            [_grav_effector([_grav_body(is_central=True, mu=3.986e14)])],
            None)


def test_reject_missing_slot0_sun_member():
    oi = _BlockStub(block=("sunStateInMsg",))
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _grav_body(mu=1.3e20,
                   link=_Reader(linked=True,
                                payload=_position([1.0, 0, 0]))),
    ])
    with pytest.raises(ValueError, match="sunStateInMsg"):
        eb.mirror_envbodies_models([eff], oi)


def test_reject_missing_slot0_moon_member():
    oi = _BlockStub(block=("moonStateInMsg",))
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _grav_body(mu=1.3e20,
                   link=_Reader(linked=True,
                                payload=_position([1.0, 0, 0]))),
        _grav_body(mu=4.9e12,
                   link=_Reader(linked=True,
                                payload=_position([2.0, 0, 0]))),
    ])
    with pytest.raises(ValueError, match="moonStateInMsg"):
        eb.mirror_envbodies_models([eff], oi)


def test_reject_missing_subscribe_moon_for_object():
    oi = _BlockStub(block=("subscribeMoonForObject",))
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _grav_body(mu=1.3e20,
                   link=_Reader(linked=True,
                                payload=_position([1.0, 0, 0]))),
        _grav_body(mu=4.9e12,
                   link=_Reader(linked=True,
                                payload=_position([2.0, 0, 0]))),
    ])
    with pytest.raises(ValueError, match="subscribeMoonForObject"):
        eb.mirror_envbodies_models([eff], oi, object_index=1)


def test_reject_missing_subscribe_sun_for_object():
    oi = _BlockStub(block=("subscribeSunForObject",))
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _grav_body(mu=1.3e20,
                   link=_Reader(linked=True,
                                payload=_position([1.0, 0, 0]))),
    ])
    with pytest.raises(ValueError, match="subscribeSunForObject"):
        eb.mirror_envbodies_models([eff], oi, object_index=1)


# --------------------------------------------------------------------------
# Per-object routing (ForObject twins)
# --------------------------------------------------------------------------

def test_forobject_routing_add_environment_body_and_sun():
    """object_index given -> ForObject twins only (hpp:172-173, :213-214)."""
    oi = _OiStub()
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _grav_body(mu=1.32712440018e20,
                   link=_Reader(linked=True,
                                payload=_position([1.496e11, 0, 0]))),
    ])
    rec = eb.mirror_envbodies_models([eff], oi, object_index=2)
    assert oi.calls == [
        ("addEnvironmentBodyForObject", 2, 1.32712440018e20, 1.496e11, 0.0,
         0.0),
        ("subscribeSunForObject", 2, rec["bodies"][1]["link"]["src"]),
    ]
    assert not any(n.startswith("addEnvironmentBody(")
                   for n in _names(oi.calls))
    assert rec["object_index"] == 2
    assert rec["bodies"][1]["setter"] == \
        "oi.addEnvironmentBodyForObject(obj, mu, ox, oy, oz)"
    assert rec["bodies"][1]["args"] == [2, 1.32712440018e20, 1.496e11, 0.0,
                                        0.0]


def test_forobject_two_bodies_sun_and_moon_twins():
    oi = _OiStub()
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _grav_body(mu=1.3e20,
                   link=_Reader(linked=True,
                                payload=_position([1.0, 0, 0]))),
        _grav_body(mu=4.9e12,
                   link=_Reader(linked=True,
                                payload=_position([2.0, 0, 0]))),
    ])
    rec = eb.mirror_envbodies_models([eff], oi, object_index=1)
    assert _names(oi.calls) == [
        "addEnvironmentBodyForObject", "subscribeSunForObject",
        "addEnvironmentBodyForObject", "subscribeMoonForObject"]
    assert oi.calls[1] == ("subscribeSunForObject", 1,
                           rec["bodies"][1]["link"]["src"])
    assert oi.calls[3] == ("subscribeMoonForObject", 1,
                           rec["bodies"][2]["link"]["src"])


def test_forobject_j2_twin():
    oi = _OiStub()
    eff = _grav_effector([_grav_body(is_central=True, mu=3.986e14)])
    eff.gravBodies[0].gravityModel = _spherical(max_deg=2)
    rec = eb.mirror_envbodies_models([eff], oi, object_index=3)
    assert oi.calls == [("setJ2ForObject", 3, _earth_j2())]
    assert rec["j2"]["setter"] == "oi.setJ2ForObject(obj, J2)"
    assert rec["j2"]["args"] == [3, _earth_j2()]


def test_object_index_zero_uses_forobject_twins():
    """Explicit 0 is the per-object path (WP25-A4), never the slot-0
    shorthand -- never route a slot through both surfaces."""
    oi = _OiStub()
    eff = _grav_effector([
        _grav_body(is_central=True, mu=3.986e14),
        _grav_body(mu=1.3e20,
                   link=_Reader(linked=True,
                                payload=_position([1.0, 0, 0]))),
    ])
    eb.mirror_envbodies_models([eff], oi, object_index=0)
    assert oi.calls[0] == ("addEnvironmentBodyForObject", 0, 1.3e20, 1.0,
                           0.0, 0.0)
    assert oi.calls[1][0] == "subscribeSunForObject" and oi.calls[1][1] == 0


# --------------------------------------------------------------------------
# STAGED discipline: not wired into swap.py, gap test stays green
# --------------------------------------------------------------------------

def test_envbodies_wired_adoption():
    """GAP-CLOSED adoption (wiring batch, PLAN_ENV_MIRROR §8, list.md §A1 --
    cf. tests/test_replacement_matrix.py::test_env_mirror_gap_open style):
    the env-bodies reader is WIRED into swap.py, so this pin adopts the
    wiring instead of asserting its absence. Red here = wiring removed or
    de-specced -- restore it or move the cell back to the open gap.

    Wiring triple: (a) the ``_mirror_env_bodies`` (+ ``_mirror_j2``) defs
    exist and fire on BOTH paths (legacy ``mirror_stock_config`` +
    ``_mirror_single_object`` via ``_mirror_env_all``); (b) the registry
    ``_MIRRORED_ENV`` covers the gravity family; (c) delegation reaches
    the staged module's public surface (``mirror_envbodies_models``): the
    sc/task consistency gate passes on a central-only census and the
    central J2 leg fires ``setJ2`` with the Earth rule value
    (``-sqrt(5) * Cbar20``, ``wp5_env.py:262-292``)."""
    import ast
    src = SWAP_SRC.read_text(encoding="utf-8")
    tree = ast.parse(src)
    fns = {n.name: n for n in ast.walk(tree)
           if isinstance(n, ast.FunctionDef)}

    def _calls(fn):
        return {n.func.id for n in ast.walk(fn)
                if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Name)}

    assert "_mirror_env_bodies" in fns and "_mirror_j2" in fns
    assert "_mirror_env_all" in fns
    assert "_mirror_env_all" in _calls(fns["mirror_stock_config"])
    assert "_mirror_env_all" in _calls(fns["_mirror_single_object"])
    assert "_mirror_env_bodies" in _calls(fns["_mirror_env_all"])
    spec = importlib.util.spec_from_file_location("swap_envbodies_adoption",
                                                  SWAP_SRC)
    swap = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(swap)
    assert {"GravityEffector"} <= set(swap._MIRRORED_ENV)
    sc = SimpleNamespace(gravField=SimpleNamespace(
        gravBodies=[_grav_body(is_central=True, mu=3.986004418e14)]))
    eff = _grav_effector(
        [_grav_body(is_central=True, mu=3.986004418e14)])
    eff.gravBodies[0].gravityModel = _spherical(max_deg=2)
    oi = _OiStub()
    rec = swap._mirror_env_bodies(sc, [eff], [], oi, None)
    assert rec["n_extra_bodies"] == 0 and rec["n_central"] == 1
    assert rec["j2"]["j2"] == pytest.approx(_earth_j2())
    assert ("setJ2", _earth_j2()) in oi.calls
    assert rec["j2_check"]["validated"] is True


def test_module_docstring_states_staged():
    assert "STAGED" in eb.__doc__


def test_central_mu_rule_citation_present():
    """The reader documents that swap.py:63-72 stays untouched."""
    assert "swap.py:63-72" in eb.__doc__


def test_nbody_two_citation_present():
    """The binding nBodies>2 gate cites its C++ source."""
    assert "omni_spacecraft.cpp:919" in eb.__doc__


def test_never_reads_string_surfaces():
    """PLAN §7.1: no planetName / displayName / modelDictionaryKey /
    localPlanet read anywhere in the reader CODE (docstrings may name them
    as never-read traps)."""
    code = _reader_code()
    for banned in ("planetName", "displayName", "modelDictionaryKey",
                   "localPlanet", "addPlanetNames", "PlanetName"):
        assert banned not in code, f"string surface {banned} referenced"
    # class dispatch is type(...).__name__ only
    assert "type(" in code and ".__name__" in code


def test_stdlib_only_imports():
    """Host-free: no Basilisk / numpy / heyoka import at module level."""
    code = _reader_code()
    for banned in ("import numpy", "import Basilisk", "from Basilisk",
                   "import heyoka", "from heyoka"):
        assert banned not in code, f"host import {banned} in reader"
    for line in code.splitlines():
        stripped = line.strip()
        if stripped.startswith(("import ", "from ")):
            root = stripped.split()[1].split(".")[0]
            assert root in ("math", "typing", "__future__"), \
                f"unexpected module-level import: {stripped}"


def _reader_code():
    """The reader source without its module docstring."""
    src = SRC.read_text(encoding="utf-8")
    end = src.index('"""', src.index('"""') + 3) + 3
    return src[end:]
