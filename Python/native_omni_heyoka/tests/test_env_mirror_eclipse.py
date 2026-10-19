"""Host-free behavioral pins for omniheyoka/env_mirror_eclipse.py (PLAN §5).

STAGED-reader pins (bare pytest, stdlib only, NO Basilisk, NO wheel, NO
numpy): the single-spacecraft head forward, the multi-spacecraft
order-match, the ambiguous-multi reject, the unlinked-LEGAL path (no
subscribe, recorded flags), the empty/list-input paths, every loud
reject, the ``object_index=None`` vs int path selection, and a pin that
the module stays STAGED (swap.py untouched, ``test_env_mirror_gap_open``
green).

Stock objects are fakes: classes built with ``type(cls_name, (), {})``
(the reader keys off ``type(model).__name__``, PLAN §7.1) carrying
EXACTLY the wrapper-verified attr names -- ``eclipseOutMsgs``
(wrapper eclipse.py:377), ``positionInMsgs`` (:376), ``planetInMsgs``
(:375), ``sunInMsg`` (:374), ``rEqCustom`` (:379) -- and the integrator
is a call-recording ``_OiStub``.  The module is loaded from its file
path, so importing it never needs the host env (same discipline as
``tests/test_replacement_gaps.py``).

If a reject is ever relaxed into a model, the corresponding test goes
red -- that red is the signal to move the cell to the host parity
suite (``tests/test_env_mirror_contract.py`` + firing legs), not to
delete it.
"""
from __future__ import annotations

import ast
import importlib.util
import io
import tokenize
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parents[1]
ECLIPSE_SRC = REPO / "omniheyoka" / "env_mirror_eclipse.py"
SWAP_SRC = REPO / "omniheyoka" / "swap.py"


def _load():
    spec = importlib.util.spec_from_file_location(
        "env_mirror_eclipse_under_test", ECLIPSE_SRC)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ecl = _load()


def _code_tokens(path):
    """Every code-level name and string-literal VALUE of the module, with
    comments and prose dropped (a docstring is one big string literal, so
    it can never masquerade as a name).  Used by the "never touch the
    deprecated alias" pin: prose may document ``illuminationFactor``,
    code may never name it."""
    src = path.read_text(encoding="utf-8")
    out = set()
    for tok in tokenize.generate_tokens(io.StringIO(src).readline):
        if tok.type == tokenize.COMMENT:
            continue
        if tok.type == tokenize.STRING:
            try:
                out.add(ast.literal_eval(tok.string))
            except Exception:
                pass
            continue
        out.add(tok.string)
    return out


def _fake(cls_name, **attrs):
    """Instance whose class name is ``cls_name`` (the reader's only key)."""
    inst = type(cls_name, (), {})()
    for k, v in attrs.items():
        setattr(inst, k, v)
    return inst


def _msgs(n):
    """``n`` fake out-messages (the objects a stock vector would hold)."""
    return [SimpleNamespace(name=f"eclipseOutMsg{i}") for i in range(n)]


class _Functor:
    """Stock ReadFunctor fake exposing the ``isLinked()`` probe."""

    def __init__(self, linked=True):
        self._linked = linked

    def isLinked(self):
        return self._linked


class _MsgLinkStub:
    """Fake slot-0 ReadFunctor member: records ``subscribeTo(src)``."""

    def __init__(self):
        self.subscribed = []

    def subscribeTo(self, src):
        self.subscribed.append(src)


class _OiStub:
    """Records oi.* calls.  ``block`` hides named surfaces so the
    missing-surface rejects can be exercised (lookup then fails)."""

    def __init__(self, block=()):
        self._block = set(block)
        self.calls = []
        self.eclipseInMsg = _MsgLinkStub()

    def __getattribute__(self, name):
        if name != "_block" and name in object.__getattribute__(
                self, "_block"):
            raise AttributeError(f"{name} blocked in this stub")
        return object.__getattribute__(self, name)

    def subscribeEclipseForObject(self, obj, src):
        self.calls.append(("subscribeEclipseForObject", obj, src))


def _eclipse(n_out=1, n_pos=None, n_planet=1, sun_linked=True, req=6378.0):
    """Fake stock ``Eclipse`` field model with the wrapper-verified
    surface (eclipse.py:352/369/374/375/376/377/379)."""
    attrs = {"sunInMsg": _Functor(sun_linked),
             "planetInMsgs": [_Functor() for _ in range(n_planet)],
             "eclipseOutMsgs": _msgs(n_out),
             "rEqCustom": req}
    if n_pos is not None:
        attrs["positionInMsgs"] = _msgs(n_pos)
    return _fake("Eclipse", **attrs)


# --------------------------------------------------------------------------
# Positive read paths (the wiring decision, host-free)
# --------------------------------------------------------------------------

def test_supported_classes_exact():
    assert ecl.SUPPORTED_CLASSES == {"Eclipse"}


def test_eclipse_wired_adoption():
    """GAP-CLOSED adoption (wiring batch, PLAN_ENV_MIRROR §8, list.md §A1 --
    cf. tests/test_replacement_matrix.py::test_env_mirror_gap_open style):
    the eclipse reader is WIRED into swap.py, so this pin adopts the wiring
    instead of asserting its absence. Red here = wiring removed or
    de-specced -- restore it or move the cell back to the open gap.

    Wiring triple: (a) the ``_mirror_eclipse_link`` def exists and fires on
    BOTH paths (legacy ``mirror_stock_config`` + ``_mirror_single_object``
    via ``_mirror_env_all``); (b) the registry ``_MIRRORED_ENV`` covers the
    eclipse family; (c) delegation reaches the staged module's public
    surface (``mirror_eclipse_models``): the single-spacecraft head
    forwards through the slot-0 ``eclipseInMsg`` member."""
    src = SWAP_SRC.read_text(encoding="utf-8")
    tree = ast.parse(src)
    fns = {n.name: n for n in ast.walk(tree)
           if isinstance(n, ast.FunctionDef)}

    def _calls(fn):
        return {n.func.id for n in ast.walk(fn)
                if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Name)}

    assert "_mirror_eclipse_link" in fns and "_mirror_env_all" in fns
    assert "_mirror_env_all" in _calls(fns["mirror_stock_config"])
    assert "_mirror_env_all" in _calls(fns["_mirror_single_object"])
    assert "_mirror_eclipse_link" in _calls(fns["_mirror_env_all"])
    spec = importlib.util.spec_from_file_location("swap_eclipse_adoption",
                                                  SWAP_SRC)
    swap = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(swap)
    # Registry: Eclipse is a stay-in-task FIELD module (plain SysModel,
    # PLAN §1.1), so it lives in the eclipse census slice -- not in
    # _MIRRORED_ENV (effectors only) -- and passes the envelope scan
    # through untouched while only its out-message is forwarded.
    assert set(swap._ECLIPSE_FAMILY) == {"Eclipse"}
    assert "Eclipse" not in swap._MIRRORED_ENV
    mdl = _eclipse(n_out=1, n_pos=1)
    oi = _OiStub()
    rec = swap._mirror_eclipse_link([mdl], oi, None)
    assert rec["n_eclipse"] == 1 and rec["class"] == "Eclipse"
    assert rec["eclipse_link"] == "eclipseInMsg.subscribeTo"
    assert oi.eclipseInMsg.subscribed == [mdl.eclipseOutMsgs[0]]


def test_mirrored_registry_still_rejects_eclipse():
    """swap.py:39-40 ``_MIRRORED`` is unchanged: the swap envelope still
    raises for the stock Eclipse module (no half-landed wiring)."""
    tree = ast.parse(SWAP_SRC.read_text(encoding="utf-8"))
    mirrored = None
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for tgt in node.targets:
                if isinstance(tgt, ast.Name) and tgt.id == "_MIRRORED":
                    mirrored = {e.value for e in node.value.elts}
    assert mirrored == {"Spacecraft", "ReactionWheelStateEffector",
                        "ThrusterDynamicEffector"}
    assert "Eclipse" not in mirrored


def test_single_spacecraft_head_forward_slot0():
    """PLAN §5: single registered spacecraft -> eclipseOutMsgs[0].
    Slot 0 has no bare ``oi.subscribeEclipse`` (hpp:211 is the only
    ``subscribeEclipse`` symbol) -- the head goes through the
    ``eclipseInMsg`` ReadFunctor member (hpp:241, cpp:576), the drag
    module's atmoDensInMsg pattern (hpp:239)."""
    mdl = _eclipse(n_out=1, n_pos=1)
    src = mdl.eclipseOutMsgs[0]
    oi = _OiStub()
    rec = ecl.mirror_eclipse_models([mdl], oi)
    assert oi.eclipseInMsg.subscribed == [src]
    assert oi.calls == []  # no ForObject call on the slot-0 path
    assert rec["n_eclipse"] == 1 and rec["class"] == "Eclipse"
    assert rec["spacecraft_registered"] == 1
    assert rec["position_msgs"] == 1
    assert rec["eclipse_link"] == "eclipseInMsg.subscribeTo"
    assert rec["eclipse_src"] is src
    assert rec["eclipse_out_index"] == 0
    assert rec["linked"] is True
    assert rec["object_index"] is None


def test_multi_spacecraft_order_match():
    """PLAN §5: multi -> order-match.  addSpacecraftToModel appends one
    out-message per spacecraft (wrapper eclipse.py:369/:377), so slot
    ``obj`` takes ``eclipseOutMsgs[obj]``."""
    mdl = _eclipse(n_out=3, n_pos=3)
    oi = _OiStub()
    rec = ecl.mirror_eclipse_models([mdl], oi, object_index=1)
    assert oi.calls == [("subscribeEclipseForObject", 1,
                         mdl.eclipseOutMsgs[1])]
    assert oi.eclipseInMsg.subscribed == []
    assert rec["spacecraft_registered"] == 3
    assert rec["eclipse_out_index"] == 1
    assert rec["eclipse_src"] is mdl.eclipseOutMsgs[1]
    assert rec["eclipse_link"] == "subscribeEclipseForObject"
    assert rec["object_index"] == 1
    # every other slot keeps its own out-message (order, not identity)
    for i in (0, 2):
        oi_i = _OiStub()
        ecl.mirror_eclipse_models([mdl], oi_i, object_index=i)
        assert oi_i.calls == [("subscribeEclipseForObject", i,
                               mdl.eclipseOutMsgs[i])]


def test_single_spacecraft_with_int_index_still_head():
    """One registered spacecraft, per-object path: the only out-message
    is the head whichever slot mirrors it."""
    mdl = _eclipse(n_out=1, n_pos=1)
    oi = _OiStub()
    rec = ecl.mirror_eclipse_models([mdl], oi, object_index=0)
    assert oi.calls == [("subscribeEclipseForObject", 0,
                         mdl.eclipseOutMsgs[0])]
    assert rec["eclipse_out_index"] == 0


def test_unlinked_legal_no_subscribe():
    """PLAN §5 / §1.5: an Eclipse model that registered NO spacecraft
    stays LEGAL -- no subscribe (the omni slot reads factor 1.0,
    cpp:1220-1222), and the state is RECORDED, never rejected."""
    mdl = _eclipse(n_out=0, n_pos=0)
    oi = _OiStub()
    rec = ecl.mirror_eclipse_models([mdl], oi)
    assert oi.calls == [] and oi.eclipseInMsg.subscribed == []
    assert rec["n_eclipse"] == 1
    assert rec["spacecraft_registered"] == 0
    assert rec["eclipse_link"] == "unlinked"
    assert rec["linked"] is False
    assert rec["unlinked_legal"] is True
    assert rec["shadow_factor_default"] == 1.0
    assert rec["eclipse_src"] is None and rec["eclipse_out_index"] is None


def test_empty_and_none_model_lists_are_legal_zero_records():
    for models in ([], None):
        oi = _OiStub()
        rec = ecl.mirror_eclipse_models(models, oi)
        assert oi.calls == [] and oi.eclipseInMsg.subscribed == []
        assert rec["n_eclipse"] == 0
        assert rec["eclipse_link"] == "unlinked"
        assert rec["linked"] is False and rec["unlinked_legal"] is True
        assert rec["shadow_factor_default"] == 1.0
        assert rec["object_index"] is None


def test_unlinked_legal_needs_no_integrator():
    """The unlinked path subscribes nothing, so it needs no oi (PLAN
    §5/§1.5): the legal state is recorded, not rejected."""
    rec = ecl.mirror_eclipse_models([_eclipse(n_out=0, n_pos=0)], None)
    assert rec["linked"] is False and rec["eclipse_link"] == "unlinked"


def test_payload_field_is_shadow_factor_not_deprecated_alias():
    """EclipseMsgPayload's ONE field is ``shadowFactor`` (wrapper
    eclipse.py:386; the only symbols in _eclipse.so);
    ``illuminationFactor`` is the deprecated C alias
    (environment.hpp:60-61) and must never be touched."""
    assert ecl.PAYLOAD_FIELD == "shadowFactor"
    assert ecl.UNLINKED_SHADOW_FACTOR == 1.0
    tree = ast.parse(ECLIPSE_SRC.read_text(encoding="utf-8"))
    consts = {n.value for n in ast.walk(tree)
              if isinstance(n, ast.Constant) and isinstance(n.value, str)}
    assert "shadowFactor" in consts
    attrs = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
    assert "illuminationFactor" not in attrs
    names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    assert "illuminationFactor" not in names
    # and only prose (docstring/comments) may name the alias at all
    assert "illuminationFactor" not in _code_tokens(ECLIPSE_SRC)
    assert "shadowFactor" in _code_tokens(ECLIPSE_SRC)
    assert "illuminationFactor" in (ecl.__doc__ or "")


def test_record_carries_ledger_fields():
    """sun / planet counts + rEqCustom are recorded (informational),
    never gated: the module passes through the task untouched (§1.1)."""
    mdl = _eclipse(n_out=1, n_pos=1, n_planet=1, sun_linked=True,
                   req=6378.0)
    rec = ecl.mirror_eclipse_models([mdl], _OiStub())
    assert rec["planet_msgs"] == 1
    assert rec["sun_linked"] is True
    assert rec["rEqCustom"] == 6378.0
    assert rec["payload_field"] == "shadowFactor"


def test_ledger_reads_never_reject_on_unreadable_surfaces():
    """PLAN §7.1 skip-on-fail: an unreadable ledger surface becomes None;
    the mirror still wires the link it can see."""
    mdl = _fake("Eclipse", eclipseOutMsgs=_msgs(1))
    # every ledger surface aborts on read (SWIG string-read trap)
    for attr in ("sunInMsg", "planetInMsgs", "rEqCustom", "positionInMsgs"):
        setattr(type(mdl), attr, property(
            lambda self: (_ for _ in ()).throw(RuntimeError("swig"))))
    oi = _OiStub()
    rec = ecl.mirror_eclipse_models([mdl], oi)
    assert rec["sun_linked"] is None and rec["planet_msgs"] is None
    assert rec["rEqCustom"] is None and rec["position_msgs"] is None
    assert rec["eclipse_link"] == "eclipseInMsg.subscribeTo"
    assert rec["linked"] is True


# --------------------------------------------------------------------------
# object_index=None (legacy slot 0) vs int (ForObject twins)
# --------------------------------------------------------------------------

def test_path_selection_none_vs_int():
    # legacy slot-0 path: exactly one registered spacecraft (unambiguous)
    mdl1 = _eclipse(n_out=1, n_pos=1)
    src = mdl1.eclipseOutMsgs[0]

    oi = _OiStub()
    ecl.mirror_eclipse_models([mdl1], oi, object_index=None)
    assert oi.eclipseInMsg.subscribed == [src] and oi.calls == []

    # per-object path: any int routes to the ForObject twin, slot 0
    # included (WP25-A4) and never through both surfaces
    for idx in (0, 4):
        mdl = _eclipse(n_out=5, n_pos=5)
        oi = _OiStub()
        ecl.mirror_eclipse_models([mdl], oi, object_index=idx)
        assert oi.calls == [("subscribeEclipseForObject", idx,
                             mdl.eclipseOutMsgs[idx])]
        assert oi.eclipseInMsg.subscribed == []


def test_position_msgs_absent_skips_cross_check():
    """The registration cross-check is informational (ledger): when the
    wrapper/build does not expose positionInMsgs the reader still wires
    the out-message and records ``position_msgs=None``."""
    attrs = {"eclipseOutMsgs": _msgs(1)}
    mdl = _fake("Eclipse", **attrs)
    oi = _OiStub()
    rec = ecl.mirror_eclipse_models([mdl], oi)
    assert rec["position_msgs"] is None
    assert rec["eclipse_link"] == "eclipseInMsg.subscribeTo"


# --------------------------------------------------------------------------
# Loud rejects (each raises ValueError pre-mirror; no subscribe ran)
# --------------------------------------------------------------------------

def test_reject_ambiguous_multi_spacecraft_legacy_path():
    """Several registered spacecraft on the legacy slot-0 path: which
    one is slot 0 is not inferable -- loud reject, no subscribe."""
    mdl = _eclipse(n_out=2, n_pos=2)
    oi = _OiStub()
    with pytest.raises(ValueError, match="registration order is "
                                         "ambiguous"):
        ecl.mirror_eclipse_models([mdl], oi)
    assert oi.calls == [] and oi.eclipseInMsg.subscribed == []
    # disambiguated by object_index, the same model mirrors fine
    oi = _OiStub()
    rec = ecl.mirror_eclipse_models([mdl], oi, object_index=1)
    assert rec["eclipse_out_index"] == 1


def test_reject_two_eclipse_models():
    with pytest.raises(ValueError,
                       match="registration order is ambiguous"):
        ecl.mirror_eclipse_models([_eclipse(), _eclipse()], _OiStub())


def test_reject_unknown_class_envelope_wording():
    """PLAN §1.2 envelope wording + concrete reason; no subscribe."""
    oi = _OiStub()
    with pytest.raises(ValueError) as ei:
        ecl.mirror_eclipse_models([_fake("EarthRadiationModel")], oi)
    msg = str(ei.value)
    assert "mirror_stock_config: task list holds EarthRadiationModel" in msg
    assert "use omni env setters instead" in msg
    assert "outside SUPPORTED_CLASSES" in msg
    assert oi.calls == [] and oi.eclipseInMsg.subscribed == []


def test_reject_object_index_out_of_range():
    """A slot with no registered spacecraft must not silently drop the
    link (that would run full sun) -- loud reject."""
    mdl = _eclipse(n_out=2, n_pos=2)
    oi = _OiStub()
    with pytest.raises(ValueError, match="has no registered spacecraft"):
        ecl.mirror_eclipse_models([mdl], oi, object_index=2)
    assert oi.calls == [] and oi.eclipseInMsg.subscribed == []


def test_reject_negative_object_index():
    with pytest.raises(ValueError, match="negative"):
        ecl.mirror_eclipse_models([_eclipse()], _OiStub(),
                                  object_index=-1)


def test_reject_non_int_object_index():
    with pytest.raises(ValueError, match="must be None"):
        ecl.mirror_eclipse_models([_eclipse()], _OiStub(),
                                  object_index="slot0")


def test_reject_registration_vectors_disagree():
    """eclipseOutMsgs vs positionInMsgs lengths must match: they grow in
    lockstep under addSpacecraftToModel, so a disagreement means the
    call order is no longer inferable."""
    mdl = _eclipse(n_out=1, n_pos=2)
    with pytest.raises(ValueError, match="positionInMsgs has 2 entries"):
        ecl.mirror_eclipse_models([mdl], _OiStub())


def test_reject_null_out_message_entry():
    """A null entry in the out-message vector cannot carry shadowFactor."""
    mdl = _fake("Eclipse", eclipseOutMsgs=[None], positionInMsgs=[None])
    oi = _OiStub()
    with pytest.raises(ValueError, match="null message entry"):
        ecl.mirror_eclipse_models([mdl], oi)
    assert oi.calls == [] and oi.eclipseInMsg.subscribed == []


def test_reject_missing_eclipse_out_msgs_attr():
    """Missing attr (shape gate) is a LOUD reject -- distinct from the
    legal unlinked path (empty vector)."""
    mdl = _fake("Eclipse", positionInMsgs=[], planetInMsgs=[])
    with pytest.raises(ValueError, match="eclipseOutMsgs unreadable"):
        ecl.mirror_eclipse_models([mdl], _OiStub())


def test_reject_none_eclipse_out_msgs_attr():
    with pytest.raises(ValueError, match="eclipseOutMsgs is None"):
        ecl.mirror_eclipse_models(
            [_fake("Eclipse", eclipseOutMsgs=None)], _OiStub())


def test_reject_eclipse_out_msgs_not_a_vector():
    with pytest.raises(ValueError, match="not a message vector"):
        ecl.mirror_eclipse_models(
            [_fake("Eclipse", eclipseOutMsgs=7)], _OiStub())


def test_reject_missing_slot0_eclipse_functor():
    oi = _OiStub(block=("eclipseInMsg",))
    with pytest.raises(ValueError, match="eclipseInMsg"):
        ecl.mirror_eclipse_models([_eclipse()], oi)


def test_reject_missing_subscribe_eclipse_for_object():
    oi = _OiStub(block=("subscribeEclipseForObject",))
    with pytest.raises(ValueError, match="subscribeEclipseForObject"):
        ecl.mirror_eclipse_models([_eclipse(n_out=3, n_pos=3)], oi,
                                  object_index=1)
    assert oi.calls == [] and oi.eclipseInMsg.subscribed == []


def test_reject_oi_none_when_link_needed():
    with pytest.raises(ValueError, match="needs an omni integrator"):
        ecl.mirror_eclipse_models([_eclipse()], None)


# --------------------------------------------------------------------------
# STAGED discipline: not wired into swap.py, gap test stays green
# --------------------------------------------------------------------------

def test_module_docstring_states_staged():
    doc = ecl.__doc__ or ""
    assert "STAGED" in doc
