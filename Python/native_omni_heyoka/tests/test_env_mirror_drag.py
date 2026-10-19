"""Host-free behavioral pins for omniheyoka/env_mirror_drag.py (PLAN_ENV_MIRROR §2.1).

Bare pytest, stdlib only, no Basilisk, no wheel: stock objects are fakes
(SimpleNamespace / MRO-named classes carrying EXACTLY the wrapper-verified
attr names -- `coreParams.{projectedArea,dragCoeff,comOffset}`,
`atmoDensInMsg`, `windVelInMsg`, `modelType`, `densityCorrectionStateName`,
wrapper `dragDynamicEffector.py:599-650`) and the integrator is a
call-recording stub.  The module is loaded from its file path (module-level
imports are stdlib-only; numpy is function-local in `_as_vec3`), so importing
it never needs the host env -- same discipline as `test_replacement_gaps.py`.

Firing-geometry pins use the measured parity fixture
(`audit_bundle/probes/swap_p1_parity.py:456-467`: A=5.0, Cd=2.2,
com=[0.05,0,0] -> setDragParams(2.2*5.0, 0.05, 0, 0) + atmo subscribe) plus
an independent CdA-math leg.  Each reject path pins its message substring;
config gates fire before any setter, so a rejected mirror leaves the oi
untouched (or with only the geometry record, for the late wind gate).
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parents[1]


def _load_drag_reader():
    path = REPO / "omniheyoka" / "env_mirror_drag.py"
    spec = importlib.util.spec_from_file_location("env_mirror_drag_under_test",
                                                  path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


drag_reader = _load_drag_reader()


def _reader(linked=False, owner=None):
    """Fake ReadFunctor proxy: `isLinked()` (AtmoPropsMsgPayload.py:350 /
    WindMsgPayload.py:350; stock's own validateConfiguration uses it, upstream
    v2.12.0 dragDynamicEffector.cpp:60-62) + `_get_source_owner()`
    (AtmoPropsMsgPayload.py:401-402 / WindMsgPayload.py:388-389 -- the
    wrapper's retention accessor used by its `__call__`)."""
    return SimpleNamespace(isLinked=lambda: linked,
                           _get_source_owner=lambda: owner)


def _drag_model(cls_name="DragDynamicEffector", atmo=None, wind=None,
                model_type="cannonball", correction_name="", area=5.0,
                cd=2.2, com=(0.05, 0.0, 0.0), core=True, abort_attr=None):
    """Fake DragDynamicEffector (or same-shaped sibling family) with the
    wrapper-verified surface.  `abort_attr` leaves a class-level property
    that raises on read -- the PLAN §7.1 SWIG string-read abort trap."""
    cls = type(cls_name, (), {})
    if abort_attr is not None:
        def _abort(self):
            raise RuntimeError("swig string read abort")
        setattr(cls, abort_attr, property(_abort))
    inst = cls()
    if core:
        inst.coreParams = SimpleNamespace(projectedArea=area, dragCoeff=cd,
                                          comOffset=list(com))
    inst.atmoDensInMsg = atmo if atmo is not None else _reader()
    inst.windVelInMsg = wind if wind is not None else _reader()
    if abort_attr != "modelType":
        inst.modelType = model_type
    if abort_attr != "densityCorrectionStateName":
        inst.densityCorrectionStateName = correction_name
    return inst


class _OiStub:
    """Records oi.* calls; the surface is exactly the header's legacy
    setters/members (vec_integrator.hpp:108, :175-176, :209-210, :239-240)
    -- any other attribute access fails loudly."""

    def __init__(self):
        self.calls = []
        self.atmoDensInMsg = SimpleNamespace(
            subscribeTo=lambda src: self.calls.append(
                ("atmoDensInMsg.subscribeTo", src)))
        self.windVelInMsg = SimpleNamespace(
            subscribeTo=lambda src: self.calls.append(
                ("windVelInMsg.subscribeTo", src)))

    def setDragParams(self, *a):
        self.calls.append(("setDragParams",) + a)

    def setDragParamsForObject(self, *a):
        self.calls.append(("setDragParamsForObject",) + a)

    def subscribeAtmoForObject(self, *a):
        self.calls.append(("subscribeAtmoForObject",) + a)

    def subscribeWindForObject(self, *a):
        self.calls.append(("subscribeWindForObject",) + a)


# --------------------------------------------------------------------------
# Positive read paths (works pins, host-free)
# --------------------------------------------------------------------------

def test_supported_classes_cannonball_only():
    assert drag_reader.SUPPORTED_CLASSES == {"DragDynamicEffector"}


def test_drag_wired_adoption():
    """GAP-CLOSED adoption (wiring batch, PLAN_ENV_MIRROR §8, list.md §A1 --
    cf. tests/test_replacement_matrix.py::test_env_mirror_gap_open style):
    the drag reader is WIRED into swap.py, so this pin adopts the wiring
    instead of asserting its absence. Red here = wiring removed or
    de-specced -- restore it or move the cell back to the open gap.

    Wiring triple: (a) the ``_mirror_drag`` def exists and fires on BOTH
    paths (legacy ``mirror_stock_config`` + ``_mirror_single_object`` via
    ``_mirror_env_all``); (b) the registry ``_MIRRORED_ENV`` covers the
    drag family; (c) delegation reaches the staged module's public
    surface (``mirror_drag_models``) on the measured parity fixture
    (``audit_bundle/probes/swap_p1_parity.py:456-467``: A=5.0, Cd=2.2,
    com=[0.05,0,0])."""
    import ast
    src = (REPO / "omniheyoka" / "swap.py").read_text(encoding="utf-8")
    tree = ast.parse(src)
    fns = {n.name: n for n in ast.walk(tree)
           if isinstance(n, ast.FunctionDef)}

    def _calls(fn):
        return {n.func.id for n in ast.walk(fn)
                if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Name)}

    assert "_mirror_drag" in fns and "_mirror_env_all" in fns
    assert "_mirror_env_all" in _calls(fns["mirror_stock_config"])
    assert "_mirror_env_all" in _calls(fns["_mirror_single_object"])
    assert "_mirror_drag" in _calls(fns["_mirror_env_all"])
    spec = importlib.util.spec_from_file_location("swap_drag_adoption",
                                                  REPO / "omniheyoka" / "swap.py")
    swap = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(swap)
    assert {"DragDynamicEffector", "FacetDragDynamicEffector"} <= set(
        swap._MIRRORED_ENV)
    asrc = object()
    oi = _OiStub()
    rec = swap._mirror_drag(
        [_drag_model(atmo=_reader(linked=True, owner=asrc))], [], [], oi,
        None)
    assert rec["n_drag"] == 1 and rec["CdA"] == pytest.approx(2.2 * 5.0)
    assert rec["comOffset"] == [0.05, 0.0, 0.0]
    assert ("setDragParams", 2.2 * 5.0, 0.05, 0.0, 0.0) in oi.calls
    assert ("atmoDensInMsg.subscribeTo", asrc) in oi.calls


def test_firing_geometry_mirror():
    """Measured parity fixture (swap_p1_parity.py:456-467): A=5.0, Cd=2.2,
    com=[0.05,0,0] -> setDragParams(2.2*5.0, 0.05, 0, 0), atmo forwarded,
    wind left unlinked (inertial fallback, §1.5)."""
    asrc = object()
    oi = _OiStub()
    rec = drag_reader.mirror_drag_models(
        [_drag_model(atmo=_reader(linked=True, owner=asrc))], oi)
    call = oi.calls[0]
    assert call[0] == "setDragParams"
    assert call[1] == pytest.approx(2.2 * 5.0)
    assert call[1] == pytest.approx(11.0)
    assert call[2:] == (0.05, 0.0, 0.0)
    assert oi.calls[1] == ("atmoDensInMsg.subscribeTo", asrc)
    assert len(oi.calls) == 2
    assert rec["n_drag"] == 1 and rec["CdA"] == pytest.approx(11.0)
    assert rec["comOffset"] == [0.05, 0.0, 0.0]
    assert rec["atmo"] == "linked" and rec["wind"] == "inertial-fallback"
    assert rec["object_index"] is None


def test_cda_math_is_coeff_times_area():
    """CdA = dragCoeff * projectedArea (PLAN §2.1), comOffset verbatim."""
    oi = _OiStub()
    drag_reader.mirror_drag_models(
        [_drag_model(atmo=_reader(linked=True, owner=object()),
                     area=12.5, cd=1.7, com=(0.1, -0.2, 0.3))], oi)
    call = oi.calls[0]
    assert call[0] == "setDragParams"
    assert call[1] == pytest.approx(1.7 * 12.5)
    assert call[2:] == (0.1, -0.2, 0.3)


def test_wind_forwarded_when_linked():
    wsrc = object()
    oi = _OiStub()
    rec = drag_reader.mirror_drag_models(
        [_drag_model(atmo=_reader(linked=True, owner=object()),
                     wind=_reader(linked=True, owner=wsrc))], oi)
    assert ("windVelInMsg.subscribeTo", wsrc) in oi.calls
    assert rec["wind"] == "forwarded"


def test_wind_unlinked_stays_unlinked_with_fallback_label():
    oi = _OiStub()
    rec = drag_reader.mirror_drag_models(
        [_drag_model(atmo=_reader(linked=True, owner=object()))], oi)
    assert not any(c[0] == "windVelInMsg.subscribeTo" for c in oi.calls)
    assert rec["wind"] == "inertial-fallback"


def test_forobject_twin_routing():
    """object_index given -> ForObject twins only (hpp:175-176, :209-210)."""
    asrc, wsrc = object(), object()
    oi = _OiStub()
    rec = drag_reader.mirror_drag_models(
        [_drag_model(atmo=_reader(linked=True, owner=asrc),
                     wind=_reader(linked=True, owner=wsrc))], oi,
        object_index=2)
    assert oi.calls[0][0] == "setDragParamsForObject" and oi.calls[0][1] == 2
    assert oi.calls[0][2] == pytest.approx(2.2 * 5.0)
    assert oi.calls[0][3:] == (0.05, 0.0, 0.0)
    assert oi.calls[1] == ("subscribeAtmoForObject", 2, asrc)
    assert oi.calls[2] == ("subscribeWindForObject", 2, wsrc)
    assert not any(c[0] == "setDragParams" for c in oi.calls)
    assert rec["object_index"] == 2


def test_empty_models_zero_record():
    oi = _OiStub()
    assert drag_reader.mirror_drag_models([], oi) == {
        "n_drag": 0, "object_index": None}
    assert oi.calls == []


# --------------------------------------------------------------------------
# Loud rejects (each raises ValueError pre-sim, mirror convention)
# --------------------------------------------------------------------------

def test_reject_non_cannonball_model_type():
    oi = _OiStub()
    with pytest.raises(ValueError, match="non-cannonball"):
        drag_reader.mirror_drag_models(
            [_drag_model(atmo=_reader(linked=True, owner=object()),
                         model_type="flatPlate")], oi)
    assert oi.calls == []  # config gates fire before any setter


def test_reject_unreadable_model_type_read():
    """PLAN §7.1 trap: an unreadable gate surface rejects loudly, never
    passes silently."""
    oi = _OiStub()
    with pytest.raises(ValueError, match="modelType"):
        drag_reader.mirror_drag_models(
            [_drag_model(atmo=_reader(linked=True, owner=object()),
                         abort_attr="modelType")], oi)
    assert oi.calls == []


def test_reject_density_correction_linkage():
    oi = _OiStub()
    with pytest.raises(ValueError, match="densityCorrection"):
        drag_reader.mirror_drag_models(
            [_drag_model(atmo=_reader(linked=True, owner=object()),
                         correction_name="densityFactor")], oi)
    assert oi.calls == []


def test_correction_unset_linkage_passes():
    """Unset linkage is the pass condition ('' and None both pass); the live
    densityCorrection value is never read (contract-test docstring policy)."""
    for name in ("", None):
        oi = _OiStub()
        rec = drag_reader.mirror_drag_models(
            [_drag_model(atmo=_reader(linked=True, owner=object()),
                         correction_name=name)], oi)
        assert rec["n_drag"] == 1


def test_reject_unreadable_correction_name():
    with pytest.raises(ValueError, match="densityCorrectionStateName"):
        drag_reader.mirror_drag_models(
            [_drag_model(atmo=_reader(linked=True, owner=object()),
                         abort_attr="densityCorrectionStateName")],
            _OiStub())


def test_reject_atmo_unlinked():
    oi = _OiStub()
    with pytest.raises(ValueError, match="atmoDensInMsg"):
        drag_reader.mirror_drag_models([_drag_model(atmo=_reader())], oi)
    assert oi.calls == []


def test_reject_atmo_source_unrecoverable():
    oi = _OiStub()
    with pytest.raises(ValueError, match="atmoDensInMsg"):
        drag_reader.mirror_drag_models(
            [_drag_model(atmo=_reader(linked=True, owner=None))], oi)
    assert oi.calls == []  # the required link is checked before the setter


def test_reject_wind_source_unrecoverable():
    oi = _OiStub()
    with pytest.raises(ValueError, match="windVelInMsg"):
        drag_reader.mirror_drag_models(
            [_drag_model(atmo=_reader(linked=True, owner=object()),
                         wind=_reader(linked=True, owner=None))], oi)
    assert oi.calls[0][0] == "setDragParams"  # geometry mirrored first
    assert not any(c[0] == "windVelInMsg.subscribeTo" for c in oi.calls)


def test_reject_facet_drag_family():
    """PLAN §2.2: FacetDragDynamicEffector is a different family (per-facet
    law structurally invisible to omni) -- envelope wording + concrete
    reason, no setter fires."""
    oi = _OiStub()
    with pytest.raises(ValueError) as ei:
        drag_reader.mirror_drag_models(
            [_drag_model(cls_name="FacetDragDynamicEffector",
                         atmo=_reader(linked=True, owner=object()))], oi)
    msg = str(ei.value)
    assert "use omni env setters instead" in msg
    assert "per-facet" in msg
    assert oi.calls == []


def test_reject_unknown_drag_family():
    with pytest.raises(ValueError, match="outside SUPPORTED_CLASSES"):
        drag_reader.mirror_drag_models(
            [_drag_model(cls_name="StochasticDragCoeff",
                         atmo=_reader(linked=True, owner=object()))],
            _OiStub())


def test_reject_two_drag_effectors():
    with pytest.raises(ValueError, match="registration order is ambiguous"):
        drag_reader.mirror_drag_models([_drag_model(), _drag_model()],
                                       _OiStub())


def test_reject_missing_core_params():
    with pytest.raises(ValueError, match="coreParams"):
        drag_reader.mirror_drag_models([_drag_model(core=False)], _OiStub())
