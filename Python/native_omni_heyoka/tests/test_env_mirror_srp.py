"""Host-free behavioral pins for omniheyoka/env_mirror_srp.py (PLAN §3 SRP).

STAGED-reader pins: exact omni setter args (incl. the cr = C_R - 1
specular-fraction mapping, C_R=1.5 -> 0.5), the cr-out-of-[0,1]
reject, the per-facet loop args (dcm-transposed normal, verbatim CoP),
ForObject routing, recorded sun/eclipse link needs (NO subscribe), and
every loud reject substring.

Bare pytest, stdlib only, no Basilisk: the module under test reads
stock objects by class name and numeric/message attributes only (never
strings), so fakes (classes made with type() + plain attrs) stand in
for stock effectors. The module is loaded from its file path --
importing the omniheyoka package would need the sibling sim package
(the test_replacement_gaps.py precedent for swap.py).

If a reject is ever relaxed into a model, the corresponding test goes
red -- that red is the signal to move the cell to the host parity
suite (tests/test_env_mirror_contract.py + firing legs), not to
delete it.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parents[1]
SRP_SRC = REPO / "omniheyoka" / "env_mirror_srp.py"
SWAP_SRC = REPO / "omniheyoka" / "swap.py"


def _load():
    spec = importlib.util.spec_from_file_location("env_mirror_srp_under_test",
                                                  SRP_SRC)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


srp = _load()

_ID3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


class _OiStub:
    """Records oi.* calls; any unexpected attribute access fails loudly
    (the stub deliberately has NO subscribeSun/subscribeEclipse -- the
    staged reader must never subscribe)."""

    def __init__(self):
        self.calls = []

    def setSRPCannonball(self, *a):
        self.calls.append(("setSRPCannonball",) + tuple(a))

    def setSRPCannonballForObject(self, *a):
        self.calls.append(("setSRPCannonballForObject",) + tuple(a))

    def addSRPFacet(self, *a):
        self.calls.append(("addSRPFacet",) + tuple(a))

    def addSRPFacetForObject(self, *a):
        self.calls.append(("addSRPFacetForObject",) + tuple(a))


class _MsgStub:
    """Fake message endpoint exposing the isLinked() probe."""

    def __init__(self, linked=True):
        self._linked = linked

    def isLinked(self):
        return self._linked


def _fake(cls_name, **attrs):
    """MRO-named fake: the readers dispatch on type(model).__name__."""
    obj = type(cls_name, (), {})()
    for k, v in attrs.items():
        setattr(obj, k, v)
    return obj


def _rp(area=1.0, C_R=1.5, **kw):
    """RadiationPressure fake with inert (empty) lookup vectors."""
    attrs = {"area": area, "coefficientReflection": C_R,
             "lookupForce_B": [], "lookupTorque_B": [], "lookupSHat_B": []}
    attrs.update(kw)
    return _fake("RadiationPressure", **attrs)


def _faceted(n=1, articulated=0):
    """FacetSRPDynamicEffector fake with the stock count getters."""
    obj = _fake("FacetSRPDynamicEffector")
    obj.getNumFacets = lambda: n
    obj.getNumArticulatedFacets = lambda: articulated
    return obj


def _spec(area=1.0, dcm=None, n_hat=(1.0, 0.0, 0.0), rot=None,
          cop=(0.0, 0.5, 0.0), cd=0.2, cs=0.3):
    """One facet spec in the stock addFacet spelling (dcm None =
    identity facet frame)."""
    return {"area": area, "dcm_F0B": dcm, "nHat_F": list(n_hat),
            "rotHat_F": rot, "r_CopB_B": list(cop),
            "diffuseCoeff": cd, "specularCoeff": cs}


def _faceted_with_lists(n_areas, dcms, nhats, rots, cops, cds, css):
    m = _faceted(n=len(n_areas))
    m.facetAreaList = n_areas
    m.facetDcm_F0BList = dcms
    m.facetNHat_FList = nhats
    m.facetRotHat_FList = rots
    m.facetR_CopB_BList = cops
    m.facetDiffuseCoeffList = cds
    m.facetSpecularCoeffList = css
    return m


# --------------------------------------------------------------------------
# Positive read paths (exact setter args)
# --------------------------------------------------------------------------

def test_supported_classes_exact():
    assert srp.SUPPORTED_CLASSES == {"RadiationPressure",
                                     "FacetSRPDynamicEffector"}


def test_cannonball_setter_args_and_record():
    oi = _OiStub()
    m = _rp(area=2.0, C_R=1.5)
    rec = srp.mirror_srp_models([m], oi)
    assert oi.calls == [("setSRPCannonball", 2.0, 0.5)]
    entry = rec["srp"][0]
    assert entry["kind"] == "cannonball" and entry["setter"] == \
        "setSRPCannonball"
    assert entry["args"] == [2.0, 0.5]
    assert entry["cr"] == 0.5 and entry["coefficientReflection"] == 1.5
    assert rec["object_index"] is None and rec["staged"] is True
    assert rec["n_models"] == 1


@pytest.mark.parametrize("c_r,cr", [(1.0, 0.0), (1.5, 0.5), (2.0, 1.0)])
def test_cannonball_cr_mapping_including_bounds(c_r, cr):
    oi = _OiStub()
    srp.mirror_srp_models([_rp(area=1.0, C_R=c_r)], oi)
    assert oi.calls == [("setSRPCannonball", 1.0, cr)]


@pytest.mark.parametrize("mode", [None, "cannonball", "SRP_CANNONBALL_MODEL"])
def test_cannonball_mode_aliases_accepted(mode):
    oi = _OiStub()
    srp.mirror_srp_models([_rp()], oi, mode=mode)
    assert oi.calls == [("setSRPCannonball", 1.0, 0.5)]


def test_facet_loop_args_slot0_identity_dcm():
    oi = _OiStub()
    rec = srp.mirror_srp_models([_faceted(n=1)], oi,
                                facet_specs=[_spec()])
    assert oi.calls == [("addSRPFacet", 1.0, 1.0, 0.0, 0.0,
                        0.0, 0.5, 0.0, 0.2, 0.3)]
    entry = rec["srp"][0]
    assert entry["kind"] == "static_facets" and entry["n_facets"] == 1
    facet = entry["facets"][0]
    assert facet["nHat_B"] == [1.0, 0.0, 0.0]
    assert facet["r_CopB_B"] == [0.0, 0.5, 0.0]
    assert (facet["cd"], facet["cs"]) == (0.2, 0.3)


def test_facet_normal_dcm_transpose():
    oi = _OiStub()
    dcm = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    srp.mirror_srp_models([_faceted(n=1)], oi,
                          facet_specs=[_spec(dcm=dcm, n_hat=(1, 0, 0))])
    # nHat_B = dcm_F0B^T . nHat_F = (0, -1, 0)
    assert oi.calls == [("addSRPFacet", 1.0, 0.0, -1.0, 0.0,
                        0.0, 0.5, 0.0, 0.2, 0.3)]


def test_facet_loop_multi_facet_order_and_for_object_routing():
    oi = _OiStub()
    specs = [_spec(), _spec(area=2.0, cd=0.1, cs=0.4, cop=(0.1, 0.2, 0.3))]
    srp.mirror_srp_models([_faceted(n=2)], oi, object_index=2,
                          facet_specs=specs)
    assert oi.calls == [
        ("addSRPFacetForObject", 2, 1.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0,
         0.2, 0.3),
        ("addSRPFacetForObject", 2, 2.0, 1.0, 0.0, 0.0, 0.1, 0.2, 0.3,
         0.1, 0.4)]


def test_facet_geometry_from_object_lists():
    oi = _OiStub()
    m = _faceted_with_lists([1.0], [_ID3], [[0.0, 1.0, 0.0]],
                            [[0.0, 0.0, 0.0]], [[0.0, 0.5, 0.0]],
                            [0.2], [0.3])
    rec = srp.mirror_srp_models([m], oi)
    assert oi.calls == [("addSRPFacet", 1.0, 0.0, 1.0, 0.0,
                        0.0, 0.5, 0.0, 0.2, 0.3)]
    assert rec["srp"][0]["n_facets"] == 1


def test_facet_spec_accepts_positional_record():
    oi = _OiStub()
    spec = (1.0, None, [1.0, 0.0, 0.0], None, [0.0, 0.5, 0.0], 0.2, 0.3)
    srp.mirror_srp_models([_faceted(n=1)], oi, facet_specs=[spec])
    assert oi.calls == [("addSRPFacet", 1.0, 1.0, 0.0, 0.0,
                        0.0, 0.5, 0.0, 0.2, 0.3)]


def test_cannonball_and_facets_combined():
    oi = _OiStub()
    srp.mirror_srp_models([_rp(), _faceted(n=1)], oi,
                          facet_specs=[_spec()])
    assert oi.calls == [("setSRPCannonball", 1.0, 0.5),
                        ("addSRPFacet", 1.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0,
                         0.2, 0.3)]


def test_no_models_is_noop():
    oi = _OiStub()
    rec = srp.mirror_srp_models([], oi)
    assert oi.calls == [] and rec["n_models"] == 0
    assert rec["link_needs"] == [] and rec["srp"] == []


# --------------------------------------------------------------------------
# Sun / eclipse link needs (recorded, never subscribed)
# --------------------------------------------------------------------------

def test_sun_eclipse_link_needs_recorded_not_subscribed():
    oi = _OiStub()
    sun = _MsgStub(True)
    m = _rp(sunEphmInMsg=sun, sunEclipseInMsg=None)
    rec = srp.mirror_srp_models([m], oi)
    needs = {n["kind"]: n for n in rec["link_needs"]}
    assert set(needs) == {"sun", "eclipse"}
    assert needs["sun"]["required"] is True
    assert needs["sun"]["attr"] == "sunEphmInMsg"
    assert needs["sun"]["src"] is sun and needs["sun"]["linked"] is True
    assert needs["sun"]["omni"]["for_object"] == \
        "oi.subscribeSunForObject(object_index, src)"
    assert needs["sun"]["omni"]["slot0"] == \
        "oi.sunStateInMsg.subscribeTo(src)"
    assert needs["sun"]["omni"]["slot0_subscribe_exists"] is False
    assert needs["eclipse"]["omni"]["slot0_subscribe_exists"] is False
    assert needs["eclipse"]["required"] is False
    assert needs["eclipse"]["attr"] == "sunEclipseInMsg"
    assert needs["eclipse"]["src"] is None
    assert needs["eclipse"]["linked"] is None
    assert [c[0] for c in oi.calls] == ["setSRPCannonball"]


def test_eclipse_link_recorded_when_linked():
    oi = _OiStub()
    ecl = _MsgStub(True)
    rec = srp.mirror_srp_models([_rp(sunEclipseInMsg=ecl)], oi)
    needs = {n["kind"]: n for n in rec["link_needs"]}
    assert needs["eclipse"]["src"] is ecl
    assert needs["eclipse"]["linked"] is True
    assert needs["sun"]["linked"] is None


def test_facet_link_attrs():
    oi = _OiStub()
    rec = srp.mirror_srp_models([_faceted(n=1)], oi, facet_specs=[_spec()])
    needs = {n["kind"]: n for n in rec["link_needs"]}
    assert needs["sun"]["attr"] == "sunInMsg"
    assert needs["sun"]["required"] is True
    assert needs["eclipse"]["attr"] == "sunEclipseInMsg"
    assert needs["eclipse"]["required"] is False


# --------------------------------------------------------------------------
# Loud rejects (each raises ValueError pre-sim; no setter ran first)
# --------------------------------------------------------------------------

@pytest.mark.parametrize("c_r", [0.5, 0.0, 2.5, 3.0])
def test_reject_cr_out_of_unit_interval(c_r):
    oi = _OiStub()
    with pytest.raises(ValueError, match="outside the specular fraction"):
        srp.mirror_srp_models([_rp(area=1.0, C_R=c_r)], oi)
    assert oi.calls == []


def test_reject_faceted_cpu_mode():
    oi = _OiStub()
    with pytest.raises(ValueError, match="faceted-CPU/lookup model"):
        srp.mirror_srp_models([_rp()], oi, mode="faceted_cpu")
    assert oi.calls == []


def test_reject_unknown_mode():
    oi = _OiStub()
    with pytest.raises(ValueError, match="unrecognized"):
        srp.mirror_srp_models([_rp()], oi, mode="bogus-mode")
    assert oi.calls == []


def test_reject_lookup_entries():
    oi = _OiStub()
    m = _rp(lookupForce_B=[[1.0, 0.0, 0.0]])
    with pytest.raises(ValueError, match="lookup-table SRP"):
        srp.mirror_srp_models([m], oi)
    assert oi.calls == []


@pytest.mark.parametrize("cls", ["FacetedSRPEffector",
                                 "FacetedSpacecraftProjectedArea"])
def test_reject_message_driven_faceted_srp(cls):
    oi = _OiStub()
    with pytest.raises(ValueError, match="message-driven facet feed"):
        srp.mirror_srp_models([_fake(cls)], oi)
    with pytest.raises(ValueError, match="use omni env setters instead"):
        srp.mirror_srp_models([_fake(cls)], oi)
    assert oi.calls == []


def test_reject_unknown_srp_class():
    oi = _OiStub()
    with pytest.raises(ValueError, match="not a mirrorable SRP model"):
        srp.mirror_srp_models([_fake("SomeOtherSRPEffector")], oi)
    assert oi.calls == []


@pytest.mark.parametrize("area", [0.0, -1.0])
def test_reject_nonpositive_cannonball_area(area):
    oi = _OiStub()
    with pytest.raises(ValueError, match="must be finite positive"):
        srp.mirror_srp_models([_rp(area=area)], oi)
    assert oi.calls == []


def test_reject_unreadable_cannonball_attrs():
    oi = _OiStub()
    with pytest.raises(ValueError,
                       match="cannot read area/coefficientReflection"):
        srp.mirror_srp_models([_rp(area=None)], oi)
    with pytest.raises(ValueError,
                       match="cannot read area/coefficientReflection"):
        srp.mirror_srp_models([_rp(C_R=None)], oi)
    assert oi.calls == []


def test_reject_nonzero_rot_hat():
    oi = _OiStub()
    m = _faceted(n=1)
    with pytest.raises(ValueError, match="rotHat stays REJECT"):
        srp.mirror_srp_models([m], oi,
                              facet_specs=[_spec(rot=(0.0, 0.0, 1.0))])
    assert oi.calls == []


def test_reject_articulated_facets():
    oi = _OiStub()
    with pytest.raises(ValueError, match="articulated/time-varying"):
        srp.mirror_srp_models([_faceted(n=1, articulated=2)], oi,
                              facet_specs=[_spec()])
    assert oi.calls == []


def test_reject_facet_count_mismatch():
    oi = _OiStub()
    with pytest.raises(ValueError, match="facet counts must agree"):
        srp.mirror_srp_models([_faceted(n=3)], oi,
                              facet_specs=[_spec()])
    assert oi.calls == []


def test_reject_facet_list_length_mismatch():
    oi = _OiStub()
    m = _faceted_with_lists([1.0, 2.0], [_ID3],
                            [[1.0, 0.0, 0.0]] * 2,
                            [[0.0, 0.0, 0.0]] * 2,
                            [[0.0, 0.5, 0.0]] * 2, [0.2] * 2, [0.3] * 2)
    with pytest.raises(ValueError, match="facet list lengths disagree"):
        srp.mirror_srp_models([m], oi)
    assert oi.calls == []


def test_reject_partially_exposed_facet_lists():
    oi = _OiStub()
    m = _faceted(n=1)
    m.facetAreaList = [1.0]
    with pytest.raises(ValueError, match="refuse a partial read"):
        srp.mirror_srp_models([m], oi)
    assert oi.calls == []


def test_reject_facet_geometry_unexposed_needs_specs():
    oi = _OiStub()
    with pytest.raises(ValueError, match="pass facet_specs"):
        srp.mirror_srp_models([_faceted(n=1)], oi)
    assert oi.calls == []


def test_reject_facet_specs_disagree_with_object_lists():
    oi = _OiStub()
    m = _faceted_with_lists([1.0], [_ID3], [[1.0, 0.0, 0.0]],
                            [[0.0, 0.0, 0.0]], [[0.0, 0.5, 0.0]],
                            [0.2], [0.3])
    with pytest.raises(ValueError, match="facet_specs disagrees"):
        srp.mirror_srp_models([m], oi, facet_specs=[_spec(), _spec()])
    assert oi.calls == []


def test_reject_facet_specs_with_multiple_effectors():
    oi = _OiStub()
    with pytest.raises(ValueError,
                       match="per-effector attribution is ambiguous"):
        srp.mirror_srp_models([_faceted(n=1), _faceted(n=1)], oi,
                              facet_specs=[_spec()])
    assert oi.calls == []


def test_reject_bad_facet_spec_shape():
    oi = _OiStub()
    with pytest.raises(ValueError, match="facet spec 0 needs the 7 addFacet"):
        srp.mirror_srp_models([_faceted(n=1)], oi,
                              facet_specs=[(1.0, None, [1.0, 0.0, 0.0])])
    assert oi.calls == []


def test_reject_missing_facet_spec_field():
    oi = _OiStub()
    bad = _spec()
    del bad["r_CopB_B"]
    with pytest.raises(ValueError, match="facet spec 0 is missing r_CopB_B"):
        srp.mirror_srp_models([_faceted(n=1)], oi, facet_specs=[bad])
    assert oi.calls == []


def test_reject_zero_area_facet():
    oi = _OiStub()
    with pytest.raises(ValueError, match="facet 0 area=0.0 must be finite"):
        srp.mirror_srp_models([_faceted(n=1)], oi,
                              facet_specs=[_spec(area=0.0)])
    assert oi.calls == []


def test_reject_missing_oi():
    m = _rp()
    with pytest.raises(ValueError, match="needs an omni integrator"):
        srp.mirror_srp_models([m], None)


# --------------------------------------------------------------------------
# STAGED discipline: not wired into swap.py, gap test stays green
# --------------------------------------------------------------------------

def test_srp_wired_adoption():
    """GAP-CLOSED adoption (wiring batch, PLAN_ENV_MIRROR §8, list.md §A1 --
    cf. tests/test_replacement_matrix.py::test_env_mirror_gap_open style):
    the SRP reader is WIRED into swap.py, so this pin adopts the wiring
    instead of asserting its absence. Red here = wiring removed or
    de-specced -- restore it or move the cell back to the open gap.

    Wiring triple: (a) the ``_mirror_srp`` def exists and fires on BOTH
    paths (legacy ``mirror_stock_config`` + ``_mirror_single_object`` via
    ``_mirror_env_all``); (b) the registry ``_MIRRORED_ENV`` covers the SRP
    family; (c) delegation reaches the staged module's public surface
    (``mirror_srp_models``) on a cannonball model (area=2.0, C_R=1.5 ->
    cr=0.5) with its REQUIRED sun link forwarded to the slot-0 member."""
    import ast
    src = SWAP_SRC.read_text(encoding="utf-8")
    tree = ast.parse(src)
    fns = {n.name: n for n in ast.walk(tree)
           if isinstance(n, ast.FunctionDef)}

    def _calls(fn):
        return {n.func.id for n in ast.walk(fn)
                if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Name)}

    assert "_mirror_srp" in fns and "_mirror_env_all" in fns
    assert "_mirror_env_all" in _calls(fns["mirror_stock_config"])
    assert "_mirror_env_all" in _calls(fns["_mirror_single_object"])
    assert "_mirror_srp" in _calls(fns["_mirror_env_all"])
    spec = importlib.util.spec_from_file_location("swap_srp_adoption",
                                                  SWAP_SRC)
    swap = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(swap)
    assert {"RadiationPressure", "FacetSRPDynamicEffector",
            "FacetedSRPEffector",
            "FacetedSpacecraftProjectedArea"} <= set(swap._MIRRORED_ENV)

    class _SwapOiStub:
        """Slot-0 env message members subscribe; every setter records."""

        def __init__(self):
            self.calls = []
            self.sunStateInMsg = SimpleNamespace(
                subscribeTo=lambda src: self.calls.append(
                    ("sunStateInMsg.subscribeTo", src)))
            self.eclipseInMsg = SimpleNamespace(
                subscribeTo=lambda src: self.calls.append(
                    ("eclipseInMsg.subscribeTo", src)))

        def setSRPCannonball(self, *a):
            self.calls.append(("setSRPCannonball",) + tuple(a))

        def setSRPCannonballForObject(self, *a):
            self.calls.append(("setSRPCannonballForObject",) + tuple(a))

        def addSRPFacet(self, *a):
            self.calls.append(("addSRPFacet",) + tuple(a))

        def addSRPFacetForObject(self, *a):
            self.calls.append(("addSRPFacetForObject",) + tuple(a))

    sun_src = object()
    sun_reader = SimpleNamespace(isLinked=lambda: True,
                                 _get_source_owner=lambda: sun_src)
    oi = _SwapOiStub()
    rec = swap._mirror_srp(
        [_rp(area=2.0, C_R=1.5, sunEphmInMsg=sun_reader,
             sunEclipseInMsg=None)], [], oi, None)
    assert rec["srp"][0]["args"] == [2.0, pytest.approx(0.5)]
    assert ("setSRPCannonball", 2.0, 0.5) in oi.calls
    assert ("sunStateInMsg.subscribeTo", sun_src) in oi.calls
    assert rec["sun_link"] == "sunStateInMsg.subscribeTo"
    assert rec["n_sun_sources"] == 1


def test_module_docstring_states_staged():
    doc = srp.__doc__ or ""
    assert "STAGED" in doc
