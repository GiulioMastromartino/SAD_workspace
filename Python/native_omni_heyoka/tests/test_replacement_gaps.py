"""Behavioral loud-reject pins: every permanent NO-GO raises before first propagation.

All tests run on BARE pytest (stdlib only; numpy importorskipped per-test
where the exercised path needs it). No Basilisk, no wheel, no sims: stock
objects are fakes (SimpleNamespace + MRO-named classes), because swap.py reads
the task lists by class name and attributes only. swap.py is loaded from its
file path (module-level imports are stdlib-only; Basilisk/numpy imports are
function-local), so importing it never needs the host env.

Each test names the envelope cell it pins. If a rejection is ever relaxed
into a model, the corresponding test goes red -- move the cell to the parity
suite (Section A of test_replacement_matrix.py), do not delete.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parents[1]


def _load_swap():
    path = REPO / "omniheyoka" / "swap.py"
    spec = importlib.util.spec_from_file_location("swap_under_test", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


swap = _load_swap()

# MRO-named fakes: _bases() reads type(model).__mro__ class names.
DynamicEffector = type("DynamicEffector", (), {})
StateEffector = type("StateEffector", (), {})


def _model(cls_name, bases=()):
    return type(cls_name, tuple(bases) + (object,), {})()


def _sim(*models, task="t"):
    return SimpleNamespace(
        TaskList=[SimpleNamespace(Name=task, TaskModels=list(models))])


class _OiStub:
    """Records oi.* calls; any unexpected attribute access fails loudly."""

    def __init__(self):
        self.calls: list = []

    def addThruster(self, *a):
        self.calls.append(("addThruster",) + tuple(a))

    def addWheel(self, *a):
        self.calls.append(("addWheel",) + tuple(a))

    def linkWheelLog(self, *a):
        self.calls.append(("linkWheelLog",) + tuple(a))


def _thr_pair(**kw):
    """Matching (cfg, data) thruster pair; kwargs override cfg fields."""
    loc, direction = [0.0, 0.0, 1.0], [0.0, 0.0, -1.0]
    cfg = {"thrLoc_B": list(loc), "thrDir_B": list(direction),
           "MinOnTime": 0.0, "steadyIsp": 220.0, "MaxThrust": 1.0}
    cfg.update(kw)
    # Stock thrusterData always carries the blowdown vectors (possibly empty);
    # the mirror reads them with a bare getattr, so fakes must too.
    return (SimpleNamespace(**cfg),
            SimpleNamespace(thrLoc_B=list(loc), thrDir_B=list(direction),
                            thrBlowDownCoeff=[], ispBlowDownCoeff=[]))


def _thr_factory_pair(**kw):
    cfg, d = _thr_pair(**kw)
    return (SimpleNamespace(thrusterList={"t0": cfg}),
            SimpleNamespace(thrusterData=[d]))


# --------------------------------------------------------------------------
# Mirror surface + positive read paths (works pins, host-free)
# --------------------------------------------------------------------------

def test_mirrored_set_exact():
    assert swap._MIRRORED == {"Spacecraft", "ReactionWheelStateEffector",
                              "ThrusterDynamicEffector"}
    # Wiring batch (PLAN_ENV_MIRROR §8, list.md §A1): the env-effector
    # classes routed to the _mirror_* readers instead of the generic
    # dynamics reject. _MIRRORED itself stays hub+RW+thruster (the probe
    # parses that literal and a staged suite pins it via AST); the UNION
    # below is the effective mirrored set. A new reader landing changes
    # one of the two sets -- red means move cells to Section A.
    assert (swap._MIRRORED | swap._MIRRORED_ENV) == {
        "Spacecraft", "ReactionWheelStateEffector",
        "ThrusterDynamicEffector", "DragDynamicEffector",
        "FacetDragDynamicEffector", "RadiationPressure",
        "FacetSRPDynamicEffector", "FacetedSRPEffector",
        "FacetedSpacecraftProjectedArea", "MtbEffector",
        "GravityEffector", "ExtForceTorque", "ThrusterStateEffector"}


def test_task_effectors_order():
    a, b, c = _model("Spacecraft"), _model("X"), _model("Y")
    sim = SimpleNamespace(TaskList=[
        SimpleNamespace(Name="t1", TaskModels=[a, b]),
        SimpleNamespace(Name="t2", TaskModels=[c])])
    assert swap.task_effectors(sim) == [("t1", a), ("t1", b), ("t2", c)]


def test_central_mu_single_body():
    sc = SimpleNamespace(gravField=SimpleNamespace(
        gravBodies=[SimpleNamespace(isCentralBody=True, mu=398600.0)]))
    assert swap._central_mu(sc) == 398600.0


def test_hub_record_reads():
    np = pytest.importorskip("numpy")
    assert np  # exercised via _as_vec3
    h = SimpleNamespace(mHub=20.0,
                        IHubPntBc_B=[[10.0, 0.0, 0.0], [0.0, 11.0, 0.0],
                                     [0.0, 0.0, 12.0]],
                        r_BcB_B=[0.0, 0.0, 0.0])
    rec = swap._hub_record(SimpleNamespace(hub=h))
    assert rec["mass"] == 20.0 and rec["rBcB"] == [0.0, 0.0, 0.0]


def test_wheel_balanced_mirror_path():
    pytest.importorskip("numpy")
    w = SimpleNamespace(RWModel="BalancedWheels", gsHat_B=[1.0, 0.0, 0.0],
                        Js=0.05)
    eff = SimpleNamespace(ReactionWheelData=[w], rwOutMsgs=[object()])
    oi = _OiStub()
    n, fric = swap._mirror_wheels([eff], oi)
    assert (n, fric) == (1, False)
    assert oi.calls[0][0] == "addWheel" and oi.calls[1][0] == "linkWheelLog"


def test_gg_and_fuel_pass_envelope_scan():
    """GravityGradientEffector + FuelTank are NOT dynamics rejects."""
    sim = _sim(_model("Spacecraft"), _model("GravityGradientEffector"),
               _model("FuelTank"))
    assert swap._reject_unmirrorable_tasks(sim) is None


def test_routed_env_classes_pass_global_scan():
    """Wiring batch: newly-mirrorable env classes no longer raise the
    generic reject (their _mirror_* reader raises its own specific
    reject when the selection is unmirrorable)."""
    models = [_model(n, bases=(DynamicEffector,))
              for n in ("DragDynamicEffector", "FacetDragDynamicEffector",
                        "RadiationPressure", "FacetSRPDynamicEffector",
                        "FacetedSRPEffector", "FacetedSpacecraftProjectedArea",
                        "MtbEffector", "GravityEffector")]
    assert swap._reject_unmirrorable_tasks(_sim(*models)) is None


# --------------------------------------------------------------------------
# Loud rejects (each raises ValueError pre-sim)
# --------------------------------------------------------------------------

def test_central_mu_with_extra_body_returns_central():
    """Wiring-batch flip BY DESIGN (PLAN_ENV_MIRROR §6.1, list.md §A1):
    one central + extras is now the mirrorable N-body shape (extras go
    to addEnvironmentBody, capped at two by the env-bodies reader per
    omni_spacecraft.cpp:919). The old single-body-only pin is replaced
    by the two-centrals reject below."""
    sc = SimpleNamespace(gravField=SimpleNamespace(gravBodies=[
        SimpleNamespace(isCentralBody=True, mu=1.0),
        SimpleNamespace(isCentralBody=False, mu=2.0)]))
    assert swap._central_mu(sc) == 1.0


def test_reject_two_central_bodies():
    sc = SimpleNamespace(gravField=SimpleNamespace(gravBodies=[
        SimpleNamespace(isCentralBody=True, mu=1.0),
        SimpleNamespace(isCentralBody=True, mu=2.0)]))
    with pytest.raises(ValueError, match="exactly one central body"):
        swap._central_mu(sc)


def test_reject_unmirrorable_env_effector():
    sim = _sim(_model("Spacecraft"),
               _model("FakeDrag", bases=(DynamicEffector,)))
    with pytest.raises(ValueError, match="use omni env setters instead"):
        swap.mirror_stock_config(sim, None, None)
    with pytest.raises(ValueError, match="use omni env setters instead"):
        swap._reject_unmirrorable_tasks(sim)


def test_reject_two_rw_effectors():
    with pytest.raises(ValueError, match="registration order is ambiguous"):
        swap._mirror_wheels([object(), object()], None)


def test_reject_rw_logs_fewer_than_wheels():
    pytest.importorskip("numpy")
    w = SimpleNamespace(RWModel="BalancedWheels", gsHat_B=[1.0, 0.0, 0.0],
                        Js=0.05)
    eff = SimpleNamespace(ReactionWheelData=[w], rwOutMsgs=[])
    with pytest.raises(ValueError, match="fewer rwOutMsgs"):
        swap._mirror_wheels([eff], _OiStub())


def test_reject_jitter_wheel():
    eff = SimpleNamespace(
        ReactionWheelData=[SimpleNamespace(RWModel="JitterSimple")],
        rwOutMsgs=[object()])
    with pytest.raises(ValueError, match="only BalancedWheels is mirrored"):
        swap._mirror_wheels([eff], None)


def test_reject_per_object_needs_explicit_lists():
    with pytest.raises(ValueError, match="needs explicit"):
        swap.mirror_stock_config(None, None, None, object_index=1)


def test_reject_sync_empty_specs():
    with pytest.raises(ValueError, match="need >= 1 object spec"):
        swap.mirror_sync_configs(None, None, [])


def test_reject_sync_spec_missing_keys():
    spec = {"sc": None, "rw_models": [], "thr_models": []}  # no fuel/use_gg
    with pytest.raises(ValueError, match="needs explicit"):
        swap.mirror_sync_configs(None, None, [spec])


def test_reject_thruster_no_factory():
    with pytest.raises(ValueError, match="thr_factory given"):
        swap._mirror_thrusters([object()], None, None, None)


def test_reject_thruster_join_mismatch():
    pytest.importorskip("numpy")
    cfg, d = _thr_pair()
    eff = SimpleNamespace(thrusterData=[d, d])  # 2 effectors vs 1 config
    with pytest.raises(ValueError, match="geometry join needs"):
        swap._mirror_thrusters([eff],
                               SimpleNamespace(thrusterList={"t0": cfg}),
                               None, _OiStub())


def test_reject_thruster_order_mismatch():
    pytest.importorskip("numpy")
    cfg, _ = _thr_pair()
    d = SimpleNamespace(thrLoc_B=[1.0, 0.0, 0.0], thrDir_B=[0.0, 0.0, -1.0])
    eff = SimpleNamespace(thrusterData=[d])
    with pytest.raises(ValueError, match="order mismatch"):
        swap._mirror_thrusters([eff],
                               SimpleNamespace(thrusterList={"t0": cfg}),
                               None, _OiStub())


def test_reject_min_pulse():
    pytest.importorskip("numpy")
    fac, eff = _thr_factory_pair(MinOnTime=0.02)
    with pytest.raises(ValueError, match="MinOnTime="):
        swap._mirror_thrusters([eff], fac, None, _OiStub())


def test_reject_cutoff_zero():
    pytest.importorskip("numpy")
    fac, eff = _thr_factory_pair()
    eff.thrusterData[0].cutoffFrequency = 0.0
    with pytest.raises(ValueError, match="degenerate"):
        swap._mirror_thrusters([eff], fac, None, _OiStub())


def test_reject_blowdown():
    pytest.importorskip("numpy")
    fac, eff = _thr_factory_pair()
    eff.thrusterData[0].thrBlowDownCoeff = [0.9]
    with pytest.raises(ValueError, match="out of the staircase envelope"):
        swap._mirror_thrusters([eff], fac, None, _OiStub())


def test_reject_swirl():
    pytest.importorskip("numpy")
    fac, eff = _thr_factory_pair(MaxSwirlTorque=0.5)
    with pytest.raises(ValueError, match="swirl is out"):
        swap._mirror_thrusters([eff], fac, None, _OiStub())


def test_reject_thruster_no_cmd_msg():
    pytest.importorskip("numpy")
    fac, eff = _thr_factory_pair()
    oi = _OiStub()
    with pytest.raises(ValueError, match="no thr_cmd_msg"):
        swap._mirror_thrusters([eff], fac, None, oi)
    assert oi.calls and oi.calls[0][0] == "addThruster"  # geometry mirrored first


def test_reject_multi_tank():
    with pytest.raises(ValueError, match="mirrors exactly one"):
        swap._mirror_fuel([object(), object()], None, [220.0])


def test_reject_tank_without_thrusters():
    with pytest.raises(ValueError, match="no mirrored thrusters"):
        swap._mirror_fuel([object()], None, [])


# --------------------------------------------------------------------------
# Env-wiring pins (PLAN_ENV_MIRROR §8 batch; all host-free fakes)
# --------------------------------------------------------------------------

class _EnvOiStub:
    """Oi stub: slot-0 env message members subscribe; every setter (or
    per-object twin) records its call (header surface hpp:106-120,
    :172-187, :209-244 -- anything else would fail loudly elsewhere)."""

    def __init__(self):
        self.calls: list = []
        for member in ("atmoDensInMsg", "windVelInMsg", "sunStateInMsg",
                       "moonStateInMsg", "eclipseInMsg", "magFieldInMsg"):
            def _sub(src, _member=member):
                self.calls.append((_member + ".subscribeTo", src))
            self.__dict__[member] = SimpleNamespace(subscribeTo=_sub)

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)

        def _rec(*args):
            self.calls.append((name,) + tuple(args))
        return _rec


def _env_oi_stub():
    return _EnvOiStub()


def _linked_reader(owner):
    return SimpleNamespace(isLinked=lambda: True,
                           _get_source_owner=lambda: owner)


def _cannonball_rp(area=2.0, c_r=1.5, sun_owner=None, eclipse_linked=False):
    m = _model("RadiationPressure")
    m.area = area
    m.coefficientReflection = c_r
    m.sunEphmInMsg = _linked_reader(sun_owner)
    m.sunEclipseInMsg = SimpleNamespace(
        isLinked=lambda: eclipse_linked,
        _get_source_owner=lambda: None)
    return m


def test_reject_per_object_needs_explicit_env_lists():
    """WP25-A4 rule extended to env: a per-object call with the main
    lists explicit but env lists missing raises (env leaves no named
    states for the C++ claim check, so None must never mean [])."""
    with pytest.raises(ValueError, match="needs explicit drag_models"):
        swap.mirror_stock_config(None, None, None, object_index=1,
                                 rw_models=[], thr_models=[],
                                 fuel_models=[], use_gg=False)


def test_reject_two_cannonball_srp():
    """Two RadiationPressure models would overwrite (not sum) the single
    omni cannonball record (verified cpp:282-291) -- loud reject."""
    models = [_cannonball_rp(), _cannonball_rp()]
    with pytest.raises(ValueError, match="ONE cannonball"):
        swap._mirror_srp(models, [], _env_oi_stub(), None)


def test_srp_cannonball_sun_link_and_record():
    """Adoption: cannonball SRP mirrors area/cr with the REQUIRED sun
    link subscribed from the recovered stock source (hpp:242)."""
    owner = object()
    oi = _env_oi_stub()
    rec = swap._mirror_srp([_cannonball_rp(sun_owner=owner)], [], oi,
                           None)
    assert rec["srp"][0]["area"] == 2.0
    assert rec["srp"][0]["cr"] == 0.5  # cr = C_R - 1 (hpp:113-116)
    assert rec["sun_link"] == "sunStateInMsg.subscribeTo"
    assert ("sunStateInMsg.subscribeTo", owner) in oi.calls


def test_env_family_sets_match_staged_surfaces():
    """Drift pin: swap.py adapts to the staged public surfaces -- the
    census sets must equal the staged module constants they delegate
    to (staged modules themselves are untouched by this diff)."""
    import importlib.util

    def _load(name):
        path = REPO / "omniheyoka" / (name + ".py")
        spec = importlib.util.spec_from_file_location(
            "gaps_staged_" + name, path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod

    drag = _load("env_mirror_drag")
    assert drag.SUPPORTED_CLASSES == {"DragDynamicEffector"}
    assert drag.SUPPORTED_CLASSES <= swap._DRAG_FAMILY
    srp = _load("env_mirror_srp")
    assert srp.SUPPORTED_CLASSES == {"RadiationPressure",
                                     "FacetSRPDynamicEffector"}
    assert srp.SUPPORTED_CLASSES <= swap._SRP_FAMILY
    mag = _load("env_mirror_mag")
    assert mag.SUPPORTED_CLASSES == (swap._MTB_FAMILY
                                     | swap._MAGFIELD_FAMILY)
    ecl = _load("env_mirror_eclipse")
    assert ecl.SUPPORTED_CLASSES == swap._ECLIPSE_FAMILY
    eb = _load("env_mirror_envbodies")
    assert eb.SUPPORTED_CLASSES == swap._GRAV_FAMILY
    assert eb.STAY_IN_TASK_CLASSES == swap._EPHEM_FAMILY
