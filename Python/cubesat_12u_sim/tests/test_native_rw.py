"""Tests for the native Heyoka RW path (cpp_native.NativeSpacecraft).

Build-independent tests use stub models (no nativerw needed). Tests marked
``needs_native`` run the real extension + Basilisk sims (600 s settled
parity, still <2 s wall time).
ISC License - see native_rw/LICENSE.
"""
import numpy as np
import pytest

from cubesat_12u_sim.cpp_native import NativeSpacecraft, available

needs_native = pytest.mark.skipif(not available(),
                                  reason="nativerw extension not built")


class FakeModel:
    """Records C++ setter calls (stands in for HeyokaRWDynamics)."""

    def __init__(self):
        self.calls = {}

    def _rec(self, name, *args):
        self.calls.setdefault(name, []).append(tuple(args))

    def setMass(self, m):
        self._rec("setMass", m)

    def setInertia(self, *args):
        self._rec("setInertia", *args)

    def setR_CN_NInit(self, *args):
        self._rec("setR_CN_NInit", *args)

    def setV_CN_NInit(self, *args):
        self._rec("setV_CN_NInit", *args)

    def setSigmaBNInit(self, *args):
        self._rec("setSigmaBNInit", *args)

    def setOmegaBNInit(self, *args):
        self._rec("setOmegaBNInit", *args)

    def setWheelSpeedsInitRPM(self, *args):
        self._rec("setWheelSpeedsInitRPM", *args)

    def setMu(self, mu):
        self._rec("setMu", mu)


class FakeBody:
    def __init__(self, mu, central=False):
        self.mu = mu
        self.isCentralBody = central


class FakeBodies(list):
    pass


def test_hub_forwarding_matrix_and_columns():
    hub = NativeSpacecraft._Hub(FakeModel())
    hub.mHub = 750.0
    hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    hub.r_CN_NInit = [[1.0], [2.0], [3.0]]
    hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]
    hub.wheelSpeedsInit = [100.0, 200.0, 300.0]
    c = hub._m.calls
    assert c["setMass"][-1] == (750.0,)
    assert c["setInertia"][-1] == (900.0, 800.0, 600.0, 0.0, 0.0, 0.0)
    assert c["setR_CN_NInit"][-1] == (1.0, 2.0, 3.0)
    assert c["setSigmaBNInit"][-1] == (0.1, 0.2, -0.3)
    assert c["setWheelSpeedsInitRPM"][-1] == (100.0, 200.0, 300.0)


def test_hub_unknown_field_warns_and_com_offset_warns():
    hub = NativeSpacecraft._Hub(FakeModel())
    with pytest.warns(UserWarning, match="not modeled"):
        hub.someFutureField = 1.0
    with pytest.warns(UserWarning, match="CoM offset"):
        hub.r_BcB_B = [0.01, 0.0, 0.0]


def test_gravfield_selects_central_mu():
    hub_model = FakeModel()
    gf = NativeSpacecraft._GravField(hub_model)
    with pytest.warns(UserWarning, match="only central-body"):
        gf.setGravBodies(FakeBodies([FakeBody(3.0e14, False), FakeBody(4.0e14, True)]))
    assert hub_model.calls["setMu"][-1] == (4.0e14,)


def test_gravfield_single_unflagged_body_and_multi_warns():
    m = FakeModel()
    gf = NativeSpacecraft._GravField(m)
    gf.setGravBodies(FakeBodies([FakeBody(3.986004418e14)]))
    assert m.calls["setMu"][-1] == (3.986004418e14,)
    with pytest.warns(UserWarning, match="only central-body"):
        gf.setGravBodies(FakeBodies([FakeBody(1.0, True), FakeBody(2.0)]))


def test_wrapper_requires_build_when_missing(monkeypatch):
    import cubesat_12u_sim.cpp_native as cn

    monkeypatch.setattr(cn, "_NATIVE", False)
    assert not cn.available()
    with pytest.raises(RuntimeError, match="nativerw"):
        cn.NativeSpacecraft()


@needs_native
def test_grav_addBodiesTo_accepted():
    from Basilisk.utilities import simIncludeGravBody

    sc = NativeSpacecraft()
    sc.hub.mHub = 750.0
    sc.hub.IHubPntBc_B = np.diag([900.0, 800.0, 600.0])
    grav = simIncludeGravBody.gravBodyFactory()
    earth = grav.createEarth()
    earth.isCentralBody = True
    grav.addBodiesTo(sc)  # must not raise; central mu forwarded
    assert len(sc.gravField.bodies) == 1
    assert float(sc.gravField.bodies[0].mu) == float(earth.mu)
    assert sc.getMu() == pytest.approx(float(earth.mu))  # reached C++


@needs_native
def test_native_parity_short():
    from cubesat_12u_sim.bsk_rw_stock import run_stock_rw
    from cubesat_12u_sim.cpp_native import run_native_rw_pipeline
    from cubesat_12u_sim.heyoka_attitude import mrp_diff

    # Full 600 s: attitude error peaks mid-transient (~4e-3 deg at 60 s)
    # and settles to ~7e-5 deg; assert the settled parity (still <0.5 s).
    s = run_stock_rw(duration=600.0, dt=0.1)
    c = run_native_rw_pipeline(duration=600.0, dt=0.1)
    assert len(c["t"]) == len(s["t"])
    att = float(np.degrees(4 * np.arctan(np.linalg.norm(
        mrp_diff(c["sigma_BN"][-1], s["sigma_BN"][-1])))))
    rate = float(np.degrees(np.linalg.norm(c["omega_BN_B"][-1] - s["omega_BN_B"][-1])))
    dw = float(np.abs(np.asarray(c["wheel_speeds"], float)
                      - np.asarray(s["wheel_speeds"], float)).max()
              * 60.0 / (2.0 * np.pi))
    dr = float(np.linalg.norm(c["r_CN_N"][-1] - s["r_CN_N"][-1]))
    # settled values: att ~7e-5 deg, rate ~1e-6 deg/s, wheels ~0.25 RPM max
    assert att < 5e-4
    assert rate < 1e-5
    assert dw < 0.5
    assert dr < 1.0


@needs_native
def test_rw_mc_smoke():
    from cubesat_12u_sim.rw_monte_carlo import (
        RWMonteCarloConfig,
        run_rw_monte_carlo,
        sample_rw_dispersions,
    )

    cfg = RWMonteCarloConfig()
    cfg.duration = 60.0
    a = sample_rw_dispersions(7, cfg)
    b = sample_rw_dispersions(7, cfg)
    assert a["mass"] == b["mass"]
    assert np.allclose(a["inertia"], b["inertia"])
    out = run_rw_monte_carlo(num_runs=2, base_seed=11, cfg=cfg,
                             backend="native", jobs=1, verbose=False,
                             save_dir=None)
    assert len(out["runs"]) == 2
    assert out["stats"]["summary"]["n_runs"] == 2
    assert set(out["runs"][0]["kpis"]) == {"final_att_err_deg",
                                           "final_rate_deg_s",
                                           "max_wheel_rpm",
                                           "settle_time_s"}


@needs_native
def test_native_is_sysmodel():
    from Basilisk.architecture import cSysModel

    sc = NativeSpacecraft()
    assert isinstance(sc, cSysModel.SysModel)
    assert sc.model is sc  # back-compat alias
