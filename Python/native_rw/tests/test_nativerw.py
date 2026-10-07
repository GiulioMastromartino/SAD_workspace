"""In-package tests for the built nativerw wheel (no sims; fast).

Run from the source tree against the installed wheel:
    ./env/bin/python -m pytest native_rw/tests -q
"""
import glob
import os
import sys
import tomllib
from pathlib import Path

SRC_ROOT = Path(__file__).resolve().parent.parent


def test_top_level_csysmodel_shim():
    import pytest

    cSysModel = pytest.importorskip("cSysModel")
    from Basilisk.architecture import cSysModel as _bsk

    assert cSysModel.SysModel is _bsk.SysModel


def test_module_instantiates_and_configures():
    import pytest

    nativerw = pytest.importorskip("nativerw")

    sc = nativerw.HeyokaRWDynamics()
    sc.ModelTag = "testSat"
    sc.setMass(750.0)
    sc.setInertia(900.0, 800.0, 600.0, 0.0, 0.0, 0.0)
    sc.setRotorInertia(0.08, 0.08, 0.08)
    sc.setUMax(0.2)
    sc.setTol(1e-12)
    sc.setMu(3.986004418e14)
    sc.setR_CN_NInit(1.0, 2.0, 3.0)
    sc.setV_CN_NInit(4.0, 5.0, 6.0)
    sc.setSigmaBNInit(0.1, 0.2, -0.3)
    sc.setOmegaBNInit(0.01, -0.02, 0.03)
    sc.setWheelSpeedsInitRPM(100.0, 200.0, 300.0)
    assert sc.getMu() == 3.986004418e14
    for attr in ("scStateOutMsg", "rwSpeedOutMsg", "rwMotorCmdInMsg"):
        assert hasattr(sc, attr), attr
    # no-scheduler round trip: first UpdateState publishes the ICs verbatim
    sc.UpdateState(0)
    assert sc.getSigmaBN(0) == pytest.approx(0.1)
    assert sc.getSigmaBN(2) == pytest.approx(-0.3)
    assert sc.getOmegaBN(1) == pytest.approx(-0.02)
    assert sc.getWheelSpeed(0) == pytest.approx(100.0 * 2 * 3.141592653589793 / 60.0)


def test_pins_lockstep():
    """pyproject pins and installed versions must agree (ABI guard)."""
    import Basilisk
    import bsk_sdk

    with open(SRC_ROOT / "pyproject.toml", "rb") as fh:
        proj = tomllib.load(fh)
    reqs = proj["build-system"]["requires"] + proj["project"].get("dependencies", [])

    def pinned(name):
        hits = [r.split("==")[1] for r in reqs if r.startswith(name + "==")]
        assert hits, f"no {name} pin in pyproject"
        assert len(set(hits)) == 1, f"conflicting {name} pins: {hits}"
        return hits[0]

    sdk_pin, bsk_pin = pinned("bsk-sdk"), pinned("bsk")
    assert sdk_pin == bsk_pin, f"sdk {sdk_pin} != bsk {bsk_pin}"
    assert Basilisk.__version__ == bsk_pin, Basilisk.__version__
    assert bsk_sdk.bsk_version() == bsk_pin, bsk_sdk.bsk_version()


def test_heyoka_linked():
    """The extension must LC_LOAD libheyoka (no dynamic_lookup dependence)."""
    import nativerw

    if sys.platform != "darwin":
        import pytest
        pytest.skip("otool check is macOS-only (use ldd on Linux)")
    pkg = Path(nativerw.__file__).resolve().parent
    sos = sorted(glob.glob(str(pkg / "_heyokaRWDynamics*.so")))
    assert sos, "compiled module not found in nativerw/"
    if sys.platform == "darwin":
        out = os.popen(f"otool -L '{sos[0]}'").read()
    elif sys.platform.startswith("linux"):
        out = os.popen(f"ldd '{sos[0]}'").read()
    else:
        import pytest
        pytest.skip(f"no linkage inspector for {sys.platform}")
    assert "heyoka" in out, f"libheyoka not linked:\n{out}"
