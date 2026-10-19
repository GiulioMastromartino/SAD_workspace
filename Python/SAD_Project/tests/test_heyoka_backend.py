"""Smoke/parity tests for the optional Heyoka mission-batch backend."""
import sys
from pathlib import Path

import numpy as np
import pytest

_HERE = Path(__file__).resolve()
sys.path.insert(0, str(_HERE.parents[1] / "python"))  # SAD_Project/python
sys.path.insert(0, str(_HERE.parents[2]))             # Python/

import sad_py
from SAD_Project.optimization.objective import build_batch_params
from SAD_Project.optimization.params import DEFAULTS


pytestmark = pytest.mark.skipif(
    not sad_py.heyoka_available(),
    reason="sad_py was built without the Heyoka backend",
)


def test_heyoka_single_run_matches_rk4():
    bp, _ = build_batch_params(DEFAULTS, disperse=True, timing=False)

    bp.use_heyoka = False
    rk = sad_py.run_mission_batch(1, 42, bp)

    bp.use_heyoka = True
    hy = sad_py.run_mission_batch(1, 42, bp)

    assert int(hy["failed"][0]) == 0
    assert int(hy["reached_track"][0]) == 1

    # The two integrators should agree far below the mission-level tolerances.
    np.testing.assert_allclose(hy["fuel_used"], rk["fuel_used"],
                               rtol=1e-6, atol=1e-12)
    np.testing.assert_allclose(hy["final_rate"], rk["final_rate"],
                               rtol=1e-5, atol=1e-11)
    np.testing.assert_allclose(hy["box_max"], rk["box_max"],
                               rtol=1e-6, atol=1e-6)
    np.testing.assert_allclose(hy["track_ang_max"], rk["track_ang_max"],
                               rtol=1e-5, atol=1e-6)


def test_heyoka_reuses_cached_integrator_across_calls():
    bp, _ = build_batch_params(DEFAULTS, disperse=True, timing=False)
    bp.use_heyoka = True

    first = sad_py.run_mission_batch(2, 7, bp)
    second = sad_py.run_mission_batch(2, 7, bp)

    assert int(np.sum(first["failed"])) == 0
    assert int(np.sum(second["failed"])) == 0
    np.testing.assert_allclose(first["final_rate"], second["final_rate"],
                               rtol=0.0, atol=1e-15)
