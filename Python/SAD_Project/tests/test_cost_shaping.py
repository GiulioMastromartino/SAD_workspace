"""Tests for the piecewise box/cone optimizer cost."""
import math

import numpy as np
import pytest

from SAD_Project.core import config as C
from SAD_Project.optimization.objective import reduce_costs, threshold_cost


def test_threshold_cost_quadratic_inside_exponential_outside():
    # Quadratic branch: (value/limit)^2, so 5 km -> 1 and 2.5 km -> 0.25.
    assert threshold_cost(0.0, 5.0) == pytest.approx(0.0)
    assert threshold_cost(2.5, 5.0) == pytest.approx(0.25)
    assert threshold_cost(5.0, 5.0) == pytest.approx(1.0)

    # Outside: exp(rate * (value/limit - 1)); continuous at the boundary.
    rate = float(C.COST_EXP_RATE)
    assert threshold_cost(7.5, 5.0, 1.0, rate) == pytest.approx(math.exp(rate * 0.5))
    assert threshold_cost(10.0, 5.0, 1.0, rate) == pytest.approx(math.exp(rate))


def test_reduce_costs_uses_max_box_and_cone_terms():
    res = {
        'final_rate': np.array([0.0]),
        'fuel_used': np.array([0.0]),
        'box_max': np.array([10.0]),
        'track_ang_max': np.array([20.0]),
        'reached_track': np.array([1]),
        'failed': np.array([0]),
        'slew_settle': np.array([4.0]),
    }
    costs, run_data = reduce_costs(res)
    rate = float(C.COST_EXP_RATE)
    expected = threshold_cost(10.0, 5.0, rate=rate) + threshold_cost(20.0, 15.0, rate=rate)
    assert costs[0] == pytest.approx(expected)
    assert run_data[0]['box_cost'] == pytest.approx(threshold_cost(10.0, 5.0, rate=rate))
    assert run_data[0]['cone_cost'] == pytest.approx(threshold_cost(20.0, 15.0, rate=rate))


@pytest.mark.parametrize("limit", [C.ORBIT_BOX_KM, C.SUN_KEEP_IN_CONE_DEG])
def test_threshold_cost_is_continuous_at_box_and_cone_edges(limit):
    """Both branches must meet at the 5 km and 15 deg boundaries."""
    rate = float(C.COST_EXP_RATE)
    at_edge = threshold_cost(limit, limit, 1.0, rate)
    assert at_edge == pytest.approx(1.0)
    assert threshold_cost(limit - 1e-9, limit, 1.0, rate) == pytest.approx(1.0, abs=2e-9)
    assert threshold_cost(limit + 1e-9, limit, 1.0, rate) == pytest.approx(1.0, abs=2e-9)
