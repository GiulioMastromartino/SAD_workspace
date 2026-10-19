"""Objective function for control parameter optimization (C++ batch kernel).

The C++ kernel can additionally return a full timing registry
(``BatchParams.timing = True``).  ``evaluate(..., return_timing=True)``
surfaces it so the optimizer and the diagnostic scripts can attribute time
to each subsystem.
"""
import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import sad_py
from SAD_Project.core import config as C


def build_batch_params(theta, disperse=True, timing=False):
    """Translate a parameter vector into a sad_py.BatchParams object."""
    from SAD_Project.optimization.params import theta_to_params
    p = theta_to_params(theta)
    bp = sad_py.BatchParams()
    bp.detumble_K = p['detumble_K']
    bp.detumble_P = p['detumble_P']
    bp.detumble_Ki = p['detumble_Ki']
    bp.slew_K = p['slew_K']
    bp.slew_P = p['slew_P']
    bp.slew_Ki = p['slew_Ki']
    bp.track_K = p['track_K']
    bp.track_P = p['track_P']
    bp.track_Ki = p['track_Ki']
    bp.TS = p['TS']
    bp.R_ORBIT = C.R_ORBIT
    bp.MU = C.MU_EARTH
    bp.mass = C.MASS
    bp.disperse = disperse
    bp.timing = timing
    bp.track_dt = C.TRACK_DT
    bp.track_embedded = True
    bp.track_sample_dt = 10.0
    bp.implicit_pd = bool(getattr(C, 'IMPLICIT_PD', True))
    bp.implicit_track_only = bool(getattr(C, 'IMPLICIT_TRACK_ONLY', True))
    bp.tau_max = float(getattr(C, 'ACS_TAU_MAX', 0.2))
    bp.disp_r_m = float(getattr(C, 'DISP_R_KM', 1.0)) * 1000.0
    bp.disp_v_mps = float(getattr(C, 'DISP_V_MPS', 0.5))
    return bp, p


def threshold_cost(value, limit, weight=1.0, rate=1.0):
    """Quadratic inside ``limit``, exponential outside (continuous at the boundary).

    ``value`` and ``limit`` are in the same units.  At ``value == limit`` both
    branches return ``weight``.
    """
    if not np.isfinite(value):
        return 0.0
    x = max(float(value), 0.0) / float(limit)
    if x <= 1.0:
        return float(weight) * x * x
    return float(weight) * float(np.exp(float(rate) * (x - 1.0)))


def reduce_costs(res):
    """Apply the optimizer cost rule to a raw batch result.

    The box and cone terms are both piecewise: quadratic inside the 5 km box /
    15 deg cone, exponential outside.  They use the per-run maximum radial
    error (``box_max``) and maximum sun angle (``track_ang_max``), so the box
    and cone requirements are directly enforced.  Returns ``(costs, run_data)``.
    """
    final_rate = np.array(res['final_rate'], dtype=float)
    fuel = np.array(res['fuel_used'], dtype=float)
    box = np.array(res.get('box_max', res.get('orbit_err', np.zeros_like(final_rate))), dtype=float)
    cone = np.array(res.get('track_ang_max', np.zeros_like(final_rate)), dtype=float)
    reached = np.array(res['reached_track'])
    failed = np.array(res['failed'])
    settle = np.array(res.get('slew_settle', np.zeros_like(final_rate)), dtype=float)
    box_limit = float(getattr(C, 'ORBIT_BOX_KM', 5.0))
    cone_limit = float(getattr(C, 'SUN_KEEP_IN_CONE_DEG', 15.0))
    box_w = float(getattr(C, 'COST_BOX_WEIGHT', 1.0))
    cone_w = float(getattr(C, 'COST_CONE_WEIGHT', 1.0))
    exp_rate = float(getattr(C, 'COST_EXP_RATE', 1.0))
    costs, run_data = [], []
    for i in range(len(final_rate)):
        t_settle = float(settle[i]) if i < settle.size else 0.0
        if not np.isfinite(t_settle) or t_settle <= 0.0:
            t_settle = 0.0
        if failed[i] or not np.isfinite(final_rate[i]):
            costs.append(50.0)
            run_data.append({'fuel': float(fuel[i]), 'time': t_settle,
                             'box_km': float(box[i]), 'cone_deg': float(cone[i]),
                             'box_cost': 50.0, 'cone_cost': 0.0, 'cost': 50.0})
            continue
        box_cost = threshold_cost(box[i], box_limit, box_w, exp_rate)
        cone_cost = threshold_cost(cone[i], cone_limit, cone_w, exp_rate)
        c = float(final_rate[i]) + (0.0 if reached[i] else 10.0)
        c += box_cost + cone_cost
        if (not np.isfinite(c)) or c > 50.0:
            c = 50.0
        costs.append(c)
        run_data.append({'fuel': float(fuel[i]), 'time': t_settle,
                         'box_km': float(box[i]), 'cone_deg': float(cone[i]),
                         'box_cost': float(box_cost), 'cone_cost': float(cone_cost),
                         'cost': float(c)})
    return np.array(costs, dtype=float), run_data


def evaluate(theta, N=5, seed=123, return_timing=False):
    """Return (scalar cost, run_data) or (cost, run_data, timing) if requested."""
    bp, _ = build_batch_params(theta, disperse=True, timing=return_timing)
    res = sad_py.run_mission_batch(N, seed, bp)
    costs, run_data = reduce_costs(res)
    m = float(np.mean(costs))
    m = m if np.isfinite(m) else 50.0
    if return_timing:
        return m, run_data, res.get('timing')
    return m, run_data
