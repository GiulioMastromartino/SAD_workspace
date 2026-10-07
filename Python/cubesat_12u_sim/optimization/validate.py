"""Validation of the winning candidates on the real simulation.

The sweep optimises against a coarse RK4 rollout; anything reported has to come
back through the same path the project actually uses — heyoka 12-state dynamics,
``throttle_command`` into the 12-jet allocation — or the numbers are not the
shipped ones. This module re-runs the best theta through that path and reports
the delta against the search-time estimate.
"""
from __future__ import annotations

import numpy as np

from .. import config as C
from ..dynamics import pack_state
from ..nmpc import semi_major_axis


def _make_integrator(y0):
    from ..heyoka_propagate import make_integrator
    return make_integrator(y0, np.zeros(len(C.THRUSTER_TMAX)))


def _y0(v_frac: float = -0.005):
    euler = np.radians(np.array([5.0, 2.0, 0.0]))
    omega = np.radians(np.array([0.05, 0.02, -0.03]))
    return pack_state(C.R0_ECI.copy(), C.V0_ECI.copy() * (1.0 + v_frac), euler, omega)


def validate_pid(theta, space, *, t_end: float = 7200.0, dt_outer: float = 10.0,
                 dt_inner: float = 1.0) -> dict:
    """OrbitPID through thruster allocation at the deployment resolution."""
    from .objective import make_pid

    ctrl = make_pid(space.from_theta(theta))
    ta = _make_integrator(_y0())
    y = _y0()
    dv, max_err = 0.0, 0.0
    n_outer = int(round(t_end / dt_outer))
    sub = int(round(dt_outer / dt_inner))
    for _ in range(n_outer):
        for _ in range(sub):
            u, F, tau = ctrl.throttle_command(y[0:3], y[3:6], y[6:9], dt_inner)
            for i, val in enumerate(np.concatenate([F, tau])):
                ta.pars[i] = float(val)
            ta.propagate_for(dt_inner)
            y = np.array(ta.state)
            dv += float(np.linalg.norm(F) / C.MASS) * dt_inner
            max_err = max(max_err, abs(float(np.linalg.norm(y[0:3])) - C.R_ORBIT))
    return {"arm": "pid", "dv": dv, "max_abs_r_err": max_err,
            "final_e_a": semi_major_axis(y[0:3], y[3:6]) - C.R_ORBIT,
            "dt_outer": dt_outer, "dt_inner": dt_inner, "t_end": t_end}


def validate_nmpc(theta, space, horizon: int, *, t_end: float = 7200.0,
                  dt_outer: float = 10.0, dt_inner: float = 1.0) -> dict:
    """HierarchicalController through thruster allocation."""
    from ..hierarchical import HierarchicalController
    from .objective import make_nmpc
    from .objective import EvalConfig

    cfg = EvalConfig(dt_outer=dt_outer, dt_inner=dt_inner)
    ctrl = make_nmpc(space.from_theta(theta), cfg, horizon)
    ta = _make_integrator(_y0())
    y = _y0()
    dv, max_err = 0.0, 0.0
    n_outer = int(round(t_end / dt_outer))
    sub = int(round(dt_outer / dt_inner))
    for _ in range(n_outer):
        for _ in range(sub):
            u, F, tau = ctrl.throttle_command(y[0:3], y[3:6], y[6:9], dt_inner,
                                              y[9:12])
            for i, val in enumerate(np.concatenate([F, tau])):
                ta.pars[i] = float(val)
            ta.propagate_for(dt_inner)
            y = np.array(ta.state)
            dv += float(np.linalg.norm(F) / C.MASS) * dt_inner
            max_err = max(max_err, abs(float(np.linalg.norm(y[0:3])) - C.R_ORBIT))
    split = ctrl.fuel_split
    return {"arm": "nmpc", "dv": dv, "max_abs_r_err": max_err,
            "final_e_a": semi_major_axis(y[0:3], y[3:6]) - C.R_ORBIT,
            "trim_frac": split["trim_frac"], "plans": split["plans"],
            "fallbacks": split["fallbacks"], "horizon": horizon,
            "dt_outer": dt_outer, "dt_inner": dt_inner, "t_end": t_end}


def propellant_g(dv: float) -> float:
    """Monopropellant mass for a given dv, in grams (Rocketry equation)."""
    return float(C.MASS * (1.0 - np.exp(-dv / (C.ISP_JET * C.G0))) * 1e3)
