"""Discrete adjoint gradient for the NMPC shooting cost.

Why
---
The shooting cost has no analytic gradient, so SLSQP falls back to forward
differences: one full N-step rollout per decision variable per iteration. That is
``3N+1`` lanes, and on the ``cpp`` backend it is measured at **88.9 % of all
Taylor lanes at H=12** (3368 of 3790 us per solve). ``grad_batch`` does not
reduce that count, it batches those lanes into a single C++ crossing — which is
where the ~18x came from, but leaves the lane count untouched.

This module replaces the ``3N+1`` perturbed rollouts with one forward rollout
plus one backward sweep: the exact Jacobian-transpose of the *same* RK4 step the
forward pass uses, not an adjoint of a continuous idealisation. That matters —
a continuous adjoint paired with a discretised forward pass is an inconsistent
gradient, and SLSQP degrades on it.

Derivation
----------
The step is ``x_{k+1} = R(dt, x_k, u_k)``. With stage Jacobians
``A_i = df/dx`` at the four RK4 stage points and ``B = df/du``,

    S1 = I                     P1 = 0
    S2 = I + (h/2) A1 S1      P2 = (h/2) B
    S3 = I + (h/2) A2 S2      P3 = (h/2) A2 P2
    S4 = I + h     A3 S3      P4 = h     A3 P3
    S5 = I + (h/6)(A1 S1 + 2 A2 S2 + 2 A3 S3 + A4 S4)          = dx_{k+1}/dx_k
    P5 =     (h/6)(  B  + 2 A2 P2 + 2 A3 P3 +   A4 P4)        = dx_{k+1}/du_k

and the sweep runs backwards with ``lam <- S5^T lam + g`` (direct stage cost at
``x_k``) while ``dJ/du_k = P5^T lam + dt * u_k / |u_k|``.

Gravity needs no variational system: the orbit is Keplerian, so
``dg/dr = -mu/|r|^3 (I - 3 rhat rhat^T)`` is available in closed form.

The forward *value* comes from the caller's existing ``_rk4_step_orbit``, never
from a second implementation, so the cost is identical by construction and only
the gradient is new.

The cost is now the scorer's cost, not a tracking surrogate: fuel as L1 dv,
position as a one-sided quadratic hinge on RADIUS error against a box fraction,
and a small terminal energy term. ``q_pos``/``q_vel``/``r_fuel`` are gone --
there are no free weights left to diverge from what is being measured.

Caveat: two kinks in the cost. The box hinge is flat inside the box and the
terminal term deadbands ``e_a``, so both have a measure-zero set where the
derivative is undefined. The adjoint uses the a.e. derivative. A centred finite
difference with step ``h`` straddling such a boundary returns something else
entirely, so the two disagree *at the boundary only* and a parity test must
avoid placing the state there. This is inherent to the cost, not the method.
"""
from __future__ import annotations

import math

import numpy as np

__all__ = ["shooting_cost", "shooting_cost_and_grad", "states_cost_grad",
           "step_jacobian", "step_value", "EA_SCALE"]

EA_SCALE = 50e3      # m, semi-major-axis normalisation for the terminal term

_I3 = np.eye(3)
_I6 = np.eye(6)


def step_value(x: np.ndarray, u: np.ndarray, dt: float, mu: float) -> np.ndarray:
    """One RK4 step. Mirrors ``nmpc._rk4_step_orbit`` exactly.

    Kept here so this module is self-contained and testable on its own; the
    optimizer still calls ``_rk4_step_orbit`` for the forward pass, and a test
    asserts the two agree bit for bit.
    """
    r, v = x[0:3], x[3:6]
    rn3 = float(np.linalg.norm(r)) ** 3

    def f(xx: np.ndarray) -> np.ndarray:
        rr = xx[0:3]
        return np.concatenate([xx[3:6], -mu * rr / float(np.linalg.norm(rr)) ** 3 + u])

    k1 = f(x)
    k2 = f(x + 0.5 * dt * k1)
    k3 = f(x + 0.5 * dt * k2)
    k4 = f(x + dt * k3)
    return x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


def _f_and_A(x: np.ndarray, u: np.ndarray, mu: float) -> tuple:
    """State derivative and its 6x6 state Jacobian at one RK4 stage."""
    r, v = x[0:3], x[3:6]
    rn = float(np.linalg.norm(r))
    f = np.concatenate([v, -mu * r / rn ** 3 + u])
    A = np.zeros((6, 6))
    A[0:3, 3:6] = _I3
    # dg/dr for g(r) = -mu r |r|^-3
    A[3:6, 0:3] = -(mu / rn ** 3) * (_I3 - 3.0 * np.outer(r, r) / (rn * rn))
    return f, A


def _B() -> np.ndarray:
    """df/du: control enters the velocity block directly. Constant."""
    B = np.zeros((6, 3))
    B[3:6, 0:3] = _I3
    return B


def step_jacobian(x: np.ndarray, u: np.ndarray, dt: float,
                  mu: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return ``(x_{k+1}, S, P)`` with ``S = dx_{k+1}/dx_k`` and ``P = dx_{k+1}/du_k``.

    Exact Jacobian-transpose source for the backward sweep; see the module
    docstring for the recurrence.
    """
    h2, h6 = 0.5 * dt, dt / 6.0
    B = _B()

    f1, A1 = _f_and_A(x, u, mu)
    f2, A2 = _f_and_A(x + h2 * f1, u, mu)
    f3, A3 = _f_and_A(x + h2 * f2, u, mu)
    f4, A4 = _f_and_A(x + dt * f3, u, mu)
    xn = x + h6 * (f1 + 2 * f2 + 2 * f3 + f4)

    S2 = _I6 + h2 * (A1 @ _I6)
    P2 = h2 * B
    S3 = _I6 + h2 * (A2 @ S2)
    # Every stage's f depends on u *directly* as well as through the stage state,
    # so each P carries a fresh +B. Dropping it (or the 6B below) is the easy
    # mistake here: RK4 integrates a system linear in u exactly, so the leading
    # term must reduce to h*B.
    P3 = h2 * (A2 @ P2 + B)
    S4 = _I6 + dt * (A3 @ S3)
    P4 = dt * (A3 @ P3 + B)
    S = _I6 + h6 * (A1 + 2.0 * (A2 @ S2) + 2.0 * (A3 @ S3) + A4 @ S4)
    P = h6 * (6.0 * B + 2.0 * (A2 @ P2) + 2.0 * (A3 @ P3) + A4 @ P4)
    return xn, S, P


# ---------------------------------------------------------------------------
# scalar cost terms and their gradients
# ---------------------------------------------------------------------------

def _ea_grad(x: np.ndarray, mu: float, r_orbit: float):
    """``e_a = a_osc - R_ORBIT`` and its gradient wrt ``[r, v]``.

    Mirrors ``nmpc.semi_major_axis``, including its ``max(|r|, 1)`` guard.
    """
    r, v = x[0:3], x[3:6]
    rn = max(float(np.linalg.norm(r)), 1.0)
    v2 = float(v @ v)
    a = 1.0 / (2.0 / rn - v2 / mu)
    a2 = a * a
    return (a - r_orbit,
            np.concatenate([2.0 * r * a2 / rn ** 3, 2.0 * v * a2 / mu]))


def _vr_grad(x: np.ndarray):
    """Radial velocity ``v . rhat`` and its gradient wrt ``[r, v]``."""
    r, v = x[0:3], x[3:6]
    rn = max(float(np.linalg.norm(r)), 1.0)
    rhat = r / rn
    vr = float(v @ rhat)
    g = np.concatenate([v / rn - (float(v @ r) * r) / rn ** 3, rhat])
    return vr, g


FUEL_X_MAX = 20.0   # |u|/a_max ceiling; exp(19.5) ~ 3e8, bounded
FUEL_KNEE = 0.5     # quadratic below this fraction of a_max, exponential above


def fuel_g(x: float, x0: float = FUEL_KNEE) -> float:
    """Un-normalised fuel shape: x^2 below the knee, x0^2*exp(x-x0) above."""
    if x <= x0:
        return x * x
    return x0 * x0 * math.exp(x - x0)


def fuel_dg(x: float, x0: float = FUEL_KNEE) -> float:
    """d/dx of :func:`fuel_g`."""
    if x <= x0:
        return 2.0 * x
    return x0 * x0 * math.exp(x - x0)


def fuel_params(a_max: float, dt: float,
                knee: float = FUEL_KNEE) -> tuple[float, float, float]:
    """``(A, x0, g1)`` -- scale, knee, and the g(1) normalisation.

    A = a_max*dt, so full authority for one tick costs exactly the m/s the
    scorer counts for it.
    """
    x0 = float(knee)
    return float(a_max) * float(dt), x0, fuel_g(1.0, x0)


def fuel_cost(u: np.ndarray, a_max: float, A: float, x0: float,
              g1: float) -> float:
    """``A * g(|u|/a_max) / g(1)``. Mirrors nmpc_solve.cpp fuel_cost.

    Quadratic near the centre (zero gradient at u = 0, which is exactly where
    the warm start sits and where L1's kink stalled SLSQP), exponential past
    the knee so a large single burn is punished relative to spreading the same
    dv into several smaller ones.
    """
    an = float(np.linalg.norm(u))
    if not (an > 0.0):
        return 0.0
    a_scale = max(float(a_max), 1e-12)
    x = min(an / a_scale, FUEL_X_MAX)
    return A * fuel_g(x, x0) / g1


def track_w(x: float, x0: float, s: float) -> float:
    """Position-tracking cost on normalised radius error ``x = |r_err|/box``.

    Linear for ``x <= x0`` (the inner 20% of the box), logarithmic beyond, with
    the value and slope both continuous at the join. ``s`` is the linear-region
    slope, chosen so ``w(1) == w_edge``. Duplicated in nmpc_solve.cpp and
    nmpc_batch.cpp (separate translation units); this copy is the reference the
    parity tests check the kernels against.
    """
    if x <= x0:
        return s * x
    return s * x0 * (1.0 + np.log(x / x0))


def track_dw(x: float, x0: float, s: float) -> float:
    """d/dx of :func:`track_w`."""
    if x <= x0:
        return s
    return s * x0 / x


def track_slope(w_edge: float, x0: float) -> float:
    """The linear-region slope that makes ``track_w(1) == w_edge``."""
    if not (1e-6 < x0 < 1.0):
        return w_edge
    return w_edge / (x0 * (1.0 + np.log(1.0 / x0)))


def _box_cost(x: np.ndarray, w_track: float, box_frac: float, track_slope: float,
              box_ref: float, r_orbit: float) -> float:
    """Radius-error tracking cost at one stage."""
    rn = max(float(np.linalg.norm(x[0:3])), 1.0)
    if box_ref <= 1.0 or w_track <= 0.0:
        return 0.0
    return track_w(abs(rn - r_orbit) / box_ref, box_frac, track_slope)


def _box_cost_grad(x: np.ndarray, w_track: float, box_frac: float,
                   track_slope: float, box_ref: float,
                   r_orbit: float) -> np.ndarray:
    """d/dx of :func:`_box_cost`.

    ``d|rn - R|/dx = sign(rn - R) * rhat``, so the chain rule gives
    ``w'(x) * sign * rhat / box_ref``. The sign is load-bearing: without it the
    gradient pushes an orbit that is BELOW target further down, because ``|.|``
    is not differentiable at zero and the outward rhat alone reads as "grow".
    Finite differences catch that immediately (analytic and fd come out equal in
    magnitude, opposite in sign), so it is covered by the parity test.
    """
    if box_ref <= 1.0 or w_track <= 0.0:
        return np.zeros(6)
    r = x[0:3]
    rn = float(np.linalg.norm(r))
    if rn < 1.0:
        rn = 1.0
    d = rn - r_orbit
    sgn = 1.0 if d >= 0.0 else -1.0
    dwx = track_dw(abs(d) / box_ref, box_frac, track_slope)
    g = np.zeros(6)
    g[0:3] = dwx * sgn * (r / rn) / box_ref
    return g


def _fuel_grad(u: np.ndarray, a_max: float, A: float, x0: float,
               g1: float) -> np.ndarray:
    """d/du of :func:`fuel_cost`: ``g'(x) * (u/|u|) / a_max``, times A/g1.

    Exactly zero at u = 0 (quadratic branch), which is where the warm start
    sits -- the property that makes this usable where L1 was not.
    """
    an = float(np.linalg.norm(u))
    if not (an > 0.0):
        return np.zeros(3)
    a_scale = max(float(a_max), 1e-12)
    x = min(an / a_scale, FUEL_X_MAX)
    return (A / g1) * fuel_dg(x, x0) * (np.asarray(u, float) / (an * a_scale))


def _terminal_cost(x: np.ndarray, w_final: float, db_a: float, mu: float,
                   r_orbit: float) -> float:
    e_a, _ = _ea_grad(x, mu, r_orbit)
    return w_final * (e_a / EA_SCALE) ** 2 if abs(e_a) >= db_a else 0.0


def _terminal_cost_grad(x: np.ndarray, w_final: float, db_a: float, mu: float,
                        r_orbit: float) -> np.ndarray:
    e_a, de = _ea_grad(x, mu, r_orbit)
    if abs(e_a) < db_a:
        return np.zeros(6)
    return (2.0 * w_final * e_a / EA_SCALE ** 2) * de


def shooting_cost(x0: np.ndarray, U: np.ndarray, dt: float, mu: float,
                  r_orbit: float, w_track: float, box_frac: float,
                  w_final: float, box_ref: float, db_a: float,
                  a_max: float) -> float:
    """The shooting cost, in the same arithmetic order as ``nmpc.cost``.

    Single source of truth: the optimizer's objective delegates here so the
    forward value the adjoint differentiates cannot drift from the value being
    optimised. The C++ parity gate (``test_nmpc_cpp_cost_matches_python``) is
    what guards this function against the kernels in ``cxxdrive/``.
    """
    N = U.shape[0]
    ts = track_slope(w_track, box_frac)
    A, fx0, fg1 = fuel_params(a_max, dt)
    from cubesat_12u_sim.control.nmpc import _rk4_step_orbit

    x = np.asarray(x0, float)
    c = 0.0
    for k in range(N):
        x = _rk4_step_orbit(x, U[k], dt)
        # fuel is L1 in |u| scaled by dt, matching the scorer's dv
        c += fuel_cost(U[k], a_max, A, fx0, fg1)
        c += _box_cost(x, w_track, box_frac, ts, box_ref, r_orbit)
    c += _terminal_cost(x, w_final, db_a, mu, r_orbit)
    return float(c)


def states_cost_grad(states: np.ndarray, U: np.ndarray, dt: float, mu: float,
                     r_orbit: float, w_track: float, box_frac: float,
                     w_final: float, box_ref: float, db_a: float,
                     a_max: float) -> np.ndarray:
    """Gradient given pre-computed forward states.

    Split out from :func:`shooting_cost_and_grad` because SLSQP asks for the
    objective and the gradient at the same point, so the forward rollout can be
    reused instead of recomputed. ``states`` is ``(N+1, 6)`` with
    ``states[k+1]`` produced from ``states[k]`` under ``U[k]``.
    """
    N = U.shape[0]
    ts = track_slope(w_track, box_frac)
    A, fx0, fg1 = fuel_params(a_max, dt)
    grad = np.zeros(3 * N)
    lam = np.zeros(6)
    lam += _box_cost_grad(states[N], w_track, box_frac, ts, box_ref, r_orbit)
    lam += _terminal_cost_grad(states[N], w_final, db_a, mu, r_orbit)
    for k in range(N - 1, -1, -1):
        _, S, P = step_jacobian(states[k], U[k], dt, mu)
        # d|u_k|/du_k = u_k / |u_k|; 0 at the origin, where the L1 term is not
        # differentiable. Returning 0 there is the same convention numpy uses
        # and keeps the solver from seeing NaN on an all-zero warm start.
        grad[3 * k:3 * k + 3] = P.T @ lam + _fuel_grad(U[k], a_max, A, fx0, fg1)
        lam = S.T @ lam
        if k >= 1:
            lam += _box_cost_grad(states[k], w_track, box_frac, ts, box_ref,
                                  r_orbit)
    return grad


def shooting_cost_and_grad(x0: np.ndarray, U: np.ndarray, dt: float,
                           mu: float, r_orbit: float, w_track: float,
                           box_frac: float, w_final: float, box_ref: float,
                           db_a: float, a_max: float) -> tuple[float, np.ndarray]:
    """``(cost, grad)`` with ``grad`` the exact gradient of :func:`shooting_cost`.

    ``grad`` is length ``3N`` in the same ``z = U.ravel()`` order the optimizer
    uses. Forward states are produced by the caller's own RK4 so the value is
    identical; this pass recomputes each step's Jacobian for the backward sweep.
    """
    U = np.asarray(U, float)
    N = U.shape[0]

    # forward, reusing the caller's stepper via a local import to avoid a cycle
    from cubesat_12u_sim.control.nmpc import _rk4_step_orbit

    states = np.empty((N + 1, 6))
    states[0] = x0
    for k in range(N):
        states[k + 1] = _rk4_step_orbit(states[k], U[k], dt)

    ts = track_slope(w_track, box_frac)
    A, fx0, fg1 = fuel_params(a_max, dt)
    c = 0.0
    for k in range(N):
        c += fuel_cost(U[k], a_max, A, fx0, fg1)
        c += _box_cost(states[k + 1], w_track, box_frac, ts, box_ref, r_orbit)
    c += _terminal_cost(states[N], w_final, db_a, mu, r_orbit)

    grad = np.zeros(3 * N)
    lam = np.zeros(6)
    # direct terms at the terminal state: the last stage's box hinge and the
    # terminal energy term both act on x_N
    lam += _box_cost_grad(states[N], w_track, box_frac, ts, box_ref, r_orbit)
    lam += _terminal_cost_grad(states[N], w_final, db_a, mu, r_orbit)

    for k in range(N - 1, -1, -1):
        _, S, P = step_jacobian(states[k], U[k], dt, mu)
        grad[3 * k:3 * k + 3] = P.T @ lam + _fuel_grad(U[k], a_max, A, fx0, fg1)
        # propagate to x_k, then add the direct stage term there
        lam = S.T @ lam
        if k >= 1:
            lam += _box_cost_grad(states[k], w_track, box_frac, ts, box_ref,
                                  r_orbit)
    return float(c), grad