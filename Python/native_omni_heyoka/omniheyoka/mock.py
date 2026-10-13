"""Pure-Python stand-in for the compiled ``omni_heyoka.OmniSystem``.

**This is not the real thing.**  It exists so the ``OmniTaylorAdaptive`` shim
and its validation can be developed, exercised and regression-tested *before*
WS1's C++ extension is built -- and after, as a cross-check that the shim
itself adds nothing.  It implements the ``OmniSystem`` contract from
``native_omni_heyoka/DESIGN.md`` in pure Python by **delegating** the symbolic
EOM to :func:`cubesat_12u_sim.backends.heyoka.heyoka_rw.build_rw_system` (the
independent hand-written reference) and evaluating them numerically with
:func:`heyoka.cfunc`.

Because it delegates rather than recomputes, the mock is deliberately limited to
the configuration that ``build_rw_system()`` encodes (750 kg hub with
``diag(900, 800, 600)``, three body-axis-aligned HR16 wheels, Earth point-mass
gravity).  Asking for anything else raises :class:`NotImplementedError` instead
of silently returning the wrong equations.  Mass, inertia, wheel inertias and
motor torques *are* runtime parameters in that reference, so they stay free.
(WS2 absolute-h fence, WP25-C3: the delegated reference encodes absolute-h,
exact for this equal-Js orthonormal set only — the `NotImplementedError`
above is the enforcement; see DESIGN.md constraint (1).)

Known, documented differences from the compiled ``OmniSystem``:

* no saturation clamp on ``u`` (the hand-written reference, like
  ``heyoka_rw.rw_allocate``, expects the caller to clip to ``±u_max``), so the
  mock's symbolic and numeric paths always agree with each other;
* ``matrixD`` / back-substitution bookkeeping does not exist here -- the
  balanced-rotor constraint and the MRP->DCM formula are native-side concerns.
"""

from __future__ import annotations

from typing import Any, Iterable, Sequence

import numpy as np

from cubesat_12u_sim.backends.heyoka import heyoka_rw as hrw
from cubesat_12u_sim.backends.heyoka.heyoka_attitude import inertia_pars
from cubesat_12u_sim.core import config as C

from .layout import par_index

__all__ = ["MockOmniSystem", "mock_rw_demo_system", "MOCK_SUPPORTED_NOTE"]

MOCK_SUPPORTED_NOTE = (
    "the pure-Python mock backend delegates to heyoka_rw.build_rw_system(), so "
    "it only supports its configuration: 3 body-axis-aligned wheels on a hub at "
    "the centre of mass, Earth point-mass gravity, mass/inertia/Js as runtime "
    "parameters."
)


def _index_of_axis(gs: Sequence[float]) -> int | None:
    """Return the body axis a wheel axis is aligned with (exactly one ±1)."""
    v = np.asarray(gs, dtype=float).reshape(-1)
    if v.shape != (3,):
        return None
    nonzero = np.flatnonzero(v)
    if len(nonzero) != 1:
        return None
    return int(nonzero[0])


class MockOmniSystem:
    """Duck-typed ``OmniSystem`` backed by ``heyoka_rw.build_rw_system()``."""

    def __init__(self) -> None:
        self._hub_mass: float | None = None
        self._inertia = np.zeros((3, 3))
        self._r_BcB_B = np.zeros(3)
        self._wheels: list[tuple[int, float, float]] = []  # (axis, Js, Jt)
        self._mu: float | None = None
        self._u_max: float | None = None
        self._sys: Any = None
        self._cfunc: Any = None

    # -- configuration -----------------------------------------------------
    def add_hub(self, mass: float, inertia: Sequence[float],
                r_BcB_B: Sequence[float] = (0.0, 0.0, 0.0)) -> None:
        r = np.asarray(r_BcB_B, dtype=float).reshape(-1)
        if r.shape != (3,) or np.any(r != 0.0):
            raise NotImplementedError(
                "mock backend: r_BcB_B must be zero (the hand-written reference "
                f"models the hub at its centre of mass), got {r.tolist()}"
            )
        self._hub_mass = float(mass)
        self._inertia = np.asarray(inertia, dtype=float).reshape(3, 3)

    def add_reaction_wheel(self, gs: Sequence[float], js: float, jt: float) -> None:
        axis = _index_of_axis(gs)
        if axis is None:
            raise NotImplementedError(
                "mock backend: only body-axis-aligned wheels are supported "
                f"(gs={np.asarray(gs).tolist()}); use the compiled OmniSystem "
                "for tilted spin axes."
            )
        self._wheels.append((axis, float(js), float(jt)))

    def set_gravity(self, mu: float) -> None:
        self._mu = float(mu)

    def set_u_max(self, u_max: float) -> None:
        self._u_max = float(u_max)

    # -- introspection -----------------------------------------------------
    @property
    def n_wheels(self) -> int:
        return len(self._wheels)

    def state_names(self) -> list[str]:
        return list(hrw.STATE_NAMES)

    def par_names(self) -> list[str]:
        """Ordered parameter names mirroring the runtime-composed layout (A2).

        Inputs first (interleaved ``u{i}``/``tauF{i}`` per wheel, WP2), then
        hub ``mass`` + ``I6``, then ``Js{i}``. No ``invI6`` slots.
        Historical note: the hand-written reference used a fixed 19-vector
        with ``invI6``; the derived layout is 16 for 3 wheels.
        """
        names: list[str] = []
        for i in range(1, len(self._wheels) + 1):
            names.extend([f"u{i}", f"tauF{i}"])
        names.extend(["mass", "Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz"])
        for i in range(1, len(self._wheels) + 1):
            names.append(f"Js{i}")
        return names

    def input_names(self) -> list[str]:
        """Ordered input-slot names (leading block of :meth:`par_names`)."""
        names: list[str] = []
        for i in range(1, len(self._wheels) + 1):
            names.extend([f"u{i}", f"tauF{i}"])
        return names

    def n_states(self) -> int:
        return len(self.state_names())

    def n_pars(self) -> int:
        return len(self.par_names())

    def default_state(self) -> list[float]:
        return mock_rw_state()

    def default_pars(self) -> list[float]:
        return mock_rw_pars()

    # -- validation --------------------------------------------------------
    def _check_supported(self) -> None:
        if self._hub_mass is None:
            raise RuntimeError("mock backend: add_hub() before building a system")
        if len(self._wheels) != 3:
            raise NotImplementedError(
                f"mock backend: heyoka_rw.build_rw_system() encodes exactly 3 "
                f"wheels, {len(self._wheels)} requested. {MOCK_SUPPORTED_NOTE}"
            )
        if sorted(a for a, _, _ in self._wheels) != [0, 1, 2]:
            raise NotImplementedError(
                "mock backend: wheel axes must be the three distinct body axes "
                f"(got {[a for a, _, _ in self._wheels]}). {MOCK_SUPPORTED_NOTE}"
            )
        if self._mu is not None and self._mu != float(C.MU_EARTH):
            raise NotImplementedError(
                f"mock backend: mu={self._mu!r} is baked into build_rw_system() "
                f"as C.MU_EARTH={float(C.MU_EARTH)!r}; a different mu needs the "
                "compiled OmniSystem."
            )

    # -- the contract ------------------------------------------------------
    def build_symbolic(self, state_names: Sequence[str] | None = None) -> Any:
        """Return ``heyoka_rw.build_rw_system()`` (the hand-written reference)."""
        self._check_supported()
        if state_names is not None and tuple(state_names) != tuple(hrw.STATE_NAMES):
            raise NotImplementedError(
                f"mock backend: variable names are fixed by build_rw_system() "
                f"({hrw.STATE_NAMES}); got {tuple(state_names)}."
            )
        self._sys = hrw.build_rw_system()
        return list(self._sys)

    def _symbolic(self) -> Any:
        sys_ = self._sys
        if sys_ is None:
            sys_ = self.build_symbolic()
        return sys_

    def _evaluator(self) -> Any:
        import heyoka as hy

        fn = self._cfunc
        if fn is None:
            sys_ = self._symbolic()
            fn = hy.cfunc([rhs for _, rhs in sys_], [var for var, _ in sys_])
            self._cfunc = fn
        return fn

    def accelerations(self, state: Sequence[float],
                      pars: Sequence[float]) -> list[float]:
        """Numeric accelerations of the hand-written system at ``(state, pars)``.

        Layout per DESIGN.md: ``[r_ddot(3), omega_dot(3), wheel_dot(n_wheels)]``
        -- the kinematic blocks (``r_dot``, ``v_dot``, ``sigma_dot``) are not
        returned.
        """
        self._check_supported()
        st = np.asarray(state, dtype=float).reshape(-1)
        ps = np.asarray(pars, dtype=float).reshape(-1)
        if st.size != self.n_states():
            raise ValueError(f"state has {st.size} components, expected {self.n_states()}")
        if ps.size != self.n_pars():
            raise ValueError(f"pars has {ps.size} components, expected {self.n_pars()}")
        names = self.par_names()
        n = len(self._wheels)
        # Friction acts like extra motor torque on the reference (same sign
        # as u, WP2 default react-on-hub); inverse rebuilt from I6 since the
        # reference multiplies by precomputed invI.
        u_eff = [float(ps[par_index(f"u{i}", names=names)])
                 + float(ps[par_index(f"tauF{i}", names=names)])
                 for i in range(1, n + 1)]
        mass = float(ps[par_index("mass", names=names)])
        ixx = float(ps[par_index("Ixx", names=names)])
        iyy = float(ps[par_index("Iyy", names=names)])
        izz = float(ps[par_index("Izz", names=names)])
        ixy = float(ps[par_index("Ixy", names=names)])
        ixz = float(ps[par_index("Ixz", names=names)])
        iyz = float(ps[par_index("Iyz", names=names)])
        inertia_mat = np.array([[ixx, ixy, ixz],
                                [ixy, iyy, iyz],
                                [ixz, iyz, izz]], dtype=float)
        js_v = [float(ps[par_index(f"Js{i}", names=names)]) for i in range(1, n + 1)]
        ref_ps = hrw.pack_rw_pars(np.asarray(u_eff, dtype=float), mass, inertia_mat,
                                  np.asarray(js_v, dtype=float))
        out = np.asarray(self._evaluator()(inputs=st, pars=ref_ps), dtype=float).reshape(-1)
        n = len(self._wheels)
        return ([float(v) for v in out[3:6]]
                + [float(v) for v in out[9:12]]
                + [float(v) for v in out[12:12 + n]])

    # -- convenience -------------------------------------------------------
    def inv_inertia(self) -> np.ndarray:
        return np.asarray(np.linalg.inv(self._inertia), dtype=float)


# ---------------------------------------------------------------------------
# ready-made mock instances / vectors
# ---------------------------------------------------------------------------
def mock_rw_demo_system(hub_mass: float | None = None,
                        inertia: np.ndarray | None = None,
                        wheels=None,
                        mu: float | None = None,
                        u_max: float | None = None) -> MockOmniSystem:
    """``MockOmniSystem`` configured exactly like :func:`presets.rw_demo_system`.

    Defaults are imported from :mod:`omniheyoka.presets` so the two can never
    drift apart; the hub/inertia/wheel values are validated by the mock exactly
    as for the native backend.
    """
    from .presets import (
        RW_DEMO_WHEELS,
        RW_INERTIA,
        RW_MASS,
        RW_MU,
        RW_U_MAX,
        wheel_specs,
    )

    sys_ = MockOmniSystem()
    sys_.add_hub(RW_MASS if hub_mass is None else hub_mass,
                 RW_INERTIA if inertia is None else inertia)
    wheel_list = RW_DEMO_WHEELS if wheels is None else wheels
    for spec in wheel_specs(wheel_list):
        sys_.add_reaction_wheel(spec[0], spec[1], spec[2])
    sys_.set_gravity(RW_MU if mu is None else mu)
    sys_.set_u_max(RW_U_MAX if u_max is None else u_max)
    return sys_


def mock_rw_state(r=None, v=None, sigma=None, omega=None, wheel_speeds=None) -> np.ndarray:
    """A representative 15-state for the RW demo (circular 12000 km orbit)."""
    r = C.R0_ECI if r is None else np.asarray(r, dtype=float)
    v = C.V0_ECI if v is None else np.asarray(v, dtype=float)
    sigma = np.zeros(3) if sigma is None else np.asarray(sigma, dtype=float)
    omega = C.OMEGA0_BODY if omega is None else np.asarray(omega, dtype=float)
    wheel_speeds = (np.radians(hrw.RW_OMEGA0_RPM) if wheel_speeds is None
                    else np.asarray(wheel_speeds, dtype=float))
    return hrw.pack_rw_state(r, v, sigma, omega, wheel_speeds)


def mock_rw_pars(u_motor=None, mass: float = 750.0,
                 inertia: np.ndarray | None = None,
                 js: np.ndarray | None = None,
                 tauF=None, system=None, par_names=None) -> list[float]:
    """Parameter vector for the RW demo, resolved BY NAME (A2).

    Historical note: the hand-written reference used a fixed 19-vector with
    ``invI6``; the derived runtime-composed layout is ``[u1, tauF1, ... ,
    mass, I6, Js...]`` (16 for 3 wheels). Defaults unchanged; only indexing
    is name-based. When ``system`` exposes ``par_names()`` it is the source
    of truth.
    """
    u_v = np.zeros(3) if u_motor is None else np.asarray(u_motor, dtype=float).reshape(-1)
    I = np.diag([900.0, 800.0, 600.0]) if inertia is None else np.asarray(inertia, dtype=float)
    I6, _invI6 = inertia_pars(I)
    js_v = np.full(3, float(hrw.RW_JS)) if js is None else np.asarray(js, dtype=float).reshape(-1)
    t_v = np.zeros_like(js_v, dtype=float) if tauF is None else np.asarray(tauF, dtype=float).reshape(-1)
    if u_v.size != js_v.size or t_v.size != js_v.size:
        raise ValueError(f"u/tauF/js length mismatch: {u_v.size}/{t_v.size}/{js_v.size}")
    values: dict[str, float] = {"mass": float(mass)}
    for key, val in zip(("Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz"), I6):
        values[key] = float(val)
    for i, (uu, tt, jj) in enumerate(zip(u_v, t_v, js_v), start=1):
        values[f"u{i}"] = float(uu)
        values[f"tauF{i}"] = float(tt)
        values[f"Js{i}"] = float(jj)
    if system is not None:
        from .layout import par_layout

        layout = par_layout(system)
        out = [0.0] * len(layout)
        for key, val in values.items():
            if key in layout:
                out[layout.index(key)] = float(val)
        return out
    names: tuple[str, ...] = tuple(par_names) if par_names is not None else tuple(
        [nm for i in range(1, len(js_v) + 1) for nm in (f"u{i}", f"tauF{i}")]
        + ["mass", "Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz"]
        + [f"Js{i}" for i in range(1, len(js_v) + 1)])
    out2 = [0.0] * len(names)
    for key, val in values.items():
        try:
            out2[par_index(key, names=names)] = float(val)
        except KeyError:
            continue
    return out2


def inertia_from_mass_props(inertia: np.ndarray) -> tuple[list[float], list[float]]:
    """Re-export of ``heyoka_attitude.inertia_pars`` for the mock's consumers."""
    return inertia_pars(np.asarray(inertia, dtype=float))
