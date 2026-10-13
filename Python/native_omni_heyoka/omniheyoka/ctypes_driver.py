"""ctypes driver for the C++ Taylor propagator (omni_taylor_api).

Replaces the abandoned string bridge: heyoka's expression string form truncates
at ~1000 chars, which silently corrupts the symbolic system for configurations
whose RHS exceeds that (e.g. tilted wheels: omx = 1009 chars).  Driving the
propagator in C++ removes strings, parsing and truncation entirely.
"""
from __future__ import annotations

import ctypes
import os
from typing import Sequence

_LIB_PATH = os.environ.get(
    "OMNI_TAYLOR_LIB",
    "/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python/env/lib/libomni_taylor.dylib",
)

_lib = ctypes.CDLL(_LIB_PATH)
_lib.omni_last_error.restype = ctypes.c_char_p

# --- explicit signatures (ctypes guesses wrongly for pointer/double mixes) ---
_DP = ctypes.POINTER(ctypes.c_double)
_lib.omni_taylor_create.restype = ctypes.c_void_p
_lib.omni_taylor_create.argtypes = [_DP, ctypes.c_double, _DP, ctypes.c_int,
                                    _DP, ctypes.c_double, ctypes.c_int,
                                    ctypes.c_double, ctypes.c_int]
_lib.omni_taylor_destroy.argtypes = [ctypes.c_void_p]
_lib.omni_taylor_n_states.restype = ctypes.c_int
_lib.omni_taylor_n_states.argtypes = [ctypes.c_void_p]
_lib.omni_taylor_n_pars.restype = ctypes.c_int
_lib.omni_taylor_n_pars.argtypes = [ctypes.c_void_p]
_lib.omni_taylor_state.restype = _DP
_lib.omni_taylor_state.argtypes = [ctypes.c_void_p]
_lib.omni_taylor_pars.restype = _DP
_lib.omni_taylor_pars.argtypes = [ctypes.c_void_p]
_lib.omni_taylor_step.restype = ctypes.c_double
_lib.omni_taylor_step.argtypes = [ctypes.c_void_p, ctypes.c_double]
_lib.omni_last_error.restype = ctypes.c_char_p


class OmniTaylor:
    def __init__(self, inertia9: Sequence[float], mass: float,
                 axes: Sequence[Sequence[float]], Js: Sequence[float],
                 mu: float, use_gg: bool = False, tol: float = 1e-13,
                 compact: bool = False, use_rpm: bool = True,
                 wheel_speeds_rpm=None, r0=None, v0=None, sigma0=None,
                 omega0=None):
        I9 = (ctypes.c_double * 9)(*[float(v) for v in inertia9])
        ax = (ctypes.c_double * 9)()
        for i, a in enumerate(axes):
            ax[3 * i] = float(a[0]); ax[3 * i + 1] = float(a[1]); ax[3 * i + 2] = float(a[2])
        js = (ctypes.c_double * len(Js))(*[float(v) for v in Js])

        self._h = ctypes.c_void_p(_lib.omni_taylor_create(
            I9, float(mass), ax, len(axes), js, float(mu), int(bool(use_gg)),
            float(tol), int(bool(compact))))
        if not self._h:
            raise RuntimeError("omni_taylor_create failed: " +
                               (_lib.omni_last_error() or b"").decode())
        self.n_states = _lib.omni_taylor_n_states(self._h)
        self.n_pars = _lib.omni_taylor_n_pars(self._h)

        self.state = (ctypes.c_double * self.n_states)()
        self.pars = (ctypes.c_double * self.n_pars)()
        ctypes.memmove(self.state, _lib.omni_taylor_state(self._h),
                       self.n_states * 8)
        ctypes.memmove(self.pars, _lib.omni_taylor_pars(self._h), self.n_pars * 8)
        # wheel speeds and ICs are set by the caller through these arrays.
        # LAYOUT FENCE (WP25-C3): numeric hub-block offsets below are the legacy
        # RW-only ABI (hub block fixed 12 by A1, wheel i in registration order).
        # Revive via r_slice()/v_slice()/sigma_slice()/omega_slice()/wheel_slice()
        # from omniheyoka.layout — do NOT copy these literals into new code.
        if use_rpm and wheel_speeds_rpm is not None:
            for i, w in enumerate(wheel_speeds_rpm):
                self.state[12 + i] = float(w) * 2.0 * 3.141592653589793 / 60.0  # LAYOUT-OK: legacy RW-only ABI, hub block fixed 12 by A1 (see fence above)
        if r0 is not None:
            self.state[0:3] = [float(v) for v in r0]  # LAYOUT-OK: legacy RW-only ABI, hub r-block 0:3 by A1 (see fence above)
        if v0 is not None:
            self.state[3:6] = [float(v) for v in v0]  # LAYOUT-OK: legacy RW-only ABI, hub v-block 3:6 by A1 (see fence above)
        if sigma0 is not None:
            self.state[6:9] = [float(v) for v in sigma0]  # LAYOUT-OK: legacy RW-only ABI, hub sigma-block 6:9 by A1 (see fence above)
        if omega0 is not None:
            self.state[9:12] = [float(v) for v in omega0]  # LAYOUT-OK: legacy RW-only ABI, hub omega-block 9:12 by A1 (see fence above)

    def __del__(self):
        if getattr(self, "_h", None):
            _lib.omni_taylor_destroy(self._h)
            self._h = None

    def step(self, dt: float) -> float:
        ctypes.memmove(_lib.omni_taylor_state(self._h), self.state,
                       self.n_states * 8)
        ctypes.memmove(_lib.omni_taylor_pars(self._h), self.pars, self.n_pars * 8)
        r = _lib.omni_taylor_step(self._h, float(dt))
        if r < 0:
            raise RuntimeError("omni_taylor_step failed: " +
                               (_lib.omni_last_error() or b"").decode())
        ctypes.memmove(self.state, _lib.omni_taylor_state(self._h),
                       self.n_states * 8)
        return r

    def state_np(self):
        import numpy as np
        return np.ctypeslib.as_array(self.state).copy()

    def set_pars(self, values: Sequence[float]):
        for i, v in enumerate(values):
            self.pars[i] = float(v)
