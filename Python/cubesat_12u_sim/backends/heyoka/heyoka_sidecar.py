"""Phase 4: Basilisk sidecar shadow propagator (live Heyoka parity check).

``HeyokaShadowPropagator`` is a Basilisk ``SysModel`` that flies alongside
truth in the same task: each tick it reads ``SCStatesMsg`` + thruster
on-time, converts on-time to a body wrench with the nominal geometry, and
steps an internal Heyoka MRP integrator (Phase 1 system) by one task step.
Divergence is logged internally and available via :meth:`summary` after the
run. No outgoing message (analysis only) -- truth dynamics are untouched.
"""
from __future__ import annotations

import numpy as np


class HeyokaShadowPropagator:
    """Factory holding a Basilisk SysModel (``.model``) plus divergence logs."""

    def __init__(self, task_step: float):
        from Basilisk.architecture import messaging, sysModel

        from cubesat_12u_sim.core import config as C
        from cubesat_12u_sim.backends.heyoka.heyoka_attitude import (
            make_mrp_integrator,
            mrp_diff,
            pack_mrp_state,
            pack_pars,
        )
        from cubesat_12u_sim.core.actuators import jets_to_wrench

        outer = self
        outer._C = C
        outer._dt = float(task_step)
        outer._ta = None
        outer.times: list[float] = []
        outer.pos_err: list[float] = []
        outer.att_err_deg: list[float] = []

        class _Shadow(sysModel.SysModel):
            def __init__(self):
                super().__init__()
                self.scStateInMsg = messaging.SCStatesMsgReader()
                # Torque demand from Basilisk's own mrpFeedback (always valid
                # once written); the sidecar mirrors the allocator in Python
                # instead of depending on the custom on-time message timing.
                self.cmdTorqueInMsg = messaging.CmdTorqueBodyMsgReader()

            def Reset(self, CurrentSimNanos):
                outer._ta = None
                outer._prev = None
                outer._warned = False
                outer._tmag: list[float] = []
                outer._om_err: list[float] = []
                outer.times.clear()
                outer.pos_err.clear()
                outer.att_err_deg.clear()

            def _warn(self, where, exc):
                if not outer._warned:
                    print(f"[heyokaShadow] {where}: {type(exc).__name__}: {exc}")
                    outer._warned = True

            def UpdateState(self, CurrentSimNanos):
                from cubesat_12u_sim.core.actuators import allocation_pinv, jets_wrench_quantized

                t = CurrentSimNanos * 1e-9
                try:
                    state = self.scStateInMsg()
                    r = np.asarray(state.r_CN_N, float)
                    v = np.asarray(state.v_CN_N, float)
                    sg = np.asarray(state.sigma_BN, float)
                    om = np.asarray(state.omega_BN_B, float)
                except Exception as exc:
                    self._warn("scState read", exc)
                    return
                try:
                    tq = np.asarray(self.cmdTorqueInMsg().torqueRequestBody,
                                    float)
                    u = allocation_pinv(
                        np.zeros(3),
                        float(outer._C.THRUSTER_CONTROL_SCALE) * tq)
                    F_b, T_b = jets_wrench_quantized(
                        np.clip(u, 0.0, 1.0), outer._C.THRUSTER_POS,
                        outer._C.THRUSTER_DIR, outer._C.THRUSTER_TMAX)
                except Exception as exc:
                    self._warn("torque read", exc)
                    F_b, T_b = np.zeros(3), np.zeros(3)
                if outer._ta is None:
                    outer._ta = make_mrp_integrator(
                        pack_mrp_state(r, v, sg, om))
                    # Truth dynamics at tick k consumed the cmd written at
                    # k-1 (mailbox semantics); hold current cmd for next step.
                    outer._prev = (F_b, T_b)
                    return
                F_prev, T_prev = outer._prev
                outer._tmag.append(float(np.linalg.norm(T_prev)))
                pars = pack_pars(F_prev, T_prev, float(outer._C.MASS),
                                 outer._C.INERTIA)
                for i, val in enumerate(pars):
                    outer._ta.pars[i] = float(val)
                outer._ta.propagate_for(outer._dt)
                outer._prev = (F_b, T_b)
                st = np.asarray(outer._ta.state, float)
                outer.times.append(t)
                outer.pos_err.append(float(np.linalg.norm(st[0:3] - r)))
                outer.att_err_deg.append(float(np.degrees(
                    4.0 * np.arctan(np.linalg.norm(mrp_diff(st[6:9], sg))))))
                outer._om_err.append(float(np.degrees(
                    np.linalg.norm(st[9:12] - om))))

        outer.model = _Shadow()
        outer.model.ModelTag = "heyokaShadow"

    def summary(self) -> dict:
        if not self.times:
            return {"samples": 0}
        import numpy as _np
        return {
            "samples": len(self.times),
            "final_pos_err_m": float(self.pos_err[-1]),
            "max_pos_err_m": float(_np.max(self.pos_err)),
            "final_att_err_deg": float(self.att_err_deg[-1]),
            "max_att_err_deg": float(_np.max(self.att_err_deg)),
            "final_omega_err_deg_s": float(self._om_err[-1]) if self._om_err else 0.0,
            "mean_applied_torque_Nm": float(_np.mean(self._tmag)) if self._tmag else 0.0,
        }
