"""Propagation harness (reference)."""
import numpy as np
from .dynamics import step_rk4

def propagate(state0, control_fn, dt, n_steps, params=None):
    state=np.asarray(state0,float).copy()
    traj=[state.copy()]
    for k in range(n_steps):
        Fb,Tb=control_fn(state,k*dt)
        state=step_rk4(state,Fb,Tb,dt,params)
        traj.append(state.copy())
    return np.array(traj)
