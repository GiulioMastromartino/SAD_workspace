"""Sensors (reference)."""
import numpy as np
from .config import STAR_SIGMA, STAR_RATE_CAP, STAR_DROPOUT

def star_tracker_measure(q_true, omega_body, sun_dir_b, rng=None):
    if rng is None: rng=np.random.default_rng(0)
    q_true=np.asarray(q_true,float)
    omega_body=np.asarray(omega_body,float)
    if np.linalg.norm(omega_body)>STAR_RATE_CAP:
        return None
    sun_dir_b=np.asarray(sun_dir_b,float)
    if np.linalg.norm(sun_dir_b)>1e-9:
        sd=sun_dir_b/np.linalg.norm(sun_dir_b)
        if sd[0]>0.9:
            return None
    if rng.random()<STAR_DROPOUT:
        return None
    err=rng.normal(0,STAR_SIGMA,3)
    a=np.linalg.norm(err)
    if a<1e-12:
        qerr=np.array([0,0,0,1.0])
    else:
        qerr=np.array([err[0]*np.sin(a/2)/a, err[1]*np.sin(a/2)/a, err[2]*np.sin(a/2)/a, np.cos(a/2)])
    from .dynamics import quat_mul, quat_normalize
    return quat_normalize(quat_mul(qerr,q_true))

def imu_measure(omega_true, bias, rng=None, dt=0.1):
    if rng is None: rng=np.random.default_rng(0)
    omega_true=np.asarray(omega_true,float)
    noise=rng.normal(0,1,3)* (0.01e-3)/np.sqrt(dt)
    return omega_true + np.asarray(bias,float) + noise
