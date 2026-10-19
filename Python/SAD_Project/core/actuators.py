"""Actuators (reference)."""
import numpy as np
from .config import THRUSTER_POS, THRUSTER_DIR, THRUSTER_TMAX, ISP_JET, G0, MASS

def thruster_allocate(Fdes, Tdes):
    Fdes=np.asarray(Fdes,float); Tdes=np.asarray(Tdes,float)
    n=len(THRUSTER_POS)
    A=np.zeros((6,n))
    for i in range(n):
        d=THRUSTER_DIR[i]; d=d/np.linalg.norm(d)
        tau=np.cross(THRUSTER_POS[i], d)
        A[:3,i]=d
        A[3:,i]=tau
    b=np.concatenate([Fdes,Tdes])
    u,res,rank,sv=np.linalg.lstsq(A,b,rcond=None)
    u=np.clip(u,0,None)
    scale=1.0
    for i in range(n):
        if u[i]>1e-12 and u[i]>THRUSTER_TMAX[i]:
            s=THRUSTER_TMAX[i]/u[i]
            if s<scale: scale=s
    if scale<1.0: u=u*scale
    u=np.clip(u,0,THRUSTER_TMAX)
    return u

def thruster_step(throttle, dt):
    # throttle array [0,1]
    throttle=np.asarray(throttle,float)
    n=len(throttle)
    F=np.zeros(3); T=np.zeros(3); mdot=0.0
    for i in range(n):
        thrust=throttle[i]*THRUSTER_TMAX[i]
        d=THRUSTER_DIR[i]/np.linalg.norm(THRUSTER_DIR[i])
        F+=thrust*d
        T+=np.cross(THRUSTER_POS[i], thrust*d)
        mdot+=thrust/(ISP_JET*G0)
    return F,T,mdot

def rw_step(torque_cmd, h, params=None):
    max_torque=0.05; max_mom=0.5; tau=0.2
    torque_cmd=np.asarray(torque_cmd,float)
    h=np.asarray(h,float).copy()
    axes=np.eye(3)
    n=3
    tau_w=np.zeros(n)
    for i in range(n):
        tau_w[i]=torque_cmd@axes[i]
        tau_w[i]=np.clip(tau_w[i],-max_torque,max_torque)
    T=np.zeros(3)
    for i in range(n):
        hdot=tau_w[i]
        if h[i]>=max_mom and hdot>0: hdot=0
        if h[i]<=-max_mom and hdot<0: hdot=0
        alpha=dt/(tau+dt)
        h[i]+=hdot*alpha*dt
        T+=hdot*alpha*axes[i]
    return T,h
