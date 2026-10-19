"""Quaternion dynamics (reference)."""
import numpy as np
from .config import MASS, INERTIA, INV_INERTIA, MU_EARTH

def quat_normalize(q):
    q = np.asarray(q,float)
    n = np.linalg.norm(q)
    if n < 1e-15: return np.array([0,0,0,1.0])
    return q/n

def quat_mul(a,b):
    a=np.asarray(a,float); b=np.asarray(b,float)
    x1,y1,z1,w1 = a
    x2,y2,z2,w2 = b
    return np.array([
        w1*x2+x1*w2+y1*z2-z1*y2,
        w1*y2-x1*z2+y1*w2+z1*x2,
        w1*z2+x1*y2-y1*x2+z1*w2,
        w1*w2-x1*x2-y1*y2-z1*z2,
    ])

def quat_conj(q):
    q=np.asarray(q,float); return np.array([-q[0],-q[1],-q[2],q[3]])

def quat_to_dcm(q):
    """Body-from-inertial DCM, matching q_dot = 0.5*q (x) [omega,0]."""
    q=quat_normalize(q)
    x,y,z,w=q
    R=np.array([
        [1-2*(y*y+z*z),2*(x*y+z*w),2*(x*z-y*w)],
        [2*(x*y-z*w),1-2*(x*x+z*z),2*(y*z+x*w)],
        [2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y)],
    ])
    return R

def dcm_to_quat(R):
    # closed form below is for the active DCM; transpose for the inverse
    R=np.asarray(R,float).T
    tr=R.trace()
    if tr>0:
        s=2*np.sqrt(tr+1); q=np.array([(R[2,1]-R[1,2])/s,(R[0,2]-R[2,0])/s,(R[1,0]-R[0,1])/s,s/4])
    elif R[0,0]>R[1,1] and R[0,0]>R[2,2]:
        s=2*np.sqrt(1+R[0,0]-R[1,1]-R[2,2]); q=np.array([s/4,(R[0,1]+R[1,0])/s,(R[0,2]+R[2,0])/s,(R[2,1]-R[1,2])/s])
    elif R[1,1]>R[2,2]:
        s=2*np.sqrt(1+R[1,1]-R[0,0]-R[2,2]); q=np.array([(R[0,1]+R[1,0])/s,s/4,(R[1,2]+R[2,1])/s,(R[0,2]-R[2,0])/s])
    else:
        s=2*np.sqrt(1+R[2,2]-R[0,0]-R[1,1]); q=np.array([(R[0,2]+R[2,0])/s,(R[1,2]+R[2,1])/s,s/4,(R[1,0]-R[0,1])/s])
    return quat_normalize(q)

def quat_derivative(q,omega):
    q=quat_normalize(q); omega=np.asarray(omega,float)
    x,y,z,w=q
    dq=np.array([
        0.5*(w*omega[0]+y*omega[2]-z*omega[1]),
        0.5*(w*omega[1]-x*omega[2]+z*omega[0]),
        0.5*(w*omega[2]+x*omega[1]-y*omega[0]),
        0.5*(-x*omega[0]-y*omega[1]-z*omega[2]),
    ])
    return dq

def quat_error_vec(qd,qc):
    qe=quat_mul(qc,quat_conj(qd))
    if qe[3]<0: qe=-qe
    return qe[:3]

def rhs(state, Fb, Tb, params=None):
    if params is None:
        mass=MASS; inertia=INERTIA; inv_inertia=INV_INERTIA; mu=MU_EARTH; gg=True
    else:
        mass=params['mass']; inertia=params['inertia']; inv_inertia=params['inv_inertia']; mu=params['mu']; gg=params.get('gg',True)
    r=state[:3]; v=state[3:6]; q=state[6:10]; w=state[10:13]
    q=quat_normalize(q)
    Rn=np.linalg.norm(r)
    if Rn<1e3: Rn=1e3
    a_grav=-mu*r/(Rn**3)
    Rbi=quat_to_dcm(q)
    Rib=Rbi.T
    Fb=np.asarray(Fb,float); Tb=np.asarray(Tb,float)
    Fi=Rib@Fb
    ai=Fi/mass + a_grav
    tgg=np.zeros(3)
    if gg:
        rb=Rbi@r
        invR5=1.0/(Rn**5)
        k=3.0*mu*invR5
        Ir=inertia@rb
        tgg=k*np.cross(rb,Ir)
    tau_b=Tb+tgg
    Iw=inertia@w
    wdot=inv_inertia@(tau_b-np.cross(w,Iw))
    qdot=quat_derivative(q,w)
    d=np.concatenate([v,ai,qdot,wdot])
    return d

def step_rk4(state, Fb, Tb, dt, params=None):
    state=np.asarray(state,float).copy()
    def f(x):
        return rhs(x,Fb,Tb,params)
    k1=f(state)
    k2=f(state+0.5*dt*k1)
    k3=f(state+0.5*dt*k2)
    k4=f(state+dt*k3)
    new=state+(dt/6.0)*(k1+2*k2+2*k3+k4)
    new[6:10]=quat_normalize(new[6:10])
    return new
