"""Sun model (reference)."""
import numpy as np

def sun_direction(t, year=365.25*86400.0, phase0=0.5*np.pi):
    ang=2*np.pi*t/year + phase0
    return np.array([np.cos(ang), np.sin(ang), 0.0])

def sun_pointing_quaternion(sun_dir_n, roll_ref=None):
    if roll_ref is None: roll_ref=np.array([1.0,0,0])
    x=sun_dir_n/np.linalg.norm(sun_dir_n)
    r=roll_ref - (roll_ref@x)*x
    if np.linalg.norm(r)<1e-9:
        r=np.array([0,0,1]) - (np.array([0,0,1])@x)*x
    if np.linalg.norm(r)<1e-9:
        r=np.array([0,1,0])
    r=r/np.linalg.norm(r)
    z=np.cross(x,r); z=z/np.linalg.norm(z)
    y=np.cross(z,x); y=y/np.linalg.norm(y)
    R=np.array([x,y,z])
    # dcm_to_quat
    from .dynamics import dcm_to_quat
    return dcm_to_quat(R)
