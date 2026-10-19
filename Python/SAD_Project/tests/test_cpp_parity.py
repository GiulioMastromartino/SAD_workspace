import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import numpy as np
import sad_py
from SAD_Project.core import dynamics as D
from SAD_Project.core import actuators as A
from SAD_Project.core import sensors as S
from SAD_Project.core import sun_model as SM

def test_dynamics_step_parity():
    # initial state
    r0=np.array([6878e3,0,0]); v0=np.array([0,7.5e3,0])
    q0=np.array([0,0,0,1.0]); w0=np.array([0.01, -0.02, 0.03])
    state0=np.concatenate([r0,v0,q0,w0])
    params={'mass':8.0,'inertia':np.diag([0.1,0.2,0.15]),'inv_inertia':np.linalg.inv(np.diag([0.1,0.2,0.15])),
            'mu':3.986004418e14,'gg':True}
    # cpp
    cpp_params=sad_py.DynamicsParams()
    cpp_params.mass=8.0
    cpp_params.mu=3.986004418e14
    cpp_params.gravity_gradient=True
    cpp_params.inertia=np.diag([0.1,0.2,0.15])
    cpp_params.inv_inertia=np.linalg.inv(np.diag([0.1,0.2,0.15]))
    cpp_params.r0=r0; cpp_params.v0=v0; cpp_params.q0=q0; cpp_params.w0=w0
    st=sad_py.DynamicsState(); sad_py.dynamics_init(st,cpp_params)
    u=sad_py.DynamicsInput(); u.force_b=np.array([0.1,0,0]); u.torque_b=np.array([0,0.01,0])
    out=sad_py.DynamicsOutput()
    sad_py.dynamics_step(u,st,out,0.1)
    # python
    new=D.step_rk4(state0,u.force_b,u.torque_b,0.1,params)
    assert np.allclose(out.q, new[6:10], atol=1e-6), (out.q, new[6:10])
    assert np.allclose(out.w, new[10:13], atol=1e-6)
    assert np.allclose(out.r, new[:3], atol=1e-3)

def test_thruster_allocate_shape():
    # just check allocation returns 8
    from SAD_Project.core.config import THRUSTER_POS, THRUSTER_DIR, THRUSTER_TMAX
    # use cpp thruster params
    tp=sad_py.ThrusterParams()
    tp.n=8
    tp.pos=[__import__('numpy').array(p) for p in THRUSTER_POS]
    tp.dir=[__import__('numpy').array(d) for d in THRUSTER_DIR]
    tp.tmax=list(THRUSTER_TMAX)
    thr=sad_py.thruster_allocate(tp, np.array([0.1,0,0]), np.array([0,0,0.01]))
    assert len(thr)==8
    assert all(t>=0 for t in thr)

def test_startracker_runs():
    sp=sad_py.StarTrackerParams(); sp.sigma=1e-4; sp.rate_cap=0.1; sp.dropout_prob=0.0; sp.seed=1
    st=sad_py.StarTrackerState(); sad_py.startracker_init(st,sp)
    inp=sad_py.StarTrackerInput(); inp.q_true=np.array([0,0,0,1.0]); inp.omega_body=np.array([0,0,0])
    out=sad_py.StarTrackerOutput()
    sad_py.startracker_step(inp,st,out,0.1)
    assert out.valid
    assert abs(np.linalg.norm(out.q_meas)-1.0)<1e-9

def test_mission_sequence():
    mp=sad_py.MissionParams()
    ms=sad_py.MissionState(); sad_py.mission_init(ms,mp)
    mi=sad_py.MissionInput(); mi.q=np.array([0,0,0,1.0]); mi.omega=np.array([0.5,0,0]); mi.q_sun=np.array([0,0,np.sin(np.pi/4),np.cos(np.pi/4)])
    mo=sad_py.MissionOutput()
    sad_py.mission_step(mi,ms,mo,0.1)
    assert mo.phase==sad_py.MissionPhase.Detumble
    # after rate drops
    mi.omega=np.array([0,0,0])
    for _ in range(60):
        sad_py.mission_step(mi,ms,mo,0.1)
    assert mo.phase in (sad_py.MissionPhase.Slew, sad_py.MissionPhase.Track)
