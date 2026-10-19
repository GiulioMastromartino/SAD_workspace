/* Level-2 C MEX S-function wrapper for sad_dynamics.
   Build with: mex sad_dynamics_sfun.c -I../../cpp -I../../cpp/core -I<eigen> ...
   Inputs : force_b(3), torque_b(3)   (6)
   Outputs: r(3), v(3), q(4), w(3) (13)
   States : dynamics state (stored in work vector)
*/
#define S_FUNCTION_NAME  sad_dynamics_sfun
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#include "sad_dynamics.h"
#include <stdlib.h>
#include <string.h>

#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 6);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 13);
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 1); /* store state pointer */
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S) {
    sad::DynamicsState *st = (sad::DynamicsState*)malloc(sizeof(sad::DynamicsState));
    sad::DynamicsParams p; p.mass = 8.0; p.mu = 3.986004418e14; p.gravity_gradient = 1;
    p.inertia = Eigen::Matrix3d::Identity(); /* placeholder */
    p.inv_inertia = Eigen::Matrix3d::Identity();
    sad::dynamics_init(*st, p);
    ssSetPWorkValue(S, 0, st);
}

static void mdlOutputs(SimStruct *S, int_T tid) {
    (void)tid;
    sad::DynamicsState *st = (sad::DynamicsState*)ssGetPWorkValue(S, 0);
    const real_T *u = ssGetInputPortRealSignal(S, 0);
    sad::DynamicsInput in; in.force_b << u[0],u[1],u[2]; in.torque_b << u[3],u[4],u[5];
    sad::DynamicsOutput out;
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    /* step using ssGetT */
    double dt = ssGetT(S) - st->t; if (dt <= 0) dt = 0.1;
    sad::dynamics_step(in, *st, out, dt);
    for (int i=0;i<3;++i){ y[i]=out.r[i]; y[3+i]=out.v[i]; }
    for (int i=0;i<4;++i) y[6+i]=out.q[i];
    for (int i=0;i<3;++i) y[10+i]=out.w[i];
}

static void mdlTerminate(SimStruct *S) {
    sad::DynamicsState *st = (sad::DynamicsState*)ssGetPWorkValue(S, 0);
    if (st) free(st);
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
