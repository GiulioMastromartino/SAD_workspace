
#
#  ISC License
#
#  Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

r"""

Overview
--------

This script discusses how to perform sweeping maneuvers on a spacecraft with respect to the Hill frame ( or an alternate, corrected Hill Frame). It sets up a 6-DOF spacecraft which is orbiting the Earth.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioSweepingSpacecraft.py

A single simulation process is created which contains both the spacecraft simulation modules, as well as the Flight Software (FSW) algorithm
modules.

The simulation setup is similar to the one used in the Hill pointing guidance example
:ref:`scenarioAttitudeGuidance`. The main difference is based on the use of the :ref:`eulerRotation` to allow 3-2-1 Euler rotations of the Hill reference frame.
This rotating Hill frame becomes the new reference frame for the attitude tracking error evaluation module.
Two inputs are added : (1) A sequence of 3-2-1 Euler angle rate commands (in rad/s) of type numpy array, (2) A sequence of time commands (in min) of type numpy array, which specifies the duration of each angle rate command.

When the simulation completes, 4 plots are shown : (1) The attitude error norm history, (2) The rate
tracking error history, (3) The control torque vector history, as well as (4) The projection of the body-frame B
axes :math:`\hat b_1`, :math:`\hat b_2` and :math:`\hat b_3` onto the
Hill frame axes :math:`\hat\imath_r`,
:math:`\hat\imath_{\theta}` and :math:`\hat\imath_h`.  This latter plot illustrates how the spacecraft is rotating with respect to the Hill frame during the sweeping maneuvers.


Illustration of Simulation Results
----------------------------------

::

    show_plots = True, useAltBodyFrame = False, angle_rate_command = np.array([[0.0,0,0.0],[0.0,0.002,0.0],[0.0,-0.002,0.0],[0.0,0,0.0]]), time_command = np.array([10,10,10,10])

The default scenario shown has the ``useAltBodyFrame`` flag turned off. It means that we seek to perform 3-2-1 Euler rotations on the Hill frame, and not an alternate, corrected Hill frame.

The resulting attitude error norm and control torque histories are shown below.  Note that the projection
of the body frame axe :math:`\hat b_2` onto the Hill frame axe :math:`\hat\imath_{\theta}` converge to :math:`|1|`, indicating that the rotation occurs with respect to :math:`\hat\imath_{\theta}`.

.. image:: /_images/Scenarios/scenarioSweepingSpacecraft10.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSweepingSpacecraft20.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSweepingSpacecraft40.svg
   :align: center

::

    show_plots = True, useAltBodyFrame = True, angle_rate_command = np.array([[0.0,0,0.0],[0.0,0.002,0.0],[0.0,-0.002,0.0],[0.0,0,0.0]]), time_command = np.array([10,10,10,10])

Here the ``useAltBodyFrame`` flag is turned on. It means that we seek to not perform 3-2-1 Euler rotations on the Hill frame but rather on an alternate, corrected Hill frame. We define the corrected Hill frame orientation as a 180 deg rotation about
:math:`\hat\imath_{\theta}`.  This flips the orientation of the first and third Hill frame axis.  This is achieved
through::

  attGuidanceEuler.angleSet = [0, np.pi, 0]

The resulting attitude error norm history, rate tracking error history and the projection of the body-frame B onto the
Hill frame are shown below.

.. image:: /_images/Scenarios/scenarioSweepingSpacecraft11.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSweepingSpacecraft21.svg
   :align: center

.. image:: /_images/Scenarios/scenarioSweepingSpacecraft41.svg
   :align: center

"""

#
# Basilisk Scenario Script
#
# Purpose:  Illustrates how to perform sweeping maneuvers on a spacecraft in a very modular manner.
# Author:   Anais Cheval
# Creation Date:  Sep. 10, 2024
#

import os
import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
# import message declarations
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import hillPoint
from Basilisk.fswAlgorithms import eulerRotation

# import FSW Algorithm related support
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import simpleNav
# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.utilities import RigidBodyKinematics
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
# attempt to import vizard
from Basilisk.utilities import vizSupport
from Basilisk.utilities import simHelpers

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


# Plotting functions

def plot_attitude_error(timeLineSet, dataSigmaBR):
    """Plot the attitude result."""
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    vectorData = dataSigmaBR
    sNorm = np.array([np.linalg.norm(v) for v in vectorData])
    plt.plot(timeLineSet, sNorm,
             color=simHelpers.getLineColor(1, 3),
             )
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error Norm $|\sigma_{B/R}|$')
    ax.set_yscale('log')

def plot_control_torque(timeLineSet, dataLr):
    """Plot the control torque response."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(timeLineSet, dataLr[:, idx],
                 color=simHelpers.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')

def plot_rate_error(timeLineSet, dataOmegaBR):
    """Plot the body angular velocity tracking error."""
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeLineSet, dataOmegaBR[:, idx],
                 color=simHelpers.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')
    return

def plot_orientation(timeLineSet, dataPos, dataVel, dataSigmaBN):
    """Plot the spacecraft orientation."""
    vectorPosData = dataPos
    vectorVelData = dataVel
    vectorMRPData = dataSigmaBN
    data = np.empty([len(vectorPosData), 3])
    for idx in range(0, len(vectorPosData)):
        ir = vectorPosData[idx] / np.linalg.norm(vectorPosData[idx])
        hv = np.cross(vectorPosData[idx], vectorVelData[idx])
        ih = hv / np.linalg.norm(hv)
        itheta = np.cross(ih, ir)
        dcmBN = RigidBodyKinematics.MRP2C(vectorMRPData[idx])
        data[idx] = [np.dot(ir, dcmBN[0]), np.dot(itheta, dcmBN[1]), np.dot(ih, dcmBN[2])]
    plt.figure(4)
    labelStrings = (r'$\hat\imath_r\cdot \hat b_1$'
                    , r'${\hat\imath}_{\theta}\cdot \hat b_2$'
                    , r'$\hat\imath_h\cdot \hat b_3$')
    for idx in range(3):
        plt.plot(timeLineSet, data[:, idx],
                 color=simHelpers.getLineColor(idx, 3),
                 label=labelStrings[idx])
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Orientation Illustration')


# Run function

def run (show_plots, useAltBodyFrame, angle_rate_command, time_command):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        useAltBodyFrame (bool): Specifies if the Hill reference frame should be corrected
        angle_rate_command (numpy array) : Sequence of 3-2-1 Euler angle rate commands (in rad/s)
        time_command (numpy array) : Sequence of time commands, which specifies the duration of each angle rate command (in min)
    """
    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    # Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # Set the simulation time variable used later on
    simulationTime = macros.min2nano(time_command[0]) # convert mins to nano-seconds

    # Create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # Create the dynamics task and specify the integration update time for this task
    simulationTimeStep = macros.sec2nano(0.1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    # Create the SIM modules
    #

    # Initialize spacecraft object and its properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]  # Inertia of the spacecraft
    scObject.hub.mHub = 750.0  # kg - Mass of the spacecraft
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(I)
    scSim.AddModelToTask(simTaskName, scObject)

    # Setup earth gravity body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu
    gravFactory.addBodiesTo(scObject)

    # Setup external torque ( no disturbance external torque set)
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    # Setup navigation sensor module
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg) # scStateInMsg Input message name for spacecraft state

    #
    #Create the FSW modules
    #

    #Setup HillPoint Guidance Module
    attGuidance = hillPoint.hillPoint()
    attGuidance.ModelTag = "hillPoint"
    attGuidance.transNavInMsg.subscribeTo(sNavObject.transOutMsg) # Incoming spacecraft tranlational message
    scSim.AddModelToTask(simTaskName, attGuidance)

    #Setup Euler rotation of the Hill reference frame
    attGuidanceEuler = eulerRotation.eulerRotation()
    attGuidanceEuler.ModelTag = "eulerRotation"
    attGuidanceEuler.attRefInMsg.subscribeTo(attGuidance.attRefOutMsg)
    scSim.AddModelToTask(simTaskName, attGuidanceEuler)
    if useAltBodyFrame:
        attGuidanceEuler.angleSet = [0, np.pi, 0]
    attGuidanceEuler.angleRates = angle_rate_command[0]


    #Setup the attitude tracking error evaluation module
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorrotatingHill"
    scSim.AddModelToTask(simTaskName, attError)
    attError.attRefInMsg.subscribeTo(attGuidanceEuler.attRefOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)

    # Setup the MRP feeback control module
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControl)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    configData = messaging.VehicleConfigMsgPayload(ISCPntB_B=I)
    configDataMsg = messaging.VehicleConfigMsg().write(configData)
    mrpControl.vehConfigInMsg.subscribeTo(configDataMsg) # The MRP feedback algorithm requires the vehicle configuration structure.
    mrpControl.K = 3.5
    mrpControl.Ki = -1.0  # make value negative to turn off integral feedback
    mrpControl.P = 30.0
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1

    # Connect torque command to external torque effector
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)

    #
    # Data logging
    #

    numDataPoints = 100
    samplingTime = simHelpers.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    mrpLog = mrpControl.cmdTorqueOutMsg.recorder(samplingTime)
    attErrLog = attError.attGuidOutMsg.recorder(samplingTime)
    snAttLog = sNavObject.attOutMsg.recorder(samplingTime)
    snTransLog = sNavObject.transOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, mrpLog)
    scSim.AddModelToTask(simTaskName, attErrLog)
    scSim.AddModelToTask(simTaskName, snAttLog)
    scSim.AddModelToTask(simTaskName, snTransLog)

    #Inititialize spacecraft state
    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0  # meters
    oe.e = 0.1
    oe.i = 33.3 * macros.D2R  # Degree to radian
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N Initial position with respect to the inertial planet frame
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N Initial velocity with respect to the inertial planet frame
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B Initial attitude with respect to the inertial planet frame
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B Initial attitude rate with respect to the inertial planet frame

    #
    # Interface the scenario with the BSK Viz
    #
    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                                  # , saveFile=fileName
                                                 )
        vizSupport.createCameraConfigMsg(viz, parentName=scObject.ModelTag,
                                         cameraID=1, fieldOfView=20 * macros.D2R,
                                         resolution=[1024, 1024], renderRate=0.,
                                         cameraPos_B=[1., 0., .0], sigma_CB=[0., np.tan(np.pi/2/4), 0.]
                                         )
        viz.settings.viewCameraFrustumHUD = 1

    #
    # Initialize Simulation
    #

    # --- OMNI-HIJACK BEGIN (generated by hijack_demo.py; only delta vs upstream pin v2.12.0)
    import os as _os
    _hijack_on = _os.environ.get("OMNI_HIJACK", "on") == "on"
    _hijack_out = _os.environ.get("HIJACK_OUT", "hijack_out.npz")
    if _hijack_on:
        import importlib.util as _ilu
        from omniheyoka.omni_heyoka import OmniVecIntegrator
        # mirror_stock_config comes from the CURRENT repo source, loaded by
        # file path (OMNI_REPO_SWAP). The installed wheel's swap.py copy lags
        # the repo (no env readers: it rejects drag/SRP/mag outright). A path
        # import -- never a sys.path insert -- keeps the wheel supplying
        # omni_heyoka + the compiled extension (conftest.py rationale).
        _swap_p = _os.environ.get("OMNI_REPO_SWAP")
        if not _swap_p:
            raise RuntimeError(
                "OMNI_REPO_SWAP must point at the repo omniheyoka/swap.py")
        _swap_spec = _ilu.spec_from_file_location("_hijack_repo_swap",
                                                  _swap_p)
        _swap = _ilu.module_from_spec(_swap_spec)
        _swap_spec.loader.exec_module(_swap)
        mirror_stock_config = _swap.mirror_stock_config
        _oi = OmniVecIntegrator(scObject)
        mirror_stock_config(scSim, scObject, _oi, thr_cmd_msg=None)
        scObject.setIntegrator(_oi)
    _hjk_state = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, _hjk_state)
    # --- OMNI-HIJACK END
    scSim.InitializeSimulation()


    #
    # Execute the simulation for the first angle rate and simulation time
    #

    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    # Update of the angle rate and simulation time
    #

    for i,j in zip(time_command[1:],angle_rate_command[1:]):
        attGuidanceEuler.angleRates = j
        simulationTime += macros.min2nano(i)
        scSim.ConfigureStopTime(simulationTime)
        scSim.ExecuteSimulation()
        # --- OMNI-HIJACK-DUMP BEGIN
        import numpy as _np2
        _hjk_d = {"t": _hjk_state.times()}
        for _k in ("r_BN_N", "v_BN_N", "sigma_BN", "omega_BN_B"):
            _hjk_d[_k] = getattr(_hjk_state, _k)
        _np2.savez(_hijack_out, **_hjk_d)
        # --- OMNI-HIJACK-DUMP END

    #
    #   retrieve the logged data
    #

    dataLr = mrpLog.torqueRequestBody
    dataSigmaBR = attErrLog.sigma_BR
    dataOmegaBR = attErrLog.omega_BR_B
    dataPos = snTransLog.r_BN_N
    dataVel = snTransLog.v_BN_N
    dataSigmaBN = snAttLog.sigma_BN

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #

    timeLineSet = attErrLog.times() * macros.NANO2MIN
    plt.close("all")  # clears out plots from earlier test runs

    plot_attitude_error(timeLineSet, dataSigmaBR)
    figureList = {}
    pltName = fileName + "1" + str(int(useAltBodyFrame))
    figureList[pltName] = plt.figure(1)

    plot_control_torque(timeLineSet, dataLr)
    pltName = fileName + "2" + str(int(useAltBodyFrame))
    figureList[pltName] = plt.figure(2)

    plot_rate_error(timeLineSet, dataOmegaBR)

    plot_orientation(timeLineSet, dataPos, dataVel, dataSigmaBN)
    pltName = fileName + "4" + str(int(useAltBodyFrame))
    figureList[pltName] = plt.figure(4)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#

if __name__ == "__main__":
    run(
        False,  # show_plots
        True,  # useAltBodyFrame
        np.array([[0.0,0,0.0],[0.0,0.002,0.0],[0.0,-0.002,0.0],[0.0,0,0.0]]), # angle_rate_command
        np.array([10,10,10,10]) # time_command
    )
