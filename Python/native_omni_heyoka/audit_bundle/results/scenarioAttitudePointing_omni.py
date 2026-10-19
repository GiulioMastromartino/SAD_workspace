#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

Demonstrates how to stabilize the attitude tumble without translational motion.
This script sets up a 6-DOF spacecraft, but without specifying any orbital motion.  Thus,
this scenario simulates the spacecraft translating in deep space.  The scenario is a simplified
version of :ref:`scenarioAttitudeFeedback` with the orbital setup removed.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioAttitudePointing.py

As with :ref:`scenarioAttitudeFeedback`, when
the simulation completes 3 plots are shown for the MRP attitude history, the rate
tracking errors, as well as the control torque vector.

The simulation layout is shown in the following illustration.  A single simulation process is created
which contains both the spacecraft simulation modules, as well as the Flight Software (FSW) algorithm
modules.

.. image:: /_images/static/test_scenarioAttitudePointing.svg
   :align: center

Illustration of Simulation Results
----------------------------------

::

    show_plots = True, useLargeTumble = False

Here a small initial tumble is simulated.  The
resulting attitude and control torque histories are shown below.  The spacecraft quickly
regains a stable orientation without tumbling past 180 degrees.

.. image:: /_images/Scenarios/scenarioAttitudePointing10.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudePointing20.svg
   :align: center

::

    show_plots = True, useLargeTumble = True

Note that, as expected,
the orientation error tumbles past 180 degrees before stabilizing to zero.  The control
torque effort is also much larger in this case.

.. image:: /_images/Scenarios/scenarioAttitudePointing11.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudePointing21.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft(), extForceTorque, simpleNav() and
#           mrpFeedback() modules.  Illustrates a 6-DOV spacecraft detumbling in deep space.
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 19, 2016
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
from Basilisk.fswAlgorithms import inertial3D
# import FSW Algorithm related support
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import simpleNav
# import simulation related support
from Basilisk.simulation import spacecraft
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
# attempt to import vizard
from Basilisk.utilities import vizSupport
from Basilisk.utilities import simHelpers

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots, useLargeTumble):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        useLargeTumble (bool): Specify if a large initial tumble rate should be used

    """

    #
    #  From here on scenario python code is found.  Above this line the code is to setup a
    #  unitTest environment.  The above code is not critical if learning how to code BSK.
    #

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(10.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(.1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(I)
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    if useLargeTumble:
        scObject.hub.omega_BN_BInit = [[0.8], [-0.6], [0.5]]  # rad/s - omega_BN_B
    else:
        scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # setup extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    #
    #   setup the FSW algorithm tasks
    #

    # setup inertial3D guidance module
    inertial3DObj = inertial3D.inertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    scSim.AddModelToTask(simTaskName, inertial3DObj)
    inertial3DObj.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation

    # setup the attitude tracking error evaluation module
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attError)

    # setup the MRP Feedback control module
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControl)
    mrpControl.K = 3.5
    mrpControl.Ki = -1  # make value negative to turn off integral feedback
    mrpControl.P = 30.0
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 50
    samplingTime = simHelpers.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    attErrorLog = attError.attGuidOutMsg.recorder(samplingTime)
    mrpLog = mrpControl.cmdTorqueOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, attErrorLog)
    scSim.AddModelToTask(simTaskName, mrpLog)

    #
    # create simulation messages
    #

    # create the FSW vehicle configuration message
    # use the same inertia in the FSW algorithm as in the simulation
    vehicleConfigOut = messaging.VehicleConfigMsgPayload(ISCPntB_B=I)
    configDataMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    #
    # connect the messages to the modules
    #
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(configDataMsg)

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                        # , saveFile=fileName
                                        )

    #
    #   initialize Simulation
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
    #   configure a simulation stop time and execute the simulation run
    #
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
    dataSigmaBR = attErrorLog.sigma_BR
    dataOmegaBR = attErrorLog.omega_BR_B
    timeAxis = attErrorLog.times()
    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, dataSigmaBR[:, idx],
                 color=simHelpers.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')
    figureList = {}
    pltName = fileName + "1" + str(int(useLargeTumble))
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, dataLr[:, idx],
                 color=simHelpers.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')
    pltName = fileName + "2" + str(int(useLargeTumble))
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, dataOmegaBR[:, idx],
                 color=simHelpers.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')

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
        False,  # useLargeTumble
    )
