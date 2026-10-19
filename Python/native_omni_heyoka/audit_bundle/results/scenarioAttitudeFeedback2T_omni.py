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

.. youtube:: tkY95HZi7Mo
   :width: 560
   :height: 315

Overview
--------

Demonstrates how to stabilize the tumble of a spacecraft orbiting the
Earth that is initially tumbling, but uses 2 separate threads for dynamics and algorithm modules.
This script sets up a 6-DOF spacecraft which is orbiting the Earth. This setup
is similar to the :ref:`scenarioAttitudeFeedback`,
but here the dynamics
simulation and the Flight Software (FSW) algorithms are run at different time steps
using two separate task groups (also called processes).

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioAttitudeFeedback2T.py

The simulation layout is shown in the following illustration.  Both a simulation process is created
which contains the spacecraft simulation modules.  A separate FSW algorithm process is run
at a different updated rate to evaluate the Flight Software (FSW) algorithm
modules.  Interface messages are now shared across SIM and FSW message passing interfaces (MPIs).

.. image:: /_images/static/test_scenarioAttitudeFeedback2T.svg
   :align: center

Note that starting with Basilisk 2.0 onwards it is trivial to connect to module messages within
another Basilisk process (task group).  There are no additional steps required to establish these connections.
Recall that this simulation is single-threaded and that the processes are evaluated sequentially.  The new BSK2
messaging system got rid of message names.  With multi-process simulations there are thus no longer any
message naming conflicts to consider as messages are directly connected.

Further, this simulation has the logging happening at a different rate then the simulation or FSW algorithm
processes.  Thus, this is a demonstration of using 3 processes running sequentially.

When the simulation completes 5 plots are shown for the MRP attitude history, the rate
tracking errors, as well as the control torque vector.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True, useUnmodeledTorque = False, useIntGain = False

Note that now the FSW algorithms are called in a separate process, in the first time step the
navigation message has not been copied over, and the initial FSW values for the tracking
errors are zero.  This is why there is a slight difference in the resulting closed loop
performance.

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T100.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T200.svg
   :align: center

::

    show_plots = True, useUnmodeledTorque = True, useIntGain = False

As expected, the orientation error doesn't settle to zero, but rather converges to a non-zero offset
proportional to the un-modeled torque being simulated.  Also, the control torques settle on
non-zero steady-state values.

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T110.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T210.svg
   :align: center

::

    show_plots = True, useUnmodeledTorque = True, useIntGain = True

In this case the orientation error does settle to zero.  The integral term changes the control torque
to settle on a value that matches the un-modeled external torque.

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T111.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedback2T211.svg
   :align: center


"""


#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft(), extForceTorque, simpleNav() and
#           mrpFeedback() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit.
#           This scenario is the same as scenarioAttitudeControl, but with the
#           difference that here the control and dynamics are executed at different
#           frequencies or time steps.
# Author:   Hanspeter Schaub
# Creation Date:  Nov. 25, 2016
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
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
# attempt to import vizard
from Basilisk.utilities import vizSupport
from Basilisk.utilities import simHelpers

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def run(show_plots, useUnmodeledTorque, useIntGain):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        useUnmodeledTorque (bool): Specify if an external torque should be included
        useIntGain (bool): Specify if the feedback control uses an integral feedback term

    """
    # Create simulation variable names
    dynTaskName = "dynTask"
    dynProcessName = "dynProcess"

    fswTaskName = "fswTask"
    fswProcessName = "fswProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(10.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(dynProcessName)
    fswProcess = scSim.CreateNewProcess(fswProcessName)

    # create the dynamics task and specify the integration update time
    simTimeStep = macros.sec2nano(0.1)
    dynProcess.addTask(scSim.CreateNewTask(dynTaskName, simTimeStep))
    fswTimeStep = macros.sec2nano(0.5)
    fswProcess.addTask(scSim.CreateNewTask(fswTaskName, fswTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(I)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(dynTaskName, scObject)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject)

    # setup extForceTorque module
    # the control torque is read in through the messaging system
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    # use the input flag to determine which external torque should be applied
    # Note that all variables are initialized to zero.  Thus, not setting this
    # vector would leave it's components all zero for the simulation.
    if useUnmodeledTorque:
        extFTObject.extTorquePntB_B = [[0.25], [-0.25], [0.1]]
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(dynTaskName, extFTObject)

    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(dynTaskName, sNavObject)

    #
    #   setup the FSW algorithm tasks
    #

    # setup inertial3D guidance module
    inertial3DObj = inertial3D.inertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    scSim.AddModelToTask(fswTaskName, inertial3DObj)
    inertial3DObj.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation

    # setup the attitude tracking error evaluation module
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(fswTaskName, attError)

    # setup the MRP Feedback control module
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(fswTaskName, mrpControl)
    mrpControl.K = 3.5
    if useIntGain:
        mrpControl.Ki = 0.0002  # make value negative to turn off integral feedback
    else:
        mrpControl.Ki = -1  # make value negative to turn off integral feedback
    mrpControl.P = 30.0
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1

    #
    #   Setup data logging before the simulation is initialized
    #
    #   Add logging object to a task group, this controls the logging rate
    numDataPoints = 100
    dataLog = scObject.scStateOutMsg.recorder(simHelpers.samplingTime(simulationTime, simTimeStep, numDataPoints))
    attErrorLog = attError.attGuidOutMsg.recorder(simHelpers.samplingTime(simulationTime,
                                                                                     fswTimeStep, numDataPoints))
    mrpLog = mrpControl.cmdTorqueOutMsg.recorder(simHelpers.samplingTime(simulationTime,
                                                                                    fswTimeStep, numDataPoints))

    scSim.AddModelToTask(dynTaskName, dataLog)
    scSim.AddModelToTask(fswTaskName, attErrorLog)
    scSim.AddModelToTask(fswTaskName, mrpLog)

    #
    # create FSW simulation messages
    #

    # create the FSW vehicle configuration message
    configData = messaging.VehicleConfigMsgPayload(ISCPntB_B=I)

    configDataMsg = messaging.VehicleConfigMsg().write(configData)

    #
    #   set initial Spacecraft States
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0  # meters
    oe.e = 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

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
    vizSupport.enableUnityVisualization(scSim, dynTaskName, scObject
                                        # , saveFile=fileName
                                        )

    #
    #   initialize Simulation
    #
    # Next, after the simulation has been initialized and the modules messages are created
    # a discover process must be called that links messages that have the same name.  This is
    # achieved through the combined initialization and message discovery macro.
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
        mirror_stock_config(scSim, scObject, _oi, thr_cmd_msg=None, ext_torque_msg=mrpControl.cmdTorqueOutMsg)
        scObject.setIntegrator(_oi)
    _hjk_state = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(dynTaskName, _hjk_state)
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
    dataPos = dataLog.r_BN_N
    dataSigmaBN = dataLog.sigma_BN
    timeAxis = dataLog.times()
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
    pltName = fileName + "1" + str(int(useUnmodeledTorque)) + str(int(useIntGain))
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, dataLr[:, idx],
                 color=simHelpers.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')
    pltName = fileName + "2" + str(int(useUnmodeledTorque)) + str(int(useIntGain))
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, dataOmegaBR[:, idx],
                 color=simHelpers.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')

    plt.figure(4)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, dataPos[:, idx] / 1000,
                 color=simHelpers.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Inertial Position [km] ')

    plt.figure(5)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, dataSigmaBN[:, idx],
                 color=simHelpers.getLineColor(idx, 3),
                 label=r'$\sigma_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Inertial MRP Attitude ')

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
        False,  # useUnmodeledTorque
        False  # useIntGain
        )
