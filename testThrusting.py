
from collections import deque
from createStationKeepingObjects import StationKeepingObjects
from load_gmat import *
from matplotlib import pyplot as plt
from xyz2ric import xyz2ric

import numpy as np

# -----------user defined variables----------------------------------
"""Please change the following variables as necessary to shape your scenario"""

# Duration of the scenario in days
maxDays = 60

# Simulation step size while coasting
dtCoast = 20.0 

# Simulation step size while thrusting
dtThrust = 5

# Orbital element set shared by the initial reference and truth satellites
orbitParam = [
    6878,   # SMA, avg alt of 500 km
    1e-3,   # ECC
    65,     # INC
    0,      # RAAN
    0,      # AOP
    0       # TA
]

# Operational bounds (+/-) to keep the truth satellite within
R_bounds = 3
I_bounds = 10
C_bounds = 3

# Maximum thruster duty time in seconds
maxDutyTime = 3600

# % of SMA deviation to target for recovery
smaRecoveryPercent = 0.75
smaOffsetFromDel_f = 0.05

I_deadband_min = 0.9

# 
maneuverArcHalfAngle = 60

# -----------output options------------------------------------------
"""
Measuring the differences as (Truth - Reference), there are 9 graphs available to plot:
    - 3D trajectory in RIC frame (plot_3D_RIC)
    - Position in each RIC axis over time (plot_RIC_v_Time)
    - Differences in the instantaneous and averaged values for each orbital element over time:
        - Semi-major Axis (del_a)
        - Eccentricity (del_e)
        - Inclination (del_i)
        - Right Ascension of the Ascending Node (del_raan)
        - Argument of Periapsis (del_aop)
        - True Anomaly (del_f)
    - Differences in the instantaneous and averaged values for the True Latitude (del_theta)
Each plot can include a point for when each thruster fired, if desired (plot_Show_Firings)

Additionally, the scenario can print to the terminal what maneuver was completed, how long it took
to complete, and at what time did the maneuver conclude (terminal_Completed_Firings)
"""

plot_3D_RIC = False

plot_rRIC_v_Time = True

plot_vRIC_v_Time = False

plot_COE_diffs = {
    "del_a": False,
    "del_e": False,
    "del_i": False,
    "del_raan": True,
    "del_aop": False,
    "del_f": False
}

plot_True_Lat_diff = False

plot_Show_Firings = True

terminal_Completed_Firings = True

# -----------create variables----------------------------------------
"""Additional variables used in the script that SHOULD NOT BE CHANGED"""
mu = 398600  # Earth’s mu in km^3/s^2
burn_duration = 0 # timer to track maneuver duration
coast_duration = 0
mean_motion = np.sqrt(mu / orbitParam[0]**3) # mean motion of intitial orbital parameters
period_sec = 2 * np.pi / mean_motion # initial orbital period in seconds

steps_per_rev = int(np.ceil(period_sec / dtCoast)) # number of simulation steps in 1 orbit around Earth
revs_to_avg = 3 # number of orbits used to average out the oscillations of the perturbed orbital solutions
steps_to_avg = int(revs_to_avg * steps_per_rev) + 5 # number of simulation steps needed to average perturbations

elapsed = 0.0 # elpased number of seconds since the start of the scenario

t = [0] # array to hold each time step

# stores the RIC history of the truth spacecraft about the reference spacecraft
offsetHistory = [[0], [0], [0], [0], [0], [0]]

# Storage of the differences in the orbital elements throughout the scenario
diffCOEs = {
    "del_a": [0],
    "del_e": [0],
    "del_i": [0],
    "del_raan": [0],
    "del_aop": [0],
    "del_f": [0]
}

# Storage of the orbital element differences across one rev about Earth
diffCOEs_buffer = {
    "del_a": deque([0], maxlen=steps_to_avg),
    "del_e": deque([0], maxlen=steps_to_avg),
    "del_i": deque([0], maxlen=steps_to_avg),
    "del_raan": deque([0], maxlen=steps_to_avg),
    "del_aop": deque([0], maxlen=steps_to_avg),
    "del_f": deque([0], maxlen=steps_to_avg)
}

# Storage of the average difference for each orbital element after each time step
diffCOEs_avg = {
    "del_a": [0],
    "del_e": [0],
    "del_i": [0],
    "del_raan": [0],
    "del_aop": [0],
    "del_f": [0]
}

# Storage of the differences in the R-axis of the RIC frame and the oscillation amplitude across one rev
R_buffer = deque([0], maxlen=steps_per_rev)
R_amp_log = deque([0], maxlen=steps_per_rev)

# Storage of the differences in the I-axis of the RIC frame and average difference across one rev
I_buffer = deque([0], maxlen=steps_per_rev)
I_buffer_avg = deque([0], maxlen=steps_per_rev)

# Storage of the differences in the C-axis of the RIC frame and the oscillation amplitude across one rev
C_buffer = deque([0], maxlen=steps_per_rev)
C_amp_log = deque([0], maxlen=steps_per_rev)

# Variables used to help complete I-axis maneuvers
del_a_target = 0 # desired increase in the difference of semi-major axis between the truth and reference satellites
del_a_recovered = False
maneuverAttempts = 0
# Log of maneuver times
burnStarts = []
burnEnds = []

# Timer to prevent controller from being stuck waiting to maneuver
numStepsWaiting = 0

# Initial state
state = "nominal"

status = "real" # "planning"

# -----------configuration preliminaries-----------------------------

# Reference Objectes
refObjs = StationKeepingObjects("reference")
refObjs.setSatCOEs(orbitParam)

# Truth Objects
truthObjs = StationKeepingObjects("truth")
truthObjs.setSatCOEs(orbitParam)
truthObjs.setManeuverable()

gmat.Initialize()

# ------------build out thruster forces------------------------------

# Reference Objectes
refObjs.preparePropInternal()
gator_ref = refObjs.prop_wrap["coast"].getIntegrator()

# Truth Objects
truthObjs.setBurnForces()
truthObjs.preparePropInternal()
gator_truth = truthObjs.prop_wrap["coast"].getIntegrator()
truth_Sat_wrapper = truthObjs.sat_wrap

# ------------run simulation-----------------------------------------

# Create the plot for 3D RIC trajectory, updates with the state change in the controller
if plot_3D_RIC:
    ax_ric_traj = plt.figure().add_subplot(projection='3d')

# Set simulation step size
dt = dtCoast

# Get initial integrator for the truth satellite
gator_truth = truthObjs.satEnginesOff()

del_a_energy = 0
# While the elapsed time is less the max number of days
while elapsed / 86400 < maxDays:
    """# The "wait for I burn" state sometimes get called when the truth satellite is not in a position to recover,
    # this check prevents the thrusters from firing
    if state == "I burn" and rvRIC[1] < I_bounds and burn_duration == 0:

        # Reset state and turn of the I+ thrusters
        state = "nominal"
        gator_truth = truthObjs.satEnginesOff(thrusterAxis)"""
    

    # Propagate spacecraft
    gator_ref.Step(dt)
    gator_truth.Step(dt)

    # Update numerical integrator references
    gator_ref.UpdateSpaceObject()
    gator_truth.UpdateSpaceObject()
    
    # Update the the elpased time
    elapsed = elapsed + dt
    t.append(elapsed)
    
    # Get the updated cartesian states for each spacecraft from the ECI frame
    rv_ref = gator_ref.GetState()
    rv_truth = gator_truth.GetState()

    # Get the corresponding cartesian state from the RIC frame
    rvRIC, rotMatrix = xyz2ric(rv_ref, rv_truth)

    # Get the updated keplerian states for each spacecraft
    refCOE = refObjs.sat_wrap.getKeplerianState()
    truthCOE = truthObjs.sat_wrap.getKeplerianState()

    """# If the truth spacecraft is just getting out of a maneuver, dt may not be equal dtCoast. Update the value of dt
    if dt != dtCoast and (state != "I burn" and state != "C burn"):
        dt = dtCoast"""

    # Only update the 1 rev buffers when elpased is a multiple of dtCoast
    if elapsed % dtCoast == 0:
        R_buffer.append(rvRIC[0])
        
        C_buffer.append(rvRIC[2])

        R_amp = (max(R_buffer) - min(R_buffer)) / 2 if len(R_buffer) > 1 else rvRIC[0]
        C_amp = (max(C_buffer) - min(C_buffer)) / 2 if len(C_buffer) > 1 else rvRIC[2]
        R_amp_log.append(R_amp)
        C_amp_log.append(C_amp)

    # Store latest RIC position vector
    [offsetHistory[i].append(rvRIC[i]) for i in range(len(rvRIC))]
    
    # Compute and store the differences in each of the 6 keplerian elements 
    coes = list(diffCOEs.keys())
    for k in range(6):
        diff = truthCOE[k] - refCOE[k]

        if k > 1 and diff > 180:
            diff = diff - 360
        elif k > 1 and diff < -180:
            diff = diff + 360
        diffCOEs[coes[k]].append(diff) # instaneous differences
        diffCOEs_buffer[coes[k]].append(diff) # instaneous differences within the past rev
        diffCOEs_avg[coes[k]].append(np.mean(diffCOEs_buffer[coes[k]])) # average differences across one rev

    # To determine if the I-axis corections have completed successfully, we will use the difference in semi-major axis based on the spacecraft's specific energy
    sma_truth = truthObjs.sat_wrap.getSMAFromEnergy()
    sma_ref = refObjs.sat_wrap.getSMAFromEnergy()
    del_a_energy = sma_truth - sma_ref

    """
    State Machine Controller Logic:
    While the spacecraft's position vector in the RIC frame are within the user-defined bounds, the spacecraft will operate in a
    nominal status. In this nominal status, all the spacecraft will do is collect state vector and deviation information. The first
    time step the spacecraft learns that it has drifted outside of the user-defined ops boundary, it will switch into a corrective 
    mode for whichever boundary was violated first.

    Violation requirements:
    - R-axis: The amplitude if the truth spacecraft's oscillations exceed that of the user-defined boundary condition
    - I-axis: The truth spacecraft's instaneous position along the I-axis exceeds the user-defined boundary condition
    - C-axis: The amplitude if the truth spacecraft's oscillations exceed that of the user-defined boundary condition

    Positional requirements prior to thruster firing:
    - R-axis: The truth spacecraft is approaching its maximum velocity in the R direction within the R-I plane
    - I-axis: The truth spacecraft must be within a 20 degree window of perigee or apogee and the truth spacecraft must have a smaller 
            semi-major axis than its reference counterpart
    - C-axis: The truth spacecraft is approaching its maximum velocity in the C direction within the I-C plane

    Maneuver termination conditions other than max duty time:
    - R-axis: The truth spacecraft is leaving the defined maneuver window
    - I-axis: The truth spacecraft's semi-major axis must be greater than the reference by the difference computed prior to the start 
            of the maneuver
    - C-axis: The truth spacecraft is leaving the defined maneuver window

    Violation recovery requirements:
    - R-axis: None, this thruster controller is still under development. The current behavior is to fire once and return to "nominal"
    - I-axis: The truth spacecraft achieved 80+% of the necessary change in semi-major axis and the average I-axis position is dropping
    - C-axis: The truth spacecraft's C-axis position amplitude has dropped to 1/3 of the boundary condition
    """
    currentTime = elapsed / 86400

    interpruptManeuver = {
        "I": rvRIC[1] > I_deadband_min * I_bounds,
        "C": C_amp > C_bounds,
        "R": R_amp > R_bounds
    }

    if interpruptManeuver["I"] and state not in ["wait for I burn", "I burn"]:
        interpruptedState = state
        state = "wait for I burn"
    elif interpruptManeuver["C"] and state not in ["wait for I burn", "I burn", "wait for C burn", "C burn", "returning to nominal from C burn"] and interpruptedState == "nominal":
        interpruptedState = state
        state = "wait for C burn"
    elif interpruptManeuver["R"] and state not in ["wait for I burn", "I burn", "wait for C burn", "C burn", "wait for R burn", "R burn", "returning to nominal from R burn"] and interpruptedState == "nominal":
        interpruptedState = state
        state = "wait for R burn"

    if interpruptManeuver["C"] and state != "I burn":
        pass

    match state:
        # -----------waiting for maneuver opoprtunties------------------------
        case "wait for R burn":
            # Increase number of steps waited
            numStepsWaiting += 1

            # Get the velocity phase angle to determine if the satellite is in position
            # velo_phase = np.arctan2(vRIC[0], vRIC[1])
            velo_phase = np.arctan2(rvRIC[0], rvRIC[1])
            # in_node_window = abs(abs(velo_phase) - np.pi /2) < np.deg2rad(15)
            
            f = truthCOE[-1]
            in_node_window = not(
                                maneuverArcHalfAngle < f <= 180 - maneuverArcHalfAngle) and \
                            not(
                                180 + maneuverArcHalfAngle < f <= 360 - maneuverArcHalfAngle)
            
            # Target thrust window has a phase angle of 15 -> 0 -> -20 deg
            if in_node_window and R_amp > R_bounds:
                n = mean_motion
                a = truthObjs.sat_wrap.getSMAFromEnergy()
                eTruth = truthCOE[1]
                eta = np.sqrt(1 - truthCOE[1])
                deltaAOP = np.deg2rad(diffCOEs_avg["del_aop"][-1])
                deltaRAAN = np.deg2rad(diffCOEs_avg["del_raan"][-1])
                i = np.deg2rad(truthCOE[2])

                fTruth = np.deg2rad(truthCOE[5])
                ETruth = 2 * np.atan(np.sqrt((1 - eTruth) / (1 + eTruth)) * np.tan(fTruth / 2))
                MTruth = ETruth - eTruth * np.sin(ETruth)

                fRef = np.deg2rad(refCOE[5])
                ERef = 2 * np.atan(np.sqrt((1 - refCOE[1]) / (1 + refCOE[1])) * np.tan(fRef / 2))
                MRef = ERef - refCOE[1] * np.sin(ERef)
                deltaM = MTruth - MRef
                deltaVp = -n * a / 4 * ((1 + eTruth)**2 / eta**2) * (deltaAOP + deltaRAAN * np.cos(i) + deltaM)
                deltaVa = -n * a / 4 * ((1 - eTruth)**2 / eta**2) * (deltaAOP + deltaRAAN * np.cos(i) + deltaM)
                
                # Based on the current speed in the R direction, fire the opposite direction thrusters
                if f <= 180 and deltaVa > 0:
                    thrusterAxis = "R+"
                elif f <= 180 and deltaVa < 0:
                    thrusterAxis = "R-"
                elif f > 180 and deltaVp > 0:
                    thrusterAxis = "R+"
                elif f > 180 and deltaVp < 0:
                    thrusterAxis = "R-"
                # thrusterAxis = "R+" if vRIC[0] < 0 else "R-"
                
                gator_truth = truthObjs.satEnginesOn(thrusterAxis)
                state = "R burn"
                
                # Add to the list of when burns start the corresponding color for this maneuver
                burnStarts.append((len(t) - 1, "m"))

                # Start maneuver duration timer
                burn_duration = 0.0
                
                # Update simulation time step
                dt = dtThrust

                # Reset R-axis position history from previous rev
                R_buffer.clear()
                R_buffer.append(rvRIC[0])
                
                # Reset the step counter
                numStepsWaiting = 0

                # If the 3D trajectory is enabled, plot the latest coasting segment
                if plot_3D_RIC:
                    plotIndex = 0 if burnEnds == [] else burnEnds[-1]
                    ax_ric_traj.plot(offsetHistory[0][plotIndex:], offsetHistory[1][plotIndex:], offsetHistory[2][plotIndex:], 'b')
                
            
            # If it has been one full rev since spacecraft entered this state, return to nominal to prevent a lock-up
            elif numStepsWaiting == steps_per_rev:
                state = "nominal"
                numStepsWaiting = 0
            else:
                continue
        case "wait for I burn":
            # Increase number of steps waited
            numStepsWaiting += 1

            # Collect the current True Anomaly value to see if the spacecraft is in the appropriate window for a maneuver
            f = truthCOE[-1]
            # in_burn_window = not(10 < f <= 170) and not(190 < f <= 350)

            in_burn_window = 160 < f <= 180
            del_a_target = smaRecoveryPercent * max(abs(del_a_energy), abs(diffCOEs_avg["del_a"][-1]))

            # Possibility for controller to trigger a maneuver when spacecraft is within user-defined bounds, this check prevents that
            valid_burn = diffCOEs_avg["del_a"][-1] < 0 and rvRIC[1] > I_deadband_min * I_bounds
            
            recentManeuver = (elapsed - (t[burnEnds[-1]] if len(burnEnds) > 0 else (3 * period_sec - elapsed)) >= 3 * period_sec)
            if in_burn_window and valid_burn and recentManeuver:
                # Update the controller state
                state = "I burn"
                thrusterAxis = "I+"
                maneuverAttempts = 0
                gator_truth = truthObjs.satEnginesOn(thrusterAxis)

                # Add to the list of when burns start the corresponding color for this maneuver
                burnStarts.append((len(t) - 1, "r"))

                # Establish recovery criteria for I-axis maneuver
                del_a_recovered = False
                
                # Change simulation step size
                dt = dtThrust
                
                # reset steps waiting counter
                numStepsWaiting = 0

                # If plotting the 3D RIC trajectory is on, plot the latest coast segment
                if plot_3D_RIC:
                    plotIndex = 0 if burnEnds == [] else burnEnds[-1]
                    ax_ric_traj.plot(offsetHistory[0][plotIndex:], offsetHistory[1][plotIndex:], offsetHistory[2][plotIndex:], 'b')  
            
            # If waiting in this state for 1 rev, return to "nominal" to prevent lock-up
            elif numStepsWaiting == steps_per_rev:
                state = "nominal"    
                numStepsWaiting = 0   
            else:
                continue
        case "wait for C burn": 
            # Increase number of steps waited
            numStepsWaiting += 1

            # velo_phase = np.arctan2(vRIC[2], vRIC[1])
            velo_phase = np.arctan2(rvRIC[1], rvRIC[2])
            
            temp = abs(abs(np.rad2deg(velo_phase)) - 90)
            # in_node_window = abs(abs(velo_phase) - np.pi /2) < np.deg2rad(20)
            
            true_lat = (truthCOE[4] + truthCOE[5]) % 360
            temp1 = diffCOEs_avg["del_raan"][-1]
            temp2 = diffCOEs_avg["del_i"][-1]
            critAngle = np.rad2deg(np.arctan(diffCOEs_avg["del_raan"][-1] / diffCOEs_avg["del_i"][-1] * refCOE[2]))
            critAngle += 360 if critAngle < 0 else 0
            if critAngle + maneuverArcHalfAngle > 360:
                in_node_window = critAngle - maneuverArcHalfAngle < true_lat or true_lat <= (critAngle + maneuverArcHalfAngle) % 360
            else:
                in_node_window = critAngle - maneuverArcHalfAngle < true_lat < critAngle + maneuverArcHalfAngle
            if in_node_window:
                thrusterAxis = "C+" if critAngle > 180 else "C-"
                
                gator_truth = truthObjs.satEnginesOn(thrusterAxis)
                state = "C burn"
                burnStarts.append((len(t) - 1, "c"))
                burn_duration = 0.0
                dt = dtThrust
                C_buffer.clear()
                C_buffer.append(rvRIC[2])
                
                numStepsWaiting = 0

                if plot_3D_RIC:
                    plotIndex = 0 if burnEnds == [] else burnEnds[-1]
                    ax_ric_traj.plot(offsetHistory[0][plotIndex:], offsetHistory[1][plotIndex:], offsetHistory[2][plotIndex:], 'b')
            elif numStepsWaiting == steps_per_rev:
                state = "nominal"
                numStepsWaiting = 0
            else:
                continue
        
        # -----------maneuvering----------------------------------------------
        case "R burn":
            burn_duration += dt
            velo_phase = np.arctan2(rvRIC[3], rvRIC[4])
            # in_cross_track_pass = abs(abs(velo_phase) - np.pi /2) < np.deg2rad(20)

            n = mean_motion
            a = truthObjs.sat_wrap.getSMAFromEnergy()
            eTruth = truthCOE[1]
            eta = np.sqrt(1 - truthCOE[1])
            deltaAOP = np.deg2rad(diffCOEs_avg["del_aop"][-1])
            deltaRAAN = np.deg2rad(diffCOEs_avg["del_raan"][-1])
            i = np.deg2rad(truthCOE[2])

            fTruth = np.deg2rad(truthCOE[5])
            ETruth = 2 * np.atan(np.sqrt((1 - eTruth) / (1 + eTruth)) * np.tan(fTruth / 2))
            MTruth = ETruth - eTruth * np.sin(ETruth)

            fRef = np.deg2rad(refCOE[5])
            ERef = 2 * np.atan(np.sqrt((1 - refCOE[1]) / (1 + refCOE[1])) * np.tan(fRef / 2))
            MRef = ERef - refCOE[1] * np.sin(ERef)
            deltaM = MTruth - MRef
            deltaVp = -n * a / 4 * ((1 + eTruth)**2 / eta**2) * (deltaAOP + deltaRAAN * np.cos(i) + deltaM)
            deltaVa = -n * a / 4 * ((1 - eTruth)**2 / eta**2) * (deltaAOP + deltaRAAN * np.cos(i) + deltaM)
                
            in_cross_track_pass = (
                            (fTruth <= maneuverArcHalfAngle or fTruth >= 360 - maneuverArcHalfAngle) or
                            (fTruth <= 180 - maneuverArcHalfAngle or fTruth >= 180 + maneuverArcHalfAngle))

            if burn_duration >= maxDutyTime or not in_cross_track_pass:
                if terminal_Completed_Firings:
                    print(f"Complete {thrusterAxis} burn duration (min) = {(burn_duration / 60):2.0f} | t = {(elapsed / 86400):2.2f} days | R-axis Amplitude = {R_amp:0.6f} | Velo phase angle = {(np.rad2deg(velo_phase)):3.2f}")
                gator_truth = truthObjs.satEnginesOff(thrusterAxis)
                state = "returning to nominal from R burn"
                burnEnds.append(len(t) - 1)
                dt = dtCoast - elapsed % dtCoast

                burn_duration = 0

                if plot_3D_RIC:
                    plotIndex = burnStarts[-1][0]
                    ax_ric_traj.plot(offsetHistory[0][plotIndex:], offsetHistory[1][plotIndex:], offsetHistory[2][plotIndex:], 'm')
                
        case "I burn":
            if thrusterAxis != "":
                burn_duration += dt

                if (burn_duration >= 660 and maneuverAttempts == 0) or maneuverAttempts > 0:
                    burnEnds.append(len(t) - 1)
                    del_a_energy_maneuver = del_a_energy

                    gator_truth = truthObjs.satEnginesOff(thrusterAxis)
                    thrusterAxis = ""

                    dt = dtCoast - elapsed % dtCoast
                    maneuverAttempts += 1
                    minIPosition = rvRIC[1]
            else:
                minIPosition = rvRIC[1] if rvRIC[1] < minIPosition else minIPosition
                coast_duration += dt
                dt = dtCoast
                if coast_duration > period_sec:
                    # Termination conditions:
                    # - achieves deadband target by the time SMA changes sign (no change)
                    # - undershoots deadband target when SMA changes sign (more thrusting)
                    # - overshoots deadband target (less thrusting)
                    if diffCOEs_avg["del_a"][-1] < 0 and -I_deadband_min * I_bounds > minIPosition >= -I_bounds:
                        if terminal_Completed_Firings:
                            terminalStr = f"t = {(t[burnStarts[-1][0]] / 86400):2.2f} days | " if (t[burnStarts[-1][0]] / 86400) >= 10 else f"t = {(t[burnStarts[-1][0]] / 86400):1.3f} days | "
                            terminalStr += f"I+ burn duration (min) = "
                            terminalStr += f"{(burn_duration / 60):2.2f} | " if (burn_duration / 60) >= 10 else f"{(burn_duration / 60):1.3f} | "
                            terminalStr += f"Recovered del_a = {(del_a_energy_maneuver):0.5f} / {(del_a_target):0.5f} | "
                            
                            accel = 0.2 / truth_Sat_wrapper.mass # m/s
                            deltaV = accel * burn_duration
                            terminalStr += f"deltaV = {deltaV:1.3f} m/s" 
                            # print(f"Complete {thrusterAxis} burn duration (min) = {(burn_duration / 60):2.3f} | t = {(elapsed / 86400):2.3f} days | Recovered del_a = {(del_a_target + del_a_energy):0.5f} out of {(2 * del_a_target):0.5f} | TA at maneuver start = {f:2.2f}")
                            print(terminalStr)
                    
                        if plot_3D_RIC:
                            plotStart = burnStarts[-1][0]
                            plotEnd = burnEnds[-1]
                            ax_ric_traj.plot(offsetHistory[0][plotStart:plotEnd], offsetHistory[1][plotStart:plotEnd], offsetHistory[2][plotStart:plotEnd], 'r') 
                        burn_duration = 0
                        # coast_duration = 0
                        state = interpruptedState
                        interpruptedState = "nominal"
                        deleteToIndex = burnEnds[-1]
                        while len(diffCOEs["del_f"]) > deleteToIndex + 1:
                            [diffCOEs[i].pop() for i in diffCOEs.keys()]
                            
                        while len(diffCOEs_avg["del_f"]) > deleteToIndex + 1:
                            [diffCOEs_avg[i].pop() for i in diffCOEs_avg.keys()]
                        
                        while len(offsetHistory[2]) > deleteToIndex + 1:
                            [offsetHistory[i].pop() for i in range(len(offsetHistory))]
                            
                        while len(t) > deleteToIndex + 1:
                            t.pop()
                        
                        for i in diffCOEs_buffer.keys():
                            for j in range(steps_to_avg+1, 1, -1):
                                diffCOEs_buffer[i].append(diffCOEs[i][-j])

                        for i in range(steps_to_avg+1, 1, -1):
                            R_amp = (max(offsetHistory[0][-(steps_to_avg+i):-i]) - min(offsetHistory[0][-(steps_to_avg+i):-i])) / 2
                            R_amp_log.append(R_amp)
                            R_buffer.append(offsetHistory[0][-i])

                            C_amp = (max(offsetHistory[2][-(steps_to_avg+i):-i]) - min(offsetHistory[2][-(steps_to_avg+i):-i])) / 2
                            C_amp_log.append(C_amp)                           
                            C_buffer.append(rvRIC[2])

                        gator_ref.Step(-coast_duration)
                        gator_truth.Step(-coast_duration)
                        elapsed = elapsed - coast_duration
                        coast_duration = 0

                        # Update numerical integrator references
                        gator_ref.UpdateSpaceObject()
                        gator_truth.UpdateSpaceObject()

                    elif diffCOEs_avg["del_a"][-1] < 0 and -I_deadband_min * I_bounds <= minIPosition:
                        # print(f"Maneuver #{maneuverAttempts}: I-position = {minIPosition:1.4}km")
                        # delete history
                        # diffcoes, diffcoes_avg, offsetHistory, t
                        deleteToIndex = burnEnds[-1]
                        while len(diffCOEs["del_f"]) > deleteToIndex + 1:
                            [diffCOEs[i].pop() for i in diffCOEs.keys()]
                            
                        while len(diffCOEs_avg["del_f"]) > deleteToIndex + 1:
                            [diffCOEs_avg[i].pop() for i in diffCOEs_avg.keys()]
                        
                        while len(offsetHistory[2]) > deleteToIndex + 1:
                            [offsetHistory[i].pop() for i in range(len(offsetHistory))]
                            
                        while len(t) > deleteToIndex + 1:
                            t.pop()

                        burnEnds.pop()
                        # Propagate spacecraft
                        gator_ref.Step(-coast_duration)
                        gator_truth.Step(-coast_duration)
                        elapsed = elapsed - coast_duration
                        coast_duration = 0

                        # Update numerical integrator references
                        gator_ref.UpdateSpaceObject()
                        gator_truth.UpdateSpaceObject()
                        
                        # Update the the elpased time

                        thrusterAxis = "I+"
                        estimateSteps = np.ceil((minIPosition + I_deadband_min * I_bounds) / 0.4) if (0 >= minIPosition) else 1
                        dt = dtThrust * estimateSteps
                        gator_truth = truthObjs.satEnginesOn(thrusterAxis)
                    elif minIPosition < -I_bounds:
                        # print(f"Maneuver #{maneuverAttempts}: I-position = {minIPosition:1.4}km | burn time = {burn_duration} sec")
                        # delete history
                        # diffcoes, diffcoes_avg, offsetHistory, t
                        deleteToIndex = burnEnds[-1] - 6
                        while len(diffCOEs["del_f"]) > deleteToIndex + 1:
                            [diffCOEs[i].pop() for i in diffCOEs.keys()]
                            
                        while len(diffCOEs_avg["del_f"]) > deleteToIndex + 1:
                            [diffCOEs_avg[i].pop() for i in diffCOEs_avg.keys()]
                        
                        while len(offsetHistory[2]) > deleteToIndex + 1:
                            [offsetHistory[i].pop() for i in range(len(offsetHistory))]
                            
                        while len(t) > deleteToIndex + 1:
                            t.pop()

                        burnEnds.pop()
                        # Propagate spacecraft
                        gator_ref.Step(-coast_duration)
                        gator_truth.Step(-coast_duration)
                        elapsed = elapsed - coast_duration
                        coast_duration = 0

                        # Update numerical integrator references
                        gator_ref.UpdateSpaceObject()
                        gator_truth.UpdateSpaceObject()
                        
                        # Update the the elpased time

                        thrusterAxis = "I+"
                        dt = dtThrust
                        gator_truth = truthObjs.satEnginesOn(thrusterAxis)
                        backPropTime = -5 * dtThrust
                        burn_duration -= 5 * dtThrust
                        gator_ref.Step(backPropTime)
                        gator_truth.Step(backPropTime)
                        # Update numerical integrator references
                        gator_ref.UpdateSpaceObject()
                        gator_truth.UpdateSpaceObject()
                        if maneuverAttempts == 25: 
                            break
                else: 
                    continue
                    
        case "C burn":
            burn_duration += dt
            velo_phase = np.arctan2(rvRIC[1], rvRIC[2])
            # in_cross_track_pass = abs(abs(velo_phase) - np.pi /2) < np.deg2rad(25)
            temp = abs(abs(np.rad2deg(velo_phase)) - 90)
            
            true_lat = (truthCOE[4] + truthCOE[5]) % 360
            critAngle = np.rad2deg(np.arctan(diffCOEs_avg["del_raan"][-1] / diffCOEs_avg["del_i"][-1] * refCOE[2]))
            critAngle += 360 if critAngle < 0 else 0
            if critAngle + maneuverArcHalfAngle > 360:
                in_cross_track_pass = critAngle - maneuverArcHalfAngle < true_lat or true_lat <= (critAngle + maneuverArcHalfAngle) % 360
            elif critAngle - maneuverArcHalfAngle < 0:
                in_cross_track_pass = (critAngle - maneuverArcHalfAngle + 360) < true_lat or true_lat <= (critAngle + maneuverArcHalfAngle) % 360
            else:
                in_cross_track_pass = critAngle - maneuverArcHalfAngle < true_lat < critAngle + maneuverArcHalfAngle
            
            if burn_duration >= maxDutyTime or not in_cross_track_pass:
                if terminal_Completed_Firings:
                    terminalStr = f"Complete {thrusterAxis} burn duration (min) = "
                    terminalStr += f"{(burn_duration / 60):2.2f} | " if (burn_duration / 60) >= 10 else f"{(burn_duration / 60):1.3f} | "
                    terminalStr += f"t = {(elapsed / 86400):2.2f} days | " if (elapsed / 86400) >= 10 else f"t = {(elapsed / 86400):1.3f} days | "
                    terminalStr += f"C-axis Amplitude = {C_amp:0.6f} km"
                    print(terminalStr)
                gator_truth = truthObjs.satEnginesOff(thrusterAxis)
                state = "returning to nominal from C burn"
                burnEnds.append(len(t) - 1)
                
                burn_duration = 0

                dt = dtCoast - elapsed % dtCoast

                if plot_3D_RIC:
                    plotIndex = burnStarts[-1][0]
                    ax_ric_traj.plot(offsetHistory[0][plotIndex:], offsetHistory[1][plotIndex:], offsetHistory[2][plotIndex:], 'c')
        
        # -----------verifying recovery---------------------------------------
        case "returning to nominal from R burn":
            R_amp_recovering = (R_amp_log[-1] <= R_amp_log[-2]) or (len(R_amp_log) < 2)
            
            if R_amp <= 1 /2 * R_bounds:
                state = interpruptedState
                interpruptedState = "nominal"
            elif not R_amp_recovering and (elapsed - t[burnEnds[-1]] > period_sec):
                state = "wait for R burn"
        case "returning to nominal from C burn":
            dt = dtCoast
            C_amp_recovering = C_amp_log[-1] <= C_amp_log[-2]
            if diffCOEs_avg["del_raan"][-1] < 0: # C_amp <= 1 /3 * C_bounds:
                state = interpruptedState
                interpruptedState = "nominal"
            else: # elif (elapsed - t[burnEnds[-1]] > .25 * period_sec): # not C_amp_recovering or
                state = "wait for C burn"       
        case _:
            continue
print(state)
# -----------plots---------------------------------------------------

if plot_3D_RIC:
    maxIndex = max(burnStarts[-1][0], burnEnds[-1])
    if state == "I burn":
        ax_ric_traj.plot(offsetHistory[0][plotIndex:], offsetHistory[1][plotIndex:], offsetHistory[2][plotIndex:], 'r') 
    elif state == "C burn":
        ax_ric_traj.plot(offsetHistory[0][plotIndex:], offsetHistory[1][plotIndex:], offsetHistory[2][plotIndex:], 'c') 
    elif state == "R burn":
        ax_ric_traj.plot(offsetHistory[0][plotIndex:], offsetHistory[1][plotIndex:], offsetHistory[2][plotIndex:], 'm')  
    else:
        ax_ric_traj.plot(offsetHistory[0][plotIndex:], offsetHistory[1][plotIndex:], offsetHistory[2][plotIndex:], 'b')  
    ax_ric_traj.set_xlabel('R (km)')
    ax_ric_traj.set_ylabel('I (km)')
    ax_ric_traj.set_zlabel('C (km)')
    ax_ric_traj.set_title('3D Trajectory of Earth Orbiter')
    ax_ric_traj.axis('equal')
    ax_ric_traj.set_title("3D RIC Positions Over Time")

# separate out the maneuvers by type
R_burns = []
I_burns = []
C_burns = []
for i in burnStarts:
    if i[1] == "m":
        R_burns.append(i[0])
    elif i[1] == "r":
        I_burns.append(i[0])
    elif i[1] == "c":
        C_burns.append(i[0])

t = np.array(t) / 86400

if plot_rRIC_v_Time:
    ax = plt.figure().add_subplot()
    ax.plot(t, offsetHistory[0], label="R")
    ax.plot(t, offsetHistory[1], label="I")
    ax.plot(t, offsetHistory[2], label="C")

    if plot_Show_Firings:
        ax.plot([t[i] for i in R_burns], [offsetHistory[0][i] for i in R_burns], "*", c="m", label="R-axis maneuver")
        ax.plot([t[i] for i in I_burns], [offsetHistory[1][i] for i in I_burns], "*", c="r", label="I-axis maneuver")
        ax.plot([t[i] for i in C_burns], [offsetHistory[2][i] for i in C_burns], "*", c="c", label="C-axis maneuver")

    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset (km)')
    ax.set_title("True Position in Reference RIC Frame vs Time")
    ax.legend()

if plot_vRIC_v_Time:
    ax = plt.figure().add_subplot()
    ax.plot(t, offsetHistory[3], label="R")
    ax.plot(t, offsetHistory[4], label="I")
    ax.plot(t, offsetHistory[5], label="C")

    if plot_Show_Firings:
        ax.plot([t[i] for i in R_burns], [offsetHistory[3][i] for i in R_burns], "*", c="m", label="R-axis maneuver")
        ax.plot([t[i] for i in I_burns], [offsetHistory[4][i] for i in I_burns], "*", c="r", label="I-axis maneuver")
        ax.plot([t[i] for i in C_burns], [offsetHistory[5][i] for i in C_burns], "*", c="c", label="C-axis maneuver")

    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset (km/sec)')
    ax.set_title("True Velocity in Reference RIC Frame vs Time")
    ax.legend()

if plot_COE_diffs["del_a"]:
    ax = plt.figure().add_subplot()
    ax.plot(t, diffCOEs["del_a"], label="del_a")
    ax.plot(t, diffCOEs_avg["del_a"], "--", label=f"{revs_to_avg} orbit average")
    
    if plot_Show_Firings:
        ax.plot([t[i] for i in R_burns], [diffCOEs_avg["del_a"][i] for i in R_burns], "*", c="m", label="R-axis maneuver")
        ax.plot([t[i] for i in I_burns], [diffCOEs_avg["del_a"][i] for i in I_burns], "*", c="r", label="I-axis maneuver")
        ax.plot([t[i] for i in C_burns], [diffCOEs_avg["del_a"][i] for i in C_burns], "*", c="c", label="C-axis maneuver")
    
    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset (km)')
    ax.set_title("Truth-Reference Differences in SMA vs Time")
    ax.legend()

if plot_COE_diffs["del_e"]:
    ax = plt.figure().add_subplot()
    ax.plot(t, diffCOEs["del_e"], label="del_e")
    ax.plot(t, diffCOEs_avg["del_e"], "--", label=f"{revs_to_avg} orbit average")
    
    if plot_Show_Firings:
        ax.plot([t[i] for i in R_burns], [diffCOEs_avg["del_e"][i] for i in R_burns], "*", c="m", label="R-axis maneuver")
        ax.plot([t[i] for i in I_burns], [diffCOEs_avg["del_e"][i] for i in I_burns], "*", c="r", label="I-axis maneuver")
        ax.plot([t[i] for i in C_burns], [diffCOEs_avg["del_e"][i] for i in C_burns], "*", c="c", label="C-axis maneuver")
    
    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset')
    ax.set_title("Truth-Reference Differences in Eccentricity vs Time")
    ax.legend()

if plot_COE_diffs["del_i"]:
    ax = plt.figure().add_subplot()
    ax.plot(t, diffCOEs["del_i"], label="del_i")
    ax.plot(t, diffCOEs_avg["del_i"], "--", label=f"{revs_to_avg} orbit average")
    
    if plot_Show_Firings:
        ax.plot([t[i] for i in R_burns], [diffCOEs_avg["del_i"][i] for i in R_burns], "*", c="m", label="R-axis maneuver")
        ax.plot([t[i] for i in I_burns], [diffCOEs_avg["del_i"][i] for i in I_burns], "*", c="r", label="I-axis maneuver")
        ax.plot([t[i] for i in C_burns], [diffCOEs_avg["del_i"][i] for i in C_burns], "*", c="c", label="C-axis maneuver")
    
    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset (deg)')
    ax.set_title("Truth-Reference Differences in Inclination vs Time")
    ax.legend()

if plot_COE_diffs["del_raan"]:
    ax = plt.figure().add_subplot()
    ax.plot(t, diffCOEs["del_raan"], label="del_raan")
    ax.plot(t, diffCOEs_avg["del_raan"], "--", label=f"{revs_to_avg} orbit average")
    
    if plot_Show_Firings:
        ax.plot([t[i] for i in R_burns], [diffCOEs_avg["del_raan"][i] for i in R_burns], "*", c="m", label="R-axis maneuver")
        ax.plot([t[i] for i in I_burns], [diffCOEs_avg["del_raan"][i] for i in I_burns], "*", c="r", label="I-axis maneuver")
        ax.plot([t[i] for i in C_burns], [diffCOEs_avg["del_raan"][i] for i in C_burns], "*", c="c", label="C-axis maneuver")
    
    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset (deg)')
    ax.set_title("Truth-Reference Differences in Right Ascension vs Time")
    ax.legend()

if plot_COE_diffs["del_aop"]:
    ax = plt.figure().add_subplot()
    ax.plot(t, diffCOEs["del_aop"], label="del_aop")
    ax.plot(t, diffCOEs_avg["del_aop"], "--", label=f"{revs_to_avg} orbit average")
    
    if plot_Show_Firings:
        ax.plot([t[i] for i in R_burns], [diffCOEs_avg["del_aop"][i] for i in R_burns], "*", c="m", label="R-axis maneuver")
        ax.plot([t[i] for i in I_burns], [diffCOEs_avg["del_aop"][i] for i in I_burns], "*", c="r", label="I-axis maneuver")
        ax.plot([t[i] for i in C_burns], [diffCOEs_avg["del_aop"][i] for i in C_burns], "*", c="c", label="C-axis maneuver")
    
    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset (deg)')
    ax.set_title("Truth-Reference Differences in Argument of Perigee vs Time")
    ax.legend()

if plot_COE_diffs["del_f"]:
    ax = plt.figure().add_subplot()
    ax.plot(t, diffCOEs["del_f"], label="del_f")
    ax.plot(t, diffCOEs_avg["del_f"], "--", label=f"{revs_to_avg} orbit average")
    
    if plot_Show_Firings:
        ax.plot([t[i] for i in R_burns], [diffCOEs_avg["del_f"][i] for i in R_burns], "*", c="m", label="R-axis maneuver")
        ax.plot([t[i] for i in I_burns], [diffCOEs_avg["del_f"][i] for i in I_burns], "*", c="r", label="I-axis maneuver")
        ax.plot([t[i] for i in C_burns], [diffCOEs_avg["del_f"][i] for i in C_burns], "*", c="c", label="C-axis maneuver")
    
    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset (deg)')
    ax.set_title("Truth-Reference Differences in True Anomaly vs Time")
    ax.legend()

if plot_True_Lat_diff:
    del_f = np.array(diffCOEs["del_f"])
    del_f_avg = np.array(diffCOEs_avg["del_f"])
    del_aop = np.array(diffCOEs["del_aop"])
    del_aop_avg = np.array(diffCOEs_avg["del_aop"])

    del_theta = del_f + del_aop
    del_theta_avg = del_f_avg + del_aop_avg

    ax = plt.figure().add_subplot()
    ax.plot(t, del_theta, label="del_theta")
    ax.plot(t, del_theta_avg, "--", label=f"{revs_to_avg} orbit average")
    
    if plot_Show_Firings:
        ax.plot([t[i] for i in R_burns], [del_theta_avg[i] for i in R_burns], "*", c="m", label="R-axis maneuver")
        ax.plot([t[i] for i in I_burns], [del_theta_avg[i] for i in I_burns], "*", c="r", label="I-axis maneuver")
        ax.plot([t[i] for i in C_burns], [del_theta_avg[i] for i in C_burns], "*", c="c", label="C-axis maneuver")
        
    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset (deg)')
    ax.set_title("Truth-Reference Differences in True Latitude vs Time")
    ax.legend()

if any([
    plot_3D_RIC,
    plot_rRIC_v_Time,
    plot_vRIC_v_Time,
    plot_COE_diffs.items()
    ]):
    
    plt.show()
