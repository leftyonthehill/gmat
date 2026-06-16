
from collections import deque
from createStationKeepingObjects import StationKeepingObjects
from load_gmat import *
from matplotlib import pyplot as plt
from xyz2ric import xyz2ric

import numpy as np

# -----------user defined variables----------------------------------
"""Please change the following variables as necessary to shape your scenario"""

# Duration of the scenario in days
maxDays = 7

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
R_bounds = 2
I_bounds = 20
C_bounds = 2

# Maximum thruster duty time in seconds
maxDutyTime = 3600

I_deadband_min = 0.85

# 
maneuverArcHalfAngle = 20

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

plot_rRIC_Amp_v_Time = True

plot_vRIC_v_Time = False

plot_vRIC_Amp_v_Time = False

plot_COE_diffs = {
    "del_a": False,
    "del_e": True,
    "del_i": False,
    "del_raan": False,
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
totalDeltaV = 0
t = [0] # array to hold each time step

# stores the RIC history of the truth spacecraft about the reference spacecraft 
RIC_History = {
    "R": {0: 0},
    "I": {0: 0},
    "C": {0: 0},
    "R_dot": {0: 0},
    "I_dot": {0: 0},
    "C_dot": {0: 0},
}

RIC_Amp_History = {
    "R": {0: 0},
    "I": {0: 0},
    "C": {0: 0},
    "R_dot": {0: 0},
    "I_dot": {0: 0},
    "C_dot": {0: 0},
}

RIC_Amp_Buffer = {
    i: deque([0], maxlen=int(1.5 * steps_per_rev)) for i in RIC_History
}

RIC_keys = [*RIC_Amp_Buffer.keys()]

# Storage of the differences in the orbital elements throughout the scenario
diffCOEs_dict = {
    "del_a": {0: 0},
    "del_e": {0: 0},
    "del_i": {0: 0},
    "del_raan": {0: 0},
    "del_aop": {0: 0},
    "del_f": {0: 0}
}

# Storage of the average difference for each orbital element after each time step
diffCOEs_avg = {
    "del_a": {0: 0},
    "del_e": {0: 0},
    "del_i": {0: 0},
    "del_raan": {0: 0},
    "del_aop": {0: 0},
    "del_f": {0: 0}
}

# Storage of the orbital element differences across three rev about Earth
diffCOEs_buffer = {
    i: deque([0], maxlen=int(steps_to_avg)) for i in diffCOEs_dict
}

COEs_keys = [*diffCOEs_dict.keys()]

# Variables used to help complete I-axis maneuvers
del_a_target = 0 # desired increase in the difference of semi-major axis between the truth and reference satellites
del_a_recovered = False
maneuverAttempts = 0
maneuverLog = []
# Log of maneuver times
burnStarts = []
burnEnds = []

# Timer to prevent controller from being stuck waiting to maneuver
numStepsWaiting = 0

# Initial state
state = "nominal"
interpruptedState = "nominal"
# -----------configuration preliminaries-----------------------------
"""Generate the necessary GMAT objects and Python wrappers"""
# Reference Objectes
refObjs = StationKeepingObjects("reference")
refObjs.setSatCOEs(orbitParam)

# Truth Objects
truthObjs = StationKeepingObjects("truth")
truthObjs.setSatCOEs(orbitParam)
truthObjs.setManeuverable()

# Initialize the scenario
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
# Set simulation step size
dt = dtCoast

# Get initial integrator for the truth satellite
gator_truth = truthObjs.satEnginesOff()
temp1234 = [[],[]]
# While the elapsed time is less the max number of days
while elapsed / 86400 < maxDays: 
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

    # Store latest RIC position vector
    for j in range(6):
        RIC_History[RIC_keys[j]][elapsed] = rvRIC[j]
        
        if state == "I burn" and thrusterAxis != "":
            # Only update the 1 rev buffers when elpased is a multiple of dtCoast
            if round(elapsed % dtCoast) == 0:
                RIC_Amp_Buffer[RIC_keys[j]].append(rvRIC[j])

            amp = (max(RIC_Amp_Buffer[RIC_keys[j]]) - min(RIC_Amp_Buffer[RIC_keys[j]])) / 2 if len(RIC_Amp_Buffer[RIC_keys[j]]) > 1 else rvRIC[j]
            RIC_Amp_History[RIC_keys[j]][elapsed] = amp
    
    # Compute and store the differences in each of the 6 keplerian elements 
    for j in range(6):
        diff = float(truthCOE[j] - refCOE[j])

        if j > 1 and diff > 180:
            diff = diff - 360
        elif j > 1 and diff < -180:
            diff = diff + 360
        diffCOEs_dict[COEs_keys[j]][elapsed] = diff # instaneous differences
        diffCOEs_buffer[COEs_keys[j]].append(diff) # instaneous differences within the past rev
        diffCOEs_avg[COEs_keys[j]][elapsed] = np.mean(diffCOEs_buffer[COEs_keys[j]]) # average differences across one rev

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

    interpruptManeuver = {
        "R": RIC_Amp_History["R"][elapsed] > R_bounds,
        "I": rvRIC[1] > I_deadband_min * I_bounds,
        "C": RIC_Amp_History["C"][elapsed] > C_bounds,
    }

    if interpruptManeuver["I"] and state not in ["wait for I burn", "R burn", "I burn", "C burn"]:
        interpruptedState = state
        state = "wait for I burn"
    elif interpruptManeuver["C"] and state not in ["wait for I burn", "wait for C burn", "R burn", "I burn", "C burn", "returning to nominal from C burn"] and interpruptedState == "nominal":
        interpruptedState = state
        state = "wait for C burn"
    elif interpruptManeuver["R"] and state not in ["wait for R burn", "wait for I burn", "wait for C burn", "R burn", "I burn", "C burn", "returning to nominal from R burn", "returning to nominal from C burn"] and interpruptedState == "nominal":
        interpruptedState = state
        state = "wait for R burn"

    match state:
        # -----------waiting for maneuver opoprtunties------------------------
        case "wait for R burn":
            # Increase number of steps waited
            numStepsWaiting += 1
            
            f = truthCOE[-1]
            in_node_window = not(
                                maneuverArcHalfAngle < f <= 180 - maneuverArcHalfAngle) and \
                            not(
                                180 + maneuverArcHalfAngle < f <= 360 - maneuverArcHalfAngle)
            
            if -0.2 <= rvRIC[0] < 0.2:
                tempStr = f"t = {(t[burnStarts[-1][0]]):2.2f} days | " if (t[burnStarts[-1][0]]) >= 10 else f"t = {(t[burnStarts[-1][0]]):1.3f} days | "
                tempStr += f"Y-AXIS Cross: R = {rvRIC[0]:1.3f} km | True lat = {(truthCOE[-2] + truthCOE[-1]) % 360} deg | "
                tempStr += f"omega = {truthCOE[-2]} deg | f = {truthCOE[-1]} deg"
                print(tempStr)
            # Target thrust window has a phase angle of 15 -> 0 -> -20 deg
            if in_node_window:
                n = mean_motion
                a = truthObjs.sat_wrap.getSMAFromEnergy()
                eTruth = truthCOE[1]
                eta = np.sqrt(1 - truthCOE[1])
                deltaAOP = np.deg2rad(diffCOEs_avg["del_aop"][elapsed])
                deltaRAAN = np.deg2rad(diffCOEs_avg["del_raan"][elapsed])
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
                
                # Reset the step counter
                numStepsWaiting = 0                
            
            # If it has been one full rev since spacecraft entered this state, return to nominal to prevent a lock-up
            elif numStepsWaiting == steps_per_rev:
                state = interpruptedState
                interpruptedState = "nominal"
                numStepsWaiting = 0
            else:
                continue
        case "wait for I burn":
            # Increase number of steps waited
            numStepsWaiting += 1

            # Collect the current True Anomaly value to see if the spacecraft is in the appropriate window for a maneuver
            f = truthCOE[-1]

            in_burn_window = 160 < f <= 180
            del_a_target = max(abs(del_a_energy), abs(diffCOEs_avg["del_a"][elapsed]))

            # Possibility for controller to trigger a maneuver when spacecraft is within user-defined bounds, this check prevents that
            subset = [diffCOEs_avg["del_a"][i] for i in t[-10 * steps_per_rev:]]
            decayRate, _ = np.polyfit(t[-10 * steps_per_rev:], subset,  1)
            del subset

            isNegativeSMA = diffCOEs_avg["del_a"][elapsed] < 0
            isNegativeSMATrend = (diffCOEs_avg["del_a"][elapsed] - diffCOEs_avg["del_a"][t[-10 * steps_per_rev]]) < 0
            isInDeadBand = rvRIC[1] > I_deadband_min * I_bounds
            valid_burn = isNegativeSMA and isNegativeSMATrend and isInDeadBand
            
            if len(burnStarts) > 0:
                if burnStarts[-1][1] != "r":
                    recentManeuver = True
                else:
                    recentManeuver = (elapsed - (t[burnEnds[-1]]) >= 3 * period_sec)
            else:
                recentManeuver = True

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
            
            # If waiting in this state for 1 rev, return to "nominal" to prevent lock-up
            elif numStepsWaiting == steps_per_rev:
                state = interpruptedState
                interpruptedState = "nominal"    
                numStepsWaiting = 0   
            else:
                continue
        case "wait for C burn": 
            # Increase number of steps waited
            numStepsWaiting += 1
            
            true_lat = (truthCOE[4] + truthCOE[5]) % 360
            critAngle = np.rad2deg(np.arctan(diffCOEs_avg["del_raan"][elapsed] / diffCOEs_avg["del_i"][elapsed] * refCOE[2]))
            critAngle += 360 if critAngle < 0 else 0
            if critAngle + maneuverArcHalfAngle > 360:
                in_node_window = critAngle - maneuverArcHalfAngle < true_lat or true_lat <= (critAngle + maneuverArcHalfAngle) % 360
            else:
                in_node_window = critAngle - maneuverArcHalfAngle < true_lat < critAngle + maneuverArcHalfAngle
            if in_node_window:
                thrusterAxis = "C-" if critAngle > 180 else "C+"
                
                gator_truth = truthObjs.satEnginesOn(thrusterAxis)
                state = "C burn"
                burnStarts.append((len(t) - 1, "c"))
                burn_duration = 0.0
                dt = dtThrust
                
                numStepsWaiting = 0

            elif numStepsWaiting == steps_per_rev:
                state = interpruptedState
                interpruptedState = "nominal"
                numStepsWaiting = 0
            else:
                continue
        
        # -----------maneuvering----------------------------------------------
        case "R burn":
            
            if -0.2 <= rvRIC[0] < 0.2:
                tempStr = f"t = {(t[burnStarts[-1][0]] / 86400):2.2f} days | " if (t[burnStarts[-1][0]] / 86400) >= 10 else f"t = {(t[burnStarts[-1][0]] / 86400):1.3f} days | "
                tempStr += f"Y-AXIS Cross: R = {rvRIC[0]:1.3f} km | True lat = {(truthCOE[-2] + truthCOE[-1]) % 360} deg | "
                tempStr += f"omega = {truthCOE[-2]} deg | f = {truthCOE[-1]} deg"
                print(tempStr)

            burn_duration += dt
            velo_phase = np.arctan2(rvRIC[3], rvRIC[4])
            # in_cross_track_pass = abs(abs(velo_phase) - np.pi /2) < np.deg2rad(20)

            n = mean_motion
            a = truthObjs.sat_wrap.getSMAFromEnergy()
            eTruth = truthCOE[1]
            eta = np.sqrt(1 - truthCOE[1])
            deltaAOP = np.deg2rad(diffCOEs_avg["del_aop"][elapsed])
            deltaRAAN = np.deg2rad(diffCOEs_avg["del_raan"][elapsed])
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
            
            in_node_window = not(
                                maneuverArcHalfAngle < f <= 180 - maneuverArcHalfAngle) and \
                            not(
                                180 + maneuverArcHalfAngle < f <= 360 - maneuverArcHalfAngle)
            
            if burn_duration >= maxDutyTime or not in_node_window:
                if terminal_Completed_Firings:
                    terminalStr = f"t = {(t[burnStarts[-1][0]] / 86400):2.2f} days | " if (t[burnStarts[-1][0]] / 86400) >= 10 else f"t = {(t[burnStarts[-1][0]] / 86400):1.3f} days | "
                    terminalStr += f"{thrusterAxis} burn duration (min) = "
                    terminalStr += f"{(burn_duration / 60):2.2f} | " if (burn_duration / 60) >= 10 else f"{(burn_duration / 60):1.3f} | "
                    terminalStr += f"R-axis Amplitude = {RIC_Amp_History["R"][elapsed]:0.6f} km         | "
                    
                    accel = 0.2 / truth_Sat_wrapper.mass # m/s
                    deltaV = accel * burn_duration
                    totalDeltaV += deltaV
                    terminalStr += f"deltaV = {deltaV:1.3f} m/s | " 
                    terminalStr += f"total deltaV = {totalDeltaV:1.3f} m/s"
                    print(terminalStr)
                gator_truth = truthObjs.satEnginesOff(thrusterAxis)
                state = "returning to nominal from R burn"
                burnEnds.append(len(t) - 1)
                dt = dtCoast - round(elapsed % dtCoast)

                burn_duration = 0
                
        case "I burn":
            
            if thrusterAxis != "":
                burn_duration += dt

                if (burn_duration >= 600 and maneuverAttempts == 0) or maneuverAttempts > 0:
                    burnEnds.append(len(t) - 1)
                    del_a_energy_maneuver = del_a_energy

                    gator_truth = truthObjs.satEnginesOff(thrusterAxis)
                    thrusterAxis = ""

                    dt = dtCoast - round(elapsed % dtCoast)
                    maneuverAttempts += 1
                    minIPosition = rvRIC[1]
            else:
                minIPosition = rvRIC[1] if rvRIC[1] < minIPosition else minIPosition
                coast_duration += dt
                dt = dtCoast
                temp = (diffCOEs_avg["del_a"][t[-1]] - diffCOEs_avg["del_a"][t[-2]]) / dt
                if coast_duration > period_sec:
                    # Termination conditions:
                    # - achieves deadband target by the time SMA changes sign (no change)
                    # - undershoots deadband target when SMA changes sign (more thrusting)
                    # - overshoots deadband target (less thrusting)
                    
                    # achieves deadband target by the time SMA changes sign (no change)
                    if diffCOEs_avg["del_a"][elapsed] < 0 and -I_deadband_min * I_bounds > minIPosition >= -I_bounds:
                        if terminal_Completed_Firings:
                            terminalStr = f"t = {(t[burnStarts[-1][0]] / 86400):2.2f} days | " if (t[burnStarts[-1][0]] / 86400) >= 10 else f"t = {(t[burnStarts[-1][0]] / 86400):1.3f} days | "
                            terminalStr += f"I+ burn duration (min) = "
                            terminalStr += f"{(burn_duration / 60):2.2f} | " if (burn_duration / 60) >= 10 else f"{(burn_duration / 60):1.3f} | "
                            terminalStr += f"Recovered del_a = {(del_a_energy_maneuver):0.5f} / {(del_a_target):0.5f} km | "
                            
                            accel = 0.2 / truth_Sat_wrapper.mass # m/s
                            deltaV = accel * burn_duration
                            totalDeltaV += deltaV
                            terminalStr += f"deltaV = {deltaV:1.3f} m/s | " 
                            terminalStr += f"total deltaV = {totalDeltaV:1.3f} m/s"
                            print(terminalStr)
                        
                        state = interpruptedState
                        interpruptedState = "nominal"
                        
                        for j in COEs_keys:
                            COEs_restore_history = {k:v for k, v in diffCOEs_dict[j].items() if t[burnEnds[-1] - steps_to_avg] < k <= t[burnEnds[-1]]}
                            diffCOEs_buffer[j] = deque([*COEs_restore_history.values()], maxlen=int(steps_to_avg))

                        for j in RIC_keys:
                            RIC_restore_history = {k:v for k, v in RIC_History[j].items() if t[burnEnds[-1] - steps_to_avg] < k <= t[burnEnds[-1]] and k % dtCoast == 0}
                            RIC_Amp_Buffer[j] = deque([*RIC_restore_history.values()], maxlen=int(1.5 * steps_per_rev))

                        t = t[:burnEnds[-1] + 1]

                        gator_ref.Step(-coast_duration)
                        gator_truth.Step(-coast_duration)
                        elapsed = elapsed - coast_duration
                        
                        burn_duration = 0
                        coast_duration = 0
                        maneuverLog = []
                        
                        # Update numerical integrator references
                        gator_ref.UpdateSpaceObject()
                        gator_truth.UpdateSpaceObject()

                    # undershoots deadband target when SMA changes sign (more thrusting)    
                    elif diffCOEs_avg["del_a"][elapsed] < 0 and (diffCOEs_avg["del_a"][elapsed] - diffCOEs_avg["del_a"][t[-10 * steps_per_rev]]) < 0 and -I_deadband_min * I_bounds <= minIPosition:
                        # print(f"Maneuver #{maneuverAttempts}: I-position = {minIPosition:1.4}km | burn time = {burn_duration} sec")
                        for j in COEs_keys:
                            COEs_restore_history = {k:v for k, v in diffCOEs_dict[j].items() if t[burnEnds[-1] - steps_to_avg] < k <= t[burnEnds[-1]]}
                            diffCOEs_buffer[j] = deque([*COEs_restore_history.values()], maxlen=int(steps_to_avg))

                        for j in RIC_keys:
                            RIC_restore_history = {k:v for k, v in RIC_History[j].items() if t[burnEnds[-1] - steps_to_avg] < k <= t[burnEnds[-1]] and k % dtCoast == 0}
                            RIC_Amp_Buffer[j] = deque([*RIC_restore_history.values()], maxlen=int(1.5 * steps_per_rev))

                        t = t[:burnEnds[-1] + 1]
                        
                        # Propagate spacecraft
                        gator_ref.Step(-coast_duration)
                        gator_truth.Step(-coast_duration)
                        elapsed = elapsed - coast_duration
                        coast_duration = 0

                        # Update numerical integrator references
                        gator_ref.UpdateSpaceObject()
                        gator_truth.UpdateSpaceObject()
                        
                        burnEnds.pop()
                        # Update the the elpased time

                        thrusterAxis = "I+"
                        estimateSteps = 1 # np.ceil((minIPosition + I_deadband_min * I_bounds) / 0.4) if (0 >= minIPosition) else 1
                        dt = dtThrust * estimateSteps
                        if burn_duration + dt in maneuverLog: 
                            estimateSteps -=1
                            dt = dtThrust * estimateSteps
                        maneuverLog.append(burn_duration)
                        gator_truth = truthObjs.satEnginesOn(thrusterAxis)
                        
                        if burn_duration + dt < 0:
                            print(f"Negative thrust time!")
                            burnStarts.pop()
                            break
                        if maneuverAttempts == 100:
                            print(f"Max burns!  current burn duration = {burn_duration} sec")
                            burnStarts.pop()
                            break
                        
                    # overshoots deadband target (less thrusting)
                    elif diffCOEs_avg["del_a"][elapsed] < 0 and (diffCOEs_avg["del_a"][elapsed] - diffCOEs_avg["del_a"][t[-10 * steps_per_rev]]) < 0 and minIPosition < -I_bounds:
                        print(f"Maneuver #{maneuverAttempts}: I-position = {minIPosition:1.4}km | burn time = {burn_duration} sec")
                        stepsToBackTrack = 5
                        if burn_duration - dtThrust * stepsToBackTrack in maneuverLog: 
                            stepsToBackTrack -=1
                        maneuverLog.append(burn_duration)
                        
                        deleteToIndex = burnEnds[-1] - stepsToBackTrack - 1
                        for k_super, v_dict in diffCOEs_dict.items():
                            for k_sub, v in v_dict.items():
                                if t[deleteToIndex] <= k_sub < t[burnEnds[-1]] and k_sub % dtCoast != 0:
                                    del diffCOEs_dict[k_super][k_sub]
                                    del diffCOEs_avg[k_super][k_sub]
                        
                        for k_super, v_dict in RIC_History.items():
                            for k_sub, v in v_dict.items():
                                if t[deleteToIndex] <= k_sub < t[burnEnds[-1]] and k_sub % dtCoast != 0:
                                    del RIC_History[k_super][k_sub]
                                    del RIC_Amp_History[k_super][k_sub]
                        
                        for j in COEs_keys:
                            COEs_restore_history = {k:v for k, v in diffCOEs_dict[j].items() if t[burnEnds[-1] - steps_to_avg] < k <= t[burnEnds[-1]]}
                            diffCOEs_buffer[j] = deque([*COEs_restore_history.values()], maxlen=int(steps_to_avg))

                        for j in RIC_keys:
                            RIC_restore_history = {k:v for k, v in RIC_History[j].items() if t[burnEnds[-1] - steps_to_avg] < k <= t[burnEnds[-1]] and k % dtCoast == 0}
                            RIC_Amp_Buffer[j] = deque([*RIC_restore_history.values()], maxlen=int(1.5 * steps_per_rev))

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
                        backPropTime = -stepsToBackTrack * dtThrust
                        burn_duration -= stepsToBackTrack * dtThrust
                        gator_ref.Step(backPropTime)
                        gator_truth.Step(backPropTime)
                        
                        # Update numerical integrator references
                        gator_ref.UpdateSpaceObject()
                        gator_truth.UpdateSpaceObject()
                        if burn_duration + dt <= 0:
                            print(f"Negative thrust time! Min I = {minIPosition}")
                            burnStarts.pop()
                            break
                        if maneuverAttempts == 100:
                            print(f"Max burns! current burn duration = {burn_duration} sec | Min I = {minIPosition}")
                            burnStarts.pop()
                            break
                else: 
                    continue
                    
        case "C burn":
            burn_duration += dt
            velo_phase = np.arctan2(rvRIC[1], rvRIC[2])
            
            true_lat = (truthCOE[4] + truthCOE[5]) % 360
            if critAngle + maneuverArcHalfAngle > 360:
                in_cross_track_pass = critAngle - maneuverArcHalfAngle < true_lat or true_lat <= (critAngle + maneuverArcHalfAngle) % 360
            elif critAngle - maneuverArcHalfAngle < 0:
                in_cross_track_pass = (critAngle - maneuverArcHalfAngle + 360) < true_lat or true_lat <= (critAngle + maneuverArcHalfAngle) % 360
            else:
                in_cross_track_pass = critAngle - maneuverArcHalfAngle < true_lat < critAngle + maneuverArcHalfAngle
            
            if burn_duration >= maxDutyTime or not in_cross_track_pass:
                if terminal_Completed_Firings:
                    terminalStr = f"t = {(t[burnStarts[-1][0]] / 86400):2.2f} days | " if (t[burnStarts[-1][0]] / 86400) >= 10 else f"t = {(t[burnStarts[-1][0]]) / 86400:1.3f} days | "
                    terminalStr += f"{thrusterAxis} burn duration (min) = "
                    terminalStr += f"{(burn_duration / 60):2.2f} | " if (burn_duration / 60) >= 10 else f"{(burn_duration / 60):1.3f} | "
                    terminalStr += f"C-axis Amplitude = {RIC_Amp_History["C"][elapsed]:0.6f} km         | "
                    
                    accel = 0.2 / truth_Sat_wrapper.mass # m/s
                    deltaV = accel * burn_duration
                    totalDeltaV += deltaV
                    terminalStr += f"deltaV = {deltaV:1.3f} m/s | " 
                    terminalStr += f"total deltaV = {totalDeltaV:1.3f} m/s"
                    print(terminalStr)

                gator_truth = truthObjs.satEnginesOff(thrusterAxis)
                thrusterAxis = ""
                state = "returning to nominal from C burn"
                burnEnds.append(len(t) - 1)
                
                burn_duration = 0

                dt = dtCoast - round(elapsed % dtCoast)
        
        # -----------verifying recovery---------------------------------------
        case "returning to nominal from R burn":
            R_amp_recovering = (RIC_Amp_History["R"][elapsed] < RIC_Amp_History["R"][t[-2]]) and (len(RIC_Amp_History["R"]) > 2)
            
            if -0.2 <= rvRIC[0] < 0.2:
                tempStr = f"t = {(t[burnStarts[-1][0]]):2.2f} days | " if (t[burnStarts[-1][0]]) >= 10 else f"t = {(t[burnStarts[-1][0]]):1.3f} days | "
                tempStr += f"Y-AXIS Cross: R = {rvRIC[0]:1.3f} km | True lat = {(truthCOE[-2] + truthCOE[-1]) % 360} deg"
                tempStr += f"omega = {truthCOE[-2]} deg | f = {truthCOE[-1]} deg"
                print(tempStr)
            if RIC_Amp_History["R"][elapsed] <= 1 /2 * R_bounds:
                state = interpruptedState
                interpruptedState = "nominal"
            elif not R_amp_recovering and (elapsed - t[burnEnds[-1]] > 0.25*period_sec):
                state = "wait for R burn"
                break
        case "returning to nominal from C burn":
            break
            dt = dtCoast
            C_amp_recovering = RIC_Amp_History["C"][elapsed] <= RIC_Amp_History["C"][t[-2]]
            if diffCOEs_avg["del_raan"][-1] < 0: # C_amp <= 1 /3 * C_bounds:
                state = interpruptedState
                interpruptedState = "nominal"
            elif (elapsed - t[burnEnds[-1]] > .75 * period_sec): # not C_amp_recovering or
                state = "wait for C burn"      
            else:
                continue 
        case _:
            continue
print(state)
# -----------plots---------------------------------------------------
t = np.array(t) / 86400
for i in COEs_keys:
    diffCOEs_dict[i] = {k / 86400: v for k, v in sorted(diffCOEs_dict[i].items())}
    diffCOEs_avg[i] = {k / 86400: v for k, v in sorted(diffCOEs_avg[i].items())}

for i in RIC_keys:
    RIC_History[i] = {k / 86400: v for k, v in sorted(RIC_History[i].items())}
    RIC_Amp_History[i] = {k / 86400: v for k, v in sorted(RIC_Amp_History[i].items())}


if plot_3D_RIC:
    ax_ric_traj = plt.figure().add_subplot(projection='3d')
    if len(burnStarts) > 0:
        burnEnds.append(0)
        for i in range(len(burnStarts)):
            R = {k:v for k, v in RIC_History["R"].items() if t[burnEnds[i-1]] <= k < t[burnStarts[i][0]]}
            I = {k:v for k, v in RIC_History["I"].items() if t[burnEnds[i-1]] <= k < t[burnStarts[i][0]]}
            C = {k:v for k, v in RIC_History["C"].items() if t[burnEnds[i-1]] <= k < t[burnStarts[i][0]]}
            ax_ric_traj.plot([*R.values()], [*I.values()], [*C.values()], 'b')

            R = {k:v for k, v in RIC_History["R"].items() if t[burnStarts[i][0]] <= k < t[burnEnds[i]]}
            I = {k:v for k, v in RIC_History["I"].items() if t[burnStarts[i][0]] <= k < t[burnEnds[i]]}
            C = {k:v for k, v in RIC_History["C"].items() if t[burnStarts[i][0]] <= k < t[burnEnds[i]]}
            ax_ric_traj.plot([*R.values()], [*I.values()], [*C.values()], burnStarts[i][1])
            
            if i == 0 and len(burnStarts) == len(burnEnds):
                burnEnds[-1] = t[-1]

        if len(burnStarts) != len(burnEnds):
            R = {k:v for k, v in RIC_History["R"].items() if t[burnEnds[-2]] <= k}
            I = {k:v for k, v in RIC_History["I"].items() if t[burnEnds[-2]] <= k}
            C = {k:v for k, v in RIC_History["C"].items() if t[burnEnds[-2]] <= k}
            ax_ric_traj.plot([*R.values()], [*I.values()], [*C.values()], 'b')
    else:
        R = {k:v for k, v in RIC_History["R"].items()}
        I = {k:v for k, v in RIC_History["I"].items()}
        C = {k:v for k, v in RIC_History["C"].items()}
        ax_ric_traj.plot([*R.values()], [*I.values()], [*C.values()], 'b')

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

if plot_rRIC_v_Time:
    ax = plt.figure().add_subplot()
    ax.plot(RIC_History["R"].keys(), RIC_History["R"].values(), label="R")
    ax.plot(RIC_History["I"].keys(), RIC_History["I"].values(), label="I")
    ax.plot(RIC_History["C"].keys(), RIC_History["C"].values(), label="C")

    if plot_Show_Firings and len(burnStarts) > 0:
        ax.plot([t[i] for i in R_burns], [RIC_History["R"][t[i]] for i in R_burns], "*", c="m", label="R-axis maneuver")
        ax.plot([t[i] for i in I_burns], [RIC_History["I"][t[i]] for i in I_burns], "*", c="r", label="I-axis maneuver")
        ax.plot([t[i] for i in C_burns], [RIC_History["C"][t[i]] for i in C_burns], "*", c="c", label="C-axis maneuver")

    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset (km)')
    ax.set_title("True Position in Reference RIC Frame vs Time")
    ax.legend()

if plot_rRIC_Amp_v_Time:
    ax = plt.figure().add_subplot()
    ax.plot(RIC_Amp_History["R"].keys(), RIC_Amp_History["R"].values(), label="R")
    ax.plot(RIC_Amp_History["I"].keys(), RIC_Amp_History["I"].values(), label="I")
    ax.plot(RIC_Amp_History["C"].keys(), RIC_Amp_History["C"].values(), label="C")

    if plot_Show_Firings and len(burnStarts) > 0:
        ax.plot([t[i] for i in R_burns], [RIC_Amp_History["R"][t[i]] for i in R_burns], "*", c="m", label="R-axis maneuver")
        ax.plot([t[i] for i in I_burns], [RIC_Amp_History["I"][t[i]] for i in I_burns], "*", c="r", label="I-axis maneuver")
        ax.plot([t[i] for i in C_burns], [RIC_Amp_History["C"][t[i]] for i in C_burns], "*", c="c", label="C-axis maneuver")

    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset (km)')
    ax.set_title("Oscillation Amplitude of Position in RIC Frame vs Time")
    ax.legend()

if plot_vRIC_v_Time:
    ax = plt.figure().add_subplot()
    ax.plot(RIC_History["R_dot"].keys(), RIC_History["R_dot"].values(), label="R_dot")
    ax.plot(RIC_History["I_dot"].keys(), RIC_History["I_dot"].values(), label="I_dot")
    ax.plot(RIC_History["C_dot"].keys(), RIC_History["C_dot"].values(), label="C_dot")

    if plot_Show_Firings and len(burnStarts) > 0:
        R = {k:v for k, v in RIC_History["R_dot"].items() if k in burnStarts[:][0]}
        I = {k:v for k, v in RIC_History["I_dot"].items() if k in burnStarts[:][0]}
        C = {k:v for k, v in RIC_History["C_dot"].items() if k in burnStarts[:][0]}
        ax.plot(R.keys(), R.values(), "*", c="m", label="R-axis maneuver")
        ax.plot(I.keys(), I.values(), "*", c="r", label="I-axis maneuver")
        ax.plot(C.keys(), C.values(), "*", c="c", label="C-axis maneuver")

    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset (km/sec)')
    ax.set_title("True Velocity in Reference RIC Frame vs Time")
    ax.legend()

if plot_vRIC_Amp_v_Time:
    ax = plt.figure().add_subplot()
    ax.plot(RIC_Amp_History["R_dot"].keys(), RIC_Amp_History["R_dot"].values(), label="R_dot")
    ax.plot(RIC_Amp_History["I_dot"].keys(), RIC_Amp_History["I_dot"].values(), label="I_dot")
    ax.plot(RIC_Amp_History["C_dot"].keys(), RIC_Amp_History["C_dot"].values(), label="C_dot")

    if plot_Show_Firings and len(burnStarts) > 0:
        R = {k:v for k, v in RIC_Amp_History["R_dot"].items() if k in burnStarts[:][0]}
        I = {k:v for k, v in RIC_Amp_History["I_dot"].items() if k in burnStarts[:][0]}
        C = {k:v for k, v in RIC_Amp_History["C_dot"].items() if k in burnStarts[:][0]}
        ax.plot(R.keys(), R.values(), "*", c="m", label="R-axis maneuver")
        ax.plot(I.keys(), I.values(), "*", c="r", label="I-axis maneuver")
        ax.plot(C.keys(), C.values(), "*", c="c", label="C-axis maneuver")

    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset (km/sec)')
    ax.set_title("True Velocity in Reference RIC Frame vs Time")
    ax.legend()

if plot_COE_diffs["del_a"]:
    ax = plt.figure().add_subplot()
    ax.plot([*diffCOEs_dict["del_a"].keys()], [*diffCOEs_dict["del_a"].values()], label="del_a")
    ax.plot([*diffCOEs_avg["del_a"].keys()], [*diffCOEs_avg["del_a"].values()], "--", label=f"{revs_to_avg} orbit average")
    
    if plot_Show_Firings and len(burnStarts) > 0:
        ax.plot([t[i] for i in R_burns], [diffCOEs_avg["del_a"][t[i]] for i in R_burns], "*", c="m", label="R-axis maneuver")
        ax.plot([t[i] for i in I_burns], [diffCOEs_avg["del_a"][t[i]] for i in I_burns], "*", c="r", label="I-axis maneuver")
        ax.plot([t[i] for i in C_burns], [diffCOEs_avg["del_a"][t[i]] for i in C_burns], "*", c="c", label="C-axis maneuver")
    
    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset (km)')
    ax.set_title("Truth-Reference Differences in SMA vs Time")
    ax.legend()

if plot_COE_diffs["del_e"]:
    ax = plt.figure().add_subplot()
    ax.plot(diffCOEs_dict["del_e"].keys(), diffCOEs_dict["del_e"].values(), label="del_e")
    ax.plot(diffCOEs_avg["del_e"].keys(), diffCOEs_avg["del_e"].values(), "--", label=f"{revs_to_avg} orbit average")
    
    if plot_Show_Firings and len(burnStarts) > 0:
        ax.plot([t[i] for i in R_burns], [diffCOEs_avg["del_e"][t[i]] for i in R_burns], "*", c="m", label="R-axis maneuver")
        ax.plot([t[i] for i in I_burns], [diffCOEs_avg["del_e"][t[i]] for i in I_burns], "*", c="r", label="I-axis maneuver")
        ax.plot([t[i] for i in C_burns], [diffCOEs_avg["del_e"][t[i]] for i in C_burns], "*", c="c", label="C-axis maneuver")
    
    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset')
    ax.set_title("Truth-Reference Differences in Eccentricity vs Time")
    ax.legend()

if plot_COE_diffs["del_i"]:
    ax = plt.figure().add_subplot()
    ax.plot(diffCOEs_dict["del_i"].keys(), diffCOEs_dict["del_i"].values(), label="del_i")
    ax.plot(diffCOEs_avg["del_i"].keys(), diffCOEs_avg["del_i"].values(), "--", label=f"{revs_to_avg} orbit average")
    
    if plot_Show_Firings and len(burnStarts) > 0:
        ax.plot([t[i] for i in R_burns], [diffCOEs_avg["del_i"][t[i]] for i in R_burns], "*", c="m", label="R-axis maneuver")
        ax.plot([t[i] for i in I_burns], [diffCOEs_avg["del_i"][t[i]] for i in I_burns], "*", c="r", label="I-axis maneuver")
        ax.plot([t[i] for i in C_burns], [diffCOEs_avg["del_i"][t[i]] for i in C_burns], "*", c="c", label="C-axis maneuver")
    
    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset (deg)')
    ax.set_title("Truth-Reference Differences in Inclination vs Time")
    ax.legend()

if plot_COE_diffs["del_raan"]:
    ax = plt.figure().add_subplot()
    ax.plot(diffCOEs_dict["del_raan"].keys(), diffCOEs_dict["del_raan"].values(), label="del_raan")
    ax.plot(diffCOEs_avg["del_raan"].keys(), diffCOEs_avg["del_raan"].values(), "--", label=f"{revs_to_avg} orbit average")
    
    if plot_Show_Firings and len(burnStarts) > 0:
        ax.plot([t[i] for i in R_burns], [diffCOEs_avg["del_raan"][i] for i in R_burns], "*", c="m", label="R-axis maneuver")
        ax.plot([t[i] for i in I_burns], [diffCOEs_avg["del_raan"][i] for i in I_burns], "*", c="r", label="I-axis maneuver")
        ax.plot([t[i] for i in C_burns], [diffCOEs_avg["del_raan"][i] for i in C_burns], "*", c="c", label="C-axis maneuver")
    
    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset (deg)')
    ax.set_title("Truth-Reference Differences in Right Ascension vs Time")
    ax.legend()

if plot_COE_diffs["del_aop"]:
    ax = plt.figure().add_subplot()
    ax.plot(diffCOEs_dict["del_aop"].keys(), diffCOEs_dict["del_aop"].values(), label="del_aop")
    ax.plot(diffCOEs_avg["del_aop"].keys(), diffCOEs_avg["del_aop"].values(), "--", label=f"{revs_to_avg} orbit average")
    
    if plot_Show_Firings and len(burnStarts) > 0:
        ax.plot([t[i] for i in R_burns], [diffCOEs_avg["del_aop"][t[i]] for i in R_burns], "*", c="m", label="R-axis maneuver")
        ax.plot([t[i] for i in I_burns], [diffCOEs_avg["del_aop"][t[i]] for i in I_burns], "*", c="r", label="I-axis maneuver")
        ax.plot([t[i] for i in C_burns], [diffCOEs_avg["del_aop"][t[i]] for i in C_burns], "*", c="c", label="C-axis maneuver")
    
    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset (deg)')
    ax.set_title("Truth-Reference Differences in Argument of Perigee vs Time")
    ax.legend()

if plot_COE_diffs["del_f"]:
    ax = plt.figure().add_subplot()
    ax.plot(diffCOEs_dict["del_f"].keys(), diffCOEs_dict["del_f"].values(), label="del_f")
    ax.plot(diffCOEs_avg["del_f"].keys(), diffCOEs_avg["del_f"].values(), "--", label=f"{revs_to_avg} orbit average")
    
    if plot_Show_Firings and len(burnStarts) > 0:
        ax.plot([t[i] for i in R_burns], [diffCOEs_avg["del_f"][t[i]] for i in R_burns], "*", c="m", label="R-axis maneuver")
        ax.plot([t[i] for i in I_burns], [diffCOEs_avg["del_f"][t[i]] for i in I_burns], "*", c="r", label="I-axis maneuver")
        ax.plot([t[i] for i in C_burns], [diffCOEs_avg["del_f"][t[i]] for i in C_burns], "*", c="c", label="C-axis maneuver")
    
    ax.set_xlabel('Time (Days)')
    ax.set_ylabel('Offset (deg)')
    ax.set_title("Truth-Reference Differences in True Anomaly vs Time")
    ax.legend()

if plot_True_Lat_diff:
    del_f = np.array([*diffCOEs_dict["del_f"].values()])
    del_f_avg = np.array([*diffCOEs_avg["del_f"].values()])
    del_aop = np.array([*diffCOEs_dict["del_aop"].values()])
    del_aop_avg = np.array([*diffCOEs_avg["del_aop"].values()])

    del_theta = del_f + del_aop
    del_theta_avg = del_f_avg + del_aop_avg

    ax = plt.figure().add_subplot()
    ax.plot(t, del_theta, label="del_theta")
    ax.plot(t, del_theta_avg, "--", label=f"{revs_to_avg} orbit average")
    
    if plot_Show_Firings and len(burnStarts) > 0:
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
