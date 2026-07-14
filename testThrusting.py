
from collections import deque
from createStationKeepingObjects import StationKeepingObjects
from load_gmat import *
from matplotlib import pyplot as plt
from plotting import terminal_Completed_Firings, outputPlots
from xyz2ric import xyz2ric

import numpy as np
import datetime

# -----------user defined variables----------------------------------
"""Please change the following variables as necessary to shape your scenario"""

# Duration of the scenario in days
maxDays = 370

# Simulation step size while coasting
dtCoast = 60.0 

# Simulation step size while thrusting
dtThrust = 5.0

# Orbital element set shared by the initial reference and truth satellites
stateVector = "new" # "new"
orbitParam = [
    6928,   # SMA, avg alt of 500 km (6878, 6903)
    1e-3,   # ECC
    65,     # INC
    0,      # RAAN
    0,      # AOP
    0,      # TA
    datetime.datetime.today() # Epoch
]
# +70 days from 6 July
refOrbitParam = [
    6862.398123976534,
    0.001222655234762756,
    65.08465834964612,
    132.8799247164418,
    103.33767643107548,
    169.9596201609888,
    "14 Sep 2026 00:00:00.000"
]

truthOrbitParam = [
    6862.354299657057,
    0.0012109994471725667,
    65.08194601412482,
    132.8802573958094,
    105.53210688316561,
    167.6482406337395,
    "14 Sep 2026 00:00:00.000"
]
# Operational bounds (+/-) to keep the truth satellite within
R_bounds = 2
I_bounds = 20
C_bounds = 4

# Maximum thruster duty time in seconds
minDutyTime = 300
maxDutyTime = 3600

I_deadband_min = 0.85

# 
maneuverArcHalfAngle = 20

# -----------create variables----------------------------------------
"""Additional variables used in the script that SHOULD NOT BE CHANGED"""
mu = 398600  # Earth’s mu in km^3/s^2
burn_duration = 0 # timer to track maneuver duration
coast_duration = 0

if stateVector == "new":
    mean_motion = np.sqrt(mu / orbitParam[0]**3) # mean motion of intitial orbital parameters
else:
    mean_motion = np.sqrt(mu / refOrbitParam[0]**3)
period_sec = 2 * np.pi / mean_motion # initial orbital period in seconds

steps_per_rev = int(np.ceil(period_sec / dtCoast)) # number of simulation steps in 1 orbit around Earth
revs_to_avg = 3 # number of orbits used to average out the oscillations of the perturbed orbital solutions
steps_to_avg = int(revs_to_avg * steps_per_rev) + 5 # number of simulation steps needed to average perturbations

elapsed = 0.0 # elpased number of seconds since the start of the scenario
totalDeltaV = 0

totalSecs = maxDays * 86400
timeSteps = int(totalSecs / dtCoast) + 1
t = [dtCoast * i for i in range(0, timeSteps)] # array to hold each time step

# stores the RIC history of the truth spacecraft about the reference spacecraft 
RIC_keys = ["R", "I", "C", "R_dot", "I_dot", "C_dot"]
RIC_History = {key: {0.0:0.0} for key in RIC_keys} # step: 0.0 for step in t
RIC_Amp_History = {key: {0.0:0.0} for key in RIC_keys}
RIC_Amp_Buffer = {key: deque([0.0], maxlen=int(1.5 * steps_per_rev)) for key in RIC_History}

# Storage of the differences in the orbital elements throughout the scenario
COEs_keys = ["del_a", "del_e", "del_i", "del_raan", "del_aop", "del_f"]
diffCOEs_dict = {key: {0.0:0.0} for key in COEs_keys}
diffCOEs_avg = {key: {0.0:0.0} for key in COEs_keys}
diffCOEs_buffer = {key: deque([0.0], maxlen=int(steps_to_avg)) for key in COEs_keys}

# Variables used to help complete I-axis maneuvers
del_a_target = 0 # desired increase in the difference of semi-major axis between the truth and reference satellites
del_a_recovered = False
maneuverAttempts = 0
maneuverLog = []
# Log of maneuver times
burnStarts = []
burnEnds = []
step_RIC = {key: 0.0 for key in RIC_keys}
step_COE = {key: 0.0 for key in COEs_keys}
# Timer to prevent controller from being stuck waiting to maneuver
numStepsWaiting = 0

# Initial state
state = "nominal"
interpruptedState = "nominal"
# -----------configuration preliminaries-----------------------------
"""Generate the necessary GMAT objects and Python wrappers"""
# Reference Objectes
refObjs = StationKeepingObjects("reference")
if stateVector == "new":
    refObjs.setSatCOEs(orbitParam)
else:
    refObjs.setSatCOEs(refOrbitParam)

# Truth Objects
truthObjs = StationKeepingObjects("truth")
if stateVector == "new":
    truthObjs.setSatCOEs(orbitParam)
else:
    truthObjs.setSatCOEs(truthOrbitParam)
truthObjs.setManeuverable()

# Initialize the scenario
gmat.Initialize()
sat_T0 = truthObjs.sat_wrap.getEpoch_GMAT()
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

# While the elapsed time is less the max number of days
while elapsed < totalSecs: 

    if state not in ["R burn", "I burn", "C burn"] and elapsed % dtCoast != 0:
        dt -= elapsed % dtCoast

    # Get the updated cartesian states for each spacecraft from the ECI frame
    rv_ref = gator_ref.GetState()
    rv_truth = gator_truth.GetState()

    # Propagate spacecraft
    gator_ref.Step(dt)
    gator_truth.Step(dt)

    # Update numerical integrator references
    gator_ref.UpdateSpaceObject()
    gator_truth.UpdateSpaceObject()
    
    # Update the the elpased time
    elapsed += dt

    if state not in ["R burn", "I burn", "C burn"] and dt != dtCoast:
        dt = dtCoast

    # Get the updated cartesian states for each spacecraft from the ECI frame
    rv_ref = gator_ref.GetState()
    rv_truth = gator_truth.GetState()

    # Get the corresponding cartesian state from the RIC frame
    rvRIC, rotMatrix = xyz2ric(rv_ref, rv_truth)

    # Get the updated keplerian states for each spacecraft
    refCOE = refObjs.sat_wrap.getKeplerianState()
    truthCOE = truthObjs.sat_wrap.getKeplerianState()

    # To determine if the I-axis corections have completed successfully, we will use the difference in semi-major axis based on the spacecraft's specific energy
    sma_truth = truthObjs.sat_wrap.getSMAFromEnergy()
    sma_ref = refObjs.sat_wrap.getSMAFromEnergy()
    del_a_energy = sma_truth - sma_ref

    if elapsed % dtCoast == 0:
        for j in range(6):
            step_RIC[RIC_keys[j]] = rvRIC[j]
            
            diff_COE = truthCOE[j] - refCOE[j]
            quad_Correction = (-360 if (j > 1 and diff_COE > 180) else (360 if j > 1 and diff_COE < -180 else 0))
            step_COE[COEs_keys[j]] = diff_COE + quad_Correction


            RIC_History[RIC_keys[j]][elapsed] = rvRIC[j]

            if state not in ["R burn", "I burn", "C burn"]:
                RIC_Amp_Buffer[RIC_keys[j]].append(rvRIC[j]) 
                amp = (max(RIC_Amp_Buffer[RIC_keys[j]]) - min(RIC_Amp_Buffer[RIC_keys[j]])) / 2 \
                    if len(RIC_Amp_Buffer[RIC_keys[j]]) == 1.5 * steps_per_rev else max(RIC_Amp_Buffer[RIC_keys[j]])
                RIC_Amp_History[RIC_keys[j]][elapsed] = amp
            else:
                RIC_Amp_Buffer[RIC_keys[j]].append(RIC_History[RIC_keys[j]][burnStarts[-1][0]])
                amp = (max(RIC_Amp_Buffer[RIC_keys[j]]) - min(RIC_Amp_Buffer[RIC_keys[j]])) / 2 \
                    if len(RIC_Amp_Buffer[RIC_keys[j]]) == 1.5 * steps_per_rev else max(RIC_Amp_Buffer[RIC_keys[j]])
                RIC_Amp_History[RIC_keys[j]][elapsed] = amp

            
            """if RIC_keys[j] == "C" and RIC_Amp_History[RIC_keys[j]][elapsed] > 1:
                print(max(RIC_Amp_Buffer[RIC_keys[j]]))
                print(min(RIC_Amp_Buffer[RIC_keys[j]]))
                
                print(len(RIC_Amp_Buffer[RIC_keys[j]]) == 1.5 * steps_per_rev)
                max(RIC_Amp_Buffer[RIC_keys[j]])
                print("woah: bad RIC")
                exit() """

            diffCOEs_dict[COEs_keys[j]][elapsed] = diff_COE + quad_Correction
            diffCOEs_buffer[COEs_keys[j]].append(diff_COE + quad_Correction)
            diffCOEs_avg[COEs_keys[j]][elapsed] = float(np.mean(diffCOEs_buffer[COEs_keys[j]])) \
                if len(diffCOEs_buffer[COEs_keys[j]]) == steps_to_avg else diff_COE + quad_Correction
            
            if COEs_keys[j] == "del_e" and diffCOEs_avg[COEs_keys[j]][elapsed] > 1:
                print("woah: bad COE")
                print(f"AVG Diff = {diffCOEs_avg[COEs_keys[j]][elapsed]}")
                for k in range(6):
                    print(f"truth = {truthCOE[k]} | ref = {refCOE[k]}")
                exit()

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
        # t.append(elapsed)

    """
    # Store latest RIC position vector
    for j in range(6):
        RIC_History[RIC_keys[j]][elapsed] = rvRIC[j]
        
        # if state == "I burn" and thrusterAxis != "":
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
        diffCOEs_avg[COEs_keys[j]][elapsed] = float(np.mean(diffCOEs_buffer[COEs_keys[j]])) # average differences across one rev
    """

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

    

    match state:
        # -----------waiting for maneuver opoprtunties------------------------
        case "wait for R burn":
            # break
            # Increase number of steps waited
            numStepsWaiting += 1
            
            f = truthCOE[-1]
            in_node_window = not( # 0 < f <= 180 - maneuverArcHalfAngle) and \
                                0 < f <= 90 - maneuverArcHalfAngle) and \
                             not(
                                90 < f <= 270 - maneuverArcHalfAngle)
            
            if diffCOEs_avg["del_e"][elapsed] > 0:
                in_node_window = 90 - maneuverArcHalfAngle < f < 90
            else:
                in_node_window = 270 - maneuverArcHalfAngle < f < 270

            minDelAOP = (-10 <= diffCOEs_dict["del_aop"][elapsed - elapsed % dtCoast] < 10)

            """if -0.2 <= rvRIC[0] < 0.2:
                tempStr = f"t = {(burnStarts[-1][0]):2.2f} days | " if (burnStarts[-1][0]) >= 10 else f"t = {(burnStarts[-1][0]):1.3f} days | "
                tempStr += f"Y-AXIS Cross: R = {rvRIC[0]:1.3f} km | True lat = {(truthCOE[-2] + truthCOE[-1]) % 360} deg | "
                tempStr += f"omega = {truthCOE[-2]} deg | f = {truthCOE[-1]} deg"
                print(tempStr)"""
            # Target thrust window has a phase angle of 15 -> 0 -> -20 deg
            if in_node_window and minDelAOP:
                n = mean_motion
                a = truthObjs.sat_wrap.getSMAFromEnergy()
                eTruth = truthCOE[1]
                eta = np.sqrt(1 - truthCOE[1])
                deltaAOP = np.deg2rad(diffCOEs_avg["del_aop"][elapsed - elapsed % dtCoast])
                deltaRAAN = np.deg2rad(diffCOEs_avg["del_raan"][elapsed - elapsed % dtCoast])
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
                thrusterAxis = "R+"
                gator_truth = truthObjs.satEnginesOn(thrusterAxis)
                state = "R burn"
                
                # Add to the list of when burns start the corresponding color for this maneuver
                burnStarts.append((elapsed, "m"))

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
            fTrue = truthCOE[-1]
            wTrue = truthCOE[-2]
            wRef = refCOE[-2]
            # trueLat = (f + w) % 360
            #in_burn_window = fTrue >= 340 # 160 < trueLat <= 180 #  # 160 < f <= 180
            
            if rvRIC[1] / I_bounds < 0.95:
                if diffCOEs_avg["del_e"][elapsed] <= 0:
                    in_burn_window = 360 - maneuverArcHalfAngle < (fTrue - diffCOEs_avg["del_aop"][elapsed]) % 360 <= 360
                else:
                    in_burn_window = 180 - maneuverArcHalfAngle < (fTrue - diffCOEs_avg["del_aop"][elapsed]) % 360 <= 180
            else:
                in_burn_window = (360 - maneuverArcHalfAngle < fTrue - diffCOEs_avg["del_aop"][elapsed] <= 360) or (180 - maneuverArcHalfAngle < fTrue - diffCOEs_avg["del_aop"][elapsed] <= 180)
            del_a_target = max(abs(del_a_energy), abs(diffCOEs_avg["del_a"][elapsed]))

            # Possibility for controller to trigger a maneuver when spacecraft is within user-defined bounds, this check prevents that
            tStep = len(t) - t.index(elapsed)
            tHistory = t[-(10 * steps_per_rev + tStep):-tStep]
            subset = [diffCOEs_avg["del_a"][j] for j in tHistory]
            decayRate, _ = np.polyfit(tHistory, subset,  1)
            del subset

            isNegativeSMA = diffCOEs_avg["del_a"][elapsed] < 0
            isNegativeSMATrend = (diffCOEs_avg["del_a"][elapsed] - diffCOEs_avg["del_a"][t[-(10 * steps_per_rev + tStep)]]) < 0
            isInDeadBand = rvRIC[1] > I_deadband_min * I_bounds
            valid_burn = isNegativeSMA and isNegativeSMATrend and isInDeadBand
            
            if len(burnStarts) > 0:
                if burnStarts[-1][1] != "r":
                    recentManeuver = True
                else:
                    recentManeuver = (elapsed - burnEnds[-1] >= 3 * period_sec)
            else:
                recentManeuver = True

            if in_burn_window and valid_burn and recentManeuver:
                # Update the controller state
                state = "I burn"
                thrusterAxis = "I+"
                maneuverAttempts = 0
                gator_truth = truthObjs.satEnginesOn(thrusterAxis)

                # Add to the list of when burns start the corresponding color for this maneuver
                burnStarts.append((elapsed, "r"))
                # print(f"f_truth = {truthCOE[-2]} deg | aop_truth = {truthCOE[-1]} deg")
                # print(f"f_ref   = {refCOE[-2]} deg | aop_ref   = {refCOE[-1]} deg")

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
            if critAngle + maneuverArcHalfAngle * 2 > 360:
                in_node_window = critAngle - maneuverArcHalfAngle * 2 < true_lat or true_lat <= (critAngle + maneuverArcHalfAngle * 2) % 360
            else:
                in_node_window = critAngle - maneuverArcHalfAngle * 2 < true_lat < critAngle + maneuverArcHalfAngle * 2
            if in_node_window:
                thrusterAxis = "C-" if critAngle > 180 else "C+"
                
                gator_truth = truthObjs.satEnginesOn(thrusterAxis)
                state = "C burn"
                burnStarts.append((elapsed, "c"))
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
            
            """if -0.2 <= rvRIC[0] < 0.2:
                tempStr = f"t = {(burnStarts[-1][0] / 86400):2.2f} days | " if (burnStarts[-1][0] / 86400) >= 10 else f"t = {(burnStarts[-1][0] / 86400):1.3f} days | "
                tempStr += f"Y-AXIS Cross: R = {rvRIC[0]:1.3f} km | True lat = {(truthCOE[-2] + truthCOE[-1]) % 360} deg | "
                tempStr += f"omega = {truthCOE[-2]} deg | f = {truthCOE[-1]} deg"
                print(tempStr)"""

            burn_duration += dt
            velo_phase = np.arctan2(rvRIC[3], rvRIC[4])
            # in_cross_track_pass = abs(abs(velo_phase) - np.pi /2) < np.deg2rad(20)

            n = mean_motion
            a = truthObjs.sat_wrap.getSMAFromEnergy()
            eTruth = truthCOE[1]
            eta = np.sqrt(1 - truthCOE[1])
            deltaAOP = np.deg2rad(diffCOEs_avg["del_aop"][elapsed - elapsed % dtCoast])
            deltaRAAN = np.deg2rad(diffCOEs_avg["del_raan"][elapsed - elapsed % dtCoast])
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


            in_node_window = not( # 0 < f <= 180 - maneuverArcHalfAngle) and \
                                0 < f <= 90 - maneuverArcHalfAngle) and \
                             not(
                                90 < f <= 270 - maneuverArcHalfAngle)
            
            in_node_window = (-10 <= diffCOEs_dict["del_aop"][elapsed - elapsed % dtCoast] < 10) and 90 - maneuverArcHalfAngle < truthCOE[-1] < 90
            
            """in_node_window = not(
                                maneuverArcHalfAngle < f <= 180 - maneuverArcHalfAngle) and \
                            not(
                                180 + maneuverArcHalfAngle < f <= 360 - maneuverArcHalfAngle)"""
            
            if (burn_duration >= maxDutyTime or not in_node_window) and burn_duration >= minDutyTime * 2:
                if terminal_Completed_Firings:
                    terminalStr = f"t = {(burnStarts[-1][0] / 86400):2.2f} days | " if (burnStarts[-1][0] / 86400) >= 10 else f"t = {(burnStarts[-1][0] / 86400):1.3f} days | "
                    terminalStr += f"{thrusterAxis} burn duration (min) = "
                    terminalStr += f"{(burn_duration / 60):2.2f} | " if (burn_duration / 60) >= 10 else f"{(burn_duration / 60):1.3f} | "
                    terminalStr += f"R-axis Amplitude = {RIC_Amp_History['R'][elapsed - elapsed % dtCoast]:0.6f} km         | "
                    
                    accel = 0.2 / truth_Sat_wrapper.mass # m/s
                    deltaV = accel * burn_duration
                    totalDeltaV += deltaV
                    terminalStr += f"deltaV = {deltaV:1.3f} m/s | " 
                    terminalStr += f"total deltaV = {totalDeltaV:1.3f} m/s"
                    print(terminalStr)
                gator_truth = truthObjs.satEnginesOff(thrusterAxis)
                state = "returning to nominal from R burn"
                burnEnds.append(elapsed)
                dt = dtCoast - round(elapsed % dtCoast)

                burn_duration = 0              
        case "I burn":
            """burn_duration += dt
            if burn_duration == minDutyTime:
                print(f"f_truth = {truthCOE[-2]} deg | aop_truth = {truthCOE[-1]} deg")
                print(f"f_ref   = {refCOE[-2]} deg | aop_ref   = {refCOE[-1]} deg")

                gator_truth = truthObjs.satEnginesOff(thrusterAxis)
                thrusterAxis = ""

                dt = dtCoast - round(elapsed % dtCoast)
                maneuverAttempts += 1
            elif thrusterAxis == "":
                break"""
            if thrusterAxis != "":
                burn_duration += dt

                if (burn_duration >= minDutyTime and maneuverAttempts == 0) or maneuverAttempts > 0:
                    burnEnds.append(elapsed)
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
                
                if coast_duration > period_sec:
                    # Termination conditions:
                    # - achieves deadband target by the time SMA changes sign (no change)
                    # - undershoots deadband target when SMA changes sign (more thrusting)
                    # - overshoots deadband target (less thrusting)
                    
                    # achieves deadband target by the time SMA changes sign (no change)
                    if diffCOEs_avg["del_a"][elapsed] < 0 and -I_deadband_min * I_bounds > minIPosition >= -I_bounds:
                        if terminal_Completed_Firings:
                            terminalStr = f"t = {(burnStarts[-1][0] / 86400):2.2f} days | " if (burnStarts[-1][0] / 86400) >= 10 else f"t = {(burnStarts[-1][0] / 86400):1.3f} days | "
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
                        
                        """for j in COEs_keys:
                            COEs_restore_history = {k:v for k, v in diffCOEs_dict[j].items() if t[burnEnds[-1] - steps_to_avg] < k <= t[burnEnds[-1]]}
                            diffCOEs_buffer[j] = deque([*COEs_restore_history.values()], maxlen=int(steps_to_avg))

                        for j in RIC_keys:
                            RIC_restore_history = {k:v for k, v in RIC_History[j].items() if t[burnEnds[-1] - steps_to_avg] < k <= t[burnEnds[-1]] and k % dtCoast == 0}
                            RIC_Amp_Buffer[j] = deque([*RIC_restore_history.values()], maxlen=int(1.5 * steps_per_rev))

                        t = t[:burnEnds[-1] + 1]"""
                        
                        restoreFromTime = burnEnds[-1] - burnEnds[-1] % dtCoast 
                        tStep = t.index(restoreFromTime)
                        tHistoryCOEs = t[(tStep - steps_to_avg):tStep]
                        for j in COEs_keys:
                            [diffCOEs_buffer[j].append(diffCOEs_dict[j][k]) for k in tHistoryCOEs]

                        tHistoryRIC = t[int(tStep - 1.5 * steps_per_rev):tStep]
                        # for j in RIC_keys:
                            # [RIC_Amp_Buffer[j].append(RIC_Amp_History[j][k]) for k in tHistoryRIC]
                            # if j =="I":
                                # [print(k) for k in RIC_Amp_Buffer[j]]

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
                    elif diffCOEs_avg["del_a"][elapsed] < 0 and (diffCOEs_avg["del_a"][elapsed] - diffCOEs_avg["del_a"][elapsed - 10 * steps_per_rev * dtCoast]) < 0 and -I_deadband_min * I_bounds <= minIPosition:
                        # print(f"Maneuver #{maneuverAttempts}: I-position = {minIPosition:1.4}km | burn time = {burn_duration} sec")
                        """
                        for j in COEs_keys:
                            COEs_restore_history = {k:v for k, v in diffCOEs_dict[j].items() if t[burnEnds[-1] - steps_to_avg] < k <= t[burnEnds[-1]]}
                            diffCOEs_buffer[j] = deque([*COEs_restore_history.values()], maxlen=int(steps_to_avg))

                        for j in RIC_keys:
                            RIC_restore_history = {k:v for k, v in RIC_History[j].items() if t[burnEnds[-1] - steps_to_avg] < k <= t[burnEnds[-1]] and k % dtCoast == 0}
                            RIC_Amp_Buffer[j] = deque([*RIC_restore_history.values()], maxlen=int(1.5 * steps_per_rev))
                        """

                        restoreFromTime = burnEnds[-1] - burnEnds[-1] % dtCoast 
                        tStep = t.index(restoreFromTime)
                        tHistoryCOEs = t[(tStep - steps_to_avg):tStep]
                        for j in COEs_keys:
                            [diffCOEs_buffer[j].append(diffCOEs_dict[j][k]) for k in tHistoryCOEs]

                        tHistoryRIC = t[int(tStep - 1.5 * steps_per_rev):tStep]
                        # for j in RIC_keys:
                            # [RIC_Amp_Buffer[j].append(RIC_Amp_History[j][k]) for k in tHistoryRIC]

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
                        estimateSteps = np.ceil((minIPosition + I_deadband_min * I_bounds) / 0.75) if (0 >= minIPosition) else 1
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
                    elif diffCOEs_avg["del_a"][elapsed] < 0 and (diffCOEs_avg["del_a"][elapsed] - diffCOEs_avg["del_a"][elapsed - 10 * steps_per_rev * dtCoast]) < 0 and minIPosition < -I_bounds:
                        # print(f"Maneuver #{maneuverAttempts}: I-position = {minIPosition:1.4}km | burn time = {burn_duration} sec")
                        truth_Tdiff = (truthObjs.sat_wrap.getEpoch_GMAT() - sat_T0) * 86400
                        ref_Tdiff = (refObjs.sat_wrap.getEpoch_GMAT() - sat_T0) * 86400

                        
                        """print(f"truth Time Elapsed = {(truth_Tdiff)}")
                        print(f"ref Time Elapsed   = {(ref_Tdiff)}")
                        print(f"truth Time Diff = {(truth_Tdiff - elapsed)}")
                        print(f"ref Time Diff   = {(ref_Tdiff - elapsed)}")
                        break"""
                        stepsToBackTrack = 5
                        if burn_duration - dtThrust * stepsToBackTrack in maneuverLog: 
                            stepsToBackTrack -=1
                        maneuverLog.append(burn_duration)
                        
                        """deleteToIndex = burnEnds[-1] - stepsToBackTrack - 1
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
                        """

                        restoreFromTime = burnEnds[-1] - burnEnds[-1] % dtCoast - (stepsToBackTrack * dtThrust // dtCoast) * dtCoast
                        tStep = t.index(restoreFromTime)
                        tHistoryCOEs = t[(tStep - steps_to_avg):tStep]
                        for j in COEs_keys:
                            [diffCOEs_buffer[j].append(diffCOEs_dict[j][k]) for k in tHistoryCOEs]

                        tHistoryRIC = t[int(tStep - 1.5 * steps_per_rev):tStep]
                        # for j in RIC_keys:
                            # [RIC_Amp_Buffer[j].append(RIC_Amp_History[j][k]) for k in tHistoryRIC]
                        burnEnds.pop()
                        
                        # Get the updated cartesian states for each spacecraft from the ECI frame
                        rv_ref = gator_ref.GetState()
                        rv_truth = gator_truth.GetState()
                        # Get the updated keplerian states for each spacecraft
                        refCOE = refObjs.sat_wrap.getKeplerianState()
                        truthCOE = truthObjs.sat_wrap.getKeplerianState()
                        
                        # Propagate spacecraft
                        # gator_ref.Step(-coast_duration)
                        # gator_truth.Step(-coast_duration)
                        backPropStepSize = coast_duration // 300
                        for i in range(300):
                            gator_ref.Step(-backPropStepSize)
                            gator_truth.Step(-backPropStepSize)

                        backStepRemainder = coast_duration % 300
                        gator_ref.Step(-backStepRemainder)
                        gator_truth.Step(-backStepRemainder)

                        elapsed = elapsed - coast_duration
                        coast_duration = 0

                        # Update numerical integrator references
                        gator_ref.UpdateSpaceObject()
                        gator_truth.UpdateSpaceObject()

                        # Get the updated cartesian states for each spacecraft from the ECI frame
                        rv_ref = gator_ref.GetState()
                        rv_truth = gator_truth.GetState()
                        # Get the updated keplerian states for each spacecraft
                        refCOE = refObjs.sat_wrap.getKeplerianState()
                        truthCOE = truthObjs.sat_wrap.getKeplerianState()
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

                        # Get the updated cartesian states for each spacecraft from the ECI frame
                        rv_ref = gator_ref.GetState()
                        rv_truth = gator_truth.GetState()

                        if burn_duration + dt <= 0:
                            print(f"Negative thrust time! Min I = {minIPosition}")
                            burnStarts.pop()
                            break
                        if maneuverAttempts == 100:
                            print(f"Max burns! current burn duration = {burn_duration} sec | Min I = {minIPosition}")
                            burnStarts.pop()
                            break
                else: 
                    RuntimeError("Invalid logic condition during I-axis deadband control")
                    exit         
        case "C burn":
            burn_duration += dt
            velo_phase = np.arctan2(rvRIC[1], rvRIC[2])
            
            true_lat = (truthCOE[4] + truthCOE[5]) % 360
            if critAngle + maneuverArcHalfAngle * 2 > 360:
                in_cross_track_pass = critAngle - maneuverArcHalfAngle * 2 < true_lat or true_lat <= (critAngle + maneuverArcHalfAngle * 2) % 360
            elif critAngle - maneuverArcHalfAngle * 2 < 0:
                in_cross_track_pass = (critAngle - maneuverArcHalfAngle * 2 + 360) < true_lat or true_lat <= (critAngle + maneuverArcHalfAngle * 2) % 360
            else:
                in_cross_track_pass = critAngle - maneuverArcHalfAngle * 2 < true_lat < critAngle + maneuverArcHalfAngle * 2
            
            if burn_duration >= maxDutyTime or not in_cross_track_pass:
                if terminal_Completed_Firings:
                    terminalStr = f"t = {(burnStarts[-1][0] / 86400):2.2f} days | " if (burnStarts[-1][0] / 86400) >= 10 else f"t = {(burnStarts[-1][0]) / 86400:1.3f} days | "
                    terminalStr += f"{thrusterAxis} burn duration (min) = "
                    terminalStr += f"{(burn_duration / 60):2.2f} | " if (burn_duration / 60) >= 10 else f"{(burn_duration / 60):1.3f} | "
                    terminalStr += f"C-axis Amplitude = {RIC_Amp_History['C'][elapsed - elapsed % dtCoast]:0.6f} km         | "
                    
                    accel = 0.2 / truth_Sat_wrapper.mass # m/s
                    deltaV = accel * burn_duration
                    totalDeltaV += deltaV
                    terminalStr += f"deltaV = {deltaV:1.3f} m/s | " 
                    terminalStr += f"total deltaV = {totalDeltaV:1.3f} m/s"
                    print(terminalStr)

                gator_truth = truthObjs.satEnginesOff(thrusterAxis)
                thrusterAxis = ""
                state = "returning to nominal from C burn"
                burnEnds.append(elapsed)
                
                burn_duration = 0

                dt = dtCoast - round(elapsed % dtCoast)
        
        # -----------verifying recovery---------------------------------------
        case "returning to nominal from R burn":
            R_amp_recovering = (RIC_Amp_History["R"][elapsed - elapsed % dtCoast] < RIC_Amp_History["R"][elapsed - elapsed % dtCoast - dtCoast]) and (len(RIC_Amp_History["R"]) > 2)
            
            if -0.2 <= rvRIC[0] < 0.2:
                tempStr = f"t = {(burnStarts[-1][0]):2.2f} days | " if (burnStarts[-1][0]) >= 10 else f"t = {(burnStarts[-1][0]):1.3f} days | "
                tempStr += f"Y-AXIS Cross: R = {rvRIC[0]:1.3f} km | True lat = {(truthCOE[-2] + truthCOE[-1]) % 360} deg | "
                tempStr += f"omega = {truthCOE[-2]} deg | f = {truthCOE[-1]} deg"
                # print(tempStr)
            if RIC_Amp_History["R"][elapsed - elapsed % dtCoast] <= 1 /2 * R_bounds:
                state = interpruptedState
                interpruptedState = "nominal"
            elif not R_amp_recovering and (elapsed - elapsed % dtCoast - burnEnds[-1] > 0.25*period_sec):
                state = "wait for R burn"
        case "returning to nominal from C burn":
            if elapsed % dtCoast != 0:
                continue

            C_amp_recovering = RIC_Amp_History["C"][elapsed - elapsed % dtCoast] <= RIC_Amp_History["C"][elapsed - elapsed % dtCoast - dtCoast]
            if diffCOEs_avg["del_raan"][elapsed - elapsed % dtCoast] > 0: # C_amp <= 1 /3 * C_bounds:
                state = interpruptedState
                interpruptedState = "nominal"
            elif (elapsed - elapsed % dtCoast - burnEnds[-1] > .75 * period_sec): # not C_amp_recovering or
                state = "wait for C burn"      
            else:
                continue 
        case _:
            continue
print(state)
print(f"Current time: T+{(elapsed / 86400)} days")

terminalTime = refObjs.sat_wrap.getEpoch_datetime() + datetime.timedelta(seconds=elapsed)
print(refObjs.sat_wrap.getEpoch_ITC(terminalTime))
print("\nReference COEs:")
[print(" " * 4 + f"{i},") for i in [*refObjs.sat_wrap.getKeplerianState()]]
print(" " * 4 + f'"{refObjs.sat_wrap.getEpoch_ddmmmyyyy(terminalTime)}"')
print("\nTruth COEs:")
[print(" " * 4 + f"{i},") for i in [*truthObjs.sat_wrap.getKeplerianState()]]
print(" " * 4 + f'"{truthObjs.sat_wrap.getEpoch_ddmmmyyyy(terminalTime)}"')

# -----------plots---------------------------------------------------
timings =  [burnEnds, burnStarts, t, revs_to_avg, dtCoast]
coes = [COEs_keys, diffCOEs_dict, diffCOEs_avg]
ric = [RIC_keys, RIC_History, RIC_Amp_History]
# outputPlots(timings, coes, ric)