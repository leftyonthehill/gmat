""" Station keeping scenario starting point. """

# Native libraries
import datetime
import sys
from collections import deque

# 3rd party libraries
import numpy as np
from createStationKeepingObjects import StationKeepingObjects
from load_gmat import gmat
from plotting import outputPlots
from simulationParameters import (
    C_bounds,
    dtCoast,
    dtThrust,
    I_bounds,
    I_deadband_min,
    maneuverArcHalfAngle,
    maxDays,
    maxDutyTime,
    minDutyTime,
    orbitParam,
    R_bounds,
    refOrbitParam,
    REVOLUTIONS_TO_AVERAGE,
    stateVector,
    truthOrbitParam,
    terminal_Completed_Firings,
)
from supportFunctions import *

# ----------- Create Variables ------------------------------------------------
MU = 398600  # Earth’s mu in km^3/s^2
burn_duration = 0 # timer to track maneuver duration
coast_duration = 0

# Depending on the state vector choice, compute the respective mean motion
if stateVector == "new":
    MEAN_MOTION = np.sqrt(MU / orbitParam[0]**3)
else:
    MEAN_MOTION = np.sqrt(MU / refOrbitParam[0]**3)

# Initial orbital period in seconds
PERIOD_IN_SECONDS = 2 * np.pi / MEAN_MOTION

# Number of simulation steps in 1 orbit around Earth
STEPS_PER_ORBIT = int(np.ceil(PERIOD_IN_SECONDS / dtCoast))

# Number of simulation steps needed to average perturbations
STEPS_PER_AVG_ORBIT = int(REVOLUTIONS_TO_AVERAGE * STEPS_PER_ORBIT) + 5

total_delta_v = 0

# Scenario time-related variables
elapsed_time = 0.0
TOTALSECONDS = maxDays * 86400
NUMBER_OF_TIME_STEPS = int(TOTALSECONDS / dtCoast) + 1
t = [dtCoast * i
     for i in range(0, NUMBER_OF_TIME_STEPS)]

# Stores the RIC history of the truth spacecraft about the reference spacecraft
RIC_KEYS = ["R", "I", "C", "R_dot", "I_dot", "C_dot"]
RIC_History = {key: {0.0:0.0}
               for key in RIC_KEYS}
RIC_Amp_History = {key: {0.0:0.0}
                   for key in RIC_KEYS}
RIC_Amp_Buffer = {key: deque([0.0], maxlen=int(1.5 * STEPS_PER_ORBIT))
                  for key in RIC_History}

# Storage of the differences in the orbital elements throughout the scenario
COE_KEYS = ["del_a", "del_e", "del_i", "del_raan", "del_aop", "del_f"]
diffCOEs_dict = {key: {0.0:0.0}
                 for key in COE_KEYS}
diffCOEs_avg = {key: {0.0:0.0}
                for key in COE_KEYS}
diffCOEs_buffer = {key: deque([0.0], maxlen=int(STEPS_PER_AVG_ORBIT))
                   for key in COE_KEYS}

# Desired increase in the difference of semi-major axis between the truth and
# reference satellites
del_a_target = 0
del_a_recovered = False
maneuver_attempts = 0
maneuver_log = []

# Log of maneuver times
burn_starts = []
burn_ends = []
step_RIC = {key: 0.0
            for key in RIC_KEYS}
step_COE = {key: 0.0
            for key in COE_KEYS}
# Timer to prevent controller from being stuck waiting to maneuver
steps_waiting = 0

# Initial state
state = "nominal"
interprupted_state = "nominal"

I_BUSY = {"wait for I burn", "R burn", "I burn", "C burn"}
C_BUSY = I_BUSY | {"wait for C burn", "returning to nominal from C burn"}
R_BUSY = C_BUSY | {"wait for R burn", "returning to nominal from R burn"}

# ----------- Configuration Preliminaries -------------------------------------
# Reference Objectes
REF_OBJ = StationKeepingObjects("reference")
if stateVector == "new":
    REF_OBJ.sat_wrap.setKeplerianState(orbitParam)
else:
    REF_OBJ.sat_wrap.setKeplerianState(refOrbitParam)

# Truth Objects
TRUTH_OBJ = StationKeepingObjects("truth")
if stateVector == "new":
    TRUTH_OBJ.sat_wrap.setKeplerianState(orbitParam)
else:
    TRUTH_OBJ.sat_wrap.setKeplerianState(truthOrbitParam)
TRUTH_OBJ.setManeuverable()
# Initialize the scenario
gmat.Initialize()
sat_T0 = orbitParam[-1] if stateVector == "new" \
    else getEpoch_As_Datetime(truthOrbitParam[-1])
# ------------ Build Out Thruster Forces --------------------------------------

# Reference Objectes
REF_OBJ.preparePropInternal()
gator_ref = REF_OBJ.prop_wrap["coast"].prop_gmat.GetPropagator()

# Truth Objects
TRUTH_OBJ.setBurnForces()
TRUTH_OBJ.preparePropInternal()
gator_truth = TRUTH_OBJ.prop_wrap["coast"].prop_gmat.GetPropagator()
TRUTH_SAT = TRUTH_OBJ.sat_wrap

# ------------ Run Simulation--------------------------------------------------
# Set simulation step size
dt = dtCoast

# Get initial integrator for the truth satellite
gator_truth = TRUTH_OBJ.satEnginesOff("coast")

# While the elapsed_time time is less the max number of days
while elapsed_time < TOTALSECONDS:

    if all([state not in ["R burn", "I burn", "C burn"],
            elapsed_time % dtCoast != 0]):
        dt -= elapsed_time % dtCoast

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
    elapsed_time += dt
    prev_major_time_step = elapsed_time - elapsed_time % dtCoast

    if state not in ["R burn", "I burn", "C burn"] and dt != dtCoast:
        dt = dtCoast

    # Get the updated cartesian states for each spacecraft from the ECI frame
    rv_ref = gator_ref.GetState()
    rv_truth = gator_truth.GetState()

    # Get the corresponding cartesian state from the RIC frame
    rv_ric, _ = xyz2ric(rv_ref, rv_truth)

    # Get the updated keplerian states for each spacecraft
    refCOE = REF_OBJ.sat_wrap.getKeplerianState()
    truthCOE = TRUTH_OBJ.sat_wrap.getKeplerianState()

    if elapsed_time % dtCoast == 0:
        for j in range(6):
            step_RIC[RIC_KEYS[j]] = rv_ric[j]

            diff_COE = truthCOE[j] - refCOE[j]

            quad_correction = 0
            if j > 1:
                if diff_COE > 180:
                    quad_correction = -360
                elif diff_COE < -180:
                    quad_correction = 360

            step_COE[COE_KEYS[j]] = diff_COE + quad_correction
            diffCOEs_dict[COE_KEYS[j]][elapsed_time] = step_COE[COE_KEYS[j]]
            diffCOEs_buffer[COE_KEYS[j]].append(step_COE[COE_KEYS[j]])

            if len(diffCOEs_buffer[COE_KEYS[j]]) == STEPS_PER_AVG_ORBIT:
                avg_value = float(np.mean(diffCOEs_buffer[COE_KEYS[j]]))
            else:
                avg_value = step_COE[COE_KEYS[j]]
            diffCOEs_avg[COE_KEYS[j]][elapsed_time] = avg_value

            RIC_History[RIC_KEYS[j]][elapsed_time] = rv_ric[j]

            if state not in ["R burn", "I burn", "C burn"]:
                RIC_Amp_Buffer[RIC_KEYS[j]].append(
                    rv_ric[j])
            else:
                RIC_Amp_Buffer[RIC_KEYS[j]].append(
                    RIC_History[RIC_KEYS[j]][burn_starts[-1][0]])

            if len(RIC_Amp_Buffer[RIC_KEYS[j]]) == 1.5 * STEPS_PER_ORBIT:
                amp = (max(RIC_Amp_Buffer[RIC_KEYS[j]])
                       - min(RIC_Amp_Buffer[RIC_KEYS[j]])) / 2
            else:
                amp = max(RIC_Amp_Buffer[RIC_KEYS[j]])

            RIC_Amp_History[RIC_KEYS[j]][elapsed_time] = amp

        interprupt_maneuver = {
            "R": RIC_Amp_History["R"][elapsed_time] > R_bounds,
            "I": rv_ric[1] > I_deadband_min * I_bounds,
            "C": RIC_Amp_History["C"][elapsed_time] > C_bounds,
        }

        if (interprupt_maneuver["I"]
            and state not in I_BUSY
        ):
            interprupted_state = state
            state = "wait for I burn"
        elif (interprupt_maneuver["C"]
              and state not in C_BUSY
              and interprupted_state == "nominal"
        ):
            interprupted_state = state
            state = "wait for C burn"
        elif (interprupt_maneuver["R"]
              and state not in R_BUSY
              and interprupted_state == "nominal"
        ):
            interprupted_state = state
            state = "wait for R burn"

    # State Machine Controller Logic:
    # While the spacecraft's position vector in the RIC frame are within the
    # user-defined bounds, the spacecraft will operate in a nominal status. In
    # this nominal status, all the spacecraft will do is collect state vector
    # and deviation information. The first time step the spacecraft learns that
    # it has drifted outside of the user-defined ops boundary, it will switch
    # into a corrective mode for whichever boundary was violated first.
    #
    # Violation requirements:
    # - R-axis: The amplitude if the truth spacecraft's oscillations exceed
    #           that of the user-defined boundary condition.
    # - I-axis: The truth spacecraft's instaneous position along the I-axis
    #           exceeds the user-defined boundary condition.
    # - C-axis: The amplitude if the truth spacecraft's oscillations exceed
    #           that of the user-defined boundary condition.
    #
    # Positional requirements prior to thruster firing:
    # - R-axis: The truth spacecraft is approaching its maximum velocity in the
    #           R direction within the R-I plane.
    # - I-axis: The truth spacecraft must be within a 20 degree window of
    #           perigee or apogee and the truth spacecraft must have a smaller
    #           semi-major axis than its reference counterpart.
    # - C-axis: The truth spacecraft is approaching its maximum velocity in the
    #           C direction within the I-C plane.
    #
    # Maneuver termination conditions other than max duty time:
    # - R-axis: The truth spacecraft is leaving the defined maneuver window.
    # - I-axis: The truth spacecraft's semi-major axis must be greater than the
    #           reference by the difference computed prior to the start.
    #           of the maneuver
    # - C-axis: The truth spacecraft is leaving the defined maneuver window.
    #
    # Violation recovery requirements:
    # - R-axis: None, this thruster controller is still under development. The
    #           current behavior is to fire once and return to "nominal".
    # - I-axis: The truth spacecraft achieved 80+% of the necessary change in
    #           semi-major axis and the average I-axis position is dropping.
    # - C-axis: The truth spacecraft's C-axis position amplitude has dropped to
    #           1/3 of the boundary condition.

    match state:
        case "wait for R burn":
            # Increase number of steps waited
            steps_waiting += 1

            f = truthCOE[-1]
            in_node_window = (90 - maneuverArcHalfAngle < f < 90
                              or 270 - maneuverArcHalfAngle < f < 270)

            del_aop = diffCOEs_dict["del_aop"][prev_major_time_step]
            in_del_aop_range = -3 <= del_aop < 3

            # Target thrust window has a phase angle of 15 -> 0 -> -20 deg
            if in_node_window and in_del_aop_range:
                if diffCOEs_avg["del_e"][elapsed_time] > 0:
                    thruster_axis = "R-" if in_node_window[0] else "R+"
                else:
                    thruster_axis = "R+" if in_node_window[0] else "R-"

                gator_truth = TRUTH_OBJ.satEnginesOn(thruster_axis)
                state = "R burn"

                # Add to the list of when burns start the corresponding color
                # for this maneuver.
                burn_starts.append((elapsed_time, "m"))

                # Start maneuver duration timer
                burn_duration = 0.0

                # Update simulation time step
                dt = dtThrust

                # Reset the step counter
                steps_waiting = 0

            # If it has been one full rev since spacecraft entered this state,
            # return to nominal to prevent a lock-up.
            elif steps_waiting == STEPS_PER_ORBIT:
                state = interprupted_state
                interprupted_state = "nominal"
                steps_waiting = 0
            else:
                continue
        case "wait for I burn":
            # Increase number of steps waited
            steps_waiting += 1

            # Collect the current True Anomaly value to see if the spacecraft
            # is in the appropriate window for a maneuver.
            fTrue = truthCOE[-1]

            f_from_ref_perigee = (
                (fTrue - diffCOEs_avg["del_aop"][elapsed_time]) % 360)
            in_apogee_pass = (
                180 - maneuverArcHalfAngle < f_from_ref_perigee <= 180)
            in_perigee_pass = (
                360 - maneuverArcHalfAngle < f_from_ref_perigee <= 360)

            if rv_ric[1] / I_bounds < 0.95:
                if diffCOEs_avg["del_e"][elapsed_time] <= 0:
                    in_burn_window = in_perigee_pass
                else:
                    in_burn_window = in_apogee_pass
            else:
                in_burn_window = in_apogee_pass or in_perigee_pass

            del_a_energy = diffCOEs_avg["del_a"][prev_major_time_step]
            del_a_target = max(
                abs(del_a_energy),
                abs(diffCOEs_avg["del_a"][elapsed_time])
            )

            # Possibility for controller to trigger a maneuver when spacecraft
            # is within user-defined bounds, this check prevents that.
            tStep = len(t) - t.index(elapsed_time)
            tHistory = t[-(10 * STEPS_PER_ORBIT + tStep):-tStep]
            subset = [diffCOEs_avg["del_a"][j] for j in tHistory]
            decayRate, _ = np.polyfit(tHistory, subset,  1)

            isNegativeSMA = diffCOEs_avg["del_a"][elapsed_time] < 0
            isNegativeSMATrend = (diffCOEs_avg["del_a"][elapsed_time]
                                  - diffCOEs_avg["del_a"][t[
                                      -(10 * STEPS_PER_ORBIT + tStep)]]) < 0
            isInDeadBand = rv_ric[1] > I_deadband_min * I_bounds
            valid_burn = isNegativeSMA and isNegativeSMATrend and isInDeadBand
            
            if len(burn_starts) > 0:
                if burn_starts[-1][1] != "r":
                    recent_maneuver = True
                else:
                    recent_maneuver = (
                        elapsed_time - burn_ends[-1] >= 3 * PERIOD_IN_SECONDS)
            else:
                recent_maneuver = True

            if in_burn_window and valid_burn and recent_maneuver:
                # Update the controller state
                state = "I burn"
                thruster_axis = "I+"
                maneuver_attempts = 0
                gator_truth = TRUTH_OBJ.satEnginesOn(thruster_axis)

                # Add to the list of when burns start the corresponding color
                # for this maneuver.
                burn_starts.append((elapsed_time, "r"))

                # Establish recovery criteria for I-axis maneuver
                del_a_recovered = False

                # Change simulation step size
                dt = dtThrust

                # reset steps waiting counter
                steps_waiting = 0

            # If waiting in this state for 1 rev, return to "nominal" to
            # prevent lock-up.
            elif steps_waiting == STEPS_PER_ORBIT:
                state = interprupted_state
                interprupted_state = "nominal"
                steps_waiting = 0
            else:
                continue
        case "wait for C burn":
            # Increase number of steps waited
            steps_waiting += 1

            true_lat = (truthCOE[4] + truthCOE[5]) % 360
            critAngle = np.rad2deg(
                np.arctan(
                    diffCOEs_avg["del_raan"][elapsed_time]
                    / diffCOEs_avg["del_i"][elapsed_time] * refCOE[2]
                )
            )
            critAngle += 360 if critAngle < 0 else 0
            if critAngle + maneuverArcHalfAngle * 2 > 360:
                in_node_window = critAngle - maneuverArcHalfAngle * 2 < true_lat or true_lat <= (critAngle + maneuverArcHalfAngle * 2) % 360
            else:
                in_node_window = critAngle - maneuverArcHalfAngle * 2 < true_lat < critAngle + maneuverArcHalfAngle * 2
            if in_node_window:
                thruster_axis = "C-" if critAngle > 180 else "C+"

                gator_truth = TRUTH_OBJ.satEnginesOn(thruster_axis)
                state = "C burn"
                burn_starts.append((elapsed_time, "c"))
                burn_duration = 0.0
                dt = dtThrust

                steps_waiting = 0

            elif steps_waiting == STEPS_PER_ORBIT:
                state = interprupted_state
                interprupted_state = "nominal"
                steps_waiting = 0
            else:
                continue

        # -----------maneuvering----------------------------------------------
        case "R burn":
            burn_duration += dt

            del_aop = diffCOEs_dict["del_aop"][prev_major_time_step]
            in_del_aop_range = -3 <= del_aop < 3

            truth_f = truthCOE[-1]
            in_burn_window = (90 - maneuverArcHalfAngle < truth_f < 90
                              or 270 - maneuverArcHalfAngle < truth_f < 270)
            in_node_window = in_del_aop_range and in_burn_window

            if (
                (burn_duration >= maxDutyTime or not in_node_window)
                and burn_duration >= minDutyTime * 2
            ):
                accel = 0.2 / TRUTH_SAT.mass # m/s
                deltaV = accel * burn_duration
                total_delta_v += deltaV
                if terminal_Completed_Firings:
                    this_burn_start = burn_starts[-1][0] / 86400
                    this_burn_duration = burn_duration / 60
                    r_amp = RIC_Amp_History['R'][prev_major_time_step]
                    get_r_axis_maneuver_print(
                        this_burn_start,
                        burn_duration,
                        thruster_axis,
                        r_amp,
                        deltaV,
                        total_delta_v
                    )
                    
                gator_truth = TRUTH_OBJ.satEnginesOff(thruster_axis)
                state = "returning to nominal from R burn"
                burn_ends.append(elapsed_time)
                dt = dtCoast - round(elapsed_time % dtCoast)

                burn_duration = 0
        case "I burn":
            del_a_energy_maneuver = diffCOEs_avg["del_a"][prev_major_time_step]
            if thruster_axis != "":
                burn_duration += dt

                if (burn_duration >= minDutyTime and maneuver_attempts == 0) or maneuver_attempts > 0:
                    burn_ends.append(elapsed_time)

                    gator_truth = TRUTH_OBJ.satEnginesOff(thruster_axis)
                    thruster_axis = ""

                    dt = dtCoast - round(elapsed_time % dtCoast)
                    maneuver_attempts += 1
                    minIPosition = rv_ric[1]
            else:
                minIPosition = rv_ric[1] if rv_ric[1] < minIPosition else minIPosition
                coast_duration += dt
                dt = dtCoast

                if coast_duration > PERIOD_IN_SECONDS:
                    # Termination conditions:
                    # - achieves deadband target by the time SMA changes sign (no change)
                    # - undershoots deadband target when SMA changes sign (more thrusting)
                    # - overshoots deadband target (less thrusting)

                    # achieves deadband target by the time SMA changes sign (no change)
                    if diffCOEs_avg["del_a"][elapsed_time] < 0 and -I_deadband_min * I_bounds > minIPosition >= -I_bounds:
                        if terminal_Completed_Firings:
                            terminalStr = f"t = {(burn_starts[-1][0] / 86400):2.2f} days | " if (burn_starts[-1][0] / 86400) >= 10 else f"t = {(burn_starts[-1][0] / 86400):1.3f} days | "
                            terminalStr += "I+ burn duration (min) = "
                            terminalStr += f"{(burn_duration / 60):2.2f} | " if (burn_duration / 60) >= 10 else f"{(burn_duration / 60):1.3f} | "
                            terminalStr += f"Recovered del_a = {(del_a_energy_maneuver):0.5f} / {(del_a_target):0.5f} km | "

                            accel = 0.2 / TRUTH_SAT.mass # m/s
                            deltaV = accel * burn_duration
                            total_delta_v += deltaV
                            terminalStr += f"deltaV = {deltaV:1.3f} m/s | " 
                            terminalStr += f"total deltaV = {total_delta_v:1.3f} m/s"
                            print(terminalStr)

                        state = interprupted_state
                        interprupted_state = "nominal"

                        restoreFromTime = burn_ends[-1] - burn_ends[-1] % dtCoast 
                        tStep = t.index(restoreFromTime)
                        tHistoryCOEs = t[(tStep - STEPS_PER_AVG_ORBIT):tStep]
                        for j in COE_KEYS:
                            [diffCOEs_buffer[j].append(diffCOEs_dict[j][k]) for k in tHistoryCOEs]

                        tHistoryRIC = t[int(tStep - 1.5 * STEPS_PER_ORBIT):tStep]

                        gator_ref.Step(-coast_duration)
                        gator_truth.Step(-coast_duration)
                        elapsed_time = elapsed_time - coast_duration

                        burn_duration = 0
                        coast_duration = 0
                        maneuver_log = []

                        # Update numerical integrator references
                        gator_ref.UpdateSpaceObject()
                        gator_truth.UpdateSpaceObject()

                    # undershoots deadband target when SMA changes sign (more thrusting)    
                    elif diffCOEs_avg["del_a"][elapsed_time] < 0 and (diffCOEs_avg["del_a"][elapsed_time] - diffCOEs_avg["del_a"][elapsed_time - 10 * STEPS_PER_ORBIT * dtCoast]) < 0 and -I_deadband_min * I_bounds <= minIPosition:
                        # print(f"Maneuver #{maneuver_attempts}: I-position = {minIPosition:1.4}km | burn time = {burn_duration} sec")

                        restoreFromTime = burn_ends[-1] - burn_ends[-1] % dtCoast 
                        tStep = t.index(restoreFromTime)
                        tHistoryCOEs = t[(tStep - STEPS_PER_AVG_ORBIT):tStep]
                        for j in COE_KEYS:
                            [diffCOEs_buffer[j].append(diffCOEs_dict[j][k]) for k in tHistoryCOEs]

                        tHistoryRIC = t[int(tStep - 1.5 * STEPS_PER_ORBIT):tStep]

                        # Propagate spacecraft
                        gator_ref.Step(-coast_duration)
                        gator_truth.Step(-coast_duration)
                        elapsed_time = elapsed_time - coast_duration
                        coast_duration = 0

                        # Update numerical integrator references
                        gator_ref.UpdateSpaceObject()
                        gator_truth.UpdateSpaceObject()

                        burn_ends.pop()
                        # Update the the elpased time

                        thruster_axis = "I+"
                        estimateSteps = np.ceil((minIPosition + I_deadband_min * I_bounds) / (0.7 if (0 >= minIPosition) else 1))
                        dt = dtThrust * estimateSteps
                        if burn_duration + dt in maneuver_log: 
                            estimateSteps -=1
                            dt = dtThrust * estimateSteps
                        maneuver_log.append(burn_duration)
                        gator_truth = TRUTH_OBJ.satEnginesOn(thruster_axis)

                        if burn_duration + dt < 0:
                            print(f"Negative thrust time!")
                            burn_starts.pop()
                            break
                        if maneuver_attempts == 100:
                            print(f"Max burns!  current burn duration = {burn_duration} sec")
                            burn_starts.pop()
                            break

                    # overshoots deadband target (less thrusting)
                    elif diffCOEs_avg["del_a"][elapsed_time] < 0 and (diffCOEs_avg["del_a"][elapsed_time] - diffCOEs_avg["del_a"][elapsed_time - 10 * STEPS_PER_ORBIT * dtCoast]) < 0 and minIPosition < -I_bounds:
                        # print(f"Maneuver #{maneuver_attempts}: I-position = {minIPosition:1.4}km | burn time = {burn_duration} sec")

                        stepsToBackTrack = abs(np.ceil(minIPosition + I_deadband_min * I_bounds)) # 5
                        if burn_duration - dtThrust * stepsToBackTrack in maneuver_log: 
                            stepsToBackTrack -=1
                        maneuver_log.append(burn_duration)

                        restoreFromTime = burn_ends[-1] - burn_ends[-1] % dtCoast - (stepsToBackTrack * dtThrust // dtCoast) * dtCoast
                        tStep = t.index(restoreFromTime)
                        tHistoryCOEs = t[(tStep - STEPS_PER_AVG_ORBIT):tStep]
                        for j in COE_KEYS:
                            [diffCOEs_buffer[j].append(diffCOEs_dict[j][k]) for k in tHistoryCOEs]

                        tHistoryRIC = t[int(tStep - 1.5 * STEPS_PER_ORBIT):tStep]
                        burn_ends.pop()

                        # Get the updated cartesian states for each spacecraft from the ECI frame
                        rv_ref = gator_ref.GetState()
                        rv_truth = gator_truth.GetState()
                        # Get the updated keplerian states for each spacecraft
                        refCOE = REF_OBJ.sat_wrap.getKeplerianState()
                        truthCOE = TRUTH_OBJ.sat_wrap.getKeplerianState()

                        # Propagate spacecraft
                        backPropStepSize = coast_duration // 300
                        for i in range(300):
                            gator_ref.Step(-backPropStepSize)
                            gator_truth.Step(-backPropStepSize)

                        backStepRemainder = coast_duration % 300
                        gator_ref.Step(-backStepRemainder)
                        gator_truth.Step(-backStepRemainder)

                        elapsed_time = elapsed_time - coast_duration
                        coast_duration = 0

                        # Update numerical integrator references
                        gator_ref.UpdateSpaceObject()
                        gator_truth.UpdateSpaceObject()

                        # Get the updated cartesian states for each spacecraft from the ECI frame
                        rv_ref = gator_ref.GetState()
                        rv_truth = gator_truth.GetState()
                        # Get the updated keplerian states for each spacecraft
                        refCOE = REF_OBJ.sat_wrap.getKeplerianState()
                        truthCOE = TRUTH_OBJ.sat_wrap.getKeplerianState()
                        # Update the the elpased time

                        thruster_axis = "I+"
                        dt = dtThrust
                        gator_truth = TRUTH_OBJ.satEnginesOn(thruster_axis)
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
                            burn_starts.pop()
                            break
                        if maneuver_attempts == 100:
                            print(f"Max burns! current burn duration = {burn_duration} sec | Min I = {minIPosition}")
                            burn_starts.pop()
                            break
                else:
                    RuntimeError("Invalid logic condition during I-axis deadband control")
                    sys.exit()
        case "C burn":
            burn_duration += dt
            velo_phase = np.arctan2(rv_ric[1], rv_ric[2])

            true_lat = (truthCOE[4] + truthCOE[5]) % 360
            if critAngle + maneuverArcHalfAngle * 2 > 360:
                in_cross_track_pass = critAngle - maneuverArcHalfAngle * 2 < true_lat or true_lat <= (critAngle + maneuverArcHalfAngle * 2) % 360
            elif critAngle - maneuverArcHalfAngle * 2 < 0:
                in_cross_track_pass = (critAngle - maneuverArcHalfAngle * 2 + 360) < true_lat or true_lat <= (critAngle + maneuverArcHalfAngle * 2) % 360
            else:
                in_cross_track_pass = critAngle - maneuverArcHalfAngle * 2 < true_lat < critAngle + maneuverArcHalfAngle * 2

            if burn_duration >= maxDutyTime or not in_cross_track_pass:
                if terminal_Completed_Firings:
                    terminalStr = f"t = {(burn_starts[-1][0] / 86400):2.2f} days | " if (burn_starts[-1][0] / 86400) >= 10 else f"t = {(burn_starts[-1][0]) / 86400:1.3f} days | "
                    terminalStr += f"{thruster_axis} burn duration (min) = "
                    terminalStr += f"{(burn_duration / 60):2.2f} | " if (burn_duration / 60) >= 10 else f"{(burn_duration / 60):1.3f} | "
                    terminalStr += f"C-axis Amplitude = {RIC_Amp_History['C'][prev_major_time_step]:0.6f} km         | "

                    accel = 0.2 / TRUTH_SAT.mass # m/s
                    deltaV = accel * burn_duration
                    total_delta_v += deltaV
                    terminalStr += f"deltaV = {deltaV:1.3f} m/s | " 
                    terminalStr += f"total deltaV = {total_delta_v:1.3f} m/s"
                    print(terminalStr)

                gator_truth = TRUTH_OBJ.satEnginesOff(thruster_axis)
                thruster_axis = ""
                state = "returning to nominal from C burn"
                burn_ends.append(elapsed_time)

                burn_duration = 0

                dt = dtCoast - round(elapsed_time % dtCoast)

        # -----------verifying recovery---------------------------------------
        case "returning to nominal from R burn":
            if RIC_Amp_History["R"][prev_major_time_step] <= 1 /2 * R_bounds:
                state = interprupted_state
                interprupted_state = "nominal"
            elif prev_major_time_step - burn_ends[-1] > 0.25*PERIOD_IN_SECONDS:
                state = "wait for R burn"
            else:
                continue

        case "returning to nominal from C burn":
            if diffCOEs_avg["del_raan"][prev_major_time_step] > 0:
                state = interprupted_state
                interprupted_state = "nominal"
            elif prev_major_time_step - burn_ends[-1] > .75 * PERIOD_IN_SECONDS:
                state = "wait for C burn"
            else:
                continue

        case _:
            continue

print(state)
print(f"Current time: T+{(elapsed_time / 86400)} days")

terminalTime = sat_T0 + datetime.timedelta(seconds=elapsed_time)
print(getEpoch_As_ModITC(terminalTime))
print("\nReference COEs:")
output_state = (" " * 4 + f"{i},"
                for i in [*REF_OBJ.sat_wrap.getKeplerianState()])
print(output_state)
print(" " * 4 + f'"{getEpoch_As_Str(terminalTime)}"')
print("\nTruth COEs:")
output_state = ((" " * 4 + f"{i},")
                for i in [*TRUTH_OBJ.sat_wrap.getKeplerianState()])
print(output_state)
print(" " * 4 + f'"{getEpoch_As_Str(terminalTime)}"')

# -----------plots---------------------------------------------------
timings =  [burn_ends, burn_starts, t, REVOLUTIONS_TO_AVERAGE, dtCoast]
coes = [diffCOEs_dict, diffCOEs_avg]
ric = [RIC_History, RIC_Amp_History]
outputPlots(timings, coes, ric)
