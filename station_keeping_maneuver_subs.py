""" Station keeping scenario starting point. 

This script drives a two-satellite (reference and truth) GMAT scenario
and uses a state-machine controller to keep the truth spacecraft within
a user-defined operational bounds of the reference spacecraft in the
Radial/In-Track/Cross-Track (RIC) frame. The reference spacecraft is
only perturbed by Earth's geopotential (4x4 model), while the truth
spacecraft carries electric thrusters in the +/-R, +/-I, +/-C
directions to counter the same Earth geopotential model, atmospheric
drag, solar radiation pressure, and third body effects (Sun and Moon).
Over time, the truth spacecraft drifts away from its reference and it
must be corrected.

Main loop
---------
Each iteration:
1. Step forward both spacecraft's RK89 integrators by `dt` (`DT_COAST`
   while coasting, `DT_THRUST` while thrusting).
2. Compute the truth spacecraft's Cartesian offset from its reference
   in the RIC frame (using xyz2ric).
3. Compute the differences between each Keplerian element between both
   spacecraft (truth_element - reference_element)
4. At each `DT_COAST`-aligned time step, perform the following updates:
   - `RIC_History` / `diffCOEs`: instantaneous values
   - `RIC_Amp_History`: RIC position/velocity oscillation amplitudes
     (via a rolling `RIC_amp_Buffer` containing one orbit's worth of
     values).
   - `diffCOEs_avg`: averaged diff_coe difference (via a rolling
     `diffCOEs_buffer` over `REVOLUTIONS_TO_AVG` orbits).
5. Check for any R/I/C boundary violations and, if the controller is
   not already performing a higher-priority correction, prepare for a
   maneuver in the corresponding "wait for <axis> burn" state.

State machine
-------------
States are tracked in `state` / `interrupted_state`. The maneuver
axis priority is I > C > R and is enforced by the sets `I_OVERRIDE`,
`C_OVERRIDE`, and `R_OVERRIDE`.

Notes
-----
- `elapsed_time` and `t` are tracked in integer seconds. Time is only
  converted to days at plot time and for any maneuver notifications in
  the terminal.
- Backwards propagation operations (<integrator>.Step(-<time>)) rely on
  RK89 being numerically irreversible only to within simulation
  tolerance. Small discontinuities at these seams are expected and
  acceptable.
- Thruster, force model, and propagator setups are delegated to
  `StationKeepingObjects`. This script only owns the control logic and
  telemetry collection.

Outputs
-------
Prints the terminal state, elapsed time, terminal epoch, and final
Keplerian elements for both spacecraft, then calls `outputPlots()` to
render RIC position/velocity, oscillation-amplitude, and diff_coe-
difference plots (see `simulationParameters.py` for which plots are
enabled and `plotting.py` for details).
"""

# Native libraries
from collections import deque

# 3rd party libraries
import numpy as np
from createStationKeepingObjects import StationKeepingObjects
from load_gmat import gmat
from data_outputs import output_plots
from simulationParameters import (
    C_BOUNDS,
    DEADBAND_TRIGGER_RATIO,
    DT_COAST,
    DT_THRUST,
    I_BOUNDS,
    MANEUVER_ARC_HALF_ANGLE,
    MAX_DAYS,
    MAX_DUTY_TIME,
    MIN_DUTY_TIME,
    ORBIT_STATE,
    PRINT_I_AXIS_MANEUVER_ATTEMPTS,
    PRINT_MANEUVER_MESSAGE,
    R_BOUNDS,
    REF_ORBIT_STATE,
    REVOLUTIONS_TO_AVG,
    STATE_VECT_SOURCE,
    TRUTH_ORBIT_STATE,
)
from supportFunctions import *

# ----------------- Create Variables ------------------------------------------
MU = 398600  # Earth’s gravitational parameter in km^3/s^2

if STATE_VECT_SOURCE == "new":
    MEAN_MOTION = np.sqrt(MU / ORBIT_STATE[0]**3)
else:
    MEAN_MOTION = np.sqrt(MU / REF_ORBIT_STATE[0]**3)

PERIOD_IN_SECONDS = 2 * np.pi / MEAN_MOTION
STEPS_PER_ORBIT = int(np.ceil(PERIOD_IN_SECONDS / DT_COAST))
STEPS_TO_AVERAGE = int(REVOLUTIONS_TO_AVG * STEPS_PER_ORBIT)


TOTALSECONDS = MAX_DAYS * 86400
NUMBER_OF_TIME_STEPS = int(TOTALSECONDS / DT_COAST) + 1
t = [DT_COAST * i
     for i in range(0, NUMBER_OF_TIME_STEPS)]

# Time keeping
elapsed_time = 0.0
burn_duration = 0.0
coast_duration = 0.0
steps_waiting_for_maneuver = 0.0

# Maneuver bookkeeping
maneuver_start_times = []
estimated_steps = 0.0
maneuver_attempt_log = []
maneuver_end_times = []
maneuver_attempts = 0.0
total_delta_v = 0.0
recent_maneuver = False

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
diffCOEs = {key: {0.0:0.0}
                 for key in COE_KEYS}
diffCOEs_avg = {key: {0.0:0.0}
                for key in COE_KEYS}
diffCOEs_buffer = {key: deque([0.0], maxlen=int(STEPS_TO_AVERAGE))
                   for key in COE_KEYS}

# Used in "wait for I burn" and "I burn" blocks to estimate and track the
# necessary increase in orbital energy to overcome atmospheric drag.
# These values are not consulted for maneuver termination, but as a way for the
# user to monitor the actual increases in orbital energy.
del_a_estimated = 0.0 # Expected increase to cross I-axis deadband
del_a_current = 0.0 # Instantaneous "del_a"
del_a_recovered = 0.0 # Achieved "del_a" post maneuver

# Tracks the minimum I-position during each maneuver attempt
min_i_pos = 0.0

# Informs C-axis maneuvers
crit_angle = 0.0
del_raan_is_neg = False

state = "nominal"
interrupted_state = "nominal"

# States that take priority over the corresponding maneuver axis
I_OVERRIDE = {"wait for I burn", "R burn", "I burn", "C burn"}
C_OVERRIDE = I_OVERRIDE | {"wait for C burn", "returning from C burn"}
R_OVERRIDE = C_OVERRIDE | {"wait for R burn", "returning from R burn"}

# ----------------- Configure Object Preliminaries ----------------------------
# Reference Objects
REF_OBJ = StationKeepingObjects("reference")
REF_SAT = REF_OBJ.sat_wrap
if STATE_VECT_SOURCE == "new":
    REF_SAT.setKeplerianState(ORBIT_STATE)
else:
    REF_SAT.setKeplerianState(REF_ORBIT_STATE)

# Truth Objects
TRUTH_OBJ = StationKeepingObjects("truth")
TRUTH_SAT = TRUTH_OBJ.sat_wrap
if STATE_VECT_SOURCE == "new":
    TRUTH_SAT.setKeplerianState(ORBIT_STATE)
else:
    TRUTH_SAT.setKeplerianState(TRUTH_ORBIT_STATE)
TRUTH_OBJ.setManeuverable()
ACCEL = TRUTH_SAT.accelerations

# Initialize the GMAT scenario
gmat.Initialize()
t0 = ORBIT_STATE[-1] if STATE_VECT_SOURCE == "new" \
    else getEpoch_As_Datetime(TRUTH_ORBIT_STATE[-1])
dt = DT_COAST
# ----------------- Build Out Thruster Forces ---------------------------------
# Reference Objects
REF_OBJ.preparePropInternal()
propagator_ref = REF_OBJ.prop_wrap["coast"].prop_gmat.GetPropagator()

# Truth Objects
TRUTH_OBJ.setBurnForces()
TRUTH_OBJ.preparePropInternal()
propagator_truth = TRUTH_OBJ.prop_wrap["coast"].prop_gmat.GetPropagator()

# ----------------- Maneuver Handling Methods ---------------------------------

# ----------------- Run Simulation---------------------------------------------
while elapsed_time < TOTALSECONDS:

    # If not maneuvering and `elapsed_time` is not aligned with `DT_COAST`,
    # temporarily change `dt` so that the next step is a major time step.
    if (state not in ["R burn", "I burn", "C burn"]
        and elapsed_time % DT_COAST != 0
    ):
        dt -= elapsed_time % DT_COAST

    propagator_ref.Step(dt)
    propagator_ref.UpdateSpaceObject()

    propagator_truth.Step(dt)
    propagator_truth.UpdateSpaceObject()

    elapsed_time += dt
    prev_major_time_step = round_to_time_step(elapsed_time)

    # If in between maneuvers, verify `dt` is equal to `DT_COAST`
    if state not in ["R burn", "I burn", "C burn"] and dt != DT_COAST:
        dt = DT_COAST

    rv_ref = propagator_ref.GetState()
    refCOE = REF_SAT.getKeplerianState()

    rv_truth = propagator_truth.GetState()
    truthCOE = TRUTH_SAT.getKeplerianState()

    rv_ric, _ = xyz2ric(rv_ref, rv_truth)
    del_a_current = truthCOE[0] - refCOE[0]

    # If `elapsed_time` is a multiple of `DT_COAST`, collect telemetry
    if elapsed_time % DT_COAST == 0:
        for j in range(6):
            RIC_History[RIC_KEYS[j]][elapsed_time] = rv_ric[j]
            RIC_Amp_Buffer[RIC_KEYS[j]].append(rv_ric[j])

            # Rolling amplitude: once the buffer holds 1.5 orbits worth of
            # data, use 1/2 of peak-to-peak. Otherwise, fall back to rolling
            # maximum.
            if len(RIC_Amp_Buffer[RIC_KEYS[j]]) == 1.5 * STEPS_PER_ORBIT:
                amp = (max(RIC_Amp_Buffer[RIC_KEYS[j]])
                       - min(RIC_Amp_Buffer[RIC_KEYS[j]])) / 2
            else:
                amp = max(RIC_Amp_Buffer[RIC_KEYS[j]])

            RIC_Amp_History[RIC_KEYS[j]][elapsed_time] = amp

            diff_coe = truthCOE[j] - refCOE[j]

            # Keep RAAN, AOP, and True Anomaly differences in [-180, 180] deg
            quad_correction = 0
            if j > 1:
                if diff_coe > 180:
                    quad_correction = -360
                elif diff_coe < -180:
                    quad_correction = 360
            corrected_coe = diff_coe + quad_correction

            diffCOEs[COE_KEYS[j]][elapsed_time] = corrected_coe
            diffCOEs_buffer[COE_KEYS[j]].append(corrected_coe)

            # Rolling average: once the buffer holds `REVOLUTIONS_TO_AVG`
            # orbits worth of data, find the average of the buffer. Otherwise,
            # use the instantaneous value as the average.
            if len(diffCOEs_buffer[COE_KEYS[j]]) == STEPS_TO_AVERAGE:
                avg_value = float(np.mean(diffCOEs_buffer[COE_KEYS[j]]))
            else:
                avg_value = corrected_coe

            diffCOEs_avg[COE_KEYS[j]][elapsed_time] = avg_value

        # At each major time step, evaluate if there have been any boundary
        # violations.
        boundary_violations = {
            "R": RIC_Amp_History["R"][elapsed_time] > R_BOUNDS,
            "I": rv_ric[1] > DEADBAND_TRIGGER_RATIO * I_BOUNDS,
            "C": RIC_Amp_History["C"][elapsed_time] > C_BOUNDS,
        }

        # If a state change is necessary, verify no higher-priority state is
        # currently selected.
        if (boundary_violations["I"]
            and state not in I_OVERRIDE
        ):
            interrupted_state = state
            state = "wait for I burn"

        elif (boundary_violations["C"]
              and state not in C_OVERRIDE
              and interrupted_state == "nominal"
        ):
            interrupted_state = state
            state = "wait for C burn"

        elif (boundary_violations["R"]
              and state not in R_OVERRIDE
              and interrupted_state == "nominal"
        ):
            interrupted_state = state
            state = "wait for R burn"

    match state:
        case "wait for R burn":
            result = controller_wait_for_r(
                elapsed_time,
                truthCOE[-1],
                diffCOEs["del_aop"][prev_major_time_step],
                diffCOEs_avg["del_e"][prev_major_time_step],
                steps_waiting_for_maneuver,
                interrupted_state
            )

            steps_waiting_for_maneuver = result["steps_waiting"]

            if "dt" in result:
                state = result["state"]
                thruster_axis = result["thruster_axis"]
                dt = result["dt"]
                starting_time = result["starting_time"]

                propagator_truth = TRUTH_OBJ.satEnginesOn(thruster_axis)
                maneuver_start_times.append(starting_time)
            elif "interrupted_state" in result:
                state = result["state"]
                interrupted_state = result["interrupted_state"]

        case "wait for I burn":
            result = controller_wait_for_i(
                elapsed_time,
                truthCOE[-1],
                diffCOEs_avg["del_e"][prev_major_time_step],
                rv_ric[1],
                steps_waiting_for_maneuver,
                maneuver_start_times,
                maneuver_end_times,
                interrupted_state
            )
            
            steps_waiting_for_maneuver = result["steps_waiting"]

            if "dt" in result:
                state = result["state"]
                thruster_axis = result["thruster_axis"]
                dt = result["dt"]
                starting_time = result["starting_time"]

                del_a_estimated = abs(diffCOEs_avg["del_a"][prev_major_time_step])
                propagator_truth = TRUTH_OBJ.satEnginesOn(thruster_axis)
                maneuver_start_times.append(starting_time)
                maneuver_attempts = 0
            elif "interrupted_state" in result:
                state = result["state"]
                interrupted_state = result["interrupted_state"]

        case "wait for C burn":
            result = controller_wait_for_c(
                elapsed_time,
                truthCOE[4],
                truthCOE[5],
                refCOE[2],
                diffCOEs_avg["del_raan"][prev_major_time_step],
                diffCOEs_avg["del_i"][prev_major_time_step],
                steps_waiting_for_maneuver,
                interrupted_state
            )

            steps_waiting_for_maneuver = result["steps_waiting"]

            if "dt" in result:
                state = result["state"]
                thruster_axis = result["thruster_axis"]
                dt = result["dt"]
                starting_time = result["starting_time"]

                propagator_truth = TRUTH_OBJ.satEnginesOn(thruster_axis)
                maneuver_start_times.append(starting_time)
                del_raan_is_neg = (
                    diffCOEs_avg["del_raan"][prev_major_time_step] < 0)
                crit_angle = result["crit_angle"]
            elif "interrupted_state" in result:
                state = result["state"]
                interrupted_state = result["interrupted_state"]

        # --------- Maneuvering -----------------------------------------------
        case "R burn":
            result = controller_r_burn(
                elapsed_time,
                dt,
                burn_duration,
                thruster_axis,
                total_delta_v,
                truthCOE[-1],
                diffCOEs["del_aop"][prev_major_time_step],
                maneuver_start_times,
                RIC_Amp_History["R"][prev_major_time_step]
            )

            burn_duration = result["burn_duration"]
            if "dt" in result:
                state = result["state"]
                total_delta_v = result["total_delta_v"]
                dt = result["dt"]

                propagator_truth = TRUTH_OBJ.satEnginesOff(thruster_axis)
                thruster_axis = result["thruster_axis"]
                maneuver_end_times.append(elapsed_time)

        case "I burn":
            result = controller_i_burn(
                elapsed_time,
                estimated_steps,
                maneuver_attempts,
                min_i_pos,
                rv_ric[1],
                dt,
                burn_duration,
                thruster_axis,
                total_delta_v,
                maneuver_start_times,
                maneuver_end_times,
                del_a_current,
                del_a_recovered,
                del_a_estimated,
                coast_duration,
                interrupted_state,
                t,
                diffCOEs,
                diffCOEs_buffer,
                propagator_ref,
                propagator_truth,
                maneuver_attempt_log
            )

            elapsed_time = result["elapsed_time"]

            if "diff_coe_buffer" in result:
                diffCOEs_buffer = result["diff_coe_buffer"]
                propagator_ref = result["propagator_ref"]
                propagator_truth = result["propagator_truth"]
                maneuver_attempt_log = result["maneuver_attempt_log"]

            if "burn_duration" in result:
                burn_duration = result["burn_duration"]
            if "coast_duration" in result:
                coast_duration = result["coast_duration"]
            if "estimated_steps" in result:                
                estimated_steps = result["estimated_steps"]
            if "maneuver_end_times" in result:
                maneuver_end_times = result["maneuver_end_times"]
            if "dt" in result:
                dt = result["dt"]
            if "maneuver_attempts" in result:
                maneuver_attempts = result["maneuver_attempts"]
            if "min_i_pos" in result:
                min_i_pos = result["min_i_pos"]
            if "del_a_recovered" in result:
                del_a_recovered = result["del_a_recovered"]

            if "state" in result:
                state = result["state"]
                interrupted_state = result["interrupted_state"]
                total_delta_v = result["total_delta_v"]
            if "thruster_axis" in result:
                thruster_axis = result["thruster_axis"]
            if "propagator_truth" in result:
                propagator_truth = result["propagator_truth"]

        case "C burn":
            result = controller_c_burn(
                elapsed_time,
                dt,
                burn_duration,
                thruster_axis,
                total_delta_v,
                truthCOE[4],
                truthCOE[5],
                crit_angle,
                maneuver_start_times,
                RIC_Amp_History["C"][prev_major_time_step]
            )

            burn_duration = result["burn_duration"]

            if "dt" in result:
                state = result["state"]
                total_delta_v = result["total_delta_v"]
                dt = result["dt"]

                propagator_truth = TRUTH_OBJ.satEnginesOff(thruster_axis)
                thruster_axis = result["thruster_axis"]
                maneuver_end_times.append(elapsed_time)
            
        # --------- Verifying Recovery ----------------------------------------
        case "returning from R burn":
            result = controller_return_from_r(
                RIC_Amp_History["R"][prev_major_time_step],
                prev_major_time_step,
                maneuver_end_times[-1],
                interrupted_state
            )

            if result is None:
                continue

            state = result["state"]
            if "interrupted_state" in result:
                interrupted_state = result["interrutped_state"]

        case "returning from C burn":
            result = controller_return_from_c(
                RIC_Amp_History["C"][prev_major_time_step],
                prev_major_time_step,
                maneuver_end_times[-1],
                interrupted_state
            )

            if result is None:
                continue
            
            state = result["state"]
            if "interrupted_state" in result:
                interrupted_state = result["interrupted_state"]

        case _:
            continue

# ----------------- Outputs ---------------------------------------------------
timings =  [maneuver_end_times, maneuver_start_times, t, REVOLUTIONS_TO_AVG, DT_COAST, STEPS_TO_AVERAGE]
coes = [diffCOEs, diffCOEs_avg]
ric = [RIC_History, RIC_Amp_History]
output_plots(timings, coes, ric)
