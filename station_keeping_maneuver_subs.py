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
from leo_station_keeping_controller import StationKeepingController
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

state = "nominal"
interrupted_state = "nominal"
thruster_axis = ""

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

# ----------------- I-axis Maneuver Support Functions -------------------------
def _back_prop(time, time_to_back_prop: float) -> None:
    time_to_back_prop = abs(time_to_back_prop)

    t_step = time_to_back_prop // 300
    for i in range(300):
        propagator_ref.Step(-t_step)
        propagator_truth.Step(-t_step)

    # Back propagate the time remaining
    remaining_time = time_to_back_prop % 300
    propagator_ref.Step(-remaining_time)
    propagator_truth.Step(-remaining_time)
    propagator_ref.UpdateSpaceObject()
    propagator_truth.UpdateSpaceObject()

    time = time - time_to_back_prop
    return time

def _reload_diff_buffers(reload_from_time: float) -> None:
    t_step = t.index(reload_from_time)
    t_history = t[(t_step - STEPS_TO_AVERAGE):t_step]

    # Reload the average diff_coe buffers with data leading
    # up to the timestamp the thruster turned off
    for key in COE_KEYS:
        for time in t_history:
            diffCOEs_buffer[key].append(diffCOEs[key][time])

    for key in RIC_KEYS:
        for time in t_history:
            RIC_Amp_Buffer[key].append(RIC_History[key][time])

# ----------------- Run Simulation---------------------------------------------
ctrl = StationKeepingController()

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
    if thruster_axis == "" and dt != DT_COAST:
        dt = DT_COAST

    rv_ref = propagator_ref.GetState()
    refCOE = REF_SAT.getKeplerianState()

    rv_truth = propagator_truth.GetState()
    truthCOE = TRUTH_SAT.getKeplerianState()

    rv_ric, _ = xyz2ric(rv_ref, rv_truth)

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

            if j == 1 and amp > 10:
                amp
                temp2 = (
                    1.5 * STEPS_PER_ORBIT == len(RIC_Amp_Buffer[RIC_KEYS[j]]))
                vbn = 1

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

    # Update internals of controller
    ctrl.amp_ric = {key: value[prev_major_time_step] for key, value in RIC_Amp_History.items()}
    ctrl.coes_instant_diff = {key: value[prev_major_time_step] for key, value in diffCOEs.items()}
    ctrl.coes_avg_diff = {key: value[prev_major_time_step] for key, value in diffCOEs_avg.items()}
    ctrl.rv_ric = rv_ric
    ctrl.truth_coes = truthCOE
    ctrl.ref_coes = refCOE

    result = ctrl.update(elapsed_time, ACCEL, thruster_axis)
    match result.get("action","continue"):
        case "start_burn":
            state = result["new_state"]
            thruster_axis = result["thruster_axis"]
            dt = result["dt"]

            propagator_truth = TRUTH_OBJ.satEnginesOn(thruster_axis)

        case "stop_waiting":
            state = result["new_state"]
            interrupted_state = result["interrupted_state"]

        case "stop_burn":
            state = result["new_state"]
            dt = result["dt"]

            if thruster_axis[0] == "R" or thruster_axis[0] == "C":
                maneuver_duration = result["maneuver_duration"]
                maneuver_delta_v = result["maneuver_delta_v"]
                total_delta_v = result["total_delta_v"]

            if PRINT_MANEUVER_MESSAGE:
                if thruster_axis[0] == "R":
                    get_r_axis_print(
                        ctrl.maneuver_starts[-1][0] / 86400,
                        maneuver_duration / 60,
                        thruster_axis,
                        ctrl.amp_ric["R"],
                        maneuver_delta_v,
                        total_delta_v
                    )
                elif thruster_axis[0] == "C":
                    get_c_axis_print(
                        ctrl.maneuver_starts[-1][0] / 86400,
                        maneuver_duration / 60,
                        thruster_axis,
                        ctrl.amp_ric["C"],
                        maneuver_delta_v,
                        total_delta_v
                    )

            propagator_truth = TRUTH_OBJ.satEnginesOff(thruster_axis)
            thruster_axis = ""

        case "successful_maneuver":
            state = result["new_state"]
            interrupted_state = result["interrupted_state"]

        case "maneuver_required":
            state = result["new_state"]

        case "successful_i_maneuver":
            state = result["new_state"]
            interrupted_state = result["interrupted_state"]
            dt = result["dt"]
            maneuver_duration = result["maneuver_duration"]
            delta_v = result["maneuver_delta_v"]
            total_delta_v = result["total_delta_v"]
            back_prop_coast_time = result["backtrack_coast_time"]

            if PRINT_MANEUVER_MESSAGE:
                get_i_axis_print(
                    ctrl.maneuver_starts[-1][0] / 86400,
                    maneuver_duration / 60,
                    ctrl.del_a_recovered,
                    ctrl.del_a_estimated,
                    delta_v,
                    total_delta_v
                )

            elapsed_time = _back_prop(elapsed_time, back_prop_coast_time)
            _reload_diff_buffers(round_to_time_step(ctrl.maneuver_ends[-1]))
            thruster_axis = ""

        case "back_prop_coast":
            dt = result["dt"]
            burn_end_time = result["burn_end_time"]
            back_prop_coast_time = result["back_track_coast_time"]

            elapsed_time = _back_prop(elapsed_time, back_prop_coast_time)
            _reload_diff_buffers(round_to_time_step(burn_end_time))

            thruster_axis = "I+"
            propagator_truth = TRUTH_OBJ.satEnginesOn(thruster_axis)

        case "back_prop_coast_and_burn":
            dt = result["dt"]
            burn_end_time = result["burn_end_time"]
            back_prop_coast_time = result["back_track_coast_time"]
            back_prop_burn_time = result["back_track_burn_time"]

            elapsed_time = _back_prop(elapsed_time, back_prop_coast_time)
            _reload_diff_buffers(round_to_time_step(burn_end_time))

            thruster_axis = "I+"
            propagator_truth = TRUTH_OBJ.satEnginesOn(thruster_axis)
            elapsed_time = _back_prop(elapsed_time, back_prop_burn_time)
        case "":
            break
        case _:
            continue

# ----------------- Outputs ---------------------------------------------------
timings =  [
    ctrl.maneuver_ends,
    ctrl.maneuver_starts,
    t,
    REVOLUTIONS_TO_AVG,
    DT_COAST,
    STEPS_TO_AVERAGE
]
coes = [
    diffCOEs,
    diffCOEs_avg
]
ric = [
    RIC_History,
    RIC_Amp_History
]
output_plots(timings, coes, ric)
