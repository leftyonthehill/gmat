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
# Waiting for maneuvers
def controller_wait_for_r(
        elapsed_time: float,
        f_true: float,
        del_aop: float,
        del_e_avg: float,
        steps_waiting: float,
        interrupted_state: str
    ) -> dict:
    """
    Evaluate if the truth spacecraft is in its ideal radial maneuver
    window.

    Returns a dict of fields for the main loop to apply.
    """
    steps_waiting += 1
    
    approaching_90 = 90 - MANEUVER_ARC_HALF_ANGLE < f_true < 90
    approaching_270 = 270 - MANEUVER_ARC_HALF_ANGLE < f_true < 270
    in_node_window = approaching_90 or approaching_270

    in_del_aop_range = abs(del_aop) <= 3

    if in_node_window and in_del_aop_range:
        # Choose thruster direction based on the sign of "del_e" and
        # whether the maneuver window is the 90 or 270 deg true anomaly
        # so that the burn torques the line of apsides toward the
        # reference spacecraft.
        if del_e_avg > 0:
            thruster_axis = "R-" if approaching_90 else "R+"
        else:
            thruster_axis = "R+" if approaching_90 else "R-"

        return {
            "steps_waiting": 0.0,
            "state": "R burn",
            "thruster_axis": thruster_axis,
            "dt": DT_THRUST,
            "starting_time": (elapsed_time, "m")
        }

    elif steps_waiting >= STEPS_PER_ORBIT:
        # Prevent permanent lock-up if the window never appears
        return {
            "steps_waiting": 0.0,
            "state": interrupted_state,
            "interrupted_state": "nominal",
        }
    else:
        return {"steps_waiting": steps_waiting}

def controller_wait_for_i(
        elapsed_time: float,
        f_true: float,
        del_e_avg: float,
        i_pos: float,
        steps_waiting: float,
        maneuver_starts: list,
        maneuver_ends: list,
        interrupted_state: str
) -> dict:
    """
    Evaluate if the truth spacecraft is in its ideal in-track maneuver
    window.

    Returns a dict of fields for the main loop to apply.
    """
    steps_waiting += 1
    
    in_apogee_pass = (
        180 - MANEUVER_ARC_HALF_ANGLE < f_true <= 180)
    in_perigee_pass = (
        360 - MANEUVER_ARC_HALF_ANGLE < f_true <= 360)

    # Prioritize the maneuver window that also reduces "del_e", unless
    # the I-position is already greater than 95% of `I_BOUNDS`. Then
    # take the first available.
    if i_pos / I_BOUNDS < 0.95:
        if del_e_avg <= 0:
            in_burn_window = in_perigee_pass
        else:
            in_burn_window = in_apogee_pass
    else:
        in_burn_window = in_apogee_pass or in_perigee_pass

    if len(maneuver_starts) > 0:
        recent_maneuver = (elapsed_time - maneuver_ends[-1]
                            >= 3 * PERIOD_IN_SECONDS)
    else:
        recent_maneuver = True

    if in_burn_window and recent_maneuver:
        return {
            "steps_waiting": 0,
            "state": "I burn",
            "thruster_axis": "I+",
            "dt": DT_THRUST,
            "starting_time": (elapsed_time, "r"),
        }

    elif steps_waiting >= STEPS_PER_ORBIT:
        # Prevent permanent lock-up if the window never appears
        return {
            "steps_waiting": 0,
            "state": interrupted_state,
            "interrupted_state": "nominal",
        }
    else:
        return {"steps_waiting": steps_waiting}

def controller_wait_for_c(
        elapsed_time: float,
        aop_true: float,
        f_true: float,
        i_ref: float,
        diff_raan_avg: float,
        diff_i_avg: float,
        steps_waiting: float,
        interrupted_state: str
) -> dict:
    """
    Evaluate if the truth spacecraft is in its ideal cross-track
    maneuver window.

    Returns a dict of fields for the main loop to apply.
    """

    steps_waiting += 1
    
    true_lat = (aop_true + f_true) % 360

    # Modified heuristic by H. Schaub and J. Junkins in 'Analytical
    # Mechanics of Space Systems', 4th Ed. This scales "del_i" by 10
    # so that "del_i" and "del_raan" have comparable magnitudes.
    # Otherwise, the original computed the wrong critical angle.
    crit_angle = np.rad2deg(np.arctan(
            diff_raan_avg / (diff_i_avg * 10) * np.sin(np.deg2rad(i_ref))
        )
    )

    # C-axis maneuvers use wider arcs (2 * `MANEUVER_ARC_HALF_ANGLE`)
    crit_angle += 360 if crit_angle < 0 else 0
    window_opens = crit_angle - MANEUVER_ARC_HALF_ANGLE * 2
    window_closes = crit_angle + MANEUVER_ARC_HALF_ANGLE * 2

    if window_opens < 0:
        in_node_window = (true_lat > window_opens + 360
                            or true_lat <= window_closes)
    elif window_closes > 360:
        in_node_window = (true_lat > window_opens
                            or true_lat <= window_closes % 360)
    else:
        in_node_window = window_opens < true_lat < window_closes

    if in_node_window:
        del_raan_is_neg = diff_raan_avg < 0
        if del_raan_is_neg:
            thruster_axis = "C-" if crit_angle >= 180 else "C+"
        else:
            thruster_axis = "C+" if crit_angle < 180 else "C-"

        return {
            "steps_waiting": 0,
            "state": "C burn",
            "thruster_axis": thruster_axis,
            "dt": DT_THRUST,
            "starting_time": (elapsed_time, "c"),
            "crit_angle": crit_angle
        }

    elif steps_waiting >= STEPS_PER_ORBIT:
        # Prevent permanent lock-up if the window never appears
        return {
            "steps_waiting": 0,
            "state": interrupted_state,
            "interrupted_state": "nominal",
        }
    else:
        return {"steps_waiting": steps_waiting}

# Maneuvering
def controller_r_burn(
        elapsed_time: float,
        dt: float,
        burn_duration: float,
        thruster_axis: str,
        total_delta_v: float,
        f_true: float,
        del_aop: float,
        maneuver_start_times: list,
        r_amp: float
) -> dict:
    """ Contains the termination criteria for the radial maneuvers. """
    
    burn_duration += dt
    
    approaching_90 = 90 - MANEUVER_ARC_HALF_ANGLE < f_true < 90
    approaching_270 = 270 - MANEUVER_ARC_HALF_ANGLE < f_true < 270
    in_burn_window = approaching_90 or approaching_270

    in_del_aop_range = abs(del_aop) <= 3

    in_node_window = in_burn_window and in_del_aop_range

    if ((burn_duration >= MAX_DUTY_TIME or not in_node_window)
        and burn_duration >= MIN_DUTY_TIME
    ):
        deltaV = ACCEL[thruster_axis] * burn_duration
        total_delta_v += deltaV

        if PRINT_MANEUVER_MESSAGE:
            this_burn_start = maneuver_start_times[-1][0] / 86400
            this_burn_duration = burn_duration / 60
            get_r_axis_maneuver_print(
                this_burn_start,
                this_burn_duration,
                thruster_axis,
                r_amp,
                deltaV,
                total_delta_v
            )

        return {
            "state": "returning from R burn",
            "burn_duration": 0,
            "total_delta_v": total_delta_v,
            "dt": DT_COAST - round(elapsed_time % DT_COAST),
            "thruster_axis": ""
        }
    else:
        return {"burn_duration": burn_duration}

def _back_prop(
        elapsed_time: float,
        back_prop_time: float,
        propagator_ref: gmat.RungeKutta89,
        propagator_truth: gmat.RungeKutta89,
) -> dict:
    if back_prop_time < 0:
        raise ValueError("The provided time must be positive.")

    backPropStepSize = back_prop_time // 300
    for i in range(300):
        propagator_ref.Step(-backPropStepSize)
        propagator_truth.Step(-backPropStepSize)

    # Back propagate the time remaining
    backStepRemainder = back_prop_time % 300
    propagator_ref.Step(-backStepRemainder)
    propagator_truth.Step(-backStepRemainder)
    propagator_ref.UpdateSpaceObject()
    propagator_truth.UpdateSpaceObject()

    elapsed_time = elapsed_time - back_prop_time

    return {
        "elapsed_time": elapsed_time,
        "propagator_ref": propagator_ref,
        "propagator_truth": propagator_truth
        }

def _reload_diff_coe_buffers(
        t: list,
        reload_from_time: float,
        diff_coe_keys: set,
        diff_coe: dict,
        diff_coe_buffer: dict
) -> dict:
    tStep = t.index(reload_from_time)
    tHistoryCOEs = t[(tStep - STEPS_TO_AVERAGE):tStep]

    # Reload the average diff_coe buffers with data leading
    # up to the timestamp the thruster turned off
    for j in diff_coe_keys:
        for k in tHistoryCOEs:
            diff_coe_buffer[j].append(diff_coe[j][k])

    return diff_coe_buffer

def _i_burn_goldilocks(
        elapsed_time: float,
        coast_duration: float,
        burn_duration: float,
        maneuver_start_time: float,
        maneuver_end_time: float,
        del_a_recovered: float,
        del_a_estimated: float,
        total_delta_v: float,
        interrupted_state: str,
        t: list,
        diff_coe_keys: set,
        diff_coe: dict,
        diff_coe_buffer: dict,
        propagator_ref: gmat.RungeKutta89,
        propagator_truth: gmat.RungeKutta89,
):
    delta_v = ACCEL["I+"] * burn_duration
    total_delta_v += delta_v

    if PRINT_MANEUVER_MESSAGE:
        get_i_axis_print(
            maneuver_start_time,
            burn_duration / 60, # minutes
            del_a_recovered,
            del_a_estimated,
            delta_v,
            total_delta_v
        )

    # Back propagate the scenario to restore buffers at the
    # end of the maneuver.
    return_to_burn_end = round_to_time_step(
        maneuver_end_time
    )

    diff_coe_buffer = _reload_diff_coe_buffers(
        t,
        return_to_burn_end,
        diff_coe_keys,
        diff_coe,
        diff_coe_buffer
    )

    back_prop_result = _back_prop(
        elapsed_time,
        coast_duration,
        propagator_ref,
        propagator_truth
    )

    elapsed_time = back_prop_result["elapsed_time"]
    propagator_ref = back_prop_result["propagator_ref"]
    propagator_truth = back_prop_result["propagator_truth"]

    return {
        "state": interrupted_state,
        "interrupted_state": "nominal",
        "elapsed_time": elapsed_time,
        "diff_coe_buffer": diff_coe_buffer,
        "propagator_ref": propagator_ref,
        "propagator_truth": propagator_truth,
        "total_delta_v": total_delta_v
    }

def _i_burn_undershoot(
        elapsed_time: float,
        coast_duration: float,
        burn_duration: float,
        maneuver_end_times: list,
        t: list,
        diff_coe_keys: set,
        diff_coe: dict,
        diff_coe_buffer: dict,
        propagator_ref: gmat.RungeKutta89,
        propagator_truth: gmat.RungeKutta89,
        maneuver_attempt_log: list,
        maneuver_attempts: int,
        min_i_pos: float,
):
    if PRINT_I_AXIS_MANEUVER_ATTEMPTS:
        i_axis_maneuver_attempt_message(
            maneuver_attempts,
            min_i_pos,
            burn_duration
        )

    maneuver_attempt_log.append(burn_duration)

    # Back propagate the scenario to restore buffers at the
    # end of the maneuver.
    return_to_burn_end = round_to_time_step(
        maneuver_end_times[-1]
    )
    diff_coe_buffer = _reload_diff_coe_buffers(
        t,
        return_to_burn_end,
        diff_coe_keys,
        diff_coe,
        diff_coe_buffer
    )
    
    back_prop_result = _back_prop(
        elapsed_time,
        coast_duration,
        propagator_ref,
        propagator_truth
    )
    
    elapsed_time = back_prop_result["elapsed_time"]
    propagator_ref = back_prop_result["propagator_ref"]
    propagator_truth = back_prop_result["propagator_truth"]
    
    # As an unsuccessful maneuver, remove its end time
    maneuver_end_times.pop()

    # Estimate the time steps needed to correct the
    # undershoot criteria (1 `DT_THRUST` time step per
    # missed Km).
    estimated_steps = np.ceil(
        (min_i_pos + DEADBAND_TRIGGER_RATIO * I_BOUNDS)
    )

    # Verify maneuver duration hasn't been tried to prevent
    # an infinite-loop.
    dt = DT_THRUST
    predicted_burn_duration = (burn_duration
                                + estimated_steps * dt)
    if predicted_burn_duration in maneuver_attempt_log:
        estimated_steps -=1

    # In case this leads to the 100th maneuver attempt,
    # notify the user and exit the station keeping loop
    if maneuver_attempts >= 100:
        raise RuntimeError("Max burns! "
                + "Current burn duration = "
                + f"{burn_duration} sec")
    else:
        return {
            "elapsed_time": elapsed_time,
            "diff_coe_buffer": diff_coe_buffer,
            "propagator_ref": propagator_ref,
            "propagator_truth": propagator_truth,
            "maneuver_end_times": maneuver_end_times,
            "thruster_axis": "I+",
            "dt": dt,
            "estimated_steps": estimated_steps,
            "maneuver_attempt_log": maneuver_attempt_log
        }

def _i_burn_overshoot(
        elapsed_time: float,
        coast_duration: float,
        burn_duration: float,
        maneuver_end_times: list,
        t: list,
        diff_coe_keys: set,
        diff_coe: dict,
        diff_coe_buffer: dict,
        propagator_ref: gmat.RungeKutta89,
        propagator_truth: gmat.RungeKutta89,
        maneuver_attempt_log: list,
        maneuver_attempts: int,
        min_i_pos: float,
):
    if PRINT_I_AXIS_MANEUVER_ATTEMPTS:
        i_axis_maneuver_attempt_message(
            maneuver_attempts,
            min_i_pos,
            burn_duration
        )

    # Estimate the time steps needed to correct the
    # overshoot (1 `DT_THRUST` time step per missed Km)
    stepsToBackTrack = abs(
        np.ceil(
            min_i_pos + DEADBAND_TRIGGER_RATIO * I_BOUNDS
        )
    )

    # Verify maneuver duration hasn't been tried to prevent
    # an infinite-loop.
    if (burn_duration - DT_THRUST * stepsToBackTrack
        in maneuver_attempt_log
    ):
        stepsToBackTrack -=1

    maneuver_attempt_log.append(burn_duration)

    # Back propagate the scenario to restore buffers at the
    # end of the maneuver.
    return_to_burn_end = round_to_time_step(
        maneuver_end_times[-1]
    )
    time_to_backtrack = (
        (stepsToBackTrack * DT_THRUST // DT_COAST)
        * DT_COAST
    )

    # The last major time step in the maneuver window and
    # its index within `t`.
    return_to_burn_end -= time_to_backtrack
    diff_coe_buffer = _reload_diff_coe_buffers(
        t,
        return_to_burn_end,
        diff_coe_keys,
        diff_coe,
        diff_coe_buffer
    )

    back_prop_result = _back_prop(
        elapsed_time,
        coast_duration,
        propagator_ref,
        propagator_truth
    )
        
    elapsed_time = back_prop_result["elapsed_time"]
    propagator_ref = back_prop_result["propagator_ref"]
    propagator_truth = back_prop_result["propagator_truth"]
        
    # As an unsuccessful maneuver, remove its end time
    maneuver_end_times.pop()
    coast_duration = 0

    # Shorten the maneuver
    propagator_truth = TRUTH_OBJ.satEnginesOn("I+")

    dt = DT_THRUST
    backPropTime = -stepsToBackTrack * DT_THRUST
    burn_duration -= stepsToBackTrack * DT_THRUST
    back_prop_result = _back_prop(
        elapsed_time,
        backPropTime,
        propagator_ref,
        propagator_truth
    )
    
    elapsed_time = back_prop_result["elapsed_time"]
    propagator_ref = back_prop_result["propagator_ref"]
    propagator_truth = back_prop_result["propagator_truth"]
        

    # In case the maneuver time goes negative while the
    # algorithm searches for the shorter maneuver time to
    # bring in the overshoot of `I_BOUNDS`, message the
    # user in the terminal and exit the station keeping
    # loop
    if burn_duration + dt <= 0:
        raise RuntimeError("Negative thrust time! Min I = "
                + str(min_i_pos)
        )
    # In case this leads to the 100th maneuver attempt,
    # notify the user and exit the station keeping loop
    elif maneuver_attempts == 100:
        raise RuntimeError("Max burns! current burn duration = "
                + str(burn_duration) + " sec | Min I = "
                + str(min_i_pos)
        )
    else:
        return {
            "elapsed_time": elapsed_time,
            "diff_coe_buffer": diff_coe_buffer,
            "propagator_ref": propagator_ref,
            "propagator_truth": propagator_truth,
            "maneuver_end_times": maneuver_end_times,
            "thruster_axis": "I+",
            "dt": dt,
            "estimated_steps": -1,
            "maneuver_attempt_log": maneuver_attempt_log
        }

def controller_i_burn(
        elapsed_time: float,
        estimated_steps: float,
        maneuver_attempts: int,
        min_i_pos: float,
        i_pos: float,
        dt: float,
        burn_duration: float,
        thruster_axis: str,
        total_delta_v: float,
        maneuver_start_times: list,
        maneuver_end_times: list,
        del_a_current: float,
        del_a_recovered: float,
        del_a_estimated: float,
        coast_duration: float,
        interrupted_state: str,
        t: list,
        diff_coes: dict,
        diff_coes_buffer: dict,
        propagator_ref: gmat.RungeKutta89,
        propagator_truth: gmat.RungeKutta89,
        maneuver_attempt_log: list
    ):
    """ Contains the termination criteria for the in-track maneuvers.
    
    The algorithm to determine the "perfect" maneuver length is as
    follows:
    - Upon entering "I burn" for the first time, fire the thrusters
      for `MIN_DUTY_TIME`
    - Turn off the thrusters and coast until "del_a" drops below 0
      (this condition signifies that the truth spacecraft is no
      longer coasting away from the reference spacecraft but rather
      beginning its approach back)
    - If the maximum negative I-axis position is not greater than
      `DEADBAND_TRIGGER_RATIO`% of `I_BOUNDS`, backwards propagate to
      the end of the maneuver and increase the burn duration
    - If the maximum negative I-axis position is greater than
      `I_BOUNDS`, backwards propagate to the end of the maneuver and
      backwards propagate into the maneuver to reduce the maneuver's
      burn duration
    - If the burn duration is commanded to be negative or the amount
      of maneuver corrections exceeds 100 attempts, the simulation is
      ended
    """
    if thruster_axis != "":
        burn_duration += dt

        # `DT_THRUST` steps remaining this maneuver attempt
        estimated_steps -= 1

        new_maneuver = (burn_duration >= MIN_DUTY_TIME
                        and maneuver_attempts == 0)
        maneuver_attempt = (maneuver_attempts > 0
                            and estimated_steps <= 0)

        if new_maneuver or maneuver_attempt:
            propagator_truth = TRUTH_OBJ.satEnginesOff("I+")
            thruster_axis = ""
            maneuver_end_times.append(elapsed_time)

            # `burn_duration` is not set to 0 here, the maneuver
            # duration may be altered later.

            # Set the simulation time step equal such that
            # `elapsed_time` is aligned with `DT_COAST`
            dt = DT_COAST - round(elapsed_time % DT_COAST)

            maneuver_attempts += 1
            min_i_pos = i_pos

            del_a_recovered = del_a_current

            return {
                "elapsed_time": elapsed_time,
                "burn_duration": burn_duration,
                "estimated_steps": estimated_steps,
                "maneuver_end_times": maneuver_end_times,
                "dt": dt,
                "maneuver_attempts": maneuver_attempts,
                "min_i_pos": min_i_pos,
                "del_a_recovered":del_a_recovered
            }

        return {
            "elapsed_time": elapsed_time,
            "burn_duration": burn_duration,
            "estimated_steps": estimated_steps,
        }
    else:
        if i_pos < min_i_pos:
            min_i_pos = i_pos

        coast_duration += dt

        dt = DT_COAST

        # Wait for at least 1 orbital period and "del_a" must be negative
        # (signifies that the truth spacecraft is now drifting to the
        # reference) before evaluating the termination
        if coast_duration > PERIOD_IN_SECONDS and del_a_current < 0:
            # Termination conditions:
            # - Achieves deadband target by the time SMA changes sign
            #   (no change).
            # - Undershoots deadband target when SMA changes sign (more
            #   thrusting required).
            # - Overshoots deadband target (less thrusting required)

            termination_conditions = [
                DEADBAND_TRIGGER_RATIO < abs(min_i_pos / I_BOUNDS) <= 1,
                abs(min_i_pos / I_BOUNDS) <= DEADBAND_TRIGGER_RATIO,
                abs(min_i_pos / I_BOUNDS) > 1
            ]

            if termination_conditions[0]:
                result = _i_burn_goldilocks(
                    elapsed_time,
                    coast_duration,
                    burn_duration,
                    maneuver_start_times[-1][0] / 86400,
                    maneuver_end_times[-1],
                    del_a_recovered,
                    del_a_estimated,
                    total_delta_v,
                    interrupted_state,
                    t,
                    diff_coes.keys(),
                    diff_coes,
                    diff_coes_buffer,
                    propagator_ref,
                    propagator_truth
                )

                result["burn_duration"] = 0
                result["coast_duration"] = 0
                result["maneuver_attempt_log"] = []

                return result
            elif termination_conditions[1]:
                result = _i_burn_undershoot(
                    elapsed_time,
                    coast_duration,
                    burn_duration,
                    maneuver_end_times,
                    t,
                    diff_coes.keys(),
                    diff_coes,
                    diff_coes_buffer,
                    propagator_ref,
                    propagator_truth,
                    maneuver_attempt_log,
                    maneuver_attempts,
                    min_i_pos
                )

                result["burn_duration"] = burn_duration
                result["coast_duration"] = coast_duration

                propagator_truth = TRUTH_OBJ.satEnginesOn("I+")
                result["propagator_truth"] = propagator_truth
                return result
            elif termination_conditions[2]:
                result = _i_burn_overshoot(
                    elapsed_time,
                    coast_duration,
                    burn_duration,
                    maneuver_end_times,
                    t,
                    diff_coes.keys(),
                    diff_coes,
                    diff_coes_buffer,
                    propagator_ref,
                    propagator_truth,
                    maneuver_attempt_log,
                    maneuver_attempts,
                    min_i_pos
                )

                result["burn_duration"] = burn_duration
                result["coast_duration"] = coast_duration

                propagator_truth = TRUTH_OBJ.satEnginesOn("I+")
                result["propagator_truth"] = propagator_truth
                return result
        return {
            "elapsed_time": elapsed_time,
            "min_i_pos": min_i_pos,
            "coast_duration": coast_duration,
            "dt": dt
        }

def controller_c_burn(
        elapsed_time: float,
        dt: float,
        burn_duration: float,
        thruster_axis: str,
        total_delta_v: float,
        aop_true: float,
        f_true: float,
        crit_angle: float,
        maneuver_start_times: list,
        c_amp: float
) -> dict:
    """
    Contains the termination criteria for the cross-track maneuvers.
    """

    burn_duration += dt

    # C-axis maneuvers use wider arcs (2 * `MANEUVER_ARC_HALF_ANGLE`)
    true_lat = (aop_true + f_true) % 360
    window_opens = crit_angle - MANEUVER_ARC_HALF_ANGLE * 2
    window_closes = crit_angle + MANEUVER_ARC_HALF_ANGLE * 2

    if window_closes > 360:
        in_cross_track_pass = (window_opens < true_lat
                                or true_lat <= window_closes % 360)
    elif window_opens < 0:
        in_cross_track_pass = ((window_opens + 360) < true_lat
                                or true_lat <= window_closes % 360)
    else:
        in_cross_track_pass = (window_opens < true_lat
                                and true_lat < window_closes)


    if burn_duration >= MAX_DUTY_TIME or not in_cross_track_pass:
        delta_v = ACCEL[thruster_axis] * burn_duration
        total_delta_v += delta_v
        if PRINT_MANEUVER_MESSAGE:
            this_burn_start = maneuver_start_times[-1][0] / 86400
            this_burn_duration = burn_duration / 60
            get_c_axis_print(
                this_burn_start,
                this_burn_duration,
                thruster_axis,
                c_amp,
                delta_v,
                total_delta_v
            )
        
        return {
            "state": "returning from C burn",
            "burn_duration": 0,
            "total_delta_v": total_delta_v,
            "dt": DT_COAST - round(elapsed_time % DT_COAST),
            "thruster_axis": ""
        }

    return {"burn_duration": burn_duration}
# Verifying Recovery
def controller_return_from_r(
        r_amp: float,
        prev_major_time_step: float,
        maneuver_end_time: float,
        interrupted_state: str
) -> dict | None:
    """ Verifies there was a good result to the radial maneuver.

    Returns a dict of fields for the main loop to apply.
    """
    # If the amplitude has dropped to less than
    # `DEADBAND_TRIGGER_RATIO` percent of `R_BOUNDS`, return to nominal
    #
    # If the amplitude has not dropped after 1/4 of an orbital period,
    # reenter "wait for R burn"
    if r_amp <= DEADBAND_TRIGGER_RATIO * R_BOUNDS:
        return {
            "state": interrupted_state,
            "interrupted_state": "nominal",
        }
    elif (prev_major_time_step
            - maneuver_end_time > 0.25 * PERIOD_IN_SECONDS
    ):
        return {"state": "wait for R burn"}
    else:
        return None
def controller_return_from_c(
        c_amp: float,
        prev_major_time_step: float,
        maneuver_end_time: float,
        interrupted_state: str
):
    """
    Verifies there was a good result to the cross-track maneuver.

    Returns a dict of fields for the main loop to apply.
    """

    # Determine if the amplitude of the C position oscillation has
    # dropped below `DEADBAND_TRIGGER_RATIO` percent of `C_BOUNDS`
    c_amp_corrected = c_amp / C_BOUNDS < DEADBAND_TRIGGER_RATIO

    if c_amp_corrected:
        return {
            "state": interrupted_state,
            "interrupted_state": "nominal"
        }
    elif (prev_major_time_step
            - maneuver_end_time > .25 * PERIOD_IN_SECONDS
    ):
        # If the amplitude hasn't recovered after 1/4 of an orbit,
        # prepare to try again
        return {"state": "wait for C burn"}
    else:
        return None

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
