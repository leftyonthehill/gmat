""" Station keeping scenario starting point. 

This script drives a two-satellite (reference and truth) GMAT scenario
and uses a state-machine controller to keep the truth spacecraft within
a user-defined operational bounds of the reference spacecraft in the
Radial/In-Track/Cross-Track (RIC) frame. The reference spacecraft is
only perturbed by Earth's geopotential (4x4 model), while the truth
spacecraft carries electric thrusters in the +/-R, +/-I, +/-C
directions to counter teh same Earth geopotential model, atmospheric
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
   - `RIC_History` / `diffCOEs_dict`: instantaneous values
   - `RIC_Amp_History`: RIC position/velocity oscillation amplitudes
     (via a rolling `RIC_amp_Buffer` containing one orbit's worth of
     values).
   - `diffCOEs_avg`: averaged COE difference (via a rolling
     `diffCOEs_buffer` over `REVOLUTIONS_TO_AVERAGE` orbits).
5. Check for any R/I/C boundary violations and, if the controller is
   not already performing a higher-priority correction, prepare for a
   maneuver in the corresponding "wait for <axis> burn" state.

State machine
-------------
States are tracked in `state`/`interupted_state`. The sets
`I_OVERRIDE`, `C_OVERRIDE`, and `R_OVERRIDE` are used to determine if
the current state can be overwritten. This script prioritizes its
maneuvers I then C then R. States:

- "nominal": Coast phase. Only orbital state information is collected
- "wait for R burn": Waits for the truth and reference arguements of
  perigee to nearly align (within +/- 3 deg) and the truth spacecraft's
  true anomaly to approach 90 deg or 270 deg (offset by
  `MANEUVER_AR_HALF_ANGLE`). When these conditions are met, turns on the
  R-axis thruster that corresponds with the sign of "del_e" (the
  truth - reference difference in eccentricity).
- "wait for I burn": Waits for the truth spacecraft's true anomaly to
  approach the reference spacecraft's perigee or apogee. The particular
  maneuver window is decided by "del_e". When this condition is met,
  turn on the I+ axis thruster.
- "wait for C burn": Waits for true latitude (phase angle) to
  approach the 'critical angle', a heuristic by H. Schuab and J.
  Junkins in 'Analytical Mechanics of Space Systems', 4th Ed, to
  compute where in the orbit is the ideal opportunity to correct both
  "del_raan" and "del_i" (the truth - reference difference in RAAN and
  inclination). Upon entering the maneuver window, turn on the C-axis
  thruster that corresponds with the sign of "del_raan" and the
  quadrant of `crit_angle`.
- "R burn": Fires until the spacecraft exits the target window (and a
  minimum duty time has elapsed) or `maxDutyTime` is reached. This
  axis's recovery logic is intentionally simple (fire once, then
  return to nominal) since R-axis drift is naturally small/bounded for
  this vehicle class and R burns are rarely required.
- "I burn": Burns for `minDutyTime`, then coasts up to one orbital
  period while tracking the minimum I-axis position (`min_i_pos`)
  reached. At the end of that coast, compares the achieved `del_a`
  recovery against `del_a_target` (a "mirror" overshoot target, not
  just past zero, to maximize coast time before the next maneuver) and
  either:
    - accepts the maneuver and returns to nominal,
    - re-ignites for additional estimated thrust duration if it
      undershot, backing up the propagators in time to resume exactly
      where the previous burn left off, or
    - backtracks (steps the propagators backward) and re-ignites for a
      shorter duration if it overshot.
  Both correction paths rebuild `diffCOEs_buffer`/`RIC_Amp_Buffer`
  history from the `t` timeline so post-maneuver averages/amplitudes
  stay consistent after the time rewind. `maneuver_attempts` guards
  against runaway retry loops.
- "C burn": Fires until the spacecraft leaves the node-crossing window
  or `maxDutyTime` is reached.
- "returning from R burn": Waits for the R-axis amplitude
  to decay to `DEADBAND_TRIGGER_RATIO`. Otherwise, returns to "wait
  for R burn" after 1/4 of an orbital period.
- "returning from C burn": Waits for the oscillation
  amplitude in the C-axis position to drop below
  `DEADBAND_TRIGGER_RATIO` of `C_BOUNDS`. Otherwise, returns to "wait
  for C burn" after 1/4 of an orbital period.

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
render RIC position/velocity, oscillation-amplitude, and COE-
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
    PRINT_I_AXIS_MANEUVER_ATTEMPS,
    PRINT_MANEUVER_MESSAGE,
    R_BOUNDS,
    REF_ORBIT_STATE,
    REVOLUTIONS_TO_AVERAGE,
    STATE_VECT_SOURCE,
    TRUTH_ORBIT_STATE,
)
from supportFunctions import *

# ----------- Create Variables ------------------------------------------------
MU = 398600  # Earth’s gravitational parameter in km^3/s^2

# Timers to measure the length of a burn or coast phase
burn_duration = 0
coast_duration = 0

# Depending on the state vector choice, compute the respective mean motion
if STATE_VECT_SOURCE == "new":
    MEAN_MOTION = np.sqrt(MU / ORBIT_STATE[0]**3)
else:
    MEAN_MOTION = np.sqrt(MU / REF_ORBIT_STATE[0]**3)

# Initial orbital period in seconds
PERIOD_IN_SECONDS = 2 * np.pi / MEAN_MOTION

# Number of simulation steps in 1 orbit around Earth
STEPS_PER_ORBIT = int(np.ceil(PERIOD_IN_SECONDS / DT_COAST))

# Number of simulation steps needed to average perturbations
STEPS_PER_AVG_ORBIT = int(REVOLUTIONS_TO_AVERAGE * STEPS_PER_ORBIT) + 5

# Collects the sum total of delta_v consumed by each maneuver
total_delta_v = 0

# Scenario time variables
elapsed_time = 0.0
TOTALSECONDS = MAX_DAYS * 86400
NUMBER_OF_TIME_STEPS = int(TOTALSECONDS / DT_COAST) + 1
t = [DT_COAST * i
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

# Estimates increase in the truth spacecraft's semi-major axis to transit
# across the deadband
del_a_target = 0

# Measures the instantaneous difference in the semi-major axis between the
# truth and reference states
del_a_current = 0
del_a_recovered = 0

# Estimates `dtThrust` steps to increase I-axis maneuver by
estimated_steps = 0

# Tracks the number of attempts during I-axis maneuvers
maneuver_attempts = 0

# Tracks the duration of prior maneuver attemps to prevent repeat maneuver
# durations
maneuver_log = []

# Prevents the retrigger of a maneuver within a specified time
recent_maneuver = False

# Logs the minimum I-axis position after an I-axis maneuver
min_i_pos = 0

# Boolean for when "del_raan" has a natrual negative drift (different force
# models in the truth and reference force models can vary the rate of
# "del_raan")
raan_decay = False

# Log of maneuver start and end times
burn_starts = []
burn_ends = []

# Step counter to prevent controller from being stuck waiting to maneuver
steps_waiting = 0

# Initial state of the simulation
state = "nominal"

# In case of a state override, what was the previous state
interprupted_state = "nominal"

# States that take priority over the corresponding maneuver axis
I_OVERRIDE = {"wait for I burn", "R burn", "I burn", "C burn"}
C_OVERRIDE = I_OVERRIDE | {"wait for C burn", "returning from C burn"}
R_OVERRIDE = C_OVERRIDE | {"wait for R burn", "returning from R burn"}

# ----------- Configure Object Preliminaries ----------------------------------
# Reference Objectes
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
# Initialize the scenario
gmat.Initialize()
t0 = ORBIT_STATE[-1] if STATE_VECT_SOURCE == "new" \
    else getEpoch_As_Datetime(TRUTH_ORBIT_STATE[-1])
# ------------ Build Out Thruster Forces --------------------------------------
# Reference Objectes
REF_OBJ.preparePropInternal()
gator_ref = REF_OBJ.prop_wrap["coast"].prop_gmat.GetPropagator()
# Truth Objects
TRUTH_OBJ.setBurnForces()
TRUTH_OBJ.preparePropInternal()
gator_truth = TRUTH_OBJ.prop_wrap["coast"].prop_gmat.GetPropagator()

# ------------ Run Simulation--------------------------------------------------
# Set simulation step size
dt = DT_COAST

# Get initial integrator for the truth satellite
gator_truth = TRUTH_OBJ.satEnginesOff("coast")

# While the elapsed_time time is less the max number of days
while elapsed_time < TOTALSECONDS:

    if all([state not in ["R burn", "I burn", "C burn"],
            elapsed_time % DT_COAST != 0]):
        dt -= elapsed_time % DT_COAST

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
    prev_major_time_step = round_to_time_step(elapsed_time)

    # If the truth satellite is not coasting, verify `dt` is equal to
    # `DT_COAST`
    if state not in ["R burn", "I burn", "C burn"] and dt != DT_COAST:
        dt = DT_COAST

    # Get the updated cartesian states for each spacecraft from the ECI frame
    rv_ref = gator_ref.GetState()
    rv_truth = gator_truth.GetState()

    # Get the corresponding cartesian state from the RIC frame
    rv_ric, _ = xyz2ric(rv_ref, rv_truth)

    # Get the updated keplerian states for each spacecraft
    refCOE = REF_SAT.getKeplerianState()
    truthCOE = TRUTH_SAT.getKeplerianState()

    # Measure the current difference of the semi-major axis
    del_a_current = truthCOE[0] - refCOE[0]

    # If the current time is a multiple of `DT_COAST`, collect telemtry data
    if elapsed_time % DT_COAST == 0:
        for j in range(6):
            # Collect RIC frame state
            RIC_History[RIC_KEYS[j]][elapsed_time] = rv_ric[j]

            # Add the current state to the amplitude buffer
            RIC_Amp_Buffer[RIC_KEYS[j]].append(rv_ric[j])

            # If the buffer is at its max length, compute the amplitude.
            # Otherwise assume the amplitude is the current value
            if len(RIC_Amp_Buffer[RIC_KEYS[j]]) == 1.5 * STEPS_PER_ORBIT:
                amp = (max(RIC_Amp_Buffer[RIC_KEYS[j]])
                       - min(RIC_Amp_Buffer[RIC_KEYS[j]])) / 2
            else:
                amp = max(RIC_Amp_Buffer[RIC_KEYS[j]])

            # Store the current amplitude
            RIC_Amp_History[RIC_KEYS[j]][elapsed_time] = amp

            # Compute the difference in Kelperian states
            coe = truthCOE[j] - refCOE[j]

            # For RAAN, AOP, and True Anomaly verify angles are in the correct
            # quadrant. Otherwise the correction should be 0
            quad_correction = 0
            if j > 1:
                if coe > 180:
                    quad_correction = -360
                elif coe < -180:
                    quad_correction = 360
            corrected_coe = coe + quad_correction

            # Store the instantaneous difference in Keplerian state
            diffCOEs_dict[COE_KEYS[j]][elapsed_time] = corrected_coe
            diffCOEs_buffer[COE_KEYS[j]].append(corrected_coe)

            # If the average buffer is at its max length, compute the average
            # difference. Otherwise, use the instantaneous value as the average
            if len(diffCOEs_buffer[COE_KEYS[j]]) == STEPS_PER_AVG_ORBIT:
                avg_value = float(np.mean(diffCOEs_buffer[COE_KEYS[j]]))
            else:
                avg_value = corrected_coe

            # Store the current average difference
            diffCOEs_avg[COE_KEYS[j]][elapsed_time] = avg_value

        # At each major time step, evaluate if there have been any boundary
        # violations
        interprupt_maneuver = {
            "R": RIC_Amp_History["R"][elapsed_time] > R_BOUNDS,
            "I": rv_ric[1] > DEADBAND_TRIGGER_RATIO * I_BOUNDS,
            "C": RIC_Amp_History["C"][elapsed_time] > C_BOUNDS,
        }

        # If a state change is necessary, verify no higher-priority state is
        # currently selected
        if (interprupt_maneuver["I"]
            and state not in I_OVERRIDE
        ):
            interprupted_state = state
            state = "wait for I burn"
        elif (interprupt_maneuver["C"]
              and state not in C_OVERRIDE
              and interprupted_state == "nominal"
        ):
            interprupted_state = state
            state = "wait for C burn"
        elif (interprupt_maneuver["R"]
              and state not in R_OVERRIDE
              and interprupted_state == "nominal"
        ):
            interprupted_state = state
            state = "wait for R burn"

    # After storing the data for the current time step, enter the state machine
    match state:
        case "wait for R burn":
            # Increase number of steps waited
            steps_waiting += 1

            # Check if the true anomaly is in the desired maneuver window
            fTrue = truthCOE[-1]
            in_node_window = (90 - MANEUVER_ARC_HALF_ANGLE < fTrue < 90
                              or 270 - MANEUVER_ARC_HALF_ANGLE < fTrue < 270)

            # Check if "del_aop" is within acceptable margins
            del_aop = diffCOEs_dict["del_aop"][prev_major_time_step]
            in_del_aop_range = abs(del_aop) <= 3

            # If the truth spacecraft is in its maneuver window, choose the
            # corresponding thruster direction
            if in_node_window and in_del_aop_range:
                # - If "del_e" is greater than 0, then the reference
                #   spacecraft's perigee is lower and apogee is higher than the
                #   truth spacecraft. When approaching apogee, the truth
                #   spacecraft should fire in the R- direction to torque the
                #   orbit and align the perigee and apogee. Then when
                #   approaching perigee the maneuver should be in the opposite
                #   direction.
                # - If "del_e" is less than 0, then the reference spacecraft's
                #   perigee is higher and apogee is lower than the truth
                #   spacecraft. When approaching apogee, the truth spacecraft
                #   should fire in the R+ direction to torque the orbit and
                #   align the perigee and apogee. Then when approaching perigee
                #   the maneuver should be in the opposite direction.
                if diffCOEs_avg["del_e"][elapsed_time] > 0:
                    thruster_axis = "R-" if in_node_window[0] else "R+"
                else:
                    thruster_axis = "R+" if in_node_window[0] else "R-"

                # Turn on the thruster and begin the maneuver
                gator_truth = TRUTH_OBJ.satEnginesOn(thruster_axis)
                state = "R burn"

                # Add to the list of when burns start the corresponding color
                # for this maneuver.
                burn_starts.append((elapsed_time, "m"))

                # Start maneuver duration timer
                burn_duration = 0.0

                # Update simulation time step
                dt = DT_THRUST

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

            # Check if the true anomaly is in the desired maneuver window
            fTrue = truthCOE[-1]
            in_apogee_pass = (
                180 - MANEUVER_ARC_HALF_ANGLE < fTrue <= 180)
            in_perigee_pass = (
                360 - MANEUVER_ARC_HALF_ANGLE < fTrue <= 360)

            # If the I-axis position is within 5% of the max `I_BOUNDS`,
            # maneuver at the first avilable opportunity. Otherwise, depending
            # on the value of "del_e", target the maneuver window that best
            # helps lower "del_e"
            if rv_ric[1] / I_BOUNDS < 0.95:
                if diffCOEs_avg["del_e"][elapsed_time] <= 0:
                    in_burn_window = in_perigee_pass
                else:
                    in_burn_window = in_apogee_pass
            else:
                in_burn_window = in_apogee_pass or in_perigee_pass

            if len(burn_starts) > 0:
                recent_maneuver = (elapsed_time - burn_ends[-1]
                                   >= 3 * PERIOD_IN_SECONDS)
            else:
                recent_maneuver = True

            # Value of "del_a" the controller is predicted to achieve to
            # correct I-axis drift
            del_a_target = abs(diffCOEs_avg["del_a"][prev_major_time_step])

            if in_burn_window and recent_maneuver:
                # Update the controller state
                state = "I burn"
                thruster_axis = "I+"

                # Begin counting the number of maneuver attemps
                maneuver_attempts = 0

                # Turn thrusters on
                gator_truth = TRUTH_OBJ.satEnginesOn(thruster_axis)

                # Add to the list of when burns start the corresponding color
                # for this maneuver.
                burn_starts.append((elapsed_time, "r"))

                # Change simulation step size
                dt = DT_THRUST

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

            # Compute the best phase angle to perform the C-axis maneuver.
            true_lat = (truthCOE[4] + truthCOE[5]) % 360
            crit_angle = np.rad2deg(
                np.arctan(
                    diffCOEs_avg["del_raan"][elapsed_time]
                    / (diffCOEs_avg["del_i"][elapsed_time] * 10)
                    * np.sin(np.deg2rad(refCOE[2]))
                )
            )
            # This equation slightly varies from heuristic by H. Schuab and
            # J. Junkins in 'Analytical Mechanics of Space Systems', 4th Ed.
            # The average differences in "del_raan" and "del_i" were off by an
            # order of magnitude. This placed the critical angle at the wrong
            # location in the orbit and "del_i" was not being corrected
            # properly. By setting the differences to the same magnitude, the
            # controller was successfully able to correct both "del_raan" and
            # "del_i" in the same maneuver

            # Maneuvers in the C-axis can operate on larger timelines than the
            # other axes so `MANEUVER_ARC_HALF_ANGLE` was doubled.
            crit_angle += 360 if crit_angle < 0 else 0
            window_opens = crit_angle - MANEUVER_ARC_HALF_ANGLE * 2
            window_closes = crit_angle + MANEUVER_ARC_HALF_ANGLE * 2
            in_node_window = window_opens < true_lat < window_closes  % 360

            # If the truth spacecraft is in its maneuver window, determine the
            # thrust axis by the rate of "del_raan" and where the maneuver
            # occurs
            if in_node_window:
                raan_decay = diffCOEs_avg["del_raan"][elapsed_time] < 0
                if raan_decay:
                    thruster_axis = "C-" if crit_angle >= 180 else "C+"
                else:
                    thruster_axis = "C+" if crit_angle < 180 else "C-"

                # Turn thrusters on
                gator_truth = TRUTH_OBJ.satEnginesOn(thruster_axis)

                # Update the controller state
                state = "C burn"

                # Add to the list of when burns start the corresponding color
                # for this maneuver.
                burn_starts.append((elapsed_time, "c"))
                burn_duration = 0.0

                # Change simulation step size
                dt = DT_THRUST

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

        # -----------maneuvering-----------------------------------------------
        case "R burn":
            # Increase the burn duration timer
            burn_duration += dt

            # Ensure the diference in the argument of perigee remains small
            del_aop = diffCOEs_dict["del_aop"][prev_major_time_step]
            in_del_aop_range = -3 <= del_aop < 3

            # Ensure the truth spacecraft's true anomaly remains in the
            # maneuver window
            truth_f = truthCOE[-1]
            in_burn_window = (90 - MANEUVER_ARC_HALF_ANGLE < truth_f < 90
                              or 270 - MANEUVER_ARC_HALF_ANGLE < truth_f < 270)
            in_node_window = in_del_aop_range and in_burn_window

            # After reaching the minimum maneuver time, if the maneuver
            # duration excedes the max duty time or the spacecraft is now
            # outside the maneuver window end the maneuver sequence
            if (
                (burn_duration >= MAX_DUTY_TIME or not in_node_window)
                and burn_duration >= MIN_DUTY_TIME
            ):
                # Compute `deltaV` from this maneuver by multipling the
                # acceleration by the duration of the maneuver
                deltaV = ACCEL[thruster_axis] * burn_duration
                total_delta_v += deltaV

                # If maneuver messages are enable, print the message to the'
                # terminal
                if PRINT_MANEUVER_MESSAGE:
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

                # Log the maneuver and set the simulation time steps such that
                # the next step is a multiple of `DT_COAST`
                burn_ends.append(elapsed_time)
                dt = DT_COAST - round(elapsed_time % DT_COAST)

                gator_truth = TRUTH_OBJ.satEnginesOff(thruster_axis)
                thruster_axis = ""

                # Update the state
                state = "returning from R burn"

                # Reset the maneuver duration timer
                burn_duration = 0
        case "I burn":
            # The algorithm to determine the "perfect" maneuver length is as
            # follows:
            # - Upon entering "I burn" for the first time, fire the thrusters
            #   for `MIN_DUTY_TIME`
            # - Turn off the thrusters and coast until "del_a" drops below 0
            #   (this conditions signifies that the truth spacecraft is no
            #   longer coasting away from the reference spacecraft but rather
            #   beginning its approach back)
            # - If the maximum negative I-axis position is not greater than
            #   `DEADBAND_TRIGGER_RATIO`% of `I_BOUNDS`, backwards propagate to
            #   the end of the maneuver and increase the burn duration
            # - If the maximum negative I-axis position is greater than
            #   `I_BOUNDS`, backwards propagate to the end of the maneuver and
            #   backwards propagate into the maneuver to reduce the maneuver's
            #   burn duration
            # - If the burn duration is commanded to be negative or the amount
            #   of maneuver corrections excedes 100 attemps, the simulation is
            #   ended
            if thruster_axis != "":
                # Increase the burn duration timer
                burn_duration += dt

                # Used when additonal manuver time is required and tracks the
                # estimated number of `DT_THRUST` steps remain to achieve the
                # desired I-axis position. When shortening the maneuver
                # duration, only one time step is necessary so this counter
                # is irrelevant
                estimated_steps -= 1

                # If this is a fresh maneuver attempt, make sure the maneuver
                # duration is greater than `MIN_DUTY_TIME`
                new_maneuver = (burn_duration >= MIN_DUTY_TIME
                                and maneuver_attempts == 0)
                # If this is a follow-on maneuver attempt, make sure maneuver
                # is propagated for the required amount of time
                maneuver_attempt = (maneuver_attempts > 0
                                    and estimated_steps <= 0)

                if new_maneuver or maneuver_attempt:
                    # Should this be the successful maneuver, store the
                    # thruster cutoff time
                    burn_ends.append(elapsed_time)

                    gator_truth = TRUTH_OBJ.satEnginesOff(thruster_axis)
                    thruster_axis = ""

                    # Set the simulation time step equal to the time it would
                    # take to get `elapsed_time` back on track to be a multiple
                    # of `DT_COAST`
                    dt = DT_COAST - round(elapsed_time % DT_COAST)

                    maneuver_attempts += 1

                    # Begin tracking the minimum I position
                    min_i_pos = rv_ric[1]

                    # Should this be the successful maneuver, store the
                    # recovered amount of "del_a" for the truth spacecraft
                    del_a_recovered = del_a_current
            else:
                # If the current I-position is less than the minimum, updated
                # the sotred value
                if rv_ric[1] < min_i_pos:
                    min_i_pos = rv_ric[1]

                coast_duration += dt

                # Since inside "I burn" loop, return `dt` to equal `DT_COAST`
                dt = DT_COAST

                # Wait for at least 1 orbital period before evaluating the
                # termination conditions for the maneuver
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
                        # Compute the deltaV consumed in the maneuver
                        deltaV = ACCEL["I+"] * burn_duration
                        total_delta_v += deltaV

                        if PRINT_MANEUVER_MESSAGE:
                            this_burn_start = burn_starts[-1][0] / 86400 # days
                            this_burn_duration = burn_duration / 60 # minutes
                            get_i_axis_print(
                                this_burn_start,
                                this_burn_duration,
                                del_a_recovered,
                                del_a_target,
                                deltaV,
                                total_delta_v
                            )

                        # With the compeltion of the maneuver, return the state
                        # machine to the previously interrupted state
                        state = interprupted_state
                        interprupted_state = "nominal"

                        # Data is not collected during the coasting propagation
                        # during the maneuver. Back propagate the scenario to
                        # the time which the thruster shut off to collect the
                        # mising data
                        restoreFromTime = round_to_time_step(burn_ends[-1])
                        tStep = t.index(restoreFromTime)
                        tHistoryCOEs = t[(tStep - STEPS_PER_AVG_ORBIT):tStep]

                        # Reload the average COE buffers with data leading up
                        # to the timestamp the thruster turned off
                        for j in diffCOEs_dict.keys():
                            for k in tHistoryCOEs:
                                diffCOEs_buffer[j].append(diffCOEs_dict[j][k])

                        # Propagate the spacecraft to the end of the maneuver
                        # and update the scenario time
                        gator_ref.Step(-coast_duration)
                        gator_truth.Step(-coast_duration)
                        elapsed_time = elapsed_time - coast_duration

                        # Prepare these tracking variables ahead of another
                        # maneuver attempt
                        burn_duration = 0
                        coast_duration = 0
                        maneuver_log = []

                        # Update numerical integrator references
                        gator_ref.UpdateSpaceObject()
                        gator_truth.UpdateSpaceObject()

                    elif termination_conditions[1]:
                        # If the debug is set to true, print information about
                        # the maneuver attempt (attempt #, minimum I-axis
                        # position, and burn duration)
                        if PRINT_I_AXIS_MANEUVER_ATTEMPS:
                            i_axis_maneuver_attempt_message(
                                maneuver_attempts,
                                min_i_pos,
                                burn_duration
                            )

                        # Log the maneuver duration from this attempt
                        maneuver_log.append(burn_duration)

                        # Determine the last major time step in the maneuver
                        # window and its index within the time keeper `t`
                        restoreFromTime = round_to_time_step(burn_ends[-1])
                        tStep = t.index(restoreFromTime)

                        # Based on the last major time step, collect the
                        # average buffer data for each COE using the data from
                        # the preceding `STEPS_PER_AVG_ORBIT` steps
                        tHistoryCOEs = t[(tStep - STEPS_PER_AVG_ORBIT):tStep]
                        for j in diffCOEs_dict.keys():
                            for k in tHistoryCOEs:
                                diffCOEs_buffer[j].append(diffCOEs_dict[j][k])

                        # Propagate the spacecraft to the end of the maneuver
                        # and update the scenario time
                        gator_ref.Step(-coast_duration)
                        gator_truth.Step(-coast_duration)
                        elapsed_time = elapsed_time - coast_duration

                        # With the coast period resetting, reset the timer
                        coast_duration = 0

                        # Update numerical integrator references
                        gator_ref.UpdateSpaceObject()
                        gator_truth.UpdateSpaceObject()

                        # Since this was an unsuccessful maneuver, remove its
                        # termination time from the log
                        burn_ends.pop()

                        # Based on the undershoot of `I_BOUNDS`, estimate the
                        # of steps needed to meet the mission criteria (1
                        # `DT_THRUST` time step per missed Km)
                        estimated_steps = np.ceil(
                            (min_i_pos + DEADBAND_TRIGGER_RATIO * I_BOUNDS)
                        )

                        # Before intiating a new maneuver, check that the
                        # predicted maneuver duration hasn't been tried
                        dt = DT_THRUST
                        predicted_burn_duration = (burn_duration
                                                   + estimated_steps * dt)
                        if predicted_burn_duration in maneuver_log:
                            estimated_steps -=1

                        # Turn the thrusters back on
                        thruster_axis = "I+"
                        gator_truth = TRUTH_OBJ.satEnginesOn(thruster_axis)

                        # In case this leads to the 100th maneuver attempt,
                        # notify the user and exit the station keeping loop
                        if maneuver_attempts == 100:
                            print("Max burns! "
                                  + "Current burn duration = "
                                  + f"{burn_duration} sec")
                            burn_starts.pop()
                            break

                    elif termination_conditions[2]:
                        # If the debug is set to true, print information about
                        # the maneuver attempt (attempt #, minimum I-axis
                        # position, and burn duration)
                        if PRINT_I_AXIS_MANEUVER_ATTEMPS:
                            i_axis_maneuver_attempt_message(
                                maneuver_attempts,
                                min_i_pos,
                                burn_duration
                            )

                        # Based on the miss distance, estimate the number of
                        # time steps needed to correct the overshoot (1
                        # `DT_THRUST` time step per missed Km)
                        stepsToBackTrack = abs(
                            np.ceil(
                                min_i_pos + DEADBAND_TRIGGER_RATIO * I_BOUNDS
                            )
                        )

                        # If the new predicted maneuver time was already tried,
                        # try an alternate maneuver duration that is one time
                        # step shorter
                        if (burn_duration
                            - DT_THRUST * stepsToBackTrack in maneuver_log
                        ):
                            stepsToBackTrack -=1

                        # Add the upcoming maneuver duration to the log
                        maneuver_log.append(burn_duration)

                        # Compute the latest major time step before the end of
                        # the maneuver
                        restoreFromTime = round_to_time_step(burn_ends[-1])

                        # Compute the number of major time steps that occur
                        # during the back propagation of the maneuver
                        time_to_backtrack = (
                            (stepsToBackTrack * DT_THRUST // DT_COAST) * DT_COAST
                        )

                        # Determine the last major time step in the maneuver
                        # window and its index within the time keeper `t`
                        restoreFromTime -= time_to_backtrack
                        tStep = t.index(restoreFromTime)

                        # Based on the last major time step, collect the
                        # average buffer data for each COE using the data from
                        # the preceding `STEPS_PER_AVG_ORBIT` steps
                        tHistoryCOEs = t[(tStep - STEPS_PER_AVG_ORBIT):tStep]
                        for j in diffCOEs_dict.keys():
                            for k in tHistoryCOEs:
                                diffCOEs_buffer[j].append(diffCOEs_dict[j][k])

                        # Since this was an unsuccessful maneuver, remove its
                        # termination time from the log
                        burn_ends.pop()

                        # Get the updated cartesian states for each spacecraft
                        # from the ECI frame.
                        rv_ref = gator_ref.GetState()
                        rv_truth = gator_truth.GetState()
                        # Get the updated keplerian states for each spacecraft.
                        refCOE = REF_SAT.getKeplerianState()
                        truthCOE = TRUTH_SAT.getKeplerianState()

                        # Back propagate spacecraft
                        # To minimize the discontinuities between the stored
                        # state and the back propagated state, the backwards
                        # time step should be small (rather than 1 large back
                        # propagation, break it into 300 small propagations).
                        backPropStepSize = coast_duration // 300
                        for i in range(300):
                            gator_ref.Step(-backPropStepSize)
                            gator_truth.Step(-backPropStepSize)

                        # If the back propagation could not be cleanly broken
                        # into 300 steps, back propagate the remaining time and
                        # update the scenario time
                        backStepRemainder = coast_duration % 300
                        gator_ref.Step(-backStepRemainder)
                        gator_truth.Step(-backStepRemainder)
                        elapsed_time = elapsed_time - coast_duration

                        # With another maneuver attmempt required, reset the
                        # timer
                        coast_duration = 0

                        # Update numerical integrator references
                        gator_ref.UpdateSpaceObject()
                        gator_truth.UpdateSpaceObject()

                        # Get the updated cartesian states for each spacecraft
                        # from the ECI frame.
                        rv_ref = gator_ref.GetState()
                        rv_truth = gator_truth.GetState()
                        # Get the updated keplerian states for each spacecraft.
                        refCOE = REF_SAT.getKeplerianState()
                        truthCOE = TRUTH_SAT.getKeplerianState()

                        # With the scenario and spacecraft aligned with the end
                        # of the previous maneuver attempt, now back propagate
                        # the maneuver to shorten it
                        thruster_axis = "I+"
                        dt = DT_THRUST
                        gator_truth = TRUTH_OBJ.satEnginesOn(thruster_axis)
                        backPropTime = -stepsToBackTrack * DT_THRUST
                        burn_duration -= stepsToBackTrack * DT_THRUST
                        gator_ref.Step(backPropTime)
                        gator_truth.Step(backPropTime)

                        # Update numerical integrator references
                        gator_ref.UpdateSpaceObject()
                        gator_truth.UpdateSpaceObject()

                        # Get the updated cartesian states for each spacecraft
                        # from the ECI frame.
                        rv_ref = gator_ref.GetState()
                        rv_truth = gator_truth.GetState()

                        # In case the maneuver time goes negative while the
                        # algorithm searches for the shorter maneuver time to
                        # bring in the overshoot of `I_BOUNDS`, message the
                        # user in the terminal and exit the station keeping
                        # loop
                        if burn_duration + dt <= 0:
                            print("Negative thrust time! Min I = "
                                  + str(min_i_pos)
                            )
                            burn_starts.pop()
                            break
                        # In case this leads to the 100th maneuver attempt,
                        # notify the user and exit the station keeping loop
                        if maneuver_attempts == 100:
                            print("Max burns! current burn duration = "
                                  + str(burn_duration) + " sec | Min I = "
                                  + str(min_i_pos)
                            )
                            burn_starts.pop()
                            break
        case "C burn":
            # Increase the burn duration timer
            burn_duration += dt

            # Verify that the truth spacecraft is still within its maneuver
            # window
            true_lat = (truthCOE[4] + truthCOE[5]) % 360
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

            # Once the spacecraft has left tis maneuver window or exceded the
            # thruster duty time, end the maneuver
            if burn_duration >= MAX_DUTY_TIME or not in_cross_track_pass:
                # Compute `deltaV` from this maneuver by multipling the
                # acceleration by the duration of the maneuver
                deltaV = ACCEL[thruster_axis] * burn_duration
                total_delta_v += deltaV
                if PRINT_MANEUVER_MESSAGE:
                    this_burn_start = burn_starts[-1][0] / 86400
                    this_burn_duration = burn_duration / 60
                    c_amp = RIC_Amp_History['C'][prev_major_time_step]
                    get_c_axis_print(
                        this_burn_start,
                        this_burn_duration,
                        thruster_axis,
                        c_amp,
                        deltaV,
                        total_delta_v
                    )

                # Log the maneuver and set the simulation time steps such that
                # the next step is a multiple of `DT_COAST`
                burn_ends.append(elapsed_time)
                dt = DT_COAST - round(elapsed_time % DT_COAST)

                gator_truth = TRUTH_OBJ.satEnginesOff(thruster_axis)
                thruster_axis = ""

                # Update the state
                state = "returning from C burn"

                # Reset the maneuver duration timer
                burn_duration = 0


        # -----------verifying recovery----------------------------------------
        case "returning from R burn":
            # Get the latest calculated oscillation amplitude
            r_amp = RIC_Amp_History["R"][prev_major_time_step]

            # If the amplitude has dropped to less than
            # `DEADBAND_TRIGGER_RATIO` percent of `R_BOUNDS`, return to nominal
            #
            # If the amplitude has not dropped after 1/4 of an orbital period,
            # reenter "wait for R burn"
            if r_amp <= DEADBAND_TRIGGER_RATIO * R_BOUNDS:
                state = interprupted_state
                interprupted_state = "nominal"
            elif (prev_major_time_step
                  - burn_ends[-1] > 0.25 * PERIOD_IN_SECONDS
            ):
                state = "wait for R burn"
            else:
                continue

        case "returning from C burn":
            # Verify the average value of del_raan has changed sign
            avg_del_raan = diffCOEs_avg["del_raan"][prev_major_time_step]
            if raan_decay:
                raan_sign_change = avg_del_raan > 0
            else:
                raan_sign_change = avg_del_raan < 0

            # Determine if the amplitude of the C position oscillation has
            # dropped below `DEADBAND_TRIGGER_RATIO` percent of `C_BOUNDS`
            c_amp = RIC_Amp_History["C"][prev_major_time_step]
            c_amp_corrected = c_amp / C_BOUNDS < DEADBAND_TRIGGER_RATIO

            # If `c_amp` has dropped suffieicently, return to nominal
            #
            # If it has been 1/4 of an orbital period and `c_amp` has not
            # reached the desired levels, return to "wait for C burn"
            if c_amp_corrected: # raan_sign_change:
                state = interprupted_state
                interprupted_state = "nominal"
            elif (prev_major_time_step
                  - burn_ends[-1] > .25 * PERIOD_IN_SECONDS
            ):
                state = "wait for C burn"
            else:
                continue

        case _:
            continue

# -----------plots---------------------------------------------------
timings =  [burn_ends, burn_starts, t, REVOLUTIONS_TO_AVERAGE, DT_COAST, STEPS_PER_AVG_ORBIT]
coes = [diffCOEs_dict, diffCOEs_avg]
ric = [RIC_History, RIC_Amp_History]
output_plots(timings, coes, ric)
