"""Logic controller for LEO Station Keeping State Machine."""

import numpy as np

from simulationParameters import *
from support_functions import *

class StationKeepingController:
    """
    Determines a spacecraft's actions that are necessary to station
    keep in a LEO environment.

    Spacecraft actions are received from update() during each time step
    to control the following:
    - Begin looking for maneuver opportunities.
    - End looking for maneuver opportunities.
    - Begin thrusting along specified axis (R+/-, I+, C+/-).
    - Shut off specified thrusters.
    - *For I-axis maneuvers only*
        - Good maneuver, return to end of thrusting and resume
          coasting.
        - Maneuver undershoots target range, return to end of previous
          maneuver and try again with a longer maneuver duration.
        - Maneuver overshoots target range, return to the end of
          previous maneuver and back propagate the maneuver to shorten
          its duration.
    
    Attributes
    ----------
    MU : float
        Earth's gravitational parameter (km^3/s^2)
    PERIOD_IN_SECONDS : float
        The orbital period of the initial truth spacecraft in seconds.
    STEPS_PER_ORBIT : int
        Based on the defined DT_COAST value, how many steps to complete
        one orbital period.
    COE_KEYS : list[str]
        Keys used to store differences in orbital elements between the
        truth and reference spacecraft.
    RIC_KEYS : list[str]
        Keys used to store the RIC position and velocity components of
        the truth spacecraft.
    I_OVERRIDE : set
        set of `state` values that cannot be interrupted to begin
        looking for I-axis maneuver opportunities.
    C_OVERRIDE : set
        set of `state` values that cannot be interrupted to begin
        looking for C-axis maneuver opportunities.
    R_OVERRIDE : set
        set of `state` values that cannot be interrupted to begin
        looking for R-axis maneuver opportunities.
    state : str
        Determines which station keeping methods to follow.
    interrupted_state : str
        In case a state override is necessary, remembers what state to
        return to upon completing the higher priority maneuver.
    steps_waiting : int
        Counts the number of steps while waiting to begin a maneuver.
    amp_ric : dict
        Updated each time step from the main loop with the oscillation
        amplitudes of the truth spacecraft's RIC state.
    coes_instant_diff : dict
        Updated each time step from the main loop with the
        instantaneous differences in Keplerian elements between the
        truth and reference spacecraft.
    coes_avg_diff : dict
        Updated each time step from the main loop with the
        average differences in Keplerian elements between the
        truth and reference spacecraft.
    rv_ric : list
        Updated each time step from the main loop with the truth
        spacecraft's RIC state vector.
    truth_coes : list
        Updated each time step from the main loop with the truth
        spacecraft's Keplerian state vector.
    ref_coes : list
        Updated each time step from the main loop with the reference
        spacecraft's Keplerian state vector.
    maneuver_starts : list
        Tracks the times of when every maneuver was initiated and what
        type of maneuver it was.
    maneuver_attempts : list
        Specific to the I-axis maneuver algorithm, tracks the durations
        of previously attempted maneuvers to prevent repeat attempts.
    maneuver_ends : list
        Tracks the times of when every maneuver was concluded.
    burn_duration : float
        Tracks the duration of the active maneuver.
    total_delta_v : float
        Tracks the sum of delta v consumed across all maneuvers
        performed.
    thruster_axis : str
        Thruster axis of the active maneuver.
    negative_time_correction_tries : int
        Specific to the I-axis maneuver algorithm, the number of tries
        the algo with a negative thrust time before terminating the
        simulation.
    estimated_steps : int
        Specific to the I-axis maneuver algorithm, the number of
        DT_THRUST steps for the truth spacecraft to add or remove from
        `burn_duration` to achieve a nominal drag recovery.
    coast_duration : float
        Specific to the I-axis maneuver algorithm, tracks the time
        since the conclusion of the maneuver for back propagation
        purposes.
    min_i_pos : float
        Specific to the I-axis maneuver algorithm, tracks the most
        negative (greatest magnitude) I-axis position during each
        coast-after-burn attempt. Goldilocks / undershoot / overshoot
        scoring uses this value (not `max_i_pos`).
    max_i_pos : float
        Specific to the I-axis maneuver algorithm, tracks the most
        positive I-axis position during the coast. Updated for
        printouts (`get_i_axis_print`); not used by the live gate or
        scoring.
    del_a_estimated : float
        Snapshot of ``abs(coes_avg_diff["del_a"])`` at I-burn start.
        Retained for possible debug prints; live successful-I print
        path uses `max_i_pos` / `min_i_pos` instead (legacy SMA-print
        path retired).
    del_a_recovered : float
        Placeholder for post-maneuver SMA difference telemetry. Not
        written by the live controller gate; kept for API compatibility
        with older print helpers.
    min_i_pos_timer : float
        Countdown (seconds) reset whenever `min_i_pos` updates. Still
        updated during the I-coast, but unused by the live termination
        gate (gate uses coast_duration > 4*PERIOD and mean del_a < 0;
        see commented ``min_i_pos_timer <= 0`` alternative in `_i_burn`).
    
    """
    MU = 398600  # Earth’s gravitational parameter in km^3/s^2

    # Compute the number of steps per orbit based on the source of the
    # state vector.
    if STATE_VECT_SOURCE == "new":
        _MEAN_MOTION = np.sqrt(MU / ORBIT_STATE[0]**3)
    else:
        _MEAN_MOTION = np.sqrt(MU / REF_ORBIT_STATE[0]**3)

    PERIOD_IN_SECONDS = 2 * np.pi / _MEAN_MOTION
    STEPS_PER_ORBIT = int(np.ceil(PERIOD_IN_SECONDS / DT_COAST))

    # dict keys to access data provided from the main loop
    COE_KEYS = ["del_a", "del_e", "del_i", "del_raan", "del_aop", "del_f"]
    RIC_KEYS = ["R", "I", "C", "R_dot", "I_dot", "C_dot"]

    # States that take priority over the corresponding maneuver axis
    I_OVERRIDE = {"wait for I burn", "R burn", "I burn", "C burn"}
    C_OVERRIDE = I_OVERRIDE | {"wait for C burn", "returning from C burn"}
    R_OVERRIDE = C_OVERRIDE | {"wait for R burn", "returning from R burn"}

    def __init__(self) -> None:
        """ Initialize the controller. """

        # State monitoring
        self.state = "nominal"
        self.interrupted_state = "nominal"
        self.steps_waiting = 0

        # Relevant telementry (MUST BE PROVIDED EVERY STEP IN MAIN LOOP)
        self.amp_ric = {i: 0 for i in self.RIC_KEYS}
        self.coes_instant_diff = {i: 0 for i in self.COE_KEYS}
        self.coes_avg_diff = {i: 0 for i in self.COE_KEYS}
        self.rv_ric = [0, 0, 0, 0, 0, 0]
        self.truth_coes = [0, 0, 0, 0, 0, 0]
        self.ref_coes = [0, 0, 0, 0, 0, 0]

        # Maneuver logging
        self.maneuver_starts = []
        self.maneuver_attempts = []
        self.maneuver_ends = []
        self.burn_duration = 0
        self.total_delta_v = 0
        self.thruster_axis = ""

        # Maneuver results (for I-burn only)
        self.negative_time_correction_tries = 5
        self.estimated_steps = 0
        self.coast_duration = 0
        self.min_i_pos = 0
        self.max_i_pos = 0
        self.del_a_estimated = 0
        self.del_a_recovered = 0
        self.min_i_pos_timer = 0

    def update(
            self,
            elapsed_time: float,
            ACCEL: dict,
            thruster_axis: str = ""
    ):
        """ Determine the station keeping action to take.

        After updating the necessary attributes, determine what the
        necessary actions are to ensure station keeping within the
        operations boundary.
        
        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation started.
        ACCEL : {str: float}
            Assuming constant thrust and mass, the acceleration map for
            each thruster axis.
        thruster_axis : str, default = ""
            Which axis has active thrusters.
        """

        # At each major time step, evaluate if there have been any operations  
        # boundary violations.
        boundary_violations = {
            "R": self.amp_ric["R"] > R_BOUNDS,
            "I": self.rv_ric[1] > DEADBAND_TRIGGER_RATIO * I_BOUNDS,
            "C": self.amp_ric["C"] > C_BOUNDS,
        }

        # If a state change is necessary, verify no higher-priority state is
        # currently selected.
        if (boundary_violations["I"]
            and self.state not in self.I_OVERRIDE
        ):
            self.interrupted_state = self.state
            self.state = "wait for I burn"

        elif (boundary_violations["C"]
                and self.state not in self.C_OVERRIDE
                and self.interrupted_state == "nominal"
        ):
            self.interrupted_state = self.state
            self.state = "wait for C burn"

        elif (boundary_violations["R"]
                and self.state not in self.R_OVERRIDE
                and self.interrupted_state == "nominal"
        ):
            self.interrupted_state = self.state
            self.state = "wait for R burn"

        match self.state:
            case "wait for R burn":
                return self._wait_for_r(elapsed_time)

            case "wait for I burn":
                return self._wait_for_i(elapsed_time)

            case "wait for C burn":
                return self._wait_for_c(elapsed_time)

            case "R burn":
                return self._r_burn(elapsed_time, ACCEL)

            case "I burn":
                return self._i_burn(elapsed_time, ACCEL, thruster_axis)

            case "C burn":
                return self._c_burn(elapsed_time, ACCEL)

            case "returning from R burn":
                return self._return_from_r(elapsed_time)

            case "returning from C burn":
                return self._return_from_c(elapsed_time)

            case _:
                # If `state` is "nominal" or any other non pre-defined terms,
                # instruct the spacecraft to operate nominally.
                return {"action": "continue"}

    # ------------- Look For Maneuver Opportunities ---------------------------
    def _wait_for_r(
            self,
            elapsed_time: float,
        ) -> dict:
        """
        Evaluate if the truth spacecraft is in its radial maneuver
        window.

        Live window (matches the code below; not the eventual ideal):
        - True anomaly approaching 90 deg or 270 deg within
          `MANEUVER_ARC_HALF_ANGLE` (open interval just before the node).
        - Instantaneous ``|del_aop| <= 3`` deg so eccentricity vectors
          are roughly aligned.

        Thruster sign (R+/-) uses mean ``del_e`` and which node
        (90 vs 270). Live gate: TA within `MANEUVER_ARC_HALF_ANGLE` of
        90/270 plus instantaneous ``|del_aop| <= 3`` deg. Per Gauss
        variational equations, radial thrust near ``cos f ≈ 0``
        (f ≈ 90/270) primarily controls *e*, not ω — it does not
        "torque the line of apsides." Polar equal-r TA is a TODO, not
        live.

        TODO — NEED TO CORRECT *IDEAL* WINDOWS: use the polar equation
        to find the TA where truth and reference altitudes match, then
        take the complementary angle ``360 - TA``. Keep the polar-equation
        notes in the body until that lands. If the window is not found
        within one orbital period, stop waiting.

        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation began.

        Returns
        -------
        dict
            Action for the main loop (``continue``, ``start_burn``, or
            ``stop_waiting``) plus any fields to apply.
        """

        ## Maneuver window identification ##
        self.steps_waiting += 1
        # NEED TO CORRECT *IDEAL* WINDOWS (docs TODO — control unchanged):
        # - use polar equation to find TA such that altitudes between truth and
        #   reference spacecraft are equal.
        # - complementary angle is 360 - TA from step above
        # Live gate below still uses approaching 90/270 +/- half-angle.
        approaching_90 = 90 - MANEUVER_ARC_HALF_ANGLE < self.truth_coes[-1] < 90
        approaching_270 = 270 - MANEUVER_ARC_HALF_ANGLE < self.truth_coes[-1] < 270
        in_node_window = approaching_90 or approaching_270

        # Keeping the value of "del_aop" small means the eccentricity vectors
        # are closely aligned and there is less work needed by the spacecraft
        # to correct the oscillation.
        in_del_aop_range = abs(self.coes_instant_diff["del_aop"]) <= 3

        ## Spacecraft action reporting ##
        if self.steps_waiting >= self.STEPS_PER_ORBIT:
            self.steps_waiting = 0
            self.state = self.interrupted_state
            self.interrupted_state = "nominal"

            # Prevent permanent lock-up if the window never appears
            return {
                "action": "stop_waiting",
                "new_state": self.state,
                "interrupted_state": self.interrupted_state,
            }

        if in_node_window and in_del_aop_range:
            # Choose thruster direction based on the sign of "del_e" and
            # whether the window is approaching 90 or 270 deg true anomaly.
            # Radial thrust near 90/270 primarily changes eccentricity
            # (not ω / line of apsides).
            if self.coes_avg_diff["del_e"] > 0:
                self.thruster_axis = "R-" if approaching_90 else "R+"
            else:
                self.thruster_axis = "R+" if approaching_90 else "R-"

            self.steps_waiting = 0
            self.maneuver_starts.append((elapsed_time, "m"))
            self.state = "R burn"
            return {
                "action": "start_burn",
                "new_state": self.state,
                "thruster_axis": self.thruster_axis,
                "dt": DT_THRUST
            }

        # If no other action is required, then continue looking for maneuver
        # opportunities.
        return {"action": "continue"}

    # TRUNCATED_FOR_TEST_DO_NOT_KEEP
