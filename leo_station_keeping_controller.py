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

    def _wait_for_i(
            self,
            elapsed_time: float
    ) -> dict:
        """
        Evaluate if the truth spacecraft is in its in-track maneuver
        window.

        Live window:
        - Near apogee (TA → 180) or perigee (TA → 360) within
          `MANEUVER_ARC_HALF_ANGLE`, preferring the pass that also
          reduces mean ``del_e`` unless I-position already exceeds
          95% of `I_BOUNDS` (then take the first available).
        - Local ``recent_maneuver`` is True when there is *no* recent
          burn: either no prior maneuver end, or
          ``elapsed_time - maneuver_ends[-1] >= 3 * PERIOD`` (i.e. True
          means "cooldown satisfied / NOT recent").
        - Instantaneous ``del_a < 0`` (truth SMA below reference so
          the burn raises energy into the deadband).

        If no window appears within one orbital period of waiting,
        return to `interrupted_state`.

        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation began.

        Returns
        -------
        dict
            Action for the main loop plus any fields to apply.
        """
        self.steps_waiting += 1

        in_apogee_pass = (
            180 - MANEUVER_ARC_HALF_ANGLE < self.truth_coes[-1] <= 180)
        in_perigee_pass = (
            360 - MANEUVER_ARC_HALF_ANGLE < self.truth_coes[-1] <= 360)

        # Prioritize the maneuver window that also reduces "del_e", unless
        # the I-position is already greater than 95% of `I_BOUNDS`. Then
        # take the first available.
        if self.rv_ric[1] / I_BOUNDS < 0.95:
            if self.coes_avg_diff["del_e"] <= 0:
                in_burn_window = in_perigee_pass
            else:
                in_burn_window = in_apogee_pass
        else:
            in_burn_window = in_apogee_pass or in_perigee_pass

        # Local flag: True means cooldown satisfied (NOT a recent burn).
        if len(self.maneuver_starts) > 0:
            recent_maneuver = (elapsed_time - self.maneuver_ends[-1]
                                >= 3 * self.PERIOD_IN_SECONDS)
        else:
            recent_maneuver = True

        del_a_settled = self.coes_instant_diff["del_a"] < 0

        if self.steps_waiting >= self.STEPS_PER_ORBIT:
            self.steps_waiting = 0
            self.state = self.interrupted_state
            self.interrupted_state = "nominal"

            # Prevent permanent lock-up if the window never appears
            return {
                "action": "stop_waiting",
                "new_state": self.state,
                "interrupted_state": self.interrupted_state
            }

        if in_burn_window and recent_maneuver and del_a_settled:
            self.steps_waiting = 0
            self.maneuver_starts.append((elapsed_time, "r"))
            self.del_a_estimated = abs(self.coes_avg_diff["del_a"])

            self.state = "I burn"
            self.thruster_axis = "I+"
            return {
                "action": "start_burn",
                "new_state": self.state,
                "thruster_axis": self.thruster_axis,
                "dt": DT_THRUST
            }

        return {"action": "continue"}

    def _calc_crit_angle(self):
        # Modified heuristic by H. Schaub and J. Junkins in 'Analytical
        # Mechanics of Space Systems', 4th Ed. Uses np.arctan (not
        # atan2): arctan(dRAAN / (dInc*10) * sin i). Scaling "del_i" by
        # 10 makes "del_i" and "del_raan" comparable; otherwise the
        # unscaled form yields the wrong critical angle.
        crit_angle = np.rad2deg(np.arctan(
                self.coes_avg_diff["del_raan"] / (self.coes_avg_diff["del_i"] * 10) * np.sin(np.deg2rad(self.ref_coes[2]))
            )
        )

        # Wrap negative arctan results into [0, 360). Live C-window is
        # +/- (4 * MANEUVER_ARC_HALF_ANGLE) about this angle (= +/-160 deg
        # when half-angle is 20); applied in `_wait_for_c` / `_c_burn`.
        crit_angle += 360 if crit_angle < 0 else 0

        return crit_angle

    def _wait_for_c(
            self,
            elapsed_time: float
    ) -> dict:
        """
        Evaluate if the truth spacecraft is in its cross-track maneuver
        window.

        Live window: true latitude ``(AOP + TA) % 360`` within
        ``crit_angle +/- (4 * MANEUVER_ARC_HALF_ANGLE)`` (= +/-160 deg
        when half-angle is 20; live 4x, not the legacy 2x in
        `testThrusting.py`). `crit_angle` from `_calc_crit_angle` uses
        ``arctan(dRAAN / (dInc*10) * sin i)`` (not atan2). Thruster sign
        follows mean ``del_raan`` and whether `crit_angle` is past 180.

        If no window appears within one orbital period of waiting,
        return to `interrupted_state`.

        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation began.

        Returns
        -------
        dict
            Action for the main loop plus any fields to apply.
        """

        self.steps_waiting += 1

        true_lat = (self.truth_coes[-2] + self.truth_coes[-1]) % 360

        crit_angle = self._calc_crit_angle()
        window_opens = crit_angle - MANEUVER_ARC_HALF_ANGLE * 4
        window_closes = crit_angle + MANEUVER_ARC_HALF_ANGLE * 4

        if window_opens < 0:
            in_node_window = (true_lat > window_opens + 360
                                or true_lat <= window_closes)
        elif window_closes > 360:
            in_node_window = (true_lat > window_opens
                                or true_lat <= window_closes % 360)
        else:
            in_node_window = window_opens < true_lat < window_closes

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

        if in_node_window:
            self.steps_waiting = 0
            self.maneuver_starts.append((elapsed_time, "c"))
            self.state = "C burn"

            del_raan_is_neg = self.coes_avg_diff["del_raan"] < 0
            if del_raan_is_neg:
                self.thruster_axis = "C-" if crit_angle >= 180 else "C+"
            else:
                self.thruster_axis = "C+" if crit_angle < 180 else "C-"

            return {
                "action": "start_burn",
                "new_state": self.state,
                "thruster_axis": self.thruster_axis,
                "dt": DT_THRUST
            }

        return {"action": "continue"}

    # Maneuvering
    def _r_burn(
            self,
            elapsed_time: float,
            ACCEL: dict
    ) -> dict:
        """Terminate the radial burn when the window ends or duty max hits.

        Continues while TA is still approaching 90/270 within the
        half-angle, ``|del_aop| <= 3``, and burn time is below
        `MAX_DUTY_TIME`. Requires at least `MIN_DUTY_TIME` before
        stopping. On stop, accumulate delta-v and enter
        ``returning from R burn``.

        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation began.
        ACCEL : dict
            Acceleration map keyed by thruster axis.

        Returns
        -------
        dict
            ``continue`` or ``stop_burn`` with bookkeeping fields.
        """
        
        self.burn_duration += DT_THRUST
        
        approaching_90 = 90 - MANEUVER_ARC_HALF_ANGLE < self.truth_coes[-1] < 90
        approaching_270 = 270 - MANEUVER_ARC_HALF_ANGLE < self.truth_coes[-1] < 270
        in_burn_window = approaching_90 or approaching_270

        in_del_aop_range = abs(self.coes_instant_diff["del_aop"]) <= 3

        in_node_window = in_burn_window and in_del_aop_range

        if ((self.burn_duration >= MAX_DUTY_TIME or not in_node_window)
            and self.burn_duration >= MIN_DUTY_TIME
        ):
            delta_v = ACCEL[self.thruster_axis] * self.burn_duration
            self.total_delta_v += delta_v

            maneuver_duration = self.burn_duration
            self.burn_duration = 0
            self.maneuver_ends.append(elapsed_time)
            self.state = "returning from R burn"
            self.thruster_axis = ""
            return {
                "action": "stop_burn",
                "new_state": self.state,
                "dt": DT_COAST - round(elapsed_time % DT_COAST),
                "maneuver_duration": maneuver_duration,
                "maneuver_delta_v": delta_v,
                "total_delta_v": self.total_delta_v,
            }

        return {"action": "continue"}

    def _i_burn_goldilocks(
            self,
            elapsed_time: float,
            ACCEL: dict
    ):
        delta_v = ACCEL["I+"] * self.burn_duration
        self.total_delta_v += delta_v

        maneuver_duration = self.burn_duration
        backtrack_time = self.coast_duration
        self.burn_duration = 0
        self.coast_duration = 0
        self.maneuver_attempts = []
        self.state = self.interrupted_state
        self.interrupted_state = "nominal"

        return {
            "action": "successful_i_maneuver",
            "new_state": self.state,
            "interrupted_state": self.interrupted_state,
            "dt": DT_COAST - round(elapsed_time % DT_COAST),
            "maneuver_duration": maneuver_duration,
            "maneuver_delta_v": delta_v,
            "total_delta_v": self.total_delta_v,
            "backtrack_coast_time": backtrack_time
        }

    def _i_burn_undershoot(
            self,
    ):
        self.maneuver_attempts.append(self.burn_duration)
        if PRINT_I_AXIS_MANEUVER_ATTEMPTS:
            i_axis_maneuver_attempt_debug_message(
                len(self.maneuver_attempts),
                self.min_i_pos,
                self.burn_duration
            )

        # As an unsuccessful maneuver, remove its end time
        burn_end_time = self.maneuver_ends[-1]
        self.maneuver_ends.pop()

        # Estimate the time steps needed to correct the
        # undershoot criteria (1 `DT_THRUST` time step per
        # missed Km).
        self.estimated_steps = np.ceil(
            (self.min_i_pos + DEADBAND_TRIGGER_RATIO * I_BOUNDS)
        )

        # Verify maneuver duration hasn't been tried to prevent
        # an infinite-loop.
        predicted_burn_duration = (self.burn_duration
                                    + self.estimated_steps * DT_THRUST)
        if predicted_burn_duration in self.maneuver_attempts:
            self.estimated_steps -=1

        # In case this leads to the 100th maneuver attempt,
        # notify the user and exit the station keeping loop
        if len(self.maneuver_attempts) >= 100:
            raise RuntimeError("Max burns! "
                    + "Current burn duration = "
                    + f"{self.burn_duration} sec")

        backtrack_time = self.coast_duration
        self.coast_duration = 0
        return {
            "action": "back_prop_coast",
            "dt": DT_THRUST,
            "burn_end_time": burn_end_time,
            "back_track_coast_time": backtrack_time
        }

    def _i_burn_overshoot(
            self
    ):
        self.maneuver_attempts.append(self.burn_duration)
        if PRINT_I_AXIS_MANEUVER_ATTEMPTS:
            i_axis_maneuver_attempt_debug_message(
                len(self.maneuver_attempts),
                self.min_i_pos,
                self.burn_duration
            )

        # Estimate the time steps needed to correct the
        # overshoot (1 `DT_THRUST` time step per missed Km)
        stepsToBackTrack = abs(
            np.ceil(
                self.min_i_pos + DEADBAND_TRIGGER_RATIO * I_BOUNDS
            )
        )

        # Verify maneuver duration hasn't been tried to prevent
        # an infinite-loop.
        if (self.burn_duration - DT_THRUST * stepsToBackTrack
            in self.maneuver_attempts
        ):
            stepsToBackTrack -=1

        backtrack_burn_time = stepsToBackTrack * DT_THRUST
        self.burn_duration -= backtrack_burn_time

        if self.burn_duration < 0:
            self.burn_duration = 5 * DT_THRUST
            self.negative_time_correction_tries -=  1
            if self.negative_time_correction_tries <= 0:
                raise RuntimeError("Too many attempts to fix a negative burn time")

        # As an unsuccessful maneuver, remove its end time
        burn_end_time = self.maneuver_ends[-1]
        self.maneuver_ends.pop()
        # In case the maneuver time goes negative while the
        # algorithm searches for the shorter maneuver time to
        # bring in the overshoot of `I_BOUNDS`, message the
        # user in the terminal and exit the station keeping
        # loop
        if self.burn_duration < 0:
            raise RuntimeError("Negative thrust time! Min I = "
                    + str(self.min_i_pos)
            )

        # In case this leads to the 100th maneuver attempt,
        # notify the user and exit the station keeping loop
        if len(self.maneuver_attempts) == 100:
            raise RuntimeError("Max burns! current burn duration = "
                    + str(self.burn_duration) + " sec | Min I = "
                    + str(self.min_i_pos)
            )

        backtrack_coast_time = self.coast_duration
        self.coast_duration = 0
        return {
            "action": "back_prop_coast_and_burn",
            "dt": DT_THRUST,
            "burn_end_time": burn_end_time,
            "back_track_coast_time": backtrack_coast_time,
            "back_track_burn_time": backtrack_burn_time
        }

    def _i_burn(
            self,
            elapsed_time: float,
            ACCEL: dict,
            thruster_axis: str
        ):
        """In-track burn: 1D finite-burn duration shooter (sim only).

        Free variable = burn duration. Reverse RK89 steps after a
        failed score are a **sim reset / targeting retry**, not a
        flyable closed-loop ops sequence.

        Algorithm:
        - First entry: fire I+ for at least `MIN_DUTY_TIME`, then coast.
        - Live gate before scoring: ``coast_duration > 4 * PERIOD`` and
          mean ``coes_avg_diff["del_a"] < 0`` (truth drifting back
          toward the reference). `min_i_pos_timer` is still updated
          but unused by the live gate (commented alt on the ``if``).
        - Score on ``min_i_pos`` (most negative I), not ``max_i_pos``:
          - Goldilocks: |min_i|/I_BOUNDS in (DEADBAND_TRIGGER_RATIO, 1]
          - Undershoot: |min_i|/I_BOUNDS <= DEADBAND_TRIGGER_RATIO
            -> reverse-step coast, lengthen burn
          - Overshoot: |min_i|/I_BOUNDS > 1
            -> reverse-step coast and burn, shorten burn
        - ``max_i_pos`` is print-only (successful-I terminal print).
        - Negative commanded burn duration or >100 attempts aborts.

        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation began.
        ACCEL : dict
            Acceleration map keyed by thruster axis.
        thruster_axis : str
            Active thruster axis, or ``""`` while coasting after cutoff.

        Returns
        -------
        dict
            ``continue``, ``stop_burn``, or an I-axis back-prop action.
        """
        if thruster_axis != "":
            self.burn_duration += DT_THRUST

            # `DT_THRUST` steps remaining this maneuver attempt
            self.estimated_steps -= 1
            maneuver_attempts = len(self.maneuver_attempts)
            new_maneuver = (self.burn_duration >= MIN_DUTY_TIME
                            and maneuver_attempts == 0)
            maneuver_attempt = (maneuver_attempts > 0
                                and self.estimated_steps <= 0)

            if new_maneuver or maneuver_attempt:
                self.maneuver_ends.append(elapsed_time)

                # `burn_duration` is not set to 0 here, the maneuver
                # duration may be altered later.

                # Set the simulation time step equal such that
                # `elapsed_time` is aligned with `DT_COAST`
                dt_to_maj_time_step = DT_COAST - round(elapsed_time % DT_COAST)

                self.max_i_pos = self.rv_ric[1]
                self.min_i_pos = self.rv_ric[1]
                self.min_i_pos_timer = 2 * self.PERIOD_IN_SECONDS

                self.coast_duration = dt_to_maj_time_step - DT_COAST
                self.state = "I burn"
                return {
                    "action": "stop_burn",
                    "new_state": self.state,
                    "dt": dt_to_maj_time_step
                }
        else:
            self.coast_duration += DT_COAST
            self.min_i_pos_timer -= DT_COAST

            if self.rv_ric[1] > self.max_i_pos:
                self.max_i_pos = self.rv_ric[1]

            if self.rv_ric[1] < self.min_i_pos:
                self.min_i_pos = self.rv_ric[1]
                self.min_i_pos_timer = 2 * self.PERIOD_IN_SECONDS

            # Live gate: coast > 4 periods AND mean del_a < 0 (leftover-
            # negative Δa at cutoff = min-coast guard). min_i_pos_timer is
            # updated above but unused here (alt gate kept commented).
            if self.coast_duration > 4 * self.PERIOD_IN_SECONDS and self.coes_avg_diff["del_a"] < 0: # self.min_i_pos_timer <= 0:
                # Termination conditions (scored on min_i_pos, not max_i_pos):
                # - Goldilocks: |min_i| / I_BOUNDS in (DEADBAND_TRIGGER_RATIO, 1]
                # - Undershoot: |min_i| too small → more thrusting
                # - Overshoot: |min_i| beyond I_BOUNDS → less thrusting

                termination_conditions = [
                    DEADBAND_TRIGGER_RATIO < abs(self.min_i_pos / I_BOUNDS) <= 1,
                    abs(self.min_i_pos / I_BOUNDS) <= DEADBAND_TRIGGER_RATIO,
                    abs(self.min_i_pos / I_BOUNDS) > 1
                ]

                if termination_conditions[0]:
                    return self._i_burn_goldilocks(elapsed_time, ACCEL)

                if termination_conditions[1]:
                    return self._i_burn_undershoot()

                if termination_conditions[2]:
                    return self._i_burn_overshoot()
        return {
            "action": "continue"
        }

    def _c_burn(
            self,
            elapsed_time: float,
            ACCEL: dict
    ) -> dict:
        """Terminate the cross-track burn when the window ends or duty max hits.

        Live window half-width is ``4 * MANEUVER_ARC_HALF_ANGLE`` about
        `crit_angle` (= 160 deg half-width when half-angle is 20; not the
        legacy 2x). On exit, accumulate delta-v and enter
        ``returning from C burn``.

        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation began.
        ACCEL : dict
            Acceleration map keyed by thruster axis.

        Returns
        -------
        dict
            ``continue`` or ``stop_burn`` with bookkeeping fields.
        """

        self.burn_duration += DT_THRUST

        # Live C-window half-width: 4 * `MANEUVER_ARC_HALF_ANGLE` (not 2×)
        true_lat = (self.truth_coes[-2] + self.truth_coes[-1]) % 360

        crit_angle = self._calc_crit_angle()
        window_opens = crit_angle - MANEUVER_ARC_HALF_ANGLE * 4
        window_closes = crit_angle + MANEUVER_ARC_HALF_ANGLE * 4

        if window_closes > 360:
            in_cross_track_pass = (window_opens < true_lat
                                    or true_lat <= window_closes % 360)
        elif window_opens < 0:
            in_cross_track_pass = ((window_opens + 360) < true_lat
                                    or true_lat <= window_closes % 360)
        else:
            in_cross_track_pass = (window_opens < true_lat
                                    and true_lat < window_closes)


        if self.burn_duration >= MAX_DUTY_TIME or not in_cross_track_pass:
            delta_v = ACCEL[self.thruster_axis] * self.burn_duration
            self.total_delta_v += delta_v

            maneuver_duration = self.burn_duration
            self.burn_duration = 0
            self.state = "returning from C burn"
            self.maneuver_ends.append(elapsed_time)

            return {
                "action": "stop_burn",
                "new_state": self.state,
                "dt": DT_COAST - round(elapsed_time % DT_COAST),
                "maneuver_duration": maneuver_duration,
                "maneuver_delta_v": delta_v,
                "total_delta_v": self.total_delta_v,
            }

        return {"action": "continue"}

    # Verifying Recovery
    def _return_from_r(
            self,
            elapsed_time: float,
    ) -> dict:
        """Verify radial maneuver recovery after thrusters cut off.

        Success when R amplitude drops to
        ``DEADBAND_TRIGGER_RATIO * R_BOUNDS``. If still high after
        0.25 orbital periods, re-enter ``wait for R burn``.

        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation began.

        Returns
        -------
        dict
            ``successful_maneuver``, ``maneuver_required``, or ``continue``.
        """
        # If the amplitude has dropped to less than
        # `DEADBAND_TRIGGER_RATIO` percent of `R_BOUNDS`, return to nominal
        #
        # If the amplitude has not dropped after 1/4 of an orbital period,
        # reenter "wait for R burn"
        if self.amp_ric["R"] <= DEADBAND_TRIGGER_RATIO * R_BOUNDS:
            self.state = self.interrupted_state
            self.interrupted_state = "nominal"

            return {
                "action": "successful_maneuver",
                "new_state": self.state,
                "interrupted_state": self.interrupted_state
            }
        if (round_to_time_step(elapsed_time)
                - self.maneuver_ends[-1] > 0.25 * self.PERIOD_IN_SECONDS
        ):
            self.state = "wait for R burn"
            return {
                "action": "maneuver_required",
                "new_state": self.state
            }

        return {"action": "continue"}

    def _return_from_c(
            self,
            elapsed_time: float
    ):
        """Verify cross-track maneuver recovery after thrusters cut off.

        Success when C amplitude drops below
        ``DEADBAND_TRIGGER_RATIO * C_BOUNDS``. If still high after
        0.25 orbital periods, re-enter ``wait for C burn``.

        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation began.

        Returns
        -------
        dict
            ``successful_maneuver``, ``maneuver_required``, or ``continue``.
        """

        # Determine if the amplitude of the C position oscillation has
        # dropped below `DEADBAND_TRIGGER_RATIO` percent of `C_BOUNDS`
        c_amp_corrected = self.amp_ric["C"] / C_BOUNDS < DEADBAND_TRIGGER_RATIO

        if c_amp_corrected:
            self.state = self.interrupted_state
            self.interrupted_state = "nominal"

            return {
                "action": "successful_maneuver",
                "new_state": self.state,
                "interrupted_state": self.interrupted_state
            }

        if (round_to_time_step(elapsed_time)
                - self.maneuver_ends[-1] > .25 * self.PERIOD_IN_SECONDS
        ):
            self.state = "wait for C burn"
            # If the amplitude hasn't recovered after 1/4 of an orbit,
            # prepare to try again
            return {
                "action": "maneuver_required",
                "new_state": self.state
            }

        return {"action": "continue"}
