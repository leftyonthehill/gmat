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
    COE_KEYS : list
        list of the keys used to store the differences in orbital
        elements between the truth and reference spacecraft.
    RIC_KEYS : list
        list of the keys used to store the RIC position and velocity
        vectors of the truth spacecraft.
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
    thrusting : bool
        Thrusters firing flag during I-axis maneuvers.
    thruster_axis : str
        Thruster axis and direction during R/C-axis maneuvers.
    negative_time_correction_tries : int
        Specific to the I-axis maneuver algorithm, the number of tries
        the controller is allowed to suggest a negative thrust time
        before terminating the simulation.
    estimated_steps : int
        Specific to the I-axis maneuver algorithm, the number of
        DT_THRUST steps for the truth spacecraft to add or remove from
        `burn_duration` to achieve a nominal drag recovery.
    coast_duration : float
        Specific to the I-axis maneuver algorithm, tracks the time
        since the conclusion of the maneuver for back propagation
        purposes.
    min_i_pos : float
        Specific to the I-axis maneuver algorithm, tracks the greatest
        negative I-axis position during each maneuver attempt.
    max_i_pos : float
        Specific to the I-axis maneuver algorithm, tracks the greatest
        positive I-axis position during each maneuver attempt.
    """

    MU = 398600  # Earth’s gravitational parameter in km^3/s^2

    # Compute the number of steps per orbit based on state vector source.
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

        # Relevant telemetry (MUST BE PROVIDED EVERY STEP IN MAIN LOOP)
        self.amp_ric = {i: 0 for i in self.RIC_KEYS}
        self.coes_instant_diff = {i: 0 for i in self.COE_KEYS}
        self.coes_avg_diff = {i: 0 for i in self.COE_KEYS}
        self.rv_ric = [0, 0, 0, 0, 0, 0]
        self.truth_coes = [0, 0, 0, 0, 0, 0]
        self.ref_coes = [0, 0, 0, 0, 0, 0]
        self.thruster_axis = ""

        # Maneuver logging
        self.maneuver_starts = []
        self.maneuver_attempts = []
        self.maneuver_ends = []
        self.burn_duration = 0
        self.total_delta_v = 0
        self.thrusting = False

        # Maneuver results (for I-burn only)
        self.negative_time_correction_tries = 5
        self.estimated_steps = 0
        self.coast_duration = 0
        self.min_i_pos = 0
        self.max_i_pos = 0

    def update(
            self,
            elapsed_time: float,
            accel: dict
    ) -> dict:
        """ Determine the station keeping action to take.

        After updating the necessary attributes, determine what the
        necessary actions are to ensure station keeping within the
        operational boundary.
        
        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation started.
        accel : {str: float}
            Assuming constant thrust and mass, the acceleration map for
            each thruster axis.
        
        Returns
        -------
        dict
            The dict will contain what the spacecraft's next action is
            and if any values in the main loop need to be updated.
        """

        # At each major time step, evaluate if there have been any
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
                return self._r_burn(elapsed_time, accel)
            case "I burn":
                return self._i_burn(elapsed_time, accel)
            case "C burn":
                return self._c_burn(elapsed_time, accel)
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
        Evaluate if the truth spacecraft is in its radial
        maneuver window.

        The maneuver window will occur when:
        - The eccentricity vectors are aligned ``|del_aop| <= 3``
        - The true anomaly is approaching within
          `MANEUVER_ARC_HALF_ANGLE` of 90 deg or 270 deg
            - With the eccentricity vectors of the truth and reference
              state nearly aligned, the driving radial deviation comes
              from the difference in eccentricity. By performing a
              maneuver near a true anomaly value of 90 deg or 270 deg,
              the corrective action can be focused on minimizing
              ``del_e``.
        
        The direction of the maneuver, either "R+" or "R-", will be
        specified by the values of ``del_e`` and the angle of the true
        anomaly prior to the start of the maneuver. When these two
        conditions are met, then alert the spacecraft to begin
        thrusting along the R-axis. Otherwise if they are not met
        within one orbital period, quit looking for maneuver
        opportunities.

        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation began.
        
        Returns
        -------
        dict
            The dict will contain what the spacecraft's next action is
            and if any values in the main loop need to be updated.
        """

        ## Maneuver window identification ##
        self.steps_waiting += 1

        approaching_90 = (90 - MANEUVER_ARC_HALF_ANGLE
                          < self.truth_coes[-1]
                          < 90)
        approaching_270 = (270 - MANEUVER_ARC_HALF_ANGLE
                           < self.truth_coes[-1]
                           < 270)
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
            # the current maneuver window.
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
        Evaluate if the truth spacecraft is in its ideal in-track
        maneuver window.

        The ideal in-track maneuver window will open when the following
        are true:
            - The truth spacecraft is within `MANEUVER_ARC_HALF_ANGLE`
              degrees of either its perigee or apogee. The location of
              the maneuver will be decided by one of two conditions:
                - The sign of "del_e"
                    - "del_e" < 0 for a maneuver at perigee
                        - The reference apogee is higher than the truth
                        and the reference perigee is lower than the
                        truth. Performing the maneuver at perigee will
                        raise the truth spacecraft's apogee to
                        meet/exceed that of the reference spacecraft.
                    - "del_e" > 0 for a maneuver at apogee
                        - The reference  apogee is lower than the truth
                        and the reference perigee is higher than the
                        truth. Performing the maneuver at apogee will
                        raise the truth's perigee to meet/exceed that
                        of the reference spacecraft.
            - The previous maneuver occurred more than 3 periods ago
            - The truth spacecraft's average sma delta is less than 0
              Km.
        
        If these conditions are not met within one orbital period, quit
        looking for maneuver opportunities.

        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation began.
        
        Returns
        -------
        dict
            The dict will contain what the spacecraft's next action is
            and if any values in the main loop need to be updated.
        """

        ## Maneuver window identification ##
        self.steps_waiting += 1
        in_apogee_pass = (
            180 - MANEUVER_ARC_HALF_ANGLE < self.truth_coes[-1] <= 180)
        in_perigee_pass = (
            360 - MANEUVER_ARC_HALF_ANGLE < self.truth_coes[-1] <= 360)

        # Prioritize the maneuver window that also reduces "del_e", unless the
        # I-position is already greater than 95% of `I_BOUNDS`. Then take the
        # first available.
        if self.rv_ric[1] / I_BOUNDS < 0.95:
            if self.coes_avg_diff["del_e"] <= 0:
                in_burn_window = in_perigee_pass
            else:
                in_burn_window = in_apogee_pass
        else:
            in_burn_window = in_apogee_pass or in_perigee_pass

        if len(self.maneuver_starts) > 0:
            no_recent_maneuver = (elapsed_time - self.maneuver_ends[-1]
                                >= 3 * self.PERIOD_IN_SECONDS)
        else:
            no_recent_maneuver = True

        del_a_settled = self.coes_avg_diff["del_a"] < 0

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

        ## Spacecraft action reporting ##
        if in_burn_window and no_recent_maneuver and del_a_settled:
            self.steps_waiting = 0
            self.maneuver_starts.append((elapsed_time, "r"))

            self.state = "I burn"
            self.thrusting = True
            return {
                "action": "start_burn",
                "new_state": self.state,
                "thruster_axis": "I+",
                "dt": DT_THRUST
            }

        # If no other action is required, then continue looking for maneuver
        # opportunities.
        return {"action": "continue"}

    def _calc_crit_angle(self) -> float:
        """
        Compute the ideal angle to correct both inclination and RAAN.
        
        This computation uses a modified version of the heuristic by H.
        Schaub and J. Junkins in 'Analytical Mechanics of Space
        Systems', 4th Ed. This version multiplies the value of "del_i"
        by 10 so that "del_i" and "del_raan" have comparable
        magnitudes. Otherwise, the original computed the wrong critical
        angle.
        
        Returns
        -------
        float
            The truth spacecraft's true latitude angle of where to
            perform the maneuver.
        """

        crit_angle = np.rad2deg(np.arctan2(
                self.coes_avg_diff["del_raan"]
                * np.sin(np.deg2rad(self.ref_coes[2])),
                self.coes_avg_diff["del_i"] * 10
            )
        )

        # Ensure `crit_angle` is between 0 and 360 degrees.
        crit_angle += 360 if crit_angle < 0 else 0

        return crit_angle

    def _wait_for_c(
            self,
            elapsed_time: float
    ) -> dict:
        """
        Evaluate if the truth spacecraft is in its ideal cross-track
        maneuver window.

        The ideal cross-track maneuver window will open when the truth
        spacecraft is within 4 * `MANEUVER_ARC_HALF_ANGLE` degrees of
        `crit_angle`. The direction of the C-axis maneuver will vary
        depending on the average value of "del_raan" and the quadrant
        of `crit_angle`. If these conditions are not met within one
        orbital period, quit looking for maneuver opportunities.

        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation began.
        
        Returns
        -------
        dict
            The dict will contain what the spacecraft's next action is
            and if any values in the main loop need to be updated.
        """

        ## Maneuver window identification ##
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

        # If no other action is required, then continue looking for maneuver
        # opportunities.
        return {"action": "continue"}

    # ------------- Maneuvering -----------------------------------------------
    def _r_burn(
            self,
            elapsed_time: float,
            accel: dict
    ) -> dict:
        """
        Terminates the radial maneuver when the maneuver window closes
        or the maneuver reaches its maximum duration.
        
        During each time step the R-axis maneuver is active, check to
        see if any of the following termination criteria have been met:
        - The maneuver duration is `MIN_DUTY_TIME` seconds or longer.
        - Either:
            - Left maneuver window.
            - Reached maximum duty time.
        
        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation began.
        accel : dict
            dict of each thruster axis and its imparted acceleration on
            the spacecraft.
        
        Returns
        -------
        dict
            The dict will contain what the spacecraft's next action is
            and if any values in the main loop need to be updated.
        """

        self.burn_duration += DT_THRUST

        # Verifying that the spacecraft is still in a valid maneuver window.
        approaching_90 = (90 - MANEUVER_ARC_HALF_ANGLE
                          < self.truth_coes[-1]
                          < 90)
        approaching_270 = (270 - MANEUVER_ARC_HALF_ANGLE
                           < self.truth_coes[-1]
                           < 270)
        in_burn_window = approaching_90 or approaching_270

        in_del_aop_range = abs(self.coes_instant_diff["del_aop"]) <= 3

        in_node_window = in_burn_window and in_del_aop_range

        ## Spacecraft action reporting ##
        if ((self.burn_duration >= MAX_DUTY_TIME or not in_node_window)
            and self.burn_duration >= MIN_DUTY_TIME
        ):
            delta_v = accel[self.thruster_axis] * self.burn_duration
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

        # If no other action is required, then continue thrusting.
        return {"action": "continue"}

    def _i_burn_goldilocks(
            self,
            elapsed_time: float,
            accel: dict
    ) -> dict:
        """
        Alert the spacecraft that a viable I-axis maneuver has been
        found and to resume nominal ops.
        
        The current return trajectory of the truth spacecraft has a
        maximum displacement between `DEADBAND_TRIGGER_RATIO` and 1
        times the distance of the negative `I_BOUNDS` boundary.
        Alert the spacecraft of the successful maneuver and reset
        attributes.
        
        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation began.
        accel : dict
            dict of each thruster axis and its imparted acceleration on
            the spacecraft.
        
        Returns
        -------
        dict
            The dict will contain what the spacecraft's next action is
            and if any values in the main loop need to be updated.
        """

        # Update delta_v consumed during mission
        delta_v = accel["I+"] * self.burn_duration
        self.total_delta_v += delta_v

        # Reset maneuver specific attributes.
        maneuver_duration = self.burn_duration
        backtrack_time = self.coast_duration
        self.burn_duration = 0
        self.coast_duration = 0
        self.maneuver_attempts = []
        self.state = self.interrupted_state
        self.interrupted_state = "nominal"

        ## Spacecraft action reporting ##
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
        """
        Alert the spacecraft that additional maneuvering time is
        required, I-axis target is currently being undershot.
        
        Returns
        -------
        dict
            The dict will contain what the spacecraft's next action is
            and if any values in the main loop need to be updated.
        
        Raises
        ------
        RuntimeError
            The controller tried 100 different maneuver attempts to
            reach its goal. To prevent an infinite loop, end the
            simulation.
        """
        # Turn thrusters on
        self.thrusting = True

        # Store maneuver duration to prevent repeats.
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

        # Estimate the time steps needed to correct the undershoot
        # (`I_BURN_STEP_GAIN` *  'miss distance ', rounded to the next whole
        # number to get the number of `DT_THRUST` steps needed).
        self.estimated_steps = np.ceil(
            (self.min_i_pos + DEADBAND_TRIGGER_RATIO * I_BOUNDS)
            * I_BURN_STEP_GAIN
        )

        # Verify maneuver duration hasn't been tried to prevent
        # an infinite-loop.
        predicted_burn_duration = (self.burn_duration
                                    + self.estimated_steps * DT_THRUST)
        if predicted_burn_duration in self.maneuver_attempts:
            self.estimated_steps -=1

        # In case this leads to the 100th maneuver attempt, notify the user and
        # exit the station keeping loop
        if len(self.maneuver_attempts) >= 100:
            raise RuntimeError("Max burns! "
                    + "Current burn duration = "
                    + f"{self.burn_duration} sec")

        # Store time to back propagate and reset coast duration timer
        backtrack_time = self.coast_duration
        self.coast_duration = 0

        ## Spacecraft action reporting ##
        return {
            "action": "back_prop_coast",
            "dt": DT_THRUST,
            "burn_end_time": burn_end_time,
            "back_track_coast_time": backtrack_time
        }

    def _i_burn_overshoot(
            self
    ):
        """
        Alert the spacecraft that less maneuvering time is required,
        I-axis target is currently being overshot.
        
        Returns
        -------
        dict
            The dict will contain what the spacecraft's next action is
            and if any values in the main loop need to be updated.
        
        Raises
        ------
        RuntimeError
            The controller tried 100 different maneuver attempts to
            reach its goal. To prevent an infinite loop, end the
            simulation.
        RuntimeError
            The controller had too many attempts where it tried using a
            negative thrust duration time. To prevent an infinite loop,
            end the simulation.
        """
        # As an unsuccessful maneuver, remove its end time
        burn_end_time = self.maneuver_ends[-1]
        self.maneuver_ends.pop()

        # Turn thrusters on
        self.thrusting = True

        # Store maneuver duration to prevent repeats.
        self.maneuver_attempts.append(self.burn_duration)

        if PRINT_I_AXIS_MANEUVER_ATTEMPTS:
            i_axis_maneuver_attempt_debug_message(
                len(self.maneuver_attempts),
                self.min_i_pos,
                self.burn_duration
            )

        # Estimate the time steps needed to correct the overshoot
        # (`I_BURN_STEP_GAIN` *  'miss distance ', rounded to the next whole
        # number to get the number of `DT_THRUST` steps needed)
        steps_back = abs(
            np.ceil(
                (self.min_i_pos + DEADBAND_TRIGGER_RATIO * I_BOUNDS)
                * I_BURN_STEP_GAIN
            )
        )

        # Verify maneuver duration hasn't been tried to prevent an
        # infinite-loop.
        if (self.burn_duration - DT_THRUST * steps_back
            in self.maneuver_attempts
        ):
            steps_back -=1

        backtrack_burn_time = steps_back * DT_THRUST
        self.burn_duration -= backtrack_burn_time

        # In case the maneuver time goes negative while the algorithm searches
        # for the shorter maneuver time to bring in the overshoot of
        # `I_BOUNDS`, there is a chance that the controller overcorrected and
        # sent the spacecraft into a negative maneuver time. Should that be the
        # the case, reset the maneuver time to 5 times `DT_THRUST` and try
        # again. If the controller cannot converge on a solution after 5
        # attempts that produce a negative thrust time, raise a RuntimeError.
        if self.burn_duration < 0:
            self.burn_duration = 5 * DT_THRUST
            self.negative_time_correction_tries -=  1
            if self.negative_time_correction_tries <= 0:
                raise RuntimeError(
                    "Too many attempts to fix a negative burn time"
                )

        # In case this leads to the 100th maneuver attempt, notify the user and
        # exit the station keeping loop.
        if len(self.maneuver_attempts) >= 100:
            raise RuntimeError("Max burns! current burn duration = "
                    + str(self.burn_duration) + " sec | Min I = "
                    + str(self.min_i_pos)
            )

        # Store time to back propagate and reset coast duration timer
        backtrack_coast_time = self.coast_duration
        self.coast_duration = 0

        ## Spacecraft action reporting ##
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
            accel: dict
        ):
        """
        Contains the termination criteria for the in-track maneuvers.
        
        The algorithm to determine the "perfect" maneuver length is as
        follows:
        - Upon entering "I burn" for the first time, fire the thrusters
          for `MIN_DUTY_TIME`.
            - Corrects for atmospheric drag and sends the spacecraft
              drifting into I- direction.
        - Coast for a minimum of 4 orbits and wait until the average 
          "del_a" drops below 0.
            - Signifies that the truth spacecraft has begun to drift
              in I+ direction.
        - If `DEADBAND_TRIGGER_RATIO <= -min_i_pos / I_BOUNDS <= 1`,
          then a "goldilocks" trajectory has been found (within the
          allowed band for drag-loss recovery).
        - If `-min_i_pos / I_BOUNDS < DEADBAND_TRIGGER_RATIO`
          (undershoot), backwards propagate to the end of the maneuver
          and increase the burn duration.
            - Prefer `-min_i_pos` over abs so `min_i_pos` > 0 cannot
              trigger a "goldilocks" trajectory.
        - If `-min_i_pos > I_BOUNDS` (overshoot), back propagate to
          the end of the maneuver and into the maneuver to reduce burn
          duration.
            - Same `-min_i_pos` preference as above.
        - If the burn duration is commanded to be negative or the
          amount of maneuver corrections exceeds 100 attempts, the
          simulation is ended.
        
        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation began.
        accel : dict
            dict of each thruster axis and its imparted acceleration on
            the spacecraft.
        
        Returns
        -------
        dict
            The dict will contain what the spacecraft's next action is
            and if any values in the main loop need to be updated.
        
        Notes
        -----
        Backwards propagation operations (<integrator>.Step(-<time>))
        rely on RK89 being numerically irreversible only to within
        simulation tolerance. Small discontinuities at these seams
        are expected and acceptable.
        """
        if self.thrusting:
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

                # `burn_duration` is not set to 0 here, the maneuver duration
                # may be altered later.

                # Set the simulation time step so that `elapsed_time`
                # is aligned with `DT_COAST`.
                dt_to_maj_time_step = DT_COAST - round(elapsed_time % DT_COAST)

                # Set the baseline min/max values
                self.max_i_pos = self.rv_ric[1]
                self.min_i_pos = self.rv_ric[1]

                # Accounts for the coast time to get the simulation,
                # post-maneuver, back onto the time grid.
                self.coast_duration = dt_to_maj_time_step - DT_COAST

                self.state = "I burn"
                self.thrusting = False
                ## Spacecraft action reporting ##
                return {
                    "action": "stop_burn",
                    "new_state": self.state,
                    "dt": dt_to_maj_time_step
                }
        else:
            self.coast_duration += DT_COAST

            # Update min/max position values
            if self.rv_ric[1] > self.max_i_pos:
                self.max_i_pos = self.rv_ric[1]

            if self.rv_ric[1] < self.min_i_pos:
                self.min_i_pos = self.rv_ric[1]

            # Wait for at least 4 orbital periods and the average "del_a" must
            # be negative (signifies that the truth spacecraft is now drifting
            # to the reference) before evaluating the termination.
            min_time_passed = self.coast_duration > 4 * self.PERIOD_IN_SECONDS
            pos_i_drift = self.coes_avg_diff["del_a"] < 0
            if min_time_passed and pos_i_drift:
                # Termination conditions:
                # - Achieves deadband target by the time SMA changes sign (no
                #   change).
                # - Undershoots deadband target when SMA changes sign (more
                #   thrusting required).
                # - Overshoots deadband target (less thrusting required)

                termination_conditions = [
                    DEADBAND_TRIGGER_RATIO <= -self.min_i_pos / I_BOUNDS <= 1,
                    -self.min_i_pos / I_BOUNDS < DEADBAND_TRIGGER_RATIO,
                    -self.min_i_pos / I_BOUNDS > 1
                ]

                if termination_conditions[0]:
                    return self._i_burn_goldilocks(elapsed_time, accel)

                if termination_conditions[1]:
                    return self._i_burn_undershoot()

                if termination_conditions[2]:
                    return self._i_burn_overshoot()

        ## Spacecraft action reporting ##
        return {
            "action": "continue"
        }

    def _c_burn(
            self,
            elapsed_time: float,
            accel: dict
    ) -> dict:
        """
        Terminates the out of plane maneuver when the maneuver window
        closes or the maneuver reaches its maximum duration.
        
        During each time step the C-axis maneuver is active, check to
        see if any of the following termination criteria have been met:
        - Left maneuver window.
        - Reached maximum duty time.
        
        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation began.
        accel : dict
            dict of each thruster axis and its imparted acceleration on
            the spacecraft.
        
        Returns
        -------
        dict
            The dict will contain what the spacecraft's next action is
            and if any values in the main loop need to be updated.
        """

        self.burn_duration += DT_THRUST

        # C-axis maneuvers use wider arcs (4 * `MANEUVER_ARC_HALF_ANGLE`)
        true_lat = (self.truth_coes[-2] + self.truth_coes[-1]) % 360

        crit_angle = self._calc_crit_angle()
        window_opens = crit_angle - MANEUVER_ARC_HALF_ANGLE * 4
        window_closes = crit_angle + MANEUVER_ARC_HALF_ANGLE * 4

        # Verifying that the spacecraft is still in a valid maneuver window.
        if window_closes > 360:
            in_cross_track_pass = (window_opens < true_lat
                                    or true_lat <= window_closes % 360)
        elif window_opens < 0:
            in_cross_track_pass = ((window_opens + 360) < true_lat
                                    or true_lat <= window_closes % 360)
        else:
            in_cross_track_pass = (window_opens < true_lat
                                    and true_lat < window_closes)

        ## Spacecraft action reporting ##
        if self.burn_duration >= MAX_DUTY_TIME or not in_cross_track_pass:
            delta_v = accel[self.thruster_axis] * self.burn_duration
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


    # ------------- Verifying Recovery ----------------------------------------
    def _return_from_r(
            self,
            elapsed_time: float,
    ) -> dict:
        """ Verifies the radial maneuver performed nominally.
        
        After an R-axis maneuver is complete, monitor the
        position's oscillation amplitude. If
        `amp_ric["R"] / R_BOUNDS <= R_TARGET_RATIO` within 75% of one
        orbit then the maneuver is deemed successful. If the
        amplitude ratio > `R_TARGET_RATIO`, then alert the spacecraft
        that additional maneuvers are required.

        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation began.
        
        Returns
        -------
        dict
            The dict will contain what the spacecraft's next action is
            and if any values in the main loop need to be updated.
        """

        if self.amp_ric["R"] / R_BOUNDS <= R_TARGET_RATIO:
            self.state = self.interrupted_state
            self.interrupted_state = "nominal"

            return {
                "action": "successful_maneuver",
                "new_state": self.state,
                "interrupted_state": self.interrupted_state
            }

        # If the amplitude hasn't recovered after 3/4 of an orbit, prepare to
        # try again.
        if (round_to_time_step(elapsed_time)
                - self.maneuver_ends[-1] > 0.75 * self.PERIOD_IN_SECONDS
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
        """ Verifies the cross-track maneuver performed nominally.
        
        After a C-axis maneuver is complete, monitor the
        position's oscillation amplitude. If
        `amp_ric["C"] / C_BOUNDS <= C_TARGET_RATIO` within 75% of one
        orbit then the maneuver is deemed successful. If the
        amplitude ratio > `C_TARGET_RATIO`, then alert the spacecraft
        that additional maneuvers are required.

        Parameters
        ----------
        elapsed_time : float
            Time in seconds since the simulation began.
        
        Returns
        -------
        dict
            The dict will contain what the spacecraft's next action is
            and if any values in the main loop need to be updated.
        """

        if self.amp_ric["C"] / C_BOUNDS <= C_TARGET_RATIO:
            self.state = self.interrupted_state
            self.interrupted_state = "nominal"

            return {
                "action": "successful_maneuver",
                "new_state": self.state,
                "interrupted_state": self.interrupted_state
            }

        # If the amplitude hasn't recovered after 3/4 of an orbit,
        # prepare to try again
        if (round_to_time_step(elapsed_time)
                - self.maneuver_ends[-1] > .75 * self.PERIOD_IN_SECONDS
        ):
            self.state = "wait for C burn"
            return {
                "action": "maneuver_required",
                "new_state": self.state
            }

        return {"action": "continue"}
