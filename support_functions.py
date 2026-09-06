"""
Time grid alignment, ECI -> RIC frame, epoch type conversion, and
maneuver log formatting.
"""

import datetime as dt
import numpy as np

from load_gmat import gmat
from simulationParameters import DT_COAST

def round_to_time_step(t: float) -> float:
    """ Round down provided time to nearest multiple of `DT_COAST`.

    The station keeping controller records the history of the truth
    spacecraft's RIC state vector, differences in the truth and
    reference spacecraft's Keplerian elements, and the rolling buffers
    along a time grid based on multiples of `DT_COAST`. Any time steps
    during a maneuver must defer to the time grid before indexing into
    the above mentioned dictionaries and deques.

    Parameters
    ----------
    t : float
        Simulated elapsed time in seconds.

    Returns
    -------
    float
        The largest multiple of `DT_COAST` that is less than or equal
        to `t`.
    """

    # Compute the modulus of t with respect to `DT_COAST` and subtract it from
    # `t` to floor the time onto the time grid.
    return t - t % DT_COAST

def xyz2ric(
        ref_state: list[float],
        true_state: list[float]
        ) -> tuple[list[float], np.ndarray]:
    """
    Rotate the truth spacecraft's offset from the reference spacecraft
    out of the Earth-centered inertial frame and into the RIC frame.

    Position: ``r_ric = C @ (r_truth - r_ref)`` with rows of ``C`` the
    RIC unit vectors built from the reference state.

    Velocity / rates: ``v_ric = C @ (v_truth - v_ref)`` — the relative
    ECI velocity is only rotated. This does **not** subtract the
    transport term ``ω × δr`` (relative velocity in a rotating RIC
    frame). Treat the returned rates as ``C*(vt-vr)``, not full RIC
    relative velocity.

    Parameters
    ----------
    ref_state : list[float]
        6-element array holding the Cartesian state vector of the
        reference spacecraft
        [x km, y km, z km, x km/s, y km/s, z km/s].
    true_state : list[float]
        6-element array holding the Cartesian state vector of the
        truth spacecraft
        [x km, y km, z km, x km/s, y km/s, z km/s].

    Returns
    -------
    tuple
        - rv_ric : list[float]
            Contains the rotated, RIC frame state vector of the
            original 'true_state'.
        - rot_matrix : np.ndarray
            Contains the 3x3 rotation matrix to rotate the ECI frame to
            RIC.

    Raises
    ------
    RuntimeError
        If the length of `ref_state` or `true_state` is not exactly 6
        elements long, alert user and end simulation.
    """
    # Verify 6 elements were provided (3 for position and 3 for velocity).
    if len(ref_state) != 6 or len(true_state) != 6:
        raise RuntimeError(
            "Invalid number of arguments! "
            "Ensure inputs are exactly 6 elements long")

    # Convert position and velocity vectors to np.array's
    ref_state = np.array(ref_state)
    true_state = np.array(true_state)

    # Compute unit vectors for RIC frame
    r_axis = ref_state[:3] / np.linalg.norm(ref_state[:3])
    h_vec = np.cross(ref_state[:3], ref_state[3:])
    c_axis = h_vec / np.linalg.norm(h_vec)
    i_axis = np.cross(c_axis, r_axis)

    # Rotation matrix from inertial to RIC
    rot_matrix = np.vstack((r_axis, i_axis, c_axis))

    # Position delta in ECI frame
    delta_r = true_state[:3] - ref_state[:3]
    delta_v = true_state[3:] - ref_state[3:]

    # Position delta in RIC frame
    r_ric = rot_matrix @ delta_r
    v_ric = rot_matrix @ delta_v

    # Combine the position and velocity vectors
    rv_ric = list(r_ric) + list(v_ric)
    return (rv_ric, rot_matrix)

# ----------------- Time instant conversions ----------------------------------
def get_epoch_as_datetime(date_str: str) -> dt.datetime:
    """ Parse a GMAT UTCGregorian epoch string into a datetime.
    
    Expected Format
    ---------------
    ``"dd mmm yyyy HH:MM:SS.fff"``
    Example: ``"26 Aug 2026 00:00:00.000"``

    Parameters
    ----------
    date_str : str
        Epoch written in GMAT's UTCGregorian format.
    
    Returns
    -------
    dt.datetime
        dt.datetime representing the same instant.
    
    Raises
    ------
    ValueError
        If `date_str` is not in the expected format.
    """

    epoch = dt.datetime.strptime(date_str, "%d %b %Y %H:%M:%S.%f")
    return epoch

def get_epoch_as_str(date: dt.datetime = dt.datetime.today()) -> str:
    """ Format a datetime as a GMAT UTCGregorian string.
    
    Parameters
    ----------
    date : dt.datetime, default=dt.datetime.today()
        Time instant to convert

    Returns
    -------
    str
        ``"dd mmm yyyy HH:MM:SS.fff"`` with millisecond precision.
        ``strftime("%f")`` provides 6 millisecond digits, however the
        last three decimal places are stripped to match GMAT's
        millisecond field width.
    """
    epoch = date.strftime("%d %b %Y %H:%M:%S.%f")
    return epoch[:-3]

def get_epoch_from_satellite(sat: gmat.Spacecraft) -> float:
    """
    Returns the spacecraft's epoch in GMAT's default time format.

    GMAT measures time using the Modified Julian Date based on
    International Atomic Time (TAIModJulian). This measures the
    number of days it has been since Nov 17, 1858 @ 0000Z without
    incorporating leap seconds.

    Parameters
    ----------
    sat : gmat.Spacecraft
        GMAT object of interest.

    Returns
    -------
    float
        The number of days since Nov 17, 1858 @ 0000Z.
    """

    return sat.GetEpoch()


def get_epoch_as_mod_itc(date: dt.datetime = dt.datetime.today()) -> str:
    """ Format a datetime as year + day-of-year + clock (not MJD/ITC).

    Despite the historical ``mod_itc`` name, this is **not** Modified
    Julian Date and not an ITC/TAI epoch. It is
    ``strftime("%Y%j%H%M%S.%f")`` truncated to millisecond precision:
    four-digit year, three-digit day-of-year, then HHMMSS.fff.

    Parameters
    ----------
    date : dt.datetime, default=dt.datetime.today()
        Time instant to convert.

    Returns
    -------
    str
        ``"yyyyDOYHHMMSS.fff"`` (e.g. ``2026249150530.068``).
        ``strftime("%f")`` provides 6 fractional digits; the last three
        are stripped to match GMAT's millisecond field width.
    """

    epoch = date.strftime("%Y%j%H%M%S.%f")
    return epoch[:-3]

# ----------------- Maneuver logging ------------------------------------------
def get_r_axis_print(
        burn_start: float,
        burn_duration: float,
        thruster_axis: str,
        r_amp: float,
        delta_v: float,
        total_delta_v: float,
    ) -> None:
    """ Print to terminal the results of a R-axis maneuver. 
    
    Parameters
    ----------
    burn_start : float
        Elapsed time in days when the maneuver begins.
    burn_duration : float
        Number of minutes the thrusters were firing.
    thruster_axis : str
        Which thruster axis and direction were the thrusters firing.
    r_amp : float
        R-axis oscillation amplitude at the termination of the
        maneuver.
    delta_v : float
        Amount of delta-v imparted onto the orbit during the maneuver.
    total_delta_v : float
        Sum of delta-v imparted onto the orbit from all maneuvers.
    """

    terminal_output = "t = "

    # Depending on the number of digits in `burn_start`, assign the
    # appropriate spacing.
    if (burn_start) >= 1000:
        terminal_output += f"{burn_start:4.2f} days | "
    elif burn_start >= 100:
        terminal_output += f"{burn_start:3.2f} days  | "
    elif burn_start >= 10:
        terminal_output += f"{burn_start:2.2f} days   | "
    else:
        terminal_output += f"{burn_start:1.2f} days    | "

    terminal_output += f"{thruster_axis} burn duration (min) = "

    # Depending on the number of digits in `burn_duration`, assign the
    # appropriate spacing.
    if (burn_duration) >= 10:
        terminal_output += f"{(burn_duration):2.2f} | "
    else:
        terminal_output += f"{(burn_duration):1.3f} | "

    # R-axis specific information
    terminal_output += f"R-axis Amplitude = {r_amp:0.3f} km"
    terminal_output += 20 * " " + "| "

    # Maneuver and total delta-v information.
    terminal_output += f"deltaV = {delta_v:1.3f} m/s | "
    terminal_output += f"total deltaV = {total_delta_v:1.3f} m/s"
    print(terminal_output)

def get_i_axis_print(
        burn_start: float,
        burn_duration: float,
        max_i_pos: float,
        min_i_pos: float,
        delta_v: float,
        total_delta_v: float,
) -> None:
    """ Print to terminal the results of a I-axis maneuver.
    
    Parameters
    ----------
    burn_start : float
        Elapsed time in days when the maneuver begins.
    burn_duration : float
        Number of minutes the thrusters were firing.
    max_i_pos : float
        The maximum I-axis position post maneuver.
    min_i_pos : float
        The minimum I-axis position post maneuver.
    delta_v : float
        Amount of delta-v imparted onto the orbit during the maneuver.
    total_delta_v : float
        Sum of delta-v imparted onto the orbit from all maneuvers.
    """
    terminal_output = "t = "

    # Depending on the number of digits in `burn_start`, assign the
    # appropriate spacing.
    if (burn_start) >= 1000:
        terminal_output += f"{burn_start:4.2f} days | "
    elif burn_start >= 100:
        terminal_output += f"{burn_start:3.2f} days  | "
    elif burn_start >= 10:
        terminal_output += f"{burn_start:2.2f} days   | "
    else:
        terminal_output += f"{burn_start:1.2f} days    | "

    terminal_output += "I+ burn duration (min) = "

    # Depending on the number of digits in `burn_duration`, assign the
    # appropriate spacing.
    if (burn_duration) >= 10:
        terminal_output += f"{(burn_duration):2.2f} | "
    else:
        terminal_output += f"{(burn_duration):1.3f} | "

    # I-axis specific information
    terminal_output += (
        f"Max/Min I-axis positions = {(max_i_pos):0.3f} / "
    )
    terminal_output += (
        f"{(min_i_pos):0.3f} km | "
    )

    # Maneuver and total delta-v information.
    terminal_output += f"deltaV = {delta_v:1.3f} m/s | "
    terminal_output += f"total deltaV = {total_delta_v:1.3f} m/s"
    print(terminal_output)

def get_c_axis_print(
        burn_start: float,
        burn_duration: float,
        thruster_axis: str,
        c_amp: float,
        delta_v: float,
        total_delta_v: float
) -> None:
    """ Print to terminal the results of a C-axis maneuver. 
    
    Parameters
    ----------
    burn_start : float
        Elapsed time in days when the maneuver begins.
    burn_duration : float
        Number of minutes the thrusters were firing.
    thruster_axis : str
        Which thruster axis and direction were the thrusters firing.
    c_amp : float
        C-axis oscillation amplitude at the termination of the
        maneuver.
    delta_v : float
        Amount of delta-v imparted onto the orbit during the maneuver.
    total_delta_v : float
        Sum of delta-v imparted onto the orbit from all maneuvers.
    """
    terminal_output = "t = "

    # Depending on the number of digits in `burn_start`, assign the
    # appropriate spacing.
    if (burn_start) >= 1000:
        terminal_output += f"{burn_start:4.2f} days | "
    elif burn_start >= 100:
        terminal_output += f"{burn_start:3.2f} days  | "
    elif burn_start >= 10:
        terminal_output += f"{burn_start:2.2f} days   | "
    else:
        terminal_output += f"{burn_start:1.2f} days    | "

    terminal_output += f"{thruster_axis} burn duration (min) = "

    # Depending on the number of digits in `burn_duration`, assign the
    # appropriate spacing.
    if (burn_duration) >= 10:
        terminal_output += f"{(burn_duration):2.2f} | "
    else:
        terminal_output += f"{(burn_duration):1.3f} | "

    # C-axis specific information
    terminal_output += f"C-axis Amplitude = {c_amp:0.3f} km"
    terminal_output += 20 * " " + "| "

    # Maneuver and total delta-v information.
    terminal_output += f"deltaV = {delta_v:1.3f} m/s | "
    terminal_output += f"total deltaV = {total_delta_v:1.3f} m/s"
    print(terminal_output)

def i_axis_maneuver_attempt_debug_message(
        maneuver_attempts : int,
        min_i_pos : float,
        burn_duration : float
) -> None:
    """
    Prints messages for each maneuver attempt during I-axis maneuver
    algorithm.
    
    Parameters
    ----------
    maneuver_attempts : int
        The previous maneuver attempt number.
    min_i_pos  : float
        The minimum I-axis position achieved during previous maneuver
        attempt.
    burn_duration : float
        How long were the thrusters firing in seconds.
    """

    maneuver_count_str = f"Maneuver #{(maneuver_attempts)}: "
    min_i_position_str = f"I-position = {min_i_pos:.4f}km | "
    burn_time_str = f"Burn time = {burn_duration} sec"
    print(maneuver_count_str + min_i_position_str + burn_time_str)
