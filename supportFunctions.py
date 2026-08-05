import datetime as dt
import numpy as np

from load_gmat import gmat
from simulationParameters import DT_COAST

def round_to_time_step(t):
    return t - t % DT_COAST

def xyz2ric(
        refState: list[float], 
        offsetState: list[float]
        ) -> (list[float] | np.ndarray[np.float64]):
    """
    Given the reference state, 'refState', compute the offset vector in
    the RIC frame.
    
    Returns
    -------
    - list[float]
        Contains the rotated, RIC frame state vector of the original
        'offsetState'.
    - np.NDArray[np.float64]
        Contains the 3x3 rotation matrix to rotate the ECI frame to RIC.
    """

    # Piece out the position and velocity vectors
    r_ref = np.array(refState[:3])
    v_ref = np.array(refState[3:6])
    r_offset = np.array(offsetState[:3])
    v_offset = np.array(offsetState[3:6])

    # Compute unit vectors for RIC frame
    R = r_ref / np.linalg.norm(r_ref)
    h_vec = np.cross(r_ref, v_ref)
    C = h_vec / np.linalg.norm(h_vec)
    I = np.cross(C, R)

    # Rotation matrix from inertial to RIC
    rotMatrix = np.vstack((R, I, C))
    
    # Position delta in ECI frame
    delta_r = r_offset - r_ref
    delta_v = v_offset - v_ref

    # Position delta in RIC frame
    r_RIC = rotMatrix @ delta_r
    v_RIC = rotMatrix @ delta_v

    # Combine the position and velocity vectors
    rv_RIC = list(r_RIC) + list(v_RIC)
    return rv_RIC, rotMatrix

def getEpoch_As_Datetime(dateStr: str) -> dt.datetime:
    """ Converts a given date string into a datetime object.
    
    Parameters
    ----------
    dateStr : str
    
    Returns
    -------
    dt.datetime
    """

    epoch = dt.datetime.strptime(dateStr, "%d %b %Y %H:%M:%S.%f")
    return epoch

def getEpoch_As_Str(date: dt.datetime = dt.datetime.today()) -> str:
    """ Converts a given date into a string.
    
    Parameters
    ----------
    date : dt.datetime, default=dt.datetime.today()

    Returns
    -------
    str
        strftime provides too many millisecond digits (6) and epoch is
        stripped to return all but the last 3 digits.

    """
    epoch = date.strftime("%d %b %Y %H:%M:%S.%f")
    return epoch[:-3]

def getEpoch_As_ModITC(date: dt.datetime = dt.datetime.today()) -> str:
    """ For a given date, return it as a string in the Modified
    International Telecommunications Corporation.

    Parameters
    ----------
    date : dt.datetime, default=dt.datetime.today()

    Returns
    -------
    str 
        strftime provides too many millisecond digits (6) and epoch is
        stripped to return all but the last 3 digits.
    """

    epoch = date.strftime("%Y%j%H%M%S.%f")
    return epoch[:-3]

def getEpoch_From_Satellite(sat: gmat.Spacecraft) -> float:
    """
    Returns the spacecraft's epoch in GMAT's default time format.

    GMAT measures time using the Modified Julian Date based on
    International Atomic Time (TAIModJulian). This measures the
    number of days it has been since Nov 17, 1858 @ 0000Z without
    incorporating leap seconds.

    Returns
    -------
    float
        The number of days since Nov 17, 1858 @ 0000Z
    """

    return sat.GetEpoch()

def get_r_axis_maneuver_print(
        burn_start,
        burn_duration,
        thruster_axis,
        r_amp,
        delta_v,
        total_delta_v,
    ):
    terminal_output = "t = "
    if (burn_start) >= 1000:
        terminal_output += f"{burn_start:4.2f} days | "
    elif burn_start >= 100:
        terminal_output += f"{burn_start:3.2f} days  | "
    elif burn_start >= 10:
        terminal_output += f"{burn_start:2.2f} days   | "
    else:
        terminal_output += f"{burn_start:1.2f} days    | "
    terminal_output += f"{thruster_axis} burn duration (min) = "

    if (burn_duration) >= 10:
        terminal_output += f"{(burn_duration):2.2f} | "
    else:
        terminal_output += f"{(burn_duration):1.3f} | "

    terminal_output += f"R-axis Amplitude = {r_amp:0.3f} km        | "
    terminal_output += f"deltaV = {delta_v:1.3f} m/s | "
    terminal_output += f"total deltaV = {total_delta_v:1.3f} m/s"
    print(terminal_output)

def get_i_axis_print(
        burn_start,
        burn_duration,
        del_a_energy_maneuver,
        del_a_target,
        delta_v,
        total_delta_v,
):
    terminal_output = "t = "
    if (burn_start) >= 1000:
        terminal_output += f"{burn_start:4.2f} days | "
    elif burn_start >= 100:
        terminal_output += f"{burn_start:3.2f} days  | "
    elif burn_start >= 10:
        terminal_output += f"{burn_start:2.2f} days   | "
    else:
        terminal_output += f"{burn_start:1.2f} days    | "

    terminal_output += "I+ burn duration (min) = "

    if (burn_duration) >= 10:
        terminal_output += f"{(burn_duration):2.2f} | "
    else:
        terminal_output += f"{(burn_duration):1.3f} | "

    terminal_output += f"Recovered del_a = {(del_a_energy_maneuver):0.3f} / {(del_a_target):0.3f} km | "

    terminal_output += f"deltaV = {delta_v:1.3f} m/s | "
    terminal_output += f"total deltaV = {total_delta_v:1.3f} m/s"
    print(terminal_output)

def get_c_axis_print(
        burn_start,
        burn_duration,
        thruster_axis,
        c_amp,
        delta_v,
        total_delta_v
):
    terminal_output = "t = "
    if (burn_start) >= 1000:
        terminal_output += f"{burn_start:4.2f} days | "
    elif burn_start >= 100:
        terminal_output += f"{burn_start:3.2f} days  | "
    elif burn_start >= 10:
        terminal_output += f"{burn_start:2.2f} days   | "
    else:
        terminal_output += f"{burn_start:1.2f} days    | "

    terminal_output += f"{thruster_axis} burn duration (min) = "

    if (burn_duration) >= 10:
        terminal_output += f"{(burn_duration):2.2f} | "
    else:
        terminal_output += f"{(burn_duration):1.3f} | "

    terminal_output += f"C-axis Amplitude = {c_amp:0.3f} km       | "

    terminal_output += f"deltaV = {delta_v:1.3f} m/s | "
    terminal_output += f"total deltaV = {total_delta_v:1.3f} m/s"
    print(terminal_output)

def i_axis_maneuver_attempt_message(
        maneuver_attempts : int,
        min_i_pos : float,
        burn_duration : float
):
    maneuver_count_str = f"Maneuver #{maneuver_attempts}:"
    min_i_position_str = f"I-position = {min_i_pos:1.4}km | "
    burn_time_str = f"Burn time = {burn_duration} sec"
    print(maneuver_count_str + min_i_position_str + burn_time_str)
