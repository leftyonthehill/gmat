""" Station keeping scenario config file.

User-editable knobs for the live driver
(`station_keeping_maneuver_subs.py`) and controller:

- Scenario control: `MAX_DAYS`, `DT_COAST`, `DT_THRUST`,
  `REVOLUTIONS_TO_AVG`.
- Satellite / orbit: `STATE_VECT_SOURCE`, `ORBIT_STATE`,
  `REF_ORBIT_STATE`, `TRUTH_ORBIT_STATE`.
  When `STATE_VECT_SOURCE == "new"`, `ORBIT_STATE` epoch uses
  `datetime.today()`, so drag/SRP runs are non-reproducible across days
  unless you pin the epoch.
- Thruster on-time limits (seconds): `MIN_DUTY_TIME`, `MAX_DUTY_TIME`.
  Note: `MIN_DUTY_TIME` (900 s default) can exceed a short R-window arc
  (~636 s for half-angle 20 deg at LEO mean motion); R burns still
  require min duty before cutoff.
- Maneuver arc half-angle: `MANEUVER_ARC_HALF_ANGLE` (deg).
- RIC operational bounds: `R_BOUNDS`, `I_BOUNDS`, `C_BOUNDS` (km) and
  `DEADBAND_TRIGGER_RATIO`.
- Plot / print flags: `PLOT_*`, `PRINT_MANEUVER_MESSAGE`,
  `PRINT_I_AXIS_MANEUVER_ATTEMPTS` (consumed by `data_outputs.py`).

Do not change control-law math here without reviewing the controller.
"""

import datetime as dt

# ----------------- Scenario Control ------------------------------------------
# Duration of the scenario in days
MAX_DAYS = 380

# Simulation step size while coasting
DT_COAST = 120.0

# Simulation step size while thrusting
DT_THRUST = 5.0

# Number of orbits used to average out the oscillations of the perturbed
# orbital solutions
REVOLUTIONS_TO_AVG = 3.0

# ----------------- Satellite Characteristics ---------------------------------
STATE_VECT_SOURCE = "new" # "existing"
# Orbital element set shared by the initial reference and truth satellites
ORBIT_STATE = [
    6903,   # SMA, avg alt of 525 km
    1e-3,   # ECC
    53,     # INC
    0,      # RAAN
    0,      # AOP
    0,      # TA
    dt.datetime.today() # Epoch (non-reproducible when STATE_VECT_SOURCE=="new")
]

REF_ORBIT_STATE = [
    6903.022615120439,
    0.0003789185228938532,
    53.02596646627463,
    172.34682023269508,
    156.8237941903876,
    19.837004359809008,
    "25 Jul 2027 15:05:29.068"
]

TRUTH_ORBIT_STATE = [
    6903.165446226063,
    0.0003454338473190065,
    53.022234750210096,
    172.19088187033176,
    167.3702942965235,
    9.480140879340864,
    "25 Jul 2027 15:05:29.068"
]

# Thruster on-time limits (seconds)
MIN_DUTY_TIME = 900  # Minimum thruster on-time (seconds)
MAX_DUTY_TIME = 3600  # Maximum thruster on-time (seconds)

MANEUVER_ARC_HALF_ANGLE = 20  # deg; C live half-width is 4x this value

# ----------------- Station Keeping Parameters --------------------------------
# Operational bounds (+/-) to keep the truth satellite within (km)
R_BOUNDS = 10  # km
I_BOUNDS = 40  # km
C_BOUNDS = 15  # km

DEADBAND_TRIGGER_RATIO = 0.85
# ----------------- Plotting --------------------------------------------------
PLOT_3D_RIC = False

PLOT_RIC_POS = True

PLOT_RIC_POS_AMP = True

PLOT_RIC_VELO = False

PLOT_RIC_VELO_AMP = False

PLOT_COE_DIFFS = {
    "del_a": True,
    "del_e": True,
    "del_i": False  ,
    "del_raan": False,
    "del_aop": False,
    "del_f": False
}

PLOT_PHASE_DIFF = False

PLOT_MANEUVER_MARKERS = True

PRINT_MANEUVER_MESSAGE = True

# ----------------- Debugging -------------------------------------------------
PRINT_I_AXIS_MANEUVER_ATTEMPTS = False
