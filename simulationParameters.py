""" Station keeping scenario config file. """

import datetime as dt

# ----------------- Scenario Control ------------------------------------------
# Duration of the scenario in days
maxDays = 125

# Simulation step size while coasting
dtCoast = 60.0

# Simulation step size while thrusting
dtThrust = 5.0

# Number of orbits used to average out the oscillations of the perturbed
# orbital solutions
REVOLUTIONS_TO_AVERAGE = 3.0

# ----------------- Satellite Characterisitcs ---------------------------------
stateVector = "new" # "new"
# Orbital element set shared by the initial reference and truth satellites
orbitParam = [
    6928,   # SMA, avg alt of 500 km (6878, 6903)
    1e-3,   # ECC
    65,     # INC
    0,      # RAAN
    0,      # AOP
    0,      # TA
    dt.datetime.today() # Epoch
]

refOrbitParam = [
    6862.398123976534,
    0.001222655234762756,
    65.08465834964612,
    132.8799247164418,
    103.33767643107548,
    169.9596201609888,
    "14 Sep 2026 00:00:00.000"
]

truthOrbitParam = [
    6862.354299657057,
    0.0012109994471725667,
    65.08194601412482,
    132.8802573958094,
    105.53210688316561,
    167.6482406337395,
    "14 Sep 2026 00:00:00.000"
]

# Maximum thruster duty time in seconds
minDutyTime = 300
maxDutyTime = 3600

# 
maneuverArcHalfAngle = 20

# ----------------- Station Keeping Parameters --------------------------------
# Operational bounds (+/-) to keep the truth satellite within
R_bounds = 2
I_bounds = 20
C_bounds = 4

I_deadband_min = 0.85
# ----------------- Plotting --------------------------------------------------
plot_3D_RIC = True

plot_rRIC_v_Time = True

plot_rRIC_Amp_v_Time = True

plot_vRIC_v_Time = True

plot_vRIC_Amp_v_Time = True

plot_COE_diffs = {
    "del_a": True,
    "del_e": True,
    "del_i": True  ,
    "del_raan": True,
    "del_aop": True,
    "del_f": True
}

plot_True_Lat_diff = True

plot_Show_Firings = True

terminal_Completed_Firings = True