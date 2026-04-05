
from collections import deque
from createStationKeepingObjects import StationKeepingObjects
from load_gmat import *
from matplotlib import pyplot as plt
from xyz2ric import xyz2ric

import numpy as np

# -----------create variables--------------------------------------

mu = 398600  # Earth’s mu in km^3/s^2
dt = 60.0 # step every 60 secs
elapsed = 0.0

orbitParam = [
    6878,   # SMA, avg alt of 500 km
    1e-3,   # ECC
    65,     # INC
    0,      # RAAN
    0,      # AOP
    0       # TA
]

# -----------configuration preliminaries----------------------------

refObjs = StationKeepingObjects("reference")
refObjs.setSatCOEs(orbitParam)

truthObjs = StationKeepingObjects("truth")
truthObjs.setSatCOEs(orbitParam)
truthObjs.setManeuverable()

gmat.Initialize()

truthObjs.setBurnForces()

refObjs.preparePropInternal()
gator_ref = refObjs.prop_wrap["coast"].getIntegrator()
truthObjs.preparePropInternal()
gator_truth = truthObjs.prop_wrap["coast"].getIntegrator()
truth_Sat_wrapper = truthObjs.sat_wrap

t = [0]
truthOffset = [[0], [0], [0]]
offsetHistory = [[0], [0], [0]]
diffCOEs = {
    "del_a": [0],
    "del_e": [0],
    "del_i": [0],
    "del_raan": [0],
    "del_aop": [0],
    "del_f": [0]
}

diffCOEs_avg = {
    "del_a": [0],
    "del_e": [0],
    "del_i": [0],
    "del_raan": [0],
    "del_aop": [0],
    "del_f": [0]
}

fig = plt.figure()
ax_ric = fig.add_subplot(projection='3d')

maxDays = 7
revs_to_avg = 3
mean_motion = np.sqrt(mu / orbitParam[0]**3)
period_sec = 2 * np.pi / mean_motion
steps_per_rev = int(np.ceil(period_sec / dt))
steps_to_avg = int(revs_to_avg * steps_per_rev) + 5

R_bounds = 1
I_bounds = 15
C_bounds = 1.5

R_buffer = deque([0], maxlen=steps_per_rev)
I_buffer = deque([0], maxlen=steps_per_rev)
C_buffer = deque([0], maxlen=steps_per_rev)
recentRmaneuver = False
recentImaneuver = False
recentCmaneuver = False

R_amp_log = deque([0], maxlen=steps_per_rev)
C_amp_log = deque([0], maxlen=steps_per_rev)

counter = 0
del_a_target = 0
del_a_recovered_history = deque([0.125], maxlen=8)
targetSMA = 0

burn_duration = 0
maxDutyTime = 30 * 60
burnStarts = []
burnEnds = []

_diffCOEs_buffer = {
    "del_a": deque([0], maxlen=steps_to_avg),
    "del_e": deque([0], maxlen=steps_to_avg),
    "del_i": deque([0], maxlen=steps_to_avg),
    "del_raan": deque([0], maxlen=steps_to_avg),
    "del_aop": deque([0], maxlen=steps_to_avg),
    "del_f": deque([0], maxlen=steps_to_avg)
}

state = "nominal"
thrusterAxis = ""
count = 0
stateOld = []
stateNew = []
rotMatrix = []
gator_truth, fm_truth = truthObjs.satEnginesOff()
while elapsed < maxDays * 86400:
    
    elapsed = elapsed + dt
    gator_ref.Step(dt)
    gator_ref.UpdateSpaceObject()
    rv_ref = gator_ref.GetState()

    gator_truth.Step(dt)
    gator_truth.UpdateSpaceObject()
    rv_truth = gator_truth.GetState()

    if dt != 60 and (state != "I burn" and state != "C burn"):
        dt = 60

    rRIC, vRIC, rotMatrix = xyz2ric(rv_ref, rv_truth)

    R_buffer.append(rRIC[0])
    I_buffer.append(rRIC[1])
    C_buffer.append(rRIC[2])

    R_amp = (max(R_buffer) - min(R_buffer)) / 2 if len(R_buffer) > 1 else rRIC[0]
    C_amp = (max(C_buffer) - min(C_buffer)) / 2 if len(C_buffer) > 1 else rRIC[2]
    R_amp_log.append(R_amp)
    C_amp_log.append(C_amp)

    truthOffset[0].append(rRIC[0])
    truthOffset[1].append(rRIC[1])
    truthOffset[2].append(rRIC[2])
    offsetHistory[0].append(rRIC[0])
    offsetHistory[1].append(rRIC[1])
    offsetHistory[2].append(rRIC[2])
    
    t.append(elapsed)
    refCOE = refObjs.sat_wrap.getKeplerianState()
    truthCOE = truthObjs.sat_wrap.getKeplerianState()
    coes = list(diffCOEs.keys())
    for k in range(6):
        diff = truthCOE[k] - refCOE[k]

        if k > 1 and diff > 180:
            diff = diff - 360
        elif k > 1 and diff < -180:
            diff = diff + 360
        diffCOEs[coes[k]].append(diff)
        _diffCOEs_buffer[coes[k]].append(diff)
        diffCOEs_avg[coes[k]].append(np.mean(_diffCOEs_buffer[coes[k]]))

    sma_truth = truthObjs.sat_wrap.getSMAFromEnergy()
    sma_ref = refObjs.sat_wrap.getSMAFromEnergy()
    del_a_energy = sma_truth - sma_ref


    match state:
        case "nominal":    
            if rRIC[1] > I_bounds : # np.mean(I_buffer) > I_bounds :
                state = "wait for I burn"
            elif C_amp > C_bounds :
                state = "wait for C burn"
            elif R_amp > R_bounds :
                state = "wait for R burn"
        case "wait for I burn":
            f = truthCOE[-1]
            if not(10 < f <= 170) and not(190 < f <= 350) and diffCOEs_avg["del_a"][-1] < 0:
                del_a_recovered_history.append(abs(diffCOEs_avg["del_a"][-1]))
                ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'b')
                truthOffset = [[truthOffset[0][-1]], [truthOffset[1][-1]], [truthOffset[2][-1]]]   
                
                state = "I burn"
                thrusterAxis = "I+"
                burnStarts.append(len(t))
                gator_truth, fm_truth = truthObjs.satEnginesOn(thrusterAxis)

                historical_del_a_avg = np.mean(del_a_recovered_history)

                del_a_target = -0.75 * del_a_energy
                dt = 5
                # I_buffer.clear()
                # I_buffer.append(rRIC[1])
        case "wait for C burn": 
            velo_phase = np.arctan2(vRIC[2], vRIC[1])
            in_node_window = abs(abs(velo_phase) - np.pi /2) < np.deg2rad(30)
            
            # true_lat = (truthCOE[4] + truthCOE[5]) % 360
            # in_node_window = (70 <= true_lat <= 110) or (250 <= true_lat <= 290)
            if in_node_window:
                thrusterAxis = "C+" if vRIC[2] < 0 else "C-"
                gator_truth, fm_truth = truthObjs.satEnginesOn(thrusterAxis)
                state = "C burn"
                burn_duration = 0.0
                total_burn_this_rev = 0.0
                ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'b')
                truthOffset = [[truthOffset[0][-1]], [truthOffset[1][-1]], [truthOffset[2][-1]]]   
                dt = 5
                C_buffer.clear()
                C_buffer.append(rRIC[2])

        case "wait for R burn":
            velo_phase = np.arctan2(vRIC[0], vRIC[1])
            in_node_window = abs(abs(velo_phase) - np.pi /2) < np.deg2rad(15)
            temp = abs(abs(velo_phase) - np.pi /2) 
            # true_lat = (truthCOE[4] + truthCOE[5]) % 360
            # in_node_window = (70 <= true_lat <= 110) or (250 <= true_lat <= 290)
            if in_node_window:
                thrusterAxis = "R+" if vRIC[2] < 0 else "R-"
                gator_truth, fm_truth = truthObjs.satEnginesOn(thrusterAxis)
                state = "R burn"
                burn_duration = 0.0
                total_burn_this_rev = 0.0
                ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'b')
                truthOffset = [[truthOffset[0][-1]], [truthOffset[1][-1]], [truthOffset[2][-1]]]    
                dt = 5
                R_buffer.clear()
                R_buffer.append(rRIC[0])

        case "I burn":
            burn_duration += dt

            del_a_recovered = del_a_energy >= del_a_target
            if del_a_recovered or burn_duration >= maxDutyTime:
                
                print(f"Complete {thrusterAxis} burn duration (min) = {(burn_duration / 60):2.0f} | t = {(elapsed / 86400):2.2f} days | Recovered del_a = {diffCOEs['del_a'][-1]:0.5f} out of {del_a_target:0.5f}")
                state = "returning to nominal from I burn"
                burnEnds.append(len(t))
                gator_truth, fm_truth = truthObjs.satEnginesOff(thrusterAxis)
                thrusterAxis = ""

                ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'r')
                truthOffset = [[truthOffset[0][-1]], [truthOffset[1][-1]], [truthOffset[2][-1]]]
                
                burn_duration = 0
                dt = 60 - elapsed % 60

        case "C burn":
            burn_duration += dt
            velo_phase = np.arctan2(vRIC[2], vRIC[1])
            in_cross_track_pass = abs(abs(velo_phase) - np.pi /2) < np.deg2rad(40)
            # true_lat = (truthCOE[4] + truthCOE[5]) % 360
            # in_cross_track_pass = (70 <= true_lat <= 110) or (250 <= true_lat <= 290)
            
            if burn_duration >= maxDutyTime or not in_cross_track_pass:
                # print(f"Complete {thrusterAxis} burn duration (min) = {(burn_duration / 60):2.0f} | t = {(elapsed / 86400):2.2f} days | C-axis Amplitude = {C_amp:0.6f} | Velo phase angle = {(np.rad2deg(velo_phase)):3.2f}")
                gator_truth, fm_truth = truthObjs.satEnginesOff(thrusterAxis)
                state = "returning to nominal from C burn"
                recentCmaneuver = False if C_amp > 1 / 3 * C_bounds else True
                burnEnds.append(len(t))
                ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'g')
                truthOffset = [[truthOffset[0][-1]], [truthOffset[1][-1]], [truthOffset[2][-1]]]  
                dt = 60 - elapsed % 60
        
        case "R burn":
            burn_duration += dt
            velo_phase = np.arctan2(vRIC[0], vRIC[1])
            in_cross_track_pass = abs(abs(velo_phase) - np.pi /2) < np.deg2rad(20)
            temp = np.rad2deg(abs(abs(velo_phase) - np.pi /2))
            # true_lat = (truthCOE[4] + truthCOE[5]) % 360
            # in_cross_track_pass = (70 <= true_lat <= 110) or (250 <= true_lat <= 290)
            
            if burn_duration >= maxDutyTime or not in_cross_track_pass:
                print(f"Complete {thrusterAxis} burn duration (min) = {(burn_duration / 60):2.0f} | t = {(elapsed / 86400):2.2f} days | R-axis Amplitude = {R_amp:0.6f} | Velo phase angle = {(np.rad2deg(velo_phase)):3.2f}")
                gator_truth, fm_truth = truthObjs.satEnginesOff(thrusterAxis)
                state = "nominal" # "returning to nominal from R burn"
                recentCmaneuver = False if R_amp > 1 / 3 * R_bounds else True
                burnEnds.append(len(t))
                ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'k')
                truthOffset = [[truthOffset[0][-1]], [truthOffset[1][-1]], [truthOffset[2][-1]]]  
                dt = 60 - elapsed % 60
    
        case "returning to nominal from I burn":

            del_a_recovered = del_a_energy >= del_a_target
            if rRIC[1] < 0.6 * I_bounds and del_a_recovered:
                state = "nominal"
                recentImaneuver = True
            elif (elapsed - t[burnEnds[-1]] > 3 * period_sec):
                state = "wait for I burn"
            else:
                continue
        case "returning to nominal from C burn":
            C_amp_recovering = C_amp_log[-1] <= C_amp_log[-2]
            if C_amp <= 1 /3 * C_bounds:
                state = "nominal"
            elif not C_amp_recovering or (elapsed - t[burnEnds[-1]] > period_sec):

                if rRIC[1] > I_bounds:
                    historical_del_a_avg = np.mean(del_a_recovered_history)
                    del_a_target = del_a_target = -0.6 * del_a_energy # 1.1 * historical_del_a_avg # abs(diffCOEs_avg["del_a"][-1]) # historical_del_a_avg 
                    state = "wait for I burn"
                else:
                    state = "wait for C burn"
            else:
                continue
        case "returning to nominal from R burn":
            R_amp_recovering = R_amp_log[-1] <= R_amp_log[-2]
            
            if R_amp <= 1 /3 * R_bounds:
                state = "nominal"
            elif not R_amp_recovering or (elapsed - t[burnEnds[-1]] > period_sec):
                break
                if rRIC[1] > I_bounds:
                    historical_del_a_avg = np.mean(del_a_recovered_history)
                    del_a_target = del_a_target = -0.6 * del_a_energy # 1.1 * historical_del_a_avg # abs(diffCOEs_avg["del_a"][-1]) # historical_del_a_avg 
                    state = "wait for I burn"
                else:
                    state = "wait for R burn"
            else:
                continue

if state == "raising":
    ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'r')
else:
    ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'b')
ax_ric.set_xlabel('R (km)')
ax_ric.set_ylabel('I (km)')
ax_ric.set_zlabel('C (km)')
ax_ric.set_title('3D Trajectory of Earth Orbiter')
# ax_ric.legend()
ax_ric.axis('equal')

t = np.array(t)
t /= 86400
ax = plt.figure().add_subplot()
ax.plot(t, diffCOEs["del_a"], label="del_a")
ax.plot(t, diffCOEs_avg["del_a"], "--", label=f"{revs_to_avg} orbit average")
#ax.plot([t[i] for i in burnStarts], [diffCOEs_avg["del_a"][i] for i in burnStarts], "*", c="r",label="Maneuvers")
ax.set_xlabel('Time (Days)')
ax.set_ylabel('Offset (km)')
ax.legend()
"""

ax = plt.figure().add_subplot()
ax.plot(t, diffCOEs["del_e"], label="del_e")
ax.plot(t, diffCOEs_avg["del_e"], "--", label=f"{revs_to_avg} orbit average")
#ax.plot([t[i] for i in burnStarts], [diffCOEs_avg["del_e"][i] for i in burnStarts], "*", c="r",label="Maneuvers")
ax.set_xlabel('Time (Days)')
ax.set_ylabel('Offset')
ax.legend()

ax = plt.figure().add_subplot()
ax.plot(t, diffCOEs["del_f"], label="del_f")
ax.plot(t, diffCOEs_avg["del_f"], "--", label=f"{revs_to_avg} orbit average")
#ax.plot([t[i] for i in burnStarts], [diffCOEs_avg["del_f"][i] for i in burnStarts], "*", c="r",label="Maneuvers")
ax.set_xlabel('Time (Days)')
ax.set_ylabel('Offset (deg)')
ax.legend()"""

ax = plt.figure().add_subplot()
ax.plot(t, diffCOEs["del_raan"], label="del_raan")
ax.plot(t, diffCOEs_avg["del_raan"], "--", label=f"{revs_to_avg} orbit average")
#ax.plot([t[i] for i in burnStarts], [diffCOEs_avg["del_raan"][i] for i in burnStarts], "*", c="r",label="Maneuvers")
ax.set_xlabel('Time (Days)')
ax.set_ylabel('Offset (deg)')
ax.legend()

ax = plt.figure().add_subplot()
ax.plot(t, diffCOEs["del_i"], label="del_i")
ax.plot(t, diffCOEs_avg["del_i"], "--", label=f"{revs_to_avg} orbit average")
#ax.plot([t[i] for i in burnStarts], [diffCOEs_avg["del_i"][i] for i in burnStarts], "*", c="r",label="Maneuvers")
ax.set_xlabel('Time (Days)')
ax.set_ylabel('Offset (deg)')
ax.legend()


ax = plt.figure().add_subplot()
ax.plot(t, offsetHistory[0], label="R")
ax.plot(t, offsetHistory[1], label="I")
ax.plot(t, offsetHistory[2], label="C")
#ax.plot([t[i] for i in burnStarts], [offsetHistory[1][i] for i in burnStarts], "*", c="r",label="Maneuvers")
ax.set_xlabel('Time (Days)')
ax.set_ylabel('Offset (km)')
ax.legend()

plt.show()
