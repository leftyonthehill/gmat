
from collections import deque
from createForceModel import ForceModel
from createPropagator import Propagator
from createSatellite import Satellite
from createStationKeepingObjects import StationKeepingObjects
# from createThruster import createThruster
from load_gmat import *
from matplotlib import pyplot as plt
from xyz2ric import xyz2ric

# import datetime as dt
import numpy as np

# -----------create variables--------------------------------------

problemDim = 3
numRandSys = 10
mu = 398600  # Earth’s mu in km^3/s^2
R = 6371 # radius of earth in km
numMinProp = 60 * 24 # take a step 60 times in an hour and for 24 hours
numMinProp = 80 # take a step 60 times in an hour and for 24 hours
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
def enginesOn(axis: str):
    prop = truthObjs.prop_wrap[axis]
    fm = truthObjs.fm_wrap[axis]

    prop.prepareInternals()

    thr_name = truth_Sat_wrapper.thrusters[axis].GetName()
    thruster = truth_Sat_wrapper.getSat().GetRefObject(gmat.THRUSTER, thr_name)
    thruster.SetField("IsFiring", True)
    truth_Sat_wrapper.getSat().IsManeuvering(True)

    prop.getPropagator().AddForce(fm.getBurnForce(axis))
    prop.getPropagator().AddPropObject(truth_Sat_wrapper.getSat())

    prop.prepareInternals()
    gator = prop.getIntegrator()
    fm = truthObjs.fm_wrap[axis].getFM()
    return gator, fm

def enginesOff(axis: str = ""):
    if axis == "":
        gator = truthObjs.prop_wrap["coast"].getIntegrator()
        fm = truthObjs.fm_wrap["coast"].getFM()
        return gator, fm
    
    prop = truthObjs.prop_wrap["coast"]
    fm = truthObjs.fm_wrap["coast"]

    prop.prepareInternals()
    
    thr_name = truthObjs.sat_wrap.thrusters[axis].GetName()
    thruster = truthObjs.sat_wrap.getSat().GetRefObject(gmat.THRUSTER, thr_name)
    thruster.SetField("IsFiring", False)
    truth_Sat_wrapper.getSat().IsManeuvering(False)
    prop.getPropagator().AddPropObject(truth_Sat_wrapper.getSat())

    prop.prepareInternals()
    gator = prop.getIntegrator()
    fm = truthObjs.fm_wrap[axis].getFM()
    return gator, fm

    

statesArrayElectric = np.zeros((numRandSys,numMinProp,problemDim))
statesArrayElectric2 = np.zeros((numRandSys,numMinProp,problemDim))
statesArrayCoast = np.zeros((numRandSys,numMinProp,problemDim))

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

maxDays = 4
revs_to_avg = 3
mean_motion = np.sqrt(mu / orbitParam[0]**3)
period_sec = 2 * np.pi / mean_motion
steps_per_rev = int(np.ceil(period_sec / dt))
steps_to_avg = int(revs_to_avg * steps_per_rev) + 5

R_bounds = 3
I_bounds = 15
C_bounds = 1.5

R_buffer = deque([0], maxlen=steps_per_rev)
I_buffer = deque([0], maxlen=steps_per_rev)
C_buffer = deque([0], maxlen=steps_per_rev)

counter = 0
del_a_to_return = 0
del_a_recovered_history = deque([0.13], maxlen=8)

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
gator_truth, fm_truth = enginesOff()
while elapsed < maxDays * 86400:
    
    elapsed = elapsed + dt
    gator_ref.Step(dt)
    gator_ref.UpdateSpaceObject()
    rv_ref = gator_ref.GetState()

    gator_truth.Step(dt)
    gator_truth.UpdateSpaceObject()
    rv_truth = gator_truth.GetState()

    rRIC, vRIC, rotMatrix = xyz2ric(rv_ref, rv_truth)

    R_buffer.append(rRIC[0])
    I_buffer.append(rRIC[1])
    C_buffer.append(rRIC[2])

    R_amp = (max(R_buffer) - min(R_buffer)) / 2 if len(R_buffer) > 1 else rRIC[0]
    C_amp = (max(C_buffer) - min(C_buffer)) / 2 if len(C_buffer) > 1 else rRIC[2]


    truthOffset[0].append(rRIC[0])
    truthOffset[1].append(rRIC[1])
    truthOffset[2].append(rRIC[2])
    offsetHistory[0].append(rRIC[0])
    offsetHistory[1].append(rRIC[1])
    offsetHistory[2].append(rRIC[2])
    
    t.append(elapsed)
    refCOE = refObjs.sat_wrap.getKeplerianState()
    truthCOE = truth_Sat_wrapper.getKeplerianState()
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
    
    match state:
        case "nominal":    
            if abs(rRIC[1]) > I_bounds:
                historical_del_a_avg = np.mean(del_a_recovered_history)
                del_a_to_return = historical_del_a_avg
                state = "wait for I burn"
            elif C_amp > C_bounds:
                state = "wait for C burn"
            elif R_amp > R_bounds:
                print(f"R ({rRIC[0]:0.3}/{R_bounds}) or C({rRIC[2]:0.3}/{C_bounds}) were violated")
                break  
        case "wait for I burn":
            if not(10 < truthCOE[-1] <= 170) and not(190 < truthCOE[-1] <= 350):
                del_a_recovered_history.append(abs(diffCOEs_avg["del_a"][-1]))

                state = "I burn"
                thrusterAxis = "I+"
                burnStarts.append(len(t))
                gator_truth, fm_truth = enginesOn(thrusterAxis)
                
                ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'b')
                truthOffset = [[truthOffset[0][-1]], [truthOffset[1][-1]], [truthOffset[2][-1]]]
                r = np.linalg.norm(rv_truth[:3])
                accelActual = fm_truth.GetDerivativesForSpacecraft(truth_Sat_wrapper.getSat())
                accel = -mu / r**3 * np.array(rv_truth[:3])
                a_diff = np.linalg.norm(accelActual[3:] - accel)   
        case "wait for C burn":
            true_lat = (truthCOE[4] + truthCOE[5]) % 360
            ture_lat_trigger = np.arctan(diffCOEs_avg["del_raan"][-1] / diffCOEs_avg["del_i"][-1] * truthCOE[2]) / np.pi * 180
            state = "C burn"
            if (90 <= true_lat < 270):
                thrusterAxis = "C+"
                gator_truth, fm_truth = enginesOn(thrusterAxis)
            else:
                thrusterAxis = "C-"
                gator_truth, fm_truth = enginesOn(thrusterAxis)
            """if (130 < true_lat < 2200) or ((310 < true_lat < 360) or (0 < true_lat < 40)):
                # print(ture_lat_trigger, true_lat, ture_lat_trigger+180)
                state = "C burn"
                burn_duration = 0
                ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'b')
                truthOffset = [[truthOffset[0][-1]], [truthOffset[1][-1]], [truthOffset[2][-1]]]
                if (130 < true_lat < 2200):
                    thrusterAxis = "C+"
                    gator_truth, fm_truth = enginesOn(thrusterAxis)
                elif (310 < true_lat < 360) or (0 < true_lat < 40):
                    thrusterAxis = "C-"
                    gator_truth, fm_truth = enginesOn(thrusterAxis)"""
                
        case "I burn":
            burn_duration += dt
            r = np.linalg.norm(rv_truth[:3])
            accelActual = fm_truth.GetDerivativesForSpacecraft(truth_Sat_wrapper.getSat())
            thrust_inertial = np.array(accelActual[3:]) + (mu / r**3) * np.array(rv_truth[:3])
            thrust_ric = rotMatrix @ thrust_inertial

            if diffCOEs_avg["del_a"][-1] >= del_a_to_return or burn_duration >= maxDutyTime:
                print(f"Complete {thrusterAxis} burn duration (min) = {(burn_duration / 60):2.0f} | t = {(elapsed / 86400):2.2f} days | Recovered del_a = {del_a_to_return:0.5f}")
                state = "returning to nominal from I burn"
                burnEnds.append(len(t))
                gator_truth, fm_truth = enginesOff(thrusterAxis)
                thrusterAxis = ""

                ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'r')
                truthOffset = [[truthOffset[0][-1]], [truthOffset[1][-1]], [truthOffset[2][-1]]]

                burn_duration = 0
        case "C burn":
            burn_duration += dt
            true_lat = (truthCOE[4] + truthCOE[5]) % 360

            maxBurnTime = (burn_duration >= maxDutyTime)
            del_raan_zeroed = diffCOEs_avg["del_raan"][-1] < 0
            maxBurnWindow = (10 <= true_lat < 170) or (190 <= true_lat < 350)
            if  maxBurnTime or del_raan_zeroed:
                
                state = "wait for C burn"
                print(f"Complete {thrusterAxis} burn duration (min) = {(burn_duration / 60):2.0f} | t = {(elapsed / 86400):2.2f} days | C-axis Amplitude = {C_amp:0.6f} | True latitude = {true_lat:3.2f}")
                
                if thrusterAxis == "C+":
                    gator_truth, fm_truth = enginesOff(thrusterAxis)
                    thrusterAxis = ""
                elif thrusterAxis == "C-":
                    gator_truth, fm_truth = enginesOff(thrusterAxis)
                    thrusterAxis = ""
                
                burnEnds.append(len(t))
                ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'g')
                truthOffset = [[truthOffset[0][-1]], [truthOffset[1][-1]], [truthOffset[2][-1]]]

                burn_duration = 0

                if diffCOEs_avg["del_raan"][-1] < 0:
                    state = "nominal"
                
        case "returning to nominal from I burn":
            # print("Returning:", rRIC)
            if diffCOEs["del_a"][-1] <= 0:
                state = "nominal"
        # case "returning to nominal from C burn":

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
