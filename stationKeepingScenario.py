
from collections import deque
from createForceModel import ForceModel
from createPropagator import Propagator
from createSatellite import Satellite
# from createThruster import createThruster
from load_gmat import *
from matplotlib import pyplot as plt
from xyz2ric import xyz2ric

import datetime as dt
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

# Spacecraft
ref_Sat_wrapper = Satellite("refSat")
ref_Sat_wrapper.setOrbitElements(orbitParam)

truth_Sat_wrapper = Satellite("truthSat")
truth_Sat_wrapper.setOrbitElements(orbitParam)
truth_Sat_wrapper.setManeuverable()

# Force model settings
ref_FM_wrapper = ForceModel("reference")
ref_FM_wrapper.setDynamics("reference")

truth_FM_wrapper = ForceModel("truth")
truth_FM_wrapper.setDynamics("truth")

truth_and_thrust_FM_wrapper = ForceModel("truth_and_thrust")
truth_and_thrust_FM_wrapper.setDynamics("truth")

# Initialize propagator object
ref_Prop_wrapper = Propagator("reference")
ref_Prop_wrapper.setIntegrator("reference")
ref_Prop_wrapper.setFM(ref_FM_wrapper.getFM())
ref_Prop_wrapper.setSat(ref_Sat_wrapper.getSat())

truth_Prop_wrapper = Propagator("truth")
truth_Prop_wrapper.setIntegrator("truth")
truth_Prop_wrapper.setFM(truth_FM_wrapper.getFM())
truth_Prop_wrapper.setSat(truth_Sat_wrapper.getSat())

truth_and_thrust_Prop_wrapper = Propagator("truth_and_thrust")
truth_and_thrust_Prop_wrapper.setIntegrator("truth")
truth_and_thrust_Prop_wrapper.setSat(truth_Sat_wrapper.getSat())

gmat.Initialize()

truth_and_thrust_FM_wrapper.setThrust(truth_Sat_wrapper)
truth_and_thrust_Prop_wrapper.setFM(truth_and_thrust_FM_wrapper.getFM())

ref_Prop_wrapper.prepareInternals()
gator_ref = ref_Prop_wrapper.getIntegrator()

truth_Prop_wrapper.prepareInternals()
truth_and_thrust_Prop_wrapper.prepareInternals()

def enginesOn():
    thruster = truth_Sat_wrapper.getSat().GetRefObject(gmat.THRUSTER, truth_Sat_wrapper.ethruster.GetName())
    thruster.SetField("IsFiring", True)
    truth_Sat_wrapper.getSat().IsManeuvering(True)
    
    truth_and_thrust_Prop_wrapper.getPropagator().AddForce(truth_and_thrust_FM_wrapper.getBurnForce())
    truth_and_thrust_Prop_wrapper.getPropagator().AddPropObject(truth_Sat_wrapper.getSat())
    
    truth_and_thrust_Prop_wrapper.getPropagator().PrepareInternals()
    gator = truth_and_thrust_Prop_wrapper.getIntegrator()
    fm = truth_and_thrust_Prop_wrapper.getPropagator().GetODEModel()
    return gator, fm

def enginesOff():
    truth_Sat_wrapper.ethruster.SetField("IsFiring", False)
    truth_Sat_wrapper.sat.IsManeuvering(False)
    truth_Prop_wrapper.prop.AddPropObject(truth_Sat_wrapper.getSat())
    
    truth_Prop_wrapper.getPropagator().PrepareInternals()
    gator = truth_Prop_wrapper.getIntegrator()
    fm = truth_Prop_wrapper.getPropagator().GetODEModel()
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

maxDays = 60
revs_to_avg = 3
mean_motion = np.sqrt(mu / orbitParam[0]**3)
period_sec = 2 * np.pi / mean_motion
steps_per_rev = int(revs_to_avg * np.ceil(period_sec / dt)) + 5

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
    "del_a": deque([0], maxlen=steps_per_rev),
    "del_e": deque([0], maxlen=steps_per_rev),
    "del_i": deque([0], maxlen=steps_per_rev),
    "del_raan": deque([0], maxlen=steps_per_rev),
    "del_aop": deque([0], maxlen=steps_per_rev),
    "del_f": deque([0], maxlen=steps_per_rev)
}

state = "nominal"
gator_truth, fm_truth = enginesOff()
while elapsed < maxDays * 86400:
    """if state == "I burn":
        rv_truth = gator_truth.GetState()
        r = np.linalg.norm(rv_truth[:3])
        accelActual = fm_truth.GetDerivativesForSpacecraft(truth_Sat_wrapper.getSat())
        accel = -mu / r**3 * np.array(rv_truth[:3])
        a_diff = np.linalg.norm(accelActual[3:] - accel)

        temp1 = rotMatrix @ accel
        temp2 = rotMatrix @ accelActual[3:]
        
        print(f"Observed Two-body acceleration: [{temp2[0]:0.5e}, {temp2[2]:0.5e}, {temp2[2]:0.5e}]")
        exit(1) """

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
    C_amp = (max(C_buffer) - min(C_buffer)) / 2 if len(C_buffer) > 1 else rRIC[2]


    truthOffset[0].append(rRIC[0])
    truthOffset[1].append(rRIC[1])
    truthOffset[2].append(rRIC[2])
    offsetHistory[0].append(rRIC[0])
    offsetHistory[1].append(rRIC[1])
    offsetHistory[2].append(rRIC[2])
    
    t.append(elapsed)
    refCOE = ref_Sat_wrapper.getKeplerianState()
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
                del_a_to_return = 1.1 * historical_del_a_avg
                state = "wait for I burn"
            elif C_amp > C_bounds:
                state = "wait for C burn"
            elif abs(rRIC[0]) > R_bounds:
                print(f"R ({rRIC[0]:0.3}/{R_bounds}) or C({rRIC[2]:0.3}/{C_bounds}) were violated")
                break  
        case "wait for I burn":
            if not(10 < truthCOE[-1] <= 170) and not(190 < truthCOE[-1] <= 350):
                del_a_recovered_history.append(abs(diffCOEs_avg["del_a"][-1]))

                state = "I burn"
                burnStarts.append(len(t))
                gator_truth, fm_truth = enginesOn()
                ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'b')
                truthOffset = [[truthOffset[0][-1]], [truthOffset[1][-1]], [truthOffset[2][-1]]]
                r = np.linalg.norm(rv_truth[:3])
                accelActual = fm_truth.GetDerivativesForSpacecraft(truth_Sat_wrapper.getSat())
                accel = -mu / r**3 * np.array(rv_truth[:3])
                a_diff = np.linalg.norm(accelActual[3:] - accel)

                temp1 = rotMatrix @ accel
                temp2 = rotMatrix @ accelActual[3:]
                
                # print(f"Observed Two-body acceleration: [{temp2[0]:0.5e}, {temp2[2]:0.5e}, {temp2[2]:0.5e}]")
    
        case "wait for C burn":
            print("<Pre-engine direction change>")
            r = np.linalg.norm(rv_truth[:3])
            accelActual = fm_truth.GetDerivativesForSpacecraft(truth_Sat_wrapper.getSat())
            accel = -mu / r**3 * np.array(rv_truth[:3])
            a_diff = np.linalg.norm(accelActual[3:] - accel)

            temp1 = rotMatrix @ accel
            temp2 = rotMatrix @ accelActual[3:]
            
            print(f"Observed Two-body acceleration: [{temp2[0]:0.5e}, {temp2[2]:0.5e}, {temp2[2]:0.5e}]")
            
            truth_Sat_wrapper.setEThrusterDirection([1e-5, 1, 1e-5])
            truth_Sat_wrapper.setManeuverable()
            truth_and_thrust_FM_wrapper.setThrust(truth_Sat_wrapper)
            truth_and_thrust_Prop_wrapper.setFM(truth_and_thrust_FM_wrapper.getFM())
            
            gator_truth, fm_truth = enginesOn()
            state = "C burn"
            burn_duration = 0
        case "I burn":
            burn_duration += dt
            if diffCOEs["del_a"][-1] >= del_a_to_return or burn_duration >= maxDutyTime:
                state = "returning to nominal from I burn"
                burnEnds.append(len(t))
                gator_truth, fm_truth = enginesOff()
                ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'r')
                truthOffset = [[truthOffset[0][-1]], [truthOffset[1][-1]], [truthOffset[2][-1]]]
                burn_duration /= 60
                print(f"Complete burn duration (min) = {burn_duration:2.1f} | t = {(elapsed / 86400):0.2f} days | Recovered del_a = {del_a_to_return:0.5f} | C-axis Amplitude = {C_amp:0.4f}")
                burn_duration = 0
        case "C burn":
            burn_duration += dt
            if burn_duration >= maxDutyTime:
                break
            r = np.linalg.norm(rv_truth[:3])
            accelActual = fm_truth.GetDerivativesForSpacecraft(truth_Sat_wrapper.getSat())
            accel = -mu / r**3 * np.array(rv_truth[:3])
            a_diff = np.linalg.norm(accelActual[3:] - accel)

            temp1 = rotMatrix @ accel
            temp2 = rotMatrix @ accelActual[3:]
            # print("<Post-engine direction change>")
            # print(f"Observed Two-body acceleration: [{temp2[0]:0.5e}, {temp2[2]:0.5e}, {temp2[2]:0.5e}]")
            # break # exit(1)
        case "returning to nominal from I burn":
            if diffCOEs["del_a"][-1] <= 0:
                state = "nominal"



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

ax = plt.figure().add_subplot()
t = np.array(t)
t /= 86400
ax.plot(t, diffCOEs["del_a"], label="del_a")
ax.plot(t, diffCOEs_avg["del_a"], "--", label=f"{revs_to_avg} orbit average")
#ax.plot([t[i] for i in burnStarts], [diffCOEs_avg["del_a"][i] for i in burnStarts], "*", c="r",label="Maneuvers")
ax.set_xlabel('Time (Days)')
ax.set_ylabel('Offset (km)')
ax.legend()

"""ax = plt.figure().add_subplot()
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
ax.plot(t, diffCOEs["del_aop"], label="del_aop")
ax.plot(t, diffCOEs_avg["del_aop"], "--", label=f"{revs_to_avg} orbit average")
#ax.plot([t[i] for i in burnStarts], [diffCOEs_avg["del_aop"][i] for i in burnStarts], "*", c="r",label="Maneuvers")
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
