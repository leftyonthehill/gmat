
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

fig = plt.figure()
ax_ric = fig.add_subplot(projection='3d')

maxDays = 31
R_bounds = .5
I_bounds = 15
C_bounds = .5

del_a_to_return = 0

state = "nominal"
gator_truth, fm_truth = enginesOff()
while elapsed < maxDays * 86400:
    
    match state:
        case "nominal":
            elapsed = elapsed + dt
            gator_ref.Step(dt)
            gator_ref.UpdateSpaceObject()
            rv_ref = gator_ref.GetState()

            gator_truth.Step(dt)
            gator_truth.UpdateSpaceObject()
            rv_truth = gator_truth.GetState()

            rRIC, vRIC, rotMatrix = xyz2ric(rv_ref, rv_truth)

            # statesArrayCoast[i,j,:] = rRIC # state_truth[0:6]
            truthOffset[0].append(rRIC[0])
            truthOffset[1].append(rRIC[1])
            truthOffset[2].append(rRIC[2])
            offsetHistory[0].append(rRIC[0])
            offsetHistory[1].append(rRIC[1])
            offsetHistory[2].append(rRIC[2])

            r = np.linalg.norm(rv_truth[:3])
            accelActual = fm_truth.GetDerivativesForSpacecraft(truth_Sat_wrapper.getSat())
            accel = -mu / r**3 * np.array(rv_truth[:3])
            a_diff = np.linalg.norm(accelActual[3:] - accel)

            temp1 = rotMatrix @ accel
            temp2 = rotMatrix @ accelActual[3:]
            # print(f"Diff: {a_diff} | Computed Two-body acceleration: {np.linalg.norm(temp1)} | Observed Two-body acceleration: {np.linalg.norm(temp2)}")
            # print(f"Step {j:2d} | diff = {a_diff:.10f} km/s²  | "
            #     f"IsFiring         = {truth_Sat_wrapper.ethruster.GetField('IsFiring')}  | ")
            
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
            
            if abs(rRIC[1]) > I_bounds:
                # state = "wait to raise"
                del_a_to_return = -0.98 * diffCOEs["del_a"][-1]
                state = "wait to raise"
                # gator_truth, fm_truth = enginesOn()
                # ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'b')
                # truthOffset = [[truthOffset[0][-1]], [truthOffset[1][-1]], [truthOffset[2][-1]]]
            elif abs(rRIC[0]) > R_bounds:
                break
            elif abs(rRIC[2]) > C_bounds:
                break
            
        case "wait to raise":
            elapsed = elapsed + dt
            gator_ref.Step(dt)
            gator_ref.UpdateSpaceObject()
            rv_ref = gator_ref.GetState()

            gator_truth.Step(dt)
            gator_truth.UpdateSpaceObject()
            rv_truth = gator_truth.GetState()

            rRIC, vRIC, rotMatrix = xyz2ric(rv_ref, rv_truth)

            # statesArrayCoast[i,j,:] = rRIC # state_truth[0:6]
            truthOffset[0].append(rRIC[0])
            truthOffset[1].append(rRIC[1])
            truthOffset[2].append(rRIC[2])
            offsetHistory[0].append(rRIC[0])
            offsetHistory[1].append(rRIC[1])
            offsetHistory[2].append(rRIC[2])

            r = np.linalg.norm(rv_truth[:3])
            accelActual = fm_truth.GetDerivativesForSpacecraft(truth_Sat_wrapper.getSat())
            accel = -mu / r**3 * np.array(rv_truth[:3])
            a_diff = np.linalg.norm(accelActual[3:] - accel)

            temp1 = rotMatrix @ accel
            temp2 = rotMatrix @ accelActual[3:]
            # print(f"Diff: {a_diff} | Computed Two-body acceleration: {np.linalg.norm(temp1)} | Observed Two-body acceleration: {np.linalg.norm(temp2)}")
            # print(f"Step {j:2d} | diff = {a_diff:.10f} km/s²  | "
            #     f"IsFiring         = {truth_Sat_wrapper.ethruster.GetField('IsFiring')}  | ")
            
            t.append(elapsed)
            refCOE = ref_Sat_wrapper.getKeplerianState()
            truthCOE = truth_Sat_wrapper.getKeplerianState()
            coes = list(diffCOEs.keys())
            # print(f"Waiting to Maneuver | T+ {elapsed} sec | TA = {truthCOE[-1]}")
            for k in range(6):
                diff = truthCOE[k] - refCOE[k]

                if k > 1 and diff > 180:
                    diff = diff - 360
                elif k > 1 and diff < -180:
                    diff = diff + 360
                diffCOEs[coes[k]].append(diff)

            if not(10 < truthCOE[-1] <= 170) and not(190 < truthCOE[-1] <= 350):
                state = "raising"
                gator_truth, fm_truth = enginesOn()
                ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'b')
                truthOffset = [[truthOffset[0][-1]], [truthOffset[1][-1]], [truthOffset[2][-1]]]
        case "raising":
            # gator_truth, fm_truth = enginesOn()
            elapsed = elapsed + dt
            gator_ref.Step(dt)
            gator_ref.UpdateSpaceObject()
            rv_ref = gator_ref.GetState()

            gator_truth.Step(dt)
            gator_truth.UpdateSpaceObject()
            rv_truth = gator_truth.GetState()

            rRIC, vRIC, rotMatrix = xyz2ric(rv_ref, rv_truth)

            # statesArrayCoast[i,j,:] = rRIC # state_truth[0:6]
            truthOffset[0].append(rRIC[0])
            truthOffset[1].append(rRIC[1])
            truthOffset[2].append(rRIC[2])
            offsetHistory[0].append(rRIC[0])
            offsetHistory[1].append(rRIC[1])
            offsetHistory[2].append(rRIC[2])

            r = np.linalg.norm(rv_truth[:3])
            accelActual = fm_truth.GetDerivativesForSpacecraft(truth_Sat_wrapper.getSat())
            accel = -mu / r**3 * np.array(rv_truth[:3])
            a_diff = np.linalg.norm(accelActual[3:] - accel)

            temp1 = rotMatrix @ accel
            temp2 = rotMatrix @ accelActual[3:]
            # print(f"Diff: {a_diff} | Computed Two-body acceleration: {np.linalg.norm(temp1)} | Observed Two-body acceleration: {np.linalg.norm(temp2)}")
            # print(f"Step {j:2d} | diff = {a_diff:.10f} km/s²  | "
            #     f"IsFiring         = {truth_Sat_wrapper.ethruster.GetField('IsFiring')}  | ")
            
            t.append(elapsed)
            refCOE = ref_Sat_wrapper.getKeplerianState()
            truthCOE = truth_Sat_wrapper.getKeplerianState()
            # print(f"Maneuvering | T+ {elapsed} sec | TA = {truthCOE[-1]}")
            coes = list(diffCOEs.keys())
            for k in range(6):
                diff = truthCOE[k] - refCOE[k]

                if k > 1 and diff > 180:
                    diff = diff - 360
                elif k > 1 and diff < -180:
                    diff = diff + 360
                diffCOEs[coes[k]].append(diff)
            # print(elapsed, diffCOEs["del_a"][-1], del_a_to_return)
            if diffCOEs["del_a"][-1] >= del_a_to_return:
                state = "returning to nominal"
                gator_truth, fm_truth = enginesOff()
                ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'r')
                truthOffset = [[truthOffset[0][-1]], [truthOffset[1][-1]], [truthOffset[2][-1]]]
            elif (10 < truthCOE[-1] <= 170) or (190 < truthCOE[-1] <= 350):
                state = "wait to raise"
                gator_truth, fm_truth = enginesOff()
                ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'r')
                truthOffset = [[truthOffset[0][-1]], [truthOffset[1][-1]], [truthOffset[2][-1]]]
                
        case "returning to nominal":
            elapsed = elapsed + dt
            gator_ref.Step(dt)
            gator_ref.UpdateSpaceObject()
            rv_ref = gator_ref.GetState()

            gator_truth.Step(dt)
            gator_truth.UpdateSpaceObject()
            rv_truth = gator_truth.GetState()

            rRIC, vRIC, rotMatrix = xyz2ric(rv_ref, rv_truth)

            # statesArrayCoast[i,j,:] = rRIC # state_truth[0:6]
            truthOffset[0].append(rRIC[0])
            truthOffset[1].append(rRIC[1])
            truthOffset[2].append(rRIC[2])
            offsetHistory[0].append(rRIC[0])
            offsetHistory[1].append(rRIC[1])
            offsetHistory[2].append(rRIC[2])

            r = np.linalg.norm(rv_truth[:3])
            accelActual = fm_truth.GetDerivativesForSpacecraft(truth_Sat_wrapper.getSat())
            accel = -mu / r**3 * np.array(rv_truth[:3])
            a_diff = np.linalg.norm(accelActual[3:] - accel)

            temp1 = rotMatrix @ accel
            temp2 = rotMatrix @ accelActual[3:]
            # print(f"Diff: {a_diff} | Computed Two-body acceleration: {np.linalg.norm(temp1)} | Observed Two-body acceleration: {np.linalg.norm(temp2)}")
            # print(f"Step {j:2d} | diff = {a_diff:.10f} km/s²  | "
            #     f"IsFiring         = {truth_Sat_wrapper.ethruster.GetField('IsFiring')}  | ")
            
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
            # print(elapsed, diffCOEs["del_a"][-1], del_a_to_return)
            if diffCOEs["del_a"][-1] <= 0:
                state = "nominal"
                # break

if state == "raising":
    ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'r')
else:
    ax_ric.plot(truthOffset[0], truthOffset[1], truthOffset[2], 'b')
# ax.plot(statesArrayElectric[0,:,0],statesArrayElectric[0,:,1],statesArrayElectric[0,:,2],label='burn',c='r')
# ax.plot(statesArrayCoast[0,:,0],statesArrayCoast[0,:,1],statesArrayCoast[0,:,2],label='coast',c='b')
# ax.plot(statesArrayElectric2[0,:,0],statesArrayElectric2[0,:,1],statesArrayElectric2[0,:,2],label='Electric',c='r')
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
ax.set_xlabel('Time (Days)')
ax.set_ylabel('Offset (km)')
ax.legend()

ax = plt.figure().add_subplot()
ax.plot(t, diffCOEs["del_e"], label="del_e")
ax.set_xlabel('Time (Days)')
ax.set_ylabel('Offset')
ax.legend()

ax = plt.figure().add_subplot()
ax.plot(t, diffCOEs["del_aop"], label="del_aop")
ax.plot(t, diffCOEs["del_f"], label="del_f")
ax.set_xlabel('Time (Days)')
ax.set_ylabel('Offset (deg)')
ax.legend()

ax = plt.figure().add_subplot()
ax.plot(t, diffCOEs["del_raan"], label="del_raan")
ax.plot(t, diffCOEs["del_i"], label="del_i")
ax.set_xlabel('Time (Days)')
ax.set_ylabel('Offset (deg)')
ax.legend()

ax = plt.figure().add_subplot()
ax.plot(t, offsetHistory[0], label="R")
ax.plot(t, offsetHistory[1], label="I")
ax.plot(t, offsetHistory[2], label="C")
ax.set_xlabel('Time (Days)')
ax.set_ylabel('Offset (km)')
ax.legend()

plt.show()
