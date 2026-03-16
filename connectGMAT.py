from load_gmat import *  # This imports gmatpy and initializes the API
from createForceModel import createForceModel
from createSatellite import createSatellite
from createPropagator import createPropagator
from createThruster import createThruster
from xyz2ric import xyz2ric

# import gmatpyplus as gmat
import datetime as dt
import numpy as np
import matplotlib.pyplot as plt
orbitParam = [
    6878,   # SMA, avg alt of 500 km
    1e-3,   # ECC
    65,     # INC
    0,      # RAAN
    0,      # AOP
    0       # TA
]

_refSat = createSatellite("Reference")
_refFM = createForceModel("Reference")
_refPropagator = createPropagator("Reference")

_truthSat = createSatellite("Truth")
_truthFM = createForceModel("Truth")
_truthPropagator = createPropagator("Truth")

_refSat.setOrbitElements(orbitParam)
#refSat = _refSat.getSat()

_truthSat.setOrbitElements(orbitParam)
#truthSat = _truthSat.getSat()

power = gmat.Construct("NuclearPowerSystem", "power")
power.SetField("InitialMaxPower", 15)
today = dt.datetime.today()
today = today.strftime("%d %b %Y 00:00:00.000")
power.SetField("InitialEpoch", today)
_truthSat.assignPower("power")
"""print(power.Help())
exit(1)"""
tank = gmat.Construct("ElectricTank", "eTank")
_refSat.assignTank("eTank")
_truthSat.assignTank("eTank")

_thruster = createThruster("mainThrust")
_thruster.assignTank("eTank")
_thruster.assignForce(1e-2)

_thruster.assignDirection("v", 1)
_thruster.assignDirection("n", 0)
_thruster.assignDirection("b", 0)

thruster = _thruster.getThruster()
_truthSat.assignThruster("mainThrust")

burn = gmat.Construct("FiniteBurn", "stationkeep")
burn.SetField("Thrusters", "mainThrust")
burn.SetSolarSystem(gmat.GetSolarSystem())

burnForce = gmat.FiniteThrust("Thrust")
burnForce.SetRefObjectName(gmat.SPACECRAFT, "truthSat")
burnForce.SetReference(burn)
gmat.ConfigManager.Instance().AddPhysicalModel(burnForce)

try:
    gmat.Initialize()
    print("GMAT Initialized!")
except Exception as e:
    print("Initialization failed ->", e)
    exit(1)

_refFM.assignSats(_refSat.getSat())
refFM = _refFM.getFM()

_truthFM.assignSats(_truthSat.getSat())
truthFM = _truthFM.getFM()

_refPropagator.assignFM(refFM)
_refPropagator.assignSat(_refSat.getSat())

_truthPropagator.assignFM(truthFM)
_truthPropagator.assignSat(_truthSat.getSat())

refPropagator = _refPropagator.getPropagator()
refIntegrator = _refPropagator.getIntegrator()
truthPropagator = _truthPropagator.getPropagator()
truthIntegrator = _truthPropagator.getIntegrator()

refSatState = [[], [], []]
truthSatState = [[], [], []]
diffCOEs = {
    "del_a": [0],
    "del_e": [0],
    "del_i": [0],
    "del_raan": [0],
    "del_aop": [0],
    "del_f": [0]
}

refState = _refSat.getCartesianState()
refSatState[0].append(0)
refSatState[1].append(0)
refSatState[2].append(0)
truthState = _truthSat.getCartesianState()

r_RIC, v_RIC, rotMatrix = xyz2ric(refState, truthState)
truthSatState[0].append(r_RIC[0])
truthSatState[1].append(r_RIC[1])
truthSatState[2].append(r_RIC[2])

t = [0]
dt = 120
# for i in range (750):    

i = 0
maxDays = 2
state = "nominal"
y_RIC_bounds = 15
del_a_bounce = 0
maxThrusting = 0

print(_truthSat.getSat().IsInitialized())
print(_thruster.getThruster().IsInitialized())

while t[-1] <= maxDays * 86400:
    i += 1
    t.append(i * dt)

    match state:
        case "nominal":
            refIntegrator.Step(dt)
            refIntegrator.UpdateSpaceObject()
            truthIntegrator.Step(dt)
            truthIntegrator.UpdateSpaceObject()

            refXYZ = _refSat.getCartesianState()
            truthXYZ = _truthSat.getCartesianState()
            refCOE = _refSat.getKeplerianState()
            truthCOE = _truthSat.getKeplerianState()
            
            r_RIC, v_RIC, rotMatrix = xyz2ric(refXYZ, truthXYZ)
            truthSatState[0].append(r_RIC[0])
            truthSatState[1].append(r_RIC[1])
            truthSatState[2].append(r_RIC[2])

            coes = list(diffCOEs.keys())
            for j in range(6):
                diff = truthCOE[j] - refCOE[j]

                if j > 1 and diff > 180:
                    diff = diff - 360
                elif j > 1 and diff < -180:
                    diff = diff + 360
                diffCOEs[coes[j]].append(diff)
            
            if abs(r_RIC[1]) > y_RIC_bounds:
                state = "wait to raise"
                
                """thruster.SetField("ThrustDirection1", -1)
                thruster.SetField("ThrustDirection2", 0)
                thruster.SetField("ThrustDirection3", 0)"""

                del_a_bounce = abs(diffCOEs["del_a"][-1])
        case "wait to raise":
            refIntegrator.Step(dt)
            refIntegrator.UpdateSpaceObject()
            truthIntegrator.Step(dt)
            truthIntegrator.UpdateSpaceObject()

            refXYZ = _refSat.getCartesianState()
            truthXYZ = _truthSat.getCartesianState()
            refCOE = _refSat.getKeplerianState()
            truthCOE = _truthSat.getKeplerianState()
            
            r_RIC, v_RIC, rotMatrix = xyz2ric(refXYZ, truthXYZ)
            truthSatState[0].append(r_RIC[0])
            truthSatState[1].append(r_RIC[1])
            truthSatState[2].append(r_RIC[2])

            coes = list(diffCOEs.keys())
            for j in range(6):
                diff = truthCOE[j] - refCOE[j]

                if j > 1 and diff > 180:
                    diff = diff - 360
                elif j > 1 and diff < -180:
                    diff = diff + 360
                diffCOEs[coes[j]].append(diff)

            if truthCOE[-1] >= 350:
                state = "raising"
                
        case "raising":

            refIntegrator.Step(0.01*dt)
            refIntegrator.UpdateSpaceObject()
            
            print(0)
            thruster.SetField("IsFiring", True)
            _truthSat.setManeuvering(True)
            burn.SetSpacecraftToManeuver(_truthSat.getSat())
            print(0.5)
            print(thruster.GetField("IsFiring"))
            # print(_truthSat.getSat().HasActiveThrusters())
            print(0.75)
            truthPropagator.AddForce(burnForce)
            psm = truthPropagator.GetPropStateManager()
            psm.BuildState()
            # psm.SetProperty("MassFlow")
            # fm.AddForce(burnForce)
            # print(fm.Help())
            # truthPropagator.AddForce(burnForce) # type: ignore
            truthPropagator.PrepareInternals() # type: ignore
            print(1)
            truthIntegrator = truthPropagator.GetPropagator() # type: ignore
            print(2)
            # print(thruster.Help())
            fm = _truthFM.getFM()
            tempSat = _truthSat.getSat()
            dv = fm.GetDerivativesForSpacecraft(tempSat)
            print("Engine ignition:  ", dv)
            # print(_truthSat.getCartesianState())
            truthIntegrator.Step(0.01*dt)
            print(3)
            truthIntegrator.UpdateSpaceObject()
            # print(_truthSat.getCartesianState())
            
            
            fm = _truthFM.getFM()
            tempSat = _truthSat.getSat()
            dv = fm.GetDerivativesForSpacecraft(tempSat)
            print("Engine shutdown:  ", dv)

            # gmat.Command("EndFiniteBurn", "stationkeep(truthSat)")
            fm.DeleteForce(burnForce)
            thruster.SetField("IsFiring", False)
            _truthSat.setManeuvering(False)
            truthPropagator.PrepareInternals() # type: ignore
            truthIntegrator = truthPropagator.GetPropagator() # type: ignore

            fm = _truthFM.getFM()
            tempSat = _truthSat.getSat()
            dv = fm.GetDerivativesForSpacecraft(tempSat)
            print("Engine disabled:  ", dv)
            # break
            refXYZ = _refSat.getCartesianState()
            truthXYZ = _truthSat.getCartesianState()
            refCOE = _refSat.getKeplerianState()
            truthCOE = _truthSat.getKeplerianState()
            
            r_RIC, v_RIC, rotMatrix = xyz2ric(refXYZ, truthXYZ)
            truthSatState[0].append(r_RIC[0])
            truthSatState[1].append(r_RIC[1])
            truthSatState[2].append(r_RIC[2])
            
            coes = list(diffCOEs.keys())
            for j in range(6):
                diff = truthCOE[j] - refCOE[j]

                if j > 1 and diff > 180:
                    diff = diff - 360
                elif j > 1 and diff < -180:
                    diff = diff + 360
                diffCOEs[coes[j]].append(diff)

            if diffCOEs["del_a"][-1] >= 0.9 * del_a_bounce:
                print(4)
                print(diffCOEs["del_a"][-1], 0.9 * del_a_bounce)
                state = "nominal"
            
            maxThrusting += 1
            if maxThrusting == 1:
                break
            else:
                state = "wait to raise"
            """if truthCOE[-1] < 350 and truthCOE[-1] >= 10:
                state = "wait to raise"
                print(state, truthCOE[-1])
            else:
                print(state, truthCOE[-1])"""
            # break
        case _:
            break

    

ax = plt.figure().add_subplot()
ax.plot(t, truthSatState[0], label="x_ric")
ax.plot(t, truthSatState[1], label="y_ric")
ax.plot(t, truthSatState[2], label="z_ric")
ax.legend()

ax = plt.figure().add_subplot()
ax.plot(t, diffCOEs["del_a"], label="del_a")
ax.plot(t, diffCOEs["del_e"], label="del_e")
ax.legend()

ax = plt.figure().add_subplot()
ax.plot(t, diffCOEs["del_i"], label="del_i")
ax.plot(t, diffCOEs["del_raan"], label="del_raan")
ax.plot(t, diffCOEs["del_aop"], label="del_aop")
ax.plot(t, diffCOEs["del_f"], label="del_f")
ax.legend()

ax = plt.figure().add_subplot(projection="3d")
ax.plot(truthSatState[0], truthSatState[1], truthSatState[2])
ax.axis("equal")
plt.show()