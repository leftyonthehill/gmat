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

tank = gmat.Construct("ElectricTank", "eTank")
_refSat.assignTank("eTank")
_truthSat.assignTank("eTank")

_thruster = createThruster("mainThrust")
_thruster.assignTank("eTank")
_thruster.assignForce()

_thruster.assignDirection("v", 1)
_thruster.assignDirection("n", 0)
_thruster.assignDirection("b", 0)

thruster = _thruster.getThruster()
_truthSat.assignThruster("mainThrust")

burn = gmat.Construct("FiniteBurn", "stationkeep")
burn.SetField("Thrusters", "mainThrust")

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
                
                thruster.SetField("ThrustDirection1", 1)
                thruster.SetField("ThrustDirection2", 0)
                thruster.SetField("ThrustDirection3", 0)

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
            refIntegrator.Step(dt)
            refIntegrator.UpdateSpaceObject()
            
            # gmat.Command("BeginFiniteBurn", "stationkeep(truthSat)")
            thruster.SetField("IsFiring", True)
            _truthSat.setManeuvering(True)
            burn.SetSpacecraftToManeuver(_truthSat.getSat())
            
            truthPropagator.AddForce(burnForce)
            truthPropagator.PrepareInternals()
            truthIntegrator = truthPropagator.GetPropagator()

            truthIntegrator.Step(dt)
            truthIntegrator.UpdateSpaceObject()
            # gmat.Command("EndFiniteBurn", "stationkeep(truthSat)")
            fm = truthPropagator.GetODEModel()
            fm.DeleteForce(burnForce)
            thruster.SetField("IsFiring", False)
            _truthSat.setManeuvering(False)
            truthPropagator.PrepareInternals()
            truthIntegrator = truthPropagator.GetPropagator()

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

            print(truthCOE)
            print(refCOE)

            t.append(i * dt)
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

            break
            if diffCOEs["del_a"][-1] >= 0.9 * del_a_bounce:
                state = "nominal"
            if truthCOE[-1] < 350 and truthCOE[-1] >= 10:
                state = "wait to raise"
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