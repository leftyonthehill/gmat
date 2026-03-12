# from load_gmat import *  # This imports gmatpy and initializes the API
from createForceModel import createForceModel
from createSatellite import createSatellite
from createPropagator import createPropagator
from createThruster import createThruster
from xyz2ric import xyz2ric

import gmatpyplus as gmat
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
refSat = _refSat.getSat()

_truthSat.setOrbitElements(orbitParam)
truthSat = _truthSat.getSat()

tank = gmat.Construct("ElectricTank", "eTank")
refSat.SetField("Tanks", "eTank")
truthSat.SetField("Tanks", "eTank")

_thruster = createThruster("mainThrust")
_thruster.assignTank("eTank")
_thruster.assignForce()

_thruster.assignDirection("v", 1)
_thruster.assignDirection("n", 0)
_thruster.assignDirection("b", 0)

thruster = _thruster.getThruster()
truthSat.SetField("Thrusters", "mainThrust")

# list(thrusters.keys())
try:
    gmat.Initialize()
    print("GMAT Initialized!")
except Exception as e:
    print("Initialization failed ->", e)
    exit(1)

_refFM.assignSats(refSat)
refFM = _refFM.getFM()

_truthFM.assignSats(truthSat)
truthFM = _truthFM.getFM()

_refPropagator.assignFM(refFM)
_refPropagator.assignSat(refSat)

_truthPropagator.assignFM(truthFM)
_truthPropagator.assignSat(truthSat)

refPropagator = _refPropagator.getProp()
truthPropagator = _truthPropagator.getProp()

refSatState = [[], [], []]
truthSatState = [[], [], []]
t = []
dt = 120
# print(f"x_ref \t y_ref \t z_ref \t x_truth \t y_truth \t z_truth \t x_ric \t y_ric \t z_ric")
for i in range (751):
    refState = np.array(refPropagator.GetState())
    refSatState[0].append(0)
    refSatState[1].append(0)
    refSatState[2].append(0)
    truthState = np.array(truthPropagator.GetState())
    
    r_RIC, v_RIC, rotMatrix = xyz2ric(refState, truthState)
    truthSatState[0].append(r_RIC[0])
    truthSatState[1].append(r_RIC[1])
    truthSatState[2].append(r_RIC[2])
    
    # print(f"{refState[0]}, {refState[1]}, {refState[2]}, {truthState[0]}, {truthState[1]}, {truthState[2]}, {r_RIC[0]}, {r_RIC[1]}, {r_RIC[2]}")

    t.append(i * dt)
    refPropagator.Step(dt)
    refPropagator.UpdateSpaceObject()
    truthPropagator.Step(dt)
    truthPropagator.UpdateSpaceObject()

print(truthSatState[0][-1], truthSatState[1][-1], truthSatState[2][-1])
ax = plt.figure().add_subplot()
ax.plot(t, truthSatState[0], label="x_ric")
ax.plot(t, truthSatState[1], label="y_ric")
ax.plot(t, truthSatState[2], label="z_ric")
ax.legend()
plt.show()

ax = plt.figure().add_subplot(projection="3d")
ax.plot(truthSatState[0], truthSatState[1], truthSatState[2])
ax.axis("equal")
plt.show()