
from load_gmat import *  # This imports gmatpy and initializes the API
from createForceModel import createForceModel
from createSatellite import createSatellite
from createPropagator import createPropagator

import numpy as np

orbitParam = [
    6878,   # SMA, avg alt of 500 km
    1e-3,   # ECC
    65,     # INC
    0,      # RAAN
    0,      # AOP
    0       # TA
]

_refSat = createSatellite("Reference")
_refSat.setOrbitElements(orbitParam)
refSat = _refSat.getSat()

_refFM = createForceModel("Reference")
_refFM.assignSats(refSat)

_refPropagator = createPropagator("Reference")
_refPropagator.assignFM(_refFM)
refPropagator = _refPropagator.getProp

_truthSat = createSatellite("Truth")
_truthSat.setOrbitElements(orbitParam)
truthSat = _truthSat.getSat()

_truthFM = createForceModel("Truth")
_truthFM.assignSats(truthSat)

_truthPropagator = createPropagator("Truth")
_truthPropagator.assignFM(_truthFM)
truthPropagator = _truthPropagator.getProp()

gmat.Initialize()
test = refPropagator.GetState()

if isinstance(test, bool):
    print("What?")

"""print(np.array(refPropagator.GetState()))
refPropagator.Step(0.01)
print(np.array(refPropagator.GetState()))"""