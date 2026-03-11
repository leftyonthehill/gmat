
from load_gmat import *  # This imports gmatpy and initializes the API
from createForceModel import createPropagator
from createSatellite import createSatellite

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

_truthSat = createSatellite("Truth")
_truthSat.setOrbitElements(orbitParam)
truthSat = _truthSat.getSat()

_refPropagator = createPropagator("Reference")
_refPropagator.assignSats(refSat)
refPropagator = _refPropagator.getProp()
_truthPropagator = createPropagator("Truth")
_truthPropagator.assignSats(truthSat)
truthPropagator = _truthPropagator.getProp()

gmat.Initialize()
test = refPropagator.GetState()

if isinstance(test, bool):
    print("What?")

"""print(np.array(refPropagator.GetState()))
refPropagator.Step(0.01)
print(np.array(refPropagator.GetState()))"""