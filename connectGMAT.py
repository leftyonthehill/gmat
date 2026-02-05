
from load_gmat import *  # This imports gmatpy and initializes the API
from createReferencePropagator import *

refPropagator = createPropagator("Reference")
truthPropagator = createPropagator("Truth")

gmat.Initialize()
gmat.ShowObjects("MissionSequence")
# print(gmat.GetGeneratingString(0))