from load_gmat import *

class createPropagator:
    def __init__(self, propType: str):
        if propType.lower() != "reference" and propType.lower() != "truth":
            raise ValueError(f"Incorrect propagation type was chosen (Provided: {propType}). The propagator type must only be 'reference' or 'truth'.")
        self.prop = self.createProp(propType)

    def getProp(self):
        return self.prop

    def createProp(self, propType: str):
        propagator = gmat.Construct("Propagator", f"{propType.lower()}Propagator")
        integrator = gmat.Construct("RungeKutta89", "Gator")
        propagator.SetReference(integrator)
        
        propagator.SetField("InitialStepSize", 60)
        propagator.SetField("MinStep", 1e-3)
        propagator.SetField("Accuracy", 1e-10)

        fm = gmat.Construct("ForceModel", f"{propType.lower()}Forces")
        if propType.lower() == "reference":
            fm = self.forceModel(
                fm
            )
        else:
            fm = self.forceModel(
                fm,
                thirdBodyEffects=True,
                atmDrag=True,
                srp=True
            )

        propagator.SetReference(fm)
        return propagator

    def forceModel(self, fm, **kwargs):
        earthGrav = gmat.Construct("GravityField")
        earthGrav.SetField("BodyName", "Earth")
        earthGrav.SetField("Degree", 4)
        earthGrav.SetField("Order", 4)
        earthGrav.SetField("PotentialFile", "JGM2.cof")
        fm.AddForce(earthGrav)
        if "thirdBodyEffects" in kwargs.keys() and kwargs["thirdBodyEffects"]:
            moonGrav = gmat.Construct("PointMassForce")
            moonGrav.SetField("BodyName", "Luna")
            fm.AddForce(moonGrav)

            sunGrav = gmat.Construct("PointMassForce")
            sunGrav.SetField("BodyName", "Sun")
            fm.AddForce(sunGrav)

        if "atmDrag" in kwargs.keys() and kwargs["thirdBodyEffects"]:
            drag = gmat.Construct("DragForce")
            drag.SetField("AtmosphereModel", "JacchiaRoberts")
            atmosphere = gmat.Construct("JacchiaRoberts")
            drag.SetReference(atmosphere)
            fm.AddForce(drag)

        if "srp" in kwargs.keys() and kwargs["thirdBodyEffects"]:
            srp = gmat.Construct("SolarRadiationPressure")
            fm.AddForce(srp)
        
        return fm

    def assignSats(self, sat):
        psm = gmat.PropagationStateManager()
        psm.SetObject(sat)
        psm.BuildState()