from load_gmat import *
earthGrav = gmat.Construct("GravityField")
earthGrav.SetField("BodyName", "Earth")
earthGrav.SetField("Degree", 4)
earthGrav.SetField("Order", 4)
earthGrav.SetField("PotentialFile", "JGM2.cof")


def createPropagator(propType: str):
    if propType.lower() != "reference" and propType.lower() != "truth":
        raise ValueError(f"Incorrect propagation type was chosen (Provided: {propType}). The propagator type must only be 'reference' or 'truth'.")
    
    propagator = gmat.Construct("Propagator", f"{propType.lower()}Propagator")
    integrator = gmat.Construct("RungeKutta89", "Gator")
    propagator.SetReference(integrator)
    
    propagator.SetField("InitialStepSize", 60)
    propagator.SetField("MinStep", 1e-3)
    propagator.SetField("Accuracy", 1e-10)

    fm = gmat.Construct("ForceModel", "forces")
    if propType.lower() == "reference":
        fm = forceModel(
            fm
        )
    else:
        fm = forceModel(
            fm,
            thirdBodyEffects=True,
            atmDrag=True,
            srp=True
        )

    propagator.SetReference(fm)
    return propagator

def forceModel(fm, **kwargs):
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