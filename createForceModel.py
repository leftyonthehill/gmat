from createSatellite import Satellite
from load_gmat import *


class ForceModel:
    def __init__(self, fmType: str):
        self.fm = gmat.Construct("ForceModel", f"{fmType}_Forces")
        self.burn = None
        self.burnForce = None

    def getBurnForce(self):
        return self.burnForce

    def getFM(self):
        return self.fm

    def setDynamics(self, propType: str):
        if propType.lower() != "reference" and propType.lower() != "truth":
            raise ValueError(f"Incorrect propagation type was chosen (Provided: {propType}). The propagator type must only be 'reference' or 'truth'.")
        
        if propType.lower() == "reference":
            self.__setForces__()
        else:
            self.__setForces__(
                thirdBodyEffects=True,
                atmDrag=True,
                srp=True
            )

    def __setForces__(self, **kwargs):
        solar = gmat.GetSolarSystem()
        self.fm.SetSolarSystem(solar)
        self.fm.SetField("CentralBody", "Earth")
        earthGrav = gmat.Construct("GravityField")
        earthGrav.SetField("BodyName", "Earth")
        earthGrav.SetField("Degree", 0)
        earthGrav.SetField("Order", 0) # 4
        earthGrav.SetField("PotentialFile", "JGM2.cof")
        earthGrav.SetField("StmLimit", 100)
        earthGrav.SetField("TideModel", "None")
        self.fm.AddForce(earthGrav)
        
        if "thirdBodyEffects" in kwargs.keys() and kwargs["thirdBodyEffects"]:
            moonGrav = gmat.Construct("PointMassForce")  
            moonGrav.SetField("BodyName", "Luna")
            self.fm.AddForce(moonGrav)

            sunGrav = gmat.Construct("PointMassForce")
            sunGrav.SetField("BodyName", "Sun")
            self.fm.AddForce(sunGrav)

        if "atmDrag" in kwargs.keys() and kwargs["atmDrag"]:
            drag = gmat.Construct("DragForce", "atmDrag")
            drag.SetField("AtmosphereModel", "JacchiaRoberts")
            atmosphere = gmat.Construct("JacchiaRoberts")
            drag.SetReference(atmosphere)
            self.fm.AddForce(drag)

        if "srp" in kwargs.keys() and kwargs["srp"]:
            srp = gmat.Construct("SolarRadiationPressure")
            self.fm.AddForce(srp)

    def setThrust(self, satObj:Satellite):
        self.burn = gmat.Construct("FiniteBurn", f"{satObj.sat.GetName()}_EBurn")
        self.burn.SetField("Thrusters", satObj.ethruster.GetName())
        self.burn.SetRefObject(satObj.ethruster, gmat.THRUSTER, satObj.ethruster.GetName())
        self.burn.SetSolarSystem(gmat.GetSolarSystem())
        self.burn.SetSpacecraftToManeuver(satObj.getSat())
        self.burn.SetRefObject(satObj.sat, gmat.SPACECRAFT, satObj.sat.GetName())

        self.burnForce = gmat.FiniteThrust("Thrust")
        self.burnForce.SetRefObjectName(gmat.SPACECRAFT, satObj.sat.GetName())
        self.burnForce.SetReference(self.burn)
    
        gmat.ConfigManager.Instance().AddPhysicalModel(self.burnForce)
        self.fm.AddForce(self.burnForce)