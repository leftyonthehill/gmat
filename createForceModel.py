from createSatellite import Satellite
from load_gmat import *


class ForceModel:
    """This class is a wrapper for a ForceModel object in GMAT.

    Provided that this force model is supposed to support the reference satellite object
    or the truth satellite, this class will assign the corresponding forces and perturbations

    Inputs during intialization:
        fmType: MUST BE EITHER "reference" or "truth" (non-case specific, determines the forces that need to be modeled)

    - If a reference force model, only perturbations considered are a 2x0 Earth geopotential model
    - If a truth force model, the following perturbations are considered:
        - 4x4 Earth geopotential model
        - Atmospheric drag
        - 3rd body effects (Lunar and Solar)
        - Solar radiation pressure

    Variables:
        fm: GMAT ForceModel Object (object containing all described forces)
        burn: GMAT FiniteBurn Object (object describing finite thrust along a corresponding RIC axis)
        burnforce: GMAT Force to be applied to a finite burn object (what gets added to the force model for propagation)
    """

    def __init__(self, fmType: str):
        """Initialize the ForceModel wrapper with ForceModel type."""
        self.fm = gmat.Construct("ForceModel", f"{fmType}_Forces")
        self.burn = {}
        self.burnForce = {}

    def getBurnForce(self, axis):
        """Return the GMAT BurnForce object"""
        return self.burnForce[axis]

    def getFM(self):
        """Return the GMAT ForceModel object"""
        return self.fm

    def setDynamics(self, propType: str):
        """Assign the corresponding forces to the ForceModel"""
        
        # ForceModel type must be either "reference" or "truth" otherwise an error is rasied
        if propType.lower() != "reference" and propType.lower() != "truth":
            raise ValueError(f"Incorrect propagation type was chosen (Provided: {propType}). The propagator type must only be 'reference' or 'truth'.")
        
        if propType.lower() == "reference":
            self._setForces(
                degree=2,
                order=0
            )
        else:
            self._setForces(
                degree=4,
                order=4,
                thirdBodyEffects=True,
                atmDrag=True,
                srp=True
            )

    def _setForces(self, **kwargs):
        solar = gmat.GetSolarSystem()
        self.fm.SetSolarSystem(solar)
        self.fm.SetField("CentralBody", "Earth")
        earthGrav = gmat.Construct("GravityField", f"{self.fm.GetName()}_Earth_Geopotential")
        earthGrav.SetField("BodyName", "Earth")
        earthGrav.SetField("Degree", kwargs["degree"])
        earthGrav.SetField("Order", kwargs["order"]) # 4
        earthGrav.SetField("PotentialFile", "JGM2.cof")
        earthGrav.SetField("StmLimit", 100)
        earthGrav.SetField("TideModel", "None")
        self.fm.AddForce(earthGrav)
        
        if "thirdBodyEffects" in kwargs.keys() and kwargs["thirdBodyEffects"]:
            moonGrav = gmat.Construct("PointMassForce", f"{self.fm.GetName()}_Lunar_Grav")  
            moonGrav.SetField("BodyName", "Luna")
            self.fm.AddForce(moonGrav)

            sunGrav = gmat.Construct("PointMassForce", f"{self.fm.GetName()}_Solar_Grav")
            sunGrav.SetField("BodyName", "Sun")
            self.fm.AddForce(sunGrav)

        if "atmDrag" in kwargs.keys() and kwargs["atmDrag"]:
            drag = gmat.Construct("DragForce", f"{self.fm.GetName()}_atmDrag")
            drag.SetField("AtmosphereModel", "JacchiaRoberts")
            atmosphere = gmat.Construct("JacchiaRoberts")
            drag.SetReference(atmosphere)
            self.fm.AddForce(drag)

        if "srp" in kwargs.keys() and kwargs["srp"]:
            srp = gmat.Construct("SolarRadiationPressure", f"{self.fm.GetName()}_srp")
            self.fm.AddForce(srp)

    def createBurnForces(self, satObj:Satellite, ax:str):
        if ax in self.burn:
            raise SyntaxError("Thrust profiles have already been produced")
            
        thr = satObj.thrusters[ax]
        self.burn[ax] = gmat.Construct("FiniteBurn", f"{self.fm.GetName()}_{ax}_Burn")
        self.burn[ax].SetField("Thrusters", thr.GetName())
        self.burn[ax].SetRefObject(thr, gmat.THRUSTER, thr.GetName())
        self.burn[ax].SetSolarSystem(gmat.GetSolarSystem())
        self.burn[ax].SetSpacecraftToManeuver(satObj.getSat())
        self.burn[ax].SetRefObject(satObj.sat, gmat.SPACECRAFT, satObj.sat.GetName())

        self.burnForce[ax] = gmat.FiniteThrust(f"{self.fm.GetName()}_{ax}_Thrust")
        self.burnForce[ax].SetRefObjectName(gmat.SPACECRAFT, satObj.sat.GetName())
        self.burnForce[ax].SetReference(self.burn[ax])

        gmat.ConfigManager.Instance().AddPhysicalModel(self.burnForce[ax])

    def setThrust(self, ax:str = ""):
        if ax != "":
            self.fm.AddForce(self.burnForce[ax])
        else:
            forceToDelete = [i.GetName() for i in self.burnForce.values()]
            for i in range(self.fm.GetNumForces()):
                f = self.fm.GetForce(i)
                if f.GetName() in forceToDelete:
                    self.fm.DeleteForce(f)
                    break
        