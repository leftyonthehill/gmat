from createSatellite import Satellite
from load_gmat import *


class ForceModel:
    """This class is a wrapper for a ForceModel object in GMAT.

    This force model wrapper is to support the reference satellite object or the truth satellite. Depending on the inputs, this class will 
    assign the corresponding forces and perturbations

    Inputs during intialization:
        - fmType (str): MUST BE EITHER "reference" or "truth" (non-case specific, determines the forces that need to be modeled)

    - If a reference force model, only perturbations considered are a 2x0 Earth geopotential model
    - If a truth force model, the following perturbations are considered:
        - 4x4 Earth geopotential model
        - Atmospheric drag
        - 3rd body effects (Lunar and Solar)
        - Solar radiation pressure

    Variables:
        - fm: GMAT ForceModel Object (object containing all described forces)
        - burn: GMAT FiniteBurn Object (object describing finite thrust along a corresponding RIC axis)
        - burnforce: GMAT Force to be applied to a finite burn object (what gets added to the force model for propagation)
    """

    def __init__(self, fmType: str):
        """Initialize the ForceModel wrapper using the ForceModel type.
        
        Inputs:
            - fmType (str): Type of ForceModel to produce (must be either "reference" or "truth")
        
        """
        self.fm = gmat.Construct("ForceModel", f"{fmType}_Forces")
        self.burn = {}
        self.burnForce = {}

    def getBurnForce(self, axis:str):
        """Returns the GMAT BurnForce object
        
        Inputs:
            - axis (str): Desired BurnForce axis
        
        Returns:
            - GMAT BurnForce object
        """
        return self.burnForce[axis]

    def getFM(self):
        """Returns the GMAT ForceModel object
        
        Retuns:
            - GMAT ForceModel object
        """
        return self.fm

    def setForcesToPropagate(self, propType: str):
        """Assign the corresponding forces to the ForceModel
        
        Inputs:
            - propType (str): Defines which set for forces to model (must be either "reference" or "truth")
        
        Raises:
            - ValueError: If the provided propType is not one of the two allowed values
        """
        
        # ForceModel type must be either "reference" or "truth" otherwise an error is rasied
        if propType.lower() != "reference" and propType.lower() != "truth":
            raise ValueError(f"Incorrect propagation type was chosen (Provided: {propType}). The propagator type must only be 'reference' or 'truth'.")
        
        # Assigning the forces for the reference ForceModel
        if propType.lower() == "reference":
            self._setForces(
                degree=2,
                order=0
            )
        # Assigning the forces for the truth ForceModel
        else:
            self._setForces(
                degree=4,
                order=4,
                thirdBodyEffects=True, # If it is desired to remove any additional forces, simply delete the kwarg(s)
                atmDrag=True,
                srp=True
            )

    def _setForces(self, **kwargs):
        """ **Internal function**
        Based on the ForceModel type, assign the corresponding forces to the ForceModel

        Inputs:
            - **kwargs: 
                Must have: degree (int), order (int)
                Optional: thirdBodyEffects (boolean), atmDrag (boolean), srp(boolean)
        """
        
        # Earth's Geopotential model
        self.fm.SetField("CentralBody", "Earth")
        earthGrav = gmat.Construct("GravityField", f"{self.fm.GetName()}_Earth_Geopotential")
        earthGrav.SetField("BodyName", "Earth")
        earthGrav.SetField("Degree", kwargs["degree"])
        earthGrav.SetField("Order", kwargs["order"]) # 4
        earthGrav.SetField("PotentialFile", "JGM2.cof")
        earthGrav.SetField("StmLimit", 100)
        earthGrav.SetField("TideModel", "None")
        self.fm.AddForce(earthGrav)
        
        # To know the popsition and velocity vectors of the Moon and Sun
        solar = gmat.GetSolarSystem()
        self.fm.SetSolarSystem(solar)

        # Adding third body effects
        if "thirdBodyEffects" in kwargs.keys() and kwargs["thirdBodyEffects"]:
            moonGrav = gmat.Construct("PointMassForce", f"{self.fm.GetName()}_Lunar_Grav")  
            moonGrav.SetField("BodyName", "Luna")
            self.fm.AddForce(moonGrav)

            sunGrav = gmat.Construct("PointMassForce", f"{self.fm.GetName()}_Solar_Grav")
            sunGrav.SetField("BodyName", "Sun")
            self.fm.AddForce(sunGrav)

        # Adding atmospheric drag effects
        if "atmDrag" in kwargs.keys() and kwargs["atmDrag"]:
            drag = gmat.Construct("DragForce", f"{self.fm.GetName()}_atmDrag")
            drag.SetField("AtmosphereModel", "JacchiaRoberts")
            atmosphere = gmat.Construct("JacchiaRoberts")
            drag.SetReference(atmosphere)
            self.fm.AddForce(drag)

        # Adding solar radiation pressure effects
        if "srp" in kwargs.keys() and kwargs["srp"]:
            srp = gmat.Construct("SolarRadiationPressure", f"{self.fm.GetName()}_srp")
            self.fm.AddForce(srp)

    def createBurnForces(self, satObj:Satellite, ax:str):
        """For a particular thruster axis on the given satObj, create a BurnForce object to be added to the Propagator later
        
        Inputs:
            - satObj (Satellite): Satellite wrapper that contains thrusters that need to have their corresponding forces created
            - ax (str): Thruster axis to create a BurnForce for

        Raises:
            - TypeError: A FiniteBurn and BurnForce objects were attempted to be created when they already exist
        """

        # Quick check to see if burn has already been created
        if ax in self.burn:
            raise TypeError("Thrust profiles have already been produced")
            
        thr = satObj.thrusters[ax]

        # Create the FiniteBurn for the thruster
        self.burn[ax] = gmat.Construct("FiniteBurn", f"{self.fm.GetName()}_{ax}_Burn")
        self.burn[ax].SetField("Thrusters", thr.GetName())
        self.burn[ax].SetRefObject(thr, gmat.THRUSTER, thr.GetName())
        self.burn[ax].SetSolarSystem(gmat.GetSolarSystem())
        self.burn[ax].SetSpacecraftToManeuver(satObj.getSat())
        self.burn[ax].SetRefObject(satObj.sat, gmat.SPACECRAFT, satObj.sat.GetName())

        # Create the BurnForce for the FiniteBurn
        self.burnForce[ax] = gmat.FiniteThrust(f"{self.fm.GetName()}_{ax}_Thrust")
        self.burnForce[ax].SetRefObjectName(gmat.SPACECRAFT, satObj.sat.GetName())
        self.burnForce[ax].SetReference(self.burn[ax])

        # Assign the BurnForce to the GMAT table of phyiscal models
        gmat.ConfigManager.Instance().AddPhysicalModel(self.burnForce[ax])        