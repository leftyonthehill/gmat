""" Support class that creates the dynamics and force model for the station
keeping scenario. """

from createSatellite import Satellite
from load_gmat import gmat


class ForceModel:
    """
    Wrapper for a ForceModel object in GMAT design to model the forces
    experienced during a station keeping scenario.

    This wrapper supports two force modes:
    - "Reference": Includes minimum perturbation load (4x4 Earth
      geopotential model) for ideal reference trajectory.
    - "Truth": High fidelity dynamics model to include atmospheric
      drag, 3rd body effects (Sun/Moon), and solar radiation pressure.

    Attributes
    ----------
    fm : gmat.ODEModel 
        GMAT object holding the list of forces that contribute to the
        spacecraft's acceleration.
    burn : {str: gmat.FiniteBurn}
        dict of GMAT objects describing the configuration of the
        thrusters and their axes.
    burnforce : {str: gmat.FiniteThrust}
        dict of GMAT forces to be applied to the gmat.PhysicalModel to
        simulate continuous-thrust acceleration for each corresponding
        thruster axis.
    """

    def __init__(self, fmType: str):
        """ Initialize the ForceModel wrapper.
        
        Parameters
        ----------
        fmType : str
            ForceModel mode to produced.      
        """

        self.fm = gmat.Construct("ForceModel", f"{fmType}_Forces")
        self.burn = {}
        self.burnForce = {}

    def setForcesToPropagate(self, propType: str):
        """ Assign the corresponding forces for a given force mode.
        
        Parameters
        ----------
        propType : str
            Defines which set for forces to model.
        
        Raises
        ------
        ValueError
            If 'propType' is not one of allowed values ("Reference" or
            "Truth").
        """
        
        if propType.lower() != "reference" and propType.lower() != "truth":
            raise ValueError(
                "Incorrect propagation type was chosen (Provided: " 
                + propType + "). The propagator type must only be "
                + "'reference' or 'truth'.")
        
        if propType.lower() == "reference":
            self._setForces(
                degree=4,
                order=4
            )
        else:
            self._setForces(
                order=4,
                degree=4,
                thirdBodyEffects=True,
                atmDrag=True,
                srp=True
            )
       
    def _setForces(self, order: int, degree: int, **kwargs):
        """ Assign the desired forces to the ForceModel.

        Parameters
        ----------
        degree : float
        order : float
        thirdBodyEffects : bool, optional
        atmDrag : bool, optional
        srp : bool, optional
        """
        
        # Assign Earth's Geopotential model
        self.fm.SetField("CentralBody", "Earth")
        earthGrav = gmat.Construct("GravityField",
                                   f"{self.fm.GetName()}_Earth_Geopotential")
        earthGrav.SetField("BodyName", "Earth")
        earthGrav.SetField("Order", order)
        earthGrav.SetField("Degree", degree)
        earthGrav.SetField("PotentialFile", "JGM2.cof")
        earthGrav.SetField("StmLimit", 100)
        earthGrav.SetField("TideModel", "None")
        self.fm.AddForce(earthGrav)
        
        # Adding third body effects
        if "thirdBodyEffects" in kwargs.keys() and kwargs["thirdBodyEffects"]:
            solar = gmat.GetSolarSystem()
            self.fm.SetSolarSystem(solar)
            
            moonGrav = gmat.Construct("PointMassForce",
                                      f"{self.fm.GetName()}_Lunar_Grav")  
            moonGrav.SetField("BodyName", "Luna")
            self.fm.AddForce(moonGrav)

            sunGrav = gmat.Construct("PointMassForce",
                                     f"{self.fm.GetName()}_Solar_Grav")
            sunGrav.SetField("BodyName", "Sun")
            self.fm.AddForce(sunGrav)

        # Adding atmospheric drag effects
        if "atmDrag" in kwargs.keys() and kwargs["atmDrag"]:
            drag = gmat.Construct("DragForce", f"{self.fm.GetName()}_atmDrag")
            drag.SetField("AtmosphereModel", "JacchiaRoberts")
            atmosphere = gmat.Construct("JacchiaRoberts")
            drag.SetReference(atmosphere)

            drag.SetField("F107", 120.0)
            drag.SetField("F107A", 120.0)
            drag.SetField("MagneticIndex", 8)

            self.fm.AddForce(drag)

        # Adding solar radiation pressure effects
        if "srp" in kwargs.keys() and kwargs["srp"]:
            srp = gmat.Construct("SolarRadiationPressure", f"{self.fm.GetName()}_srp")
            self.fm.AddForce(srp)

    def createBurnForces(self, satObj:Satellite, ax:str):
        """
        For a given thruster axis, add its dynamics to the scenario's
        physical model.
        
        Parameters
        ----------
        satObj : Satellite
            Contains the thrusters to be added to the physics model.
        ax : str
            Thruster axis to create a BurnForce for.

        Raises
        ------
        RuntimeError
            FiniteBurn and BurnForce objects were attempted to be
            created a second time.
        """

        # Check to see if burn has already been created
        if ax in self.burn:
            raise RuntimeError("Thrust profiles have already been produced")

        # Create the FiniteBurn for the thruster
        thr = satObj.thrusters[ax]
        self.burn[ax] = gmat.Construct("FiniteBurn", f"{self.fm.GetName()}_{ax}_Burn")
        self.burn[ax].SetField("Thrusters", thr.GetName())
        self.burn[ax].SetRefObject(thr, gmat.THRUSTER, thr.GetName())
        self.burn[ax].SetSolarSystem(gmat.GetSolarSystem())
        self.burn[ax].SetSpacecraftToManeuver(satObj.getGMATSat())
        self.burn[ax].SetRefObject(satObj.sat, gmat.SPACECRAFT, satObj.sat.GetName())

        # Create the BurnForce for the FiniteBurn
        self.burnForce[ax] = gmat.FiniteThrust(f"{self.fm.GetName()}_{ax}_Thrust")
        self.burnForce[ax].SetRefObjectName(gmat.SPACECRAFT, satObj.sat.GetName())
        self.burnForce[ax].SetReference(self.burn[ax])

        # Assign the BurnForce to the GMAT table of phyiscal models
        gmat.ConfigManager.Instance().AddPhysicalModel(self.burnForce[ax])        