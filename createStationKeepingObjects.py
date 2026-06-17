from createForceModel import ForceModel
from createPropagator import Propagator
from createSatellite import Satellite
from load_gmat import *

class StationKeepingObjects:
    """Creates the necesasary GMAT objects to support the modeling of either a reference or a truth spacecraft
    
    Based on the provided objectType, this class will create the supporting force model, propagator, and spacecraft to be defined as the 
    "coast" key. In the case of a maneuverable spacecraft this class will also create force models along each thruster axis and a 
    propagator for each force model .
    
    Variables:
        - objType (str): What type of objects are being created
        - coes (list): List of the keplerian/cartesian state vector assign to the spacecraft
        - sat_wrap (Satellite): GMAT Spacecraft wrapper
        - fm_wrap (dict): dict containing the GMAT ForceModel wrapper for the coasting period and any axes with thrusters
        - prop_wrap (dict): dict containing the GMAT Propagator wrapper for the coasting period and any axes with thrusters
    """

    def __init__(self, objectType: str):
        """Initialize the StationKeepingObjects for a given objectType
        
        Inputs:
            - objectType (str): Describes what objects need to made
            
        Raises:
            ValueError: Checks to see if the provided objectType is either "truth" or "reference"
        """

        if objectType.lower() != "truth" and objectType.lower() != "reference":
            raise ValueError("Object typing can only be 'Truth' or 'Reference'.")

        self.objType = objectType
        self.coes = []

        # Create object wrappers
        self.sat_wrap    = Satellite(f"{objectType}_Sat")
        self.fm_wrap     = {"coast": ForceModel(objectType)}
        self.prop_wrap   = {"coast": Propagator(objectType)}

        # For the coasting period, assign the corresponding forces and satellite to the propagator
        self.fm_wrap["coast"].setForcesToPropagate(objectType)
        self.prop_wrap["coast"].setIntegrator(objectType)

        self.prop_wrap["coast"].setFM(self.fm_wrap["coast"].getFM())
        
        self.prop_wrap["coast"].setSat(self.sat_wrap.getSat())

    def setSatCOEs(self, coes: list):
        """Provided a 6-element list of classical orbit elements, assign the keplerian state vector to the spacecraft
        
        Inputs:
            - coes (list):  [
                            Semi-major axis, 
                            Eccentricity, 
                            Inclination, 
                            Right Ascension of the Ascending Node, 
                            Argument of Periapsis, 
                            True Anomaly
                            ]
        """
        self.coes = coes
        self.sat_wrap.setOrbitElements(coes)
    
    def setManeuverable(self):
        """Whether a satellite has custom thrusters or the default want to be used, this function calls the sat_wrap's 
        setManeuverable() function to associate its thrusters with the bus, fuel tank, and power supply.
        
        Additionally, this function will create a ForceModel and a Propagator to be able to properly model the thruster's 
        effects during simulation.
        """
        self.sat_wrap.setManeuverable()
        
        # For each thruster key in sat_wrap's thruster dict, create its own Propagator and ForceModel
        for ax in self.sat_wrap.thrusters.keys():
            self.fm_wrap[ax]     = ForceModel(f"{self.objType}_{ax}")
            self.fm_wrap[ax].setForcesToPropagate(self.objType)

            self.prop_wrap[ax]   = Propagator(f"{self.objType}_{ax}")
            self.prop_wrap[ax].setIntegrator(self.objType)
            self.prop_wrap[ax].setFM(self.fm_wrap[ax].getFM())
            self.prop_wrap[ax].setSat(self.sat_wrap.getSat())
    
    def setBurnForces(self):
        """After initializing the GMAT scenario, call this function to assign the ForceModels to a GMAT BurnForce object"""
        for ax in self.sat_wrap.thrusters.keys():
            self.fm_wrap[ax].createBurnForces(self.sat_wrap, ax)
            self.prop_wrap[ax].prepareInternals()

    def preparePropInternal(self):
        """Shortcut to prepare the internals of all associated propagators, not just the current one"""
        for prop in self.prop_wrap.values():
            prop.prepareInternals()

    def satEnginesOn(self, axis:str):
        """During the scenario when it is time to maneuver, this function changes which propagator is used to account for the 
        corresponding firing thruster and updates the satellite reference in the new propagator.
        
        Inputs:
            - axis (str): The corresponding axis in which the thrusters will fire"""
        
        # Collect the Propagator and ForceModel for the new axis
        prop = self.prop_wrap[axis]
        fm = self.fm_wrap[axis]

        # Update the latest internal values for the propagator
        prop.prepareInternals()

        # Collect the thruster we want to fire
        thr_name = self.sat_wrap.thrusters[axis].GetName()
        thruster = self.sat_wrap.getSat().GetRefObject(gmat.THRUSTER, thr_name)
        
        # Turn on thruster and set Spacecraft to maneuverable
        thruster.SetField("IsFiring", True)
        self.sat_wrap.getSat().IsManeuvering(True)

        # Add the thruster's force to the Propagator
        prop.getPropagator().AddForce(fm.getBurnForce(axis))

        # Update the Propagator's satellite reference
        prop.getPropagator().AddPropObject(self.sat_wrap.getSat())

        # Update the latest internal values for the propagator
        prop.prepareInternals()
        
        # Collect new numerical integrator and ForceModel for modeling
        gator = prop.getIntegrator()
        return gator

    def satEnginesOff(self, axis:str = "coast"):
        """During the scenario when it is time to turn off the thrusters, this function turns off the corresponding thruster 
        and returns the satellite object to a "coast" mode. """
        
        # Only at the beginning of the scenario should no axis be given. This sets the scene for the upcoming propagation
        if axis == "coast":
            gator = self.prop_wrap[axis].getIntegrator()
            return gator
        
        # Otherwise should any axis be given, perform the following:


        # Collect the Propagator and ForceModel for coast period
        prop = self.prop_wrap["coast"]

        # Update the latest internal values for the propagator
        prop.prepareInternals()
        
        # Collect the thruster we want to turn off
        thr_name = self.sat_wrap.thrusters[axis].GetName()
        thruster = self.sat_wrap.getSat().GetRefObject(gmat.THRUSTER, thr_name)

        # Turn off the thruster and set the spacecraft to be no longer maneuverable
        thruster.SetField("IsFiring", False)
        self.sat_wrap.getSat().IsManeuvering(False)

        # Update the spacecraft reference in the propagator
        prop.getPropagator().AddPropObject(self.sat_wrap.getSat())

        # Update the latest internal values for the propagator
        prop.prepareInternals()

        # Collect the new numerical integrator and ForceModel for simulation
        gator = prop.getIntegrator()
        return gator