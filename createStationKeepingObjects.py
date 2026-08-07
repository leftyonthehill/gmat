""" Support class that combines the satellite, force model, and propagator
objects to support the study of a station keeping scenario. """

from createForceModel import ForceModel
from createPropagator import Propagator
from createSatellite import Satellite
from load_gmat import gmat

class StationKeepingObjects:
    """ Creates GMAT objects for a station keeping scenario.
    
    Rather than only storing the information for one state at a time
    (either coasting or thrusting along one of the spacecraft's
    thruster directions), the necessary ForceModel and Propagator
    wrappers are stored under their respective "coast" or thruster
    axis keys (such as "R-", "I+", "C+" for example).

    Attributes
    ----------
    objType : str
        What type of objects to be created.
    state : list
        List of the Keplerian/Cartesian state vector assigned to the
        spacecraft.
    sat_wrap : Satellite
        GMAT Spacecraft wrapper.
    fm_wrap : dict
        Dict containing the GMAT ForceModel wrappers for the coasting
        period and any axes with thrusters.
    prop_wrap : dict
        Dict containing the GMAT Propagator wrapper for the coasting
        period and any axes with thrusters.
    """

    def __init__(self, objectType: str):
        """ Create GMAT objects for the provided object type.
        
        By default, a new satellite object is set to non-maneuverable. 

        Parameters
        ----------
        objectType : str
            Describes what objects need to made.
            
        Raises
        ------
        ValueError
            Checks to see if the provided objectType is either "truth"
            or "reference".
        """

        if all([objectType.lower() != "truth",
                objectType.lower() != "reference"]):
            raise ValueError(
                "Object typing can only be 'Truth' or 'Reference'.")

        self.objType = objectType
        self.thrustAxis = "coast"

        # Create object wrappers
        self.sat_wrap = Satellite(f"{objectType}_Sat")
        self.sat_gmat = self.sat_wrap.sat
        self.fm_wrap = {self.thrustAxis: ForceModel(objectType)}
        self.prop_wrap = {self.thrustAxis: Propagator(objectType)}

        # For the coasting period, assign the corresponding forces and
        # satellite to the propagator
        self.fm_wrap[self.thrustAxis].setForcesToPropagate(objectType)
        self.prop_wrap[self.thrustAxis].setIntegrator()
        self.prop_wrap[self.thrustAxis].setFM(
            self.fm_wrap[self.thrustAxis].fm)
        self.prop_wrap[self.thrustAxis].setSat(self.sat_gmat)
    
    def setManeuverable(self):
        """
        If a satellite is determined to be maneuverable, this function
        calls the 'sat_wrap' function, setManeuverable(), and creates
        ForceModels and Propagators for each thruster attached to the
        vehicle.
        """

        self.sat_wrap.setManeuverable()
        
        # For each thruster key in sat_wrap's thruster dict, create its own
        # Propagator and ForceModel.
        for ax in self.sat_wrap.thrusters.keys():
            self.fm_wrap[ax] = ForceModel(f"{self.objType}_{ax}")
            self.fm_wrap[ax].setForcesToPropagate(self.objType)
            fm_gmat = self.fm_wrap[ax].fm

            self.prop_wrap[ax] = Propagator(f"{self.objType}_{ax}")
            self.prop_wrap[ax].setIntegrator()
            self.prop_wrap[ax].setFM(fm_gmat)
            self.prop_wrap[ax].setSat(self.sat_gmat)
    
    def setBurnForces(self):
        """
        After initializing the GMAT scenario, call this function to
        assign the ForceModels to a GMAT BurnForce object.
        """
        for ax in self.sat_wrap.thrusters.keys():
            self.fm_wrap[ax].createBurnForces(self.sat_wrap, ax)
            self.prop_wrap[ax].prop_gmat.PrepareInternals()

    def preparePropInternal(self):
        """
        Prepare the internals of all associated propagators with this
        Satellite.
        """
        for prop in self.prop_wrap.values():
            prop.prop_gmat.PrepareInternals()

    def satEnginesOn(self, axis:str) -> gmat.RungeKutta89:
        """ Turn the thrusters on of the given axis.

        For the provided value of 'axis', update the corresponding
        Propagator with the latest Satellite state and the thruster's
        force.
        
        Parameters
        ----------
        axis : str
            The corresponding axis in which the thrusters fire.

        Returns
        -------
        gmat.RungeKutta90
            The gmat object representing the numerical integrator which
            contains all the forces to be modeled.
        """
        
        
        # Collect the Propagator and ForceModel for the new axis
        self.thrustAxis = axis
        prop = self.prop_wrap[self.thrustAxis]
        fm = self.fm_wrap[self.thrustAxis]

        # Update the latest internal values for the propagator
        prop.prop_gmat.PrepareInternals()

        # Collect the thruster we want to fire
        thr_name = self.sat_wrap.thrusters[self.thrustAxis].GetName()
        thruster = self.sat_gmat.GetRefObject(
            gmat.THRUSTER, thr_name)
        
        # Turn on thruster and set Spacecraft to maneuverable
        thruster.SetField("IsFiring", True)
        self.sat_wrap.getGMATSat().IsManeuvering(True)

        # Add the thruster's force to the Propagator
        prop.prop_gmat.AddForce(fm.burnForce[self.thrustAxis])

        # Update the Propagator's satellite reference
        prop.prop_gmat.AddPropObject(self.sat_gmat)

        # Update the latest internal values for the propagator
        prop.prop_gmat.PrepareInternals()
        
        # Collect new numerical integrator and ForceModel for modeling
        gator = prop.prop_gmat.GetPropagator()
        return gator

    def satEnginesOff(self, axis:str) -> gmat.RungeKutta89:
        """ Turn off any active thrusters on the Satellite.

        Based on the provided axis, turn off the corresponding
        thrusters.

        Parameters
        ----------
        axis : str
            The thruster axis we want to turn off.
        
        Returns
        -------
        gmat.RungeKutta90
            The gmat object representing the numerical integrator which
            contains all the forces to be modeled.
        """

        # Collect the Propagator and ForceModel for coast period
        self.thrustAxis = axis
        prop = self.prop_wrap["coast"]

        if self.thrustAxis != "coast":
            # Update the latest internal values for the propagator
            prop.prop_gmat.PrepareInternals()

            # Collect the thruster we want to turn off
            thr_name = self.sat_wrap.thrusters[self.thrustAxis].GetName()
            thruster = self.sat_gmat.GetRefObject(gmat.THRUSTER, thr_name)

            # Turn off the thruster and set the spacecraft to be no longer
            # maneuverable.
            thruster.SetField("IsFiring", False)
            self.sat_gmat.IsManeuvering(False)

            # Update the spacecraft reference in the propagator
            prop.prop_gmat.AddPropObject(self.sat_gmat)

        # Update the latest internal values for the propagator
        prop.prop_gmat.PrepareInternals()

        # Collect the new numerical integrator and ForceModel for simulation
        gator = prop.prop_gmat.GetPropagator()
        return gator
