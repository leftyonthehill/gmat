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
    object_type : str
        What type of objects to be created.
    thrust_axis : str
        Actively maneuvering thrust axis.
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

    def __init__(self, object_type: str):
        """ Create GMAT objects for the provided object type.
        
        By default, a new satellite object is set to non-maneuverable. 

        Parameters
        ----------
        object_type : str
            Describes what objects need to made.
            
        Raises
        ------
        ValueError
            Checks to see if the provided `object_type` is either "truth"
            or "reference".
        """

        if all([object_type.lower() != "truth",
                object_type.lower() != "reference"]):
            raise ValueError(
                "Object typing can only be 'Truth' or 'Reference'.")

        self.object_type = object_type
        self.thrust_axis = "coast"

        # Create object wrappers
        self.sat_wrap = Satellite(f"{object_type}_Sat")
        self.sat_gmat = self.sat_wrap.sat
        self.fm_wrap = {self.thrust_axis: ForceModel(object_type)}
        self.prop_wrap = {self.thrust_axis: Propagator(object_type)}

        # For the coasting period, assign the corresponding forces and
        # satellite to the propagator
        self.fm_wrap[self.thrust_axis].set_forces_to_propagate(object_type)
        self.prop_wrap[self.thrust_axis].set_integrator()
        self.prop_wrap[self.thrust_axis].set_fm(
            self.fm_wrap[self.thrust_axis].fm)
        self.prop_wrap[self.thrust_axis].set_sat(self.sat_gmat)

    def set_maneuverable(self) -> None:
        """
        If a satellite is determined to be maneuverable, this function
        calls the 'sat_wrap' function, setManeuverable(), and creates
        ForceModels and Propagators for each thruster attached to the
        vehicle.
        """

        self.sat_wrap.setManeuverable()

        thuster_axes = self.sat_wrap.thrusters.keys()
        for ax in thuster_axes:
            self.fm_wrap[ax] = ForceModel(f"{self.object_type}_{ax}")
            self.fm_wrap[ax].set_forces_to_propagate(self.object_type)
            fm_gmat = self.fm_wrap[ax].fm

            self.prop_wrap[ax] = Propagator(f"{self.object_type}_{ax}")
            self.prop_wrap[ax].set_integrator()
            self.prop_wrap[ax].set_fm(fm_gmat)
            self.prop_wrap[ax].set_sat(self.sat_gmat)

    def set_burn_forces(self) -> None:
        """
        **After initializing the GMAT scenario**, call this function to
        assign the ForceModels to a GMAT BurnForce object for each
        thruster axis.

        Returns
        -------
        None
        """

        thuster_axes = self.sat_wrap.thrusters.keys()
        for ax in thuster_axes:
            self.fm_wrap[ax].create_burn_forces(self.sat_wrap, ax)
            self.prop_wrap[ax].prop_gmat.PrepareInternals()

    def prepare_propagators(self) -> None:
        """
        Prepare the internals of all associated propagators with this
        Satellite.

        Returns
        -------
        None
        """
        for prop in self.prop_wrap.values():
            prop.prop_gmat.PrepareInternals()

    def thruster_on(self, axis:str) -> gmat.RungeKutta89:
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
        gmat.RungeKutta89
            The gmat object representing the numerical integrator which
            contains all the forces to be modeled.
        """

        # Collect the Propagator and ForceModel for the new axis
        self.thrust_axis = axis
        prop = self.prop_wrap[self.thrust_axis]
        fm = self.fm_wrap[self.thrust_axis]

        # Update the latest internal values for the propagator
        prop.prop_gmat.PrepareInternals()

        # Collect the thruster we want to fire
        thr_name = self.sat_wrap.thrusters[self.thrust_axis].GetName()
        thruster = self.sat_gmat.GetRefObject(
            gmat.THRUSTER, thr_name)

        # Turn on thruster and set Spacecraft to maneuverable
        thruster.SetField("IsFiring", True)
        self.sat_wrap.getGMATSat().IsManeuvering(True)

        # Add the thruster's force to the Propagator
        prop.prop_gmat.AddForce(fm.burn_force[self.thrust_axis])

        # Update the Propagator's satellite reference
        prop.prop_gmat.AddPropObject(self.sat_gmat)

        # Update the latest internal values for the propagator
        prop.prop_gmat.PrepareInternals()

        # Collect new numerical integrator and ForceModel for modeling
        integrator = prop.prop_gmat.GetPropagator()
        return integrator

    def thruster_off(self, axis:str) -> gmat.RungeKutta89:
        """ Turn off any active thrusters on the Satellite.

        Based on the provided axis, turn off the corresponding
        thrusters.

        Parameters
        ----------
        axis : str
            The thruster axis we want to turn off.
        
        Returns
        -------
        gmat.RungeKutta89
            The gmat object representing the numerical integrator which
            contains all the forces to be modeled.
        """

        # Collect the Propagator and ForceModel for coast period
        self.thrust_axis = axis
        prop = self.prop_wrap["coast"]

        if self.thrust_axis != "coast":
            # Update the latest internal values for the propagator
            prop.prop_gmat.PrepareInternals()

            # Collect the thruster we want to turn off
            thr_name = self.sat_wrap.thrusters[self.thrust_axis].GetName()
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
        integrator = prop.prop_gmat.GetPropagator()
        return integrator
