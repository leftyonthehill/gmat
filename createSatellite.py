""" Support class that creates the satellites for a station keeping
scenario. """

import datetime as dt

from load_gmat import gmat


class Satellite:
    """ Wrapper for a Spacecraft object in GMAT. 
    
    - If this is for the reference spacecraft, no other function calls
      are required. 
    - If this is for the maneuvering spacecraft (the truth state in a
      station keeping scenario), make sure to call setManeuverable() to
      create thrusters along the +/-R, +/-I, +/-C axes in the RIC
      reference frame. If a different combination of thrusters are
      required, make sure to call setEThruster(axis, engineSpecs)
      before calling setManeuverable(). The +/-R, +/-I, +/-C thrust
      vectors are created if there are no other thrusters assigned to
      the vehicle.

    Attributes
    ----------
    epoch : str
        String of state vector epoch.
    mass : float
        Spacecraft mass in kg.
    sat : gmat_py.Spacecraft
        GMAT Spacecraft Object.
    thrusters : {str: gmat_py.ElectricThruster}
        Dict containing any and all of the thrusters assigned to this 
        spacecraft associated with the corresponding thruster axis.
    accelerations : {str: float}
        Dict containing the acceleration of the spacecraft while
        thrusting with the corresponding thruster axis.
    """
    
    def __init__(self, sat_name: str):
        """ Initialize the Satellite wrapper.
        
        Parameters
        ----------
        sat_name : str
            Name of the Satellite.
        """

        self.epoch = ""
        self.thrusters = {}
        self.accelerations = {}

        # Spacecraft creation and assigning its physical parameters
        #
        # Default parameters (match SetField calls below):
        #   DisplayStateType = Keplerian
        #   Area effected by solar radiation pressure | SRPArea = 6 m^2
        #   Coefficient of reflectivity | Cr = 1.8
        #   Area effected by atmospheric drag | DragArea = 5 m^2
        #   Coefficient of drag | Cd = 1.5
        #   Satellite dry mass | DryMass = 900 kg
        self.sat = gmat.Construct("Spacecraft", sat_name)
        self.sat.SetField("DisplayStateType", "Keplerian")
        self.sat.SetField("SRPArea", 6)
        self.sat.SetField("Cr", 1.8)
        self.sat.SetField("DragArea", 5)
        self.sat.SetField("Cd", 1.5)
        self.sat.SetField("DryMass", 900)
        self.mass = 900

        # Spacecraft coordinate system reference
        self.sat.SetField("CoordinateSystem", "EarthMJ2000Eq")
