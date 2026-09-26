""" Support class that creates the satellites for a station keeping
scenario. """

import datetime as dt

from load_gmat import gmat


class Satellite:
    """ Wrapper for a Spacecraft object in GMAT. 
    
    - If this is for the reference spacecraft, no other function calls
      are required. 
    - If this is for the maneuvering spacecraft (the truth state in a
      station keeping scenario), make sure to call
      `set_maneuverable()` to create thrusters along the +/-R, +/-I,
      +/-C axes in the RIC reference frame. If a different combination
      of thrusters is required, make sure to call
      `_set_ethruster(axis, engine_specs)` before `set_maneuverable()`.
      The +/-R, +/-I, +/-C thrust vectors are created if there are no
      other thrusters assigned to the vehicle.

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
        # Default parameters:
        #   DisplayStateType = Keplerian
        #   Area affected by solar radiation pressure | SRPArea = 6 m^2
        #   Coefficient of reflectivity | Cr = 1.8
        #   Area affected by atmospheric drag | DragArea = 5 m^2
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

    def set_sat_param(self, sat_physical_param: list[float]):
        """ Customize the physical parameters of the spacecraft.

        Parameters
        ----------
        sat_physical_param : list[float]
        - [0]
            a_d (Cross-sectional area exposed to atmospheric
            drag)
        - [1]
            a_r (Cross-sectional area exposed to solar radiation
            pressure)
        - [2]
            c_d (Coefficient of drag)
        - [3]       
            c_r (Coefficient of reflectivity)
        - [4]
            m (Satellite mass)        
        """

        a_d, a_r, c_d, c_r, m = sat_physical_param

        # Updating the physical parameters of the spacecraft
        self.sat.SetField("DragArea", a_d)
        self.sat.SetField("SRPArea", a_r)
        self.sat.SetField("Cd", c_d)
        self.sat.SetField("Cr", c_r)
        self.sat.SetField("DryMass", m)
        self.mass = m

    def set_keplerian_state(self, coes: list[float | str]):
        """ Set the spacecraft state vector using Keplerian elements.
        
        The provided list must contain an element for each classical
        orbital element and, optionally, the epoch associated with the
        state vector. If an epoch is included, it must be in the form
        of "dd mmm yyyy HH:MM:SS.SSS".

        Parameters
        ----------
        coes : list[float | str]
            - Semi-major axis, 
            - Eccentricity, 
            - Inclination, 
            - Right Ascension of the Ascending Node, 
            - Argument of Periapsis, 
            - True Anomaly,
            - State Vector Epoch ("dd mmm yyyy HH:MM:SS.SSS")
                            
        Raises
        ------
        ValueError
            If the provided list is not exactly 6 or 7 elements long.
        SyntaxError
            If the provided epoch string is not in the correct format.
        """

        if len(coes) < 6 or len(coes) > 7:
            raise ValueError("Incorrect amount of elements passed. There "
                             + "needs to be either 6 (COEs) or 7 (COEs + "
                             + "Epoch) elements. In this case "
                             + str(len(coes)) + " elements were passed.")

        # State vector setting
        a, e, i, raan, aop, f = coes[:6]

        self.sat.SetField("SMA", a)
        self.sat.SetField("ECC", e)
        self.sat.SetField("INC", i)
        self.sat.SetField("RAAN", raan)
        self.sat.SetField("AOP", aop)
        self.sat.SetField("TA", f) 

        # State vector epoch setting
        if len(coes) == 6:
            epoch = dt.datetime.today()
        else:
            epoch = coes[-1]

        if isinstance(epoch, dt.datetime):
            self.epoch = epoch.strftime("%d %b %Y 00:00:00.000")
        elif isinstance(epoch, str):
            self.epoch = epoch
        else:
            raise SyntaxError("Invalid date type. The epoch must be either a " \
                            "datetime.datetime object or a string")

        self.sat.SetField("DateFormat", "UTCGregorian")
        self.sat.SetField("Epoch", self.epoch)

    def set_cartesian_state(self, xyz: list):
        """ Set the spacecraft state vector using Cartesian elements.
                
        The provided list must contain an element for each Cartesian
        element from the ECI frame and, optionally, the epoch
        associated with the state vector. If an epoch is included,
        it must be in the form of "dd mmm yyyy HH:MM:SS.SSS".

        Parameters
        ----------
        xyz : list[float | str]
            - X, 
            - Y, 
            - Z, 
            - V_X, 
            - V_Y, 
            - V_Z,
            - State Vector Epoch ("dd mmm yyyy HH:MM:SS.SSS")
                            
        Raises
        ------
        ValueError
            If the provided list is not exactly 6 or 7 elements long.
        SyntaxError
            If the provided epoch string is not in the correct format.
        """

        if len(xyz) < 6 or len(xyz) > 7:
            raise ValueError("Incorrect amount of elements passed. There "
                             + "needs to be either 6 or 7 elements. "
                             + "In this case " + str(len(xyz))
                             + " elements were passed.")

        x, y, z, xdot, ydot, zdot = xyz[:6]

        self.sat.SetField("DisplayStateType", "Cartesian")
        self.sat.SetField("X", x)
        self.sat.SetField("Y", y)
        self.sat.SetField("Z", z)
        self.sat.SetField("VX", xdot)
        self.sat.SetField("VY", ydot)
        self.sat.SetField("VZ", zdot)

        # State vector epoch setting
        if len(xyz) == 6:
            epoch = dt.datetime.today()
        else:
            epoch = xyz[-1]

        if isinstance(epoch, dt.datetime):
            self.epoch = epoch.strftime("%d %b %Y 00:00:00.000")
        elif isinstance(epoch, str):
            self.epoch = epoch
        else:
            raise SyntaxError("Invalid date type. The epoch must be either a " \
                            "datetime.datetime object or a string")

        self.sat.SetField("DateFormat", "UTCGregorian")
        self.sat.SetField("Epoch", self.epoch)

        self.sat.SetField("DisplayStateType", "Keplerian")

    def set_maneuverable(self):
        """ Prepare the components needed to make the spacecraft
        maneuverable.

        If custom components have not been created for this spacecraft,
        create a generic electric fuel tank, nuclear power system, and
        thruster for each axis of the RIC frame.
        """

        # Check for missing tank
        if self.sat.GetField("Tanks") == "{}":
            self._set_etank()

        # Name of fuel tank for thrusters
        etank_name = self.sat.GetField("Tanks")[1:-1]

        # Check for missing power supply
        if self.sat.GetField("PowerSystem") == "":
            self._set_power_system()

        # Check for missing thrusters
        if not self.thrusters:
            thruster_axes = {
                "R+": (0.2, 3000), 
                "R-": (0.2, 3000), 
                "I+": (0.2, 3000), 
                "I-": (0.2, 3000), 
                "C+": (0.2, 3000), 
                "C-": (0.2, 3000)}

            # Create a thruster for each thruster direction 
            for ax, thruster_param in thruster_axes.items():
                self._set_ethruster(ax, thruster_param)

        # String together all onboard thrusters and assign each to main fuel
        # tank.
        thruster_names = [i.GetName() for i in self.thrusters.values()]
        thruster_array = "{" + ", ".join(thruster_names) + "}"
        self.sat.SetField("Thrusters", thruster_array)
        for i in self.thrusters.values():
            i.SetField("Tank", etank_name)

    def get_keplerian_state(self) -> list[float]:
        """ Return the Keplerian state vector of the spacecraft.
        
        Returns
        -------
        list[float]
            The spacecraft's current Keplerian state vector.
        """

        # state comes in the form of GMAT R6Vector that is not human readable
        # in Python. Converting the vector to a list makes debugging
        # spacecraft states much easier
        state = self.sat.GetKeplerianState()
        x = [float(state[i]) for i in range(6)]
        return x

    def get_cartesian_state(self) -> list[float]:
        """ Returns the Cartesian state vector of the spacecraft. 
        
        Returns
        -------
        list[float]
            The spacecraft's current Cartesian state vector.
        """

        # state comes in the form of GMAT R6Vector that is not human readable
        # in Python. Converting the vector to a list makes debugging
        # spacecraft states much easier
        state = self.sat.GetCartesianState()
        x = [float(state[i]) for i in range(6)]
        return x

    def _set_etank(self, mass: float = 30):
        """
        Create the fuel tank that the onboard electric thrusters will
        use.
        
        Parameters
        ----------
        mass : float, default: 30 kg
            How much gas is in the tank.
        """

        # create GMAT electric fuel tank
        etank = gmat.Construct("ElectricTank",
                                    f"{self.sat.GetName()}_tank")
        etank.SetField("FuelMass", mass)

        # add the tank mass to the satellite's total mass
        self.mass += mass

        # Assign the tank to the spacecraft
        self.sat.SetField("Tanks", etank.GetName())

    def _set_ethruster(self, axis:str = "I+",
                     engine_specs: tuple = (0.2, 3000)):
        """ Create a thruster on the spacecraft.
        
        Parameters
        ----------
        axis : str, default="I+"
            Which axis in the Radial(R+/-)/In-track(I+/-)/Cross-Track
            (C+/-) coordinate frame (RIC) will this thruster fire.
        engine_specs : tuple, default=(0.2 N, 3000 sec)
            Two element tuple containing the engine force and ISP,
            respectively.
        
        Raises
        ------
        ValueError
            Checks to see if the provided axis is valid in the RIC
            frame.
        """

        if axis not in ("R+", "R-", "I+", "I-", "C+", "C-"):
            raise ValueError(axis + " axis not found. Acceptable values are:"
                             + "R+, R-, I+, I-, C+, C-")

        # Create the thruster
        thrust = engine_specs[0]
        isp = engine_specs[1]
        ethruster = gmat.Construct("ElectricThruster",
                                   self.sat.GetName() + "_electric_thruster_"
                                    + axis)
        ethruster.SetField("Isp", isp)
        ethruster.SetField("ConstantThrust", thrust)
        ethruster.SetField("ThrustModel", "ConstantThrustAndIsp")
        ethruster.SetField("DecrementMass", True)

        # Add to the wrapper's dict of thrusters
        self.thrusters[axis] = ethruster
        self.accelerations[axis] = thrust / self.mass

        # Based on the thruster axis, assign its thrust direction
        self._set_ethruster_direction(axis)

    def _set_ethruster_direction(self, axis):
        """ Assign the thruster's direction.
        
        The spacecraft's thrusters are created referencing the
        spacecraft's Velocity/Normal/Bi-Normal (VNB) reference frame.
        A conversion map is used to relate the RIC and VNB frames.

        During the development of this class, it was discovered that
        thrusters in the N axis of the VNB frame (or the C axis of the
        RIC frame) could not be assigned a thrust vector direction
        aligned exactly with the axis. The smallest deviations that
        could be included in the thrust vector were +/- 1e-5 in the
        supplementary axes.

        Parameters
        ----------
        axis : str
            Which RIC axis the thruster fires along.
        
        Raises
        ------
        ValueError
            Checks to see if the provided axis is valid in the RIC
            frame.
        """

        if axis not in ("R+", "R-", "I+", "I-", "C+", "C-"):
            raise ValueError(axis + "axis not found. Acceptable values are:"
                             + " R+, R-, I+, I-, C+, C-")


        # The following map is used to correlate the RIC frame to the VNB
        # frame
        axis_map = {
            "R+": [0, 0, 1], 
            "R-": [0, 0, -1], 
            "I+": [1, 0, 0], 
            "I-": [-1, 0, 0], 
            "C+": [1e-5, 1, 1e-5],
            "C-": [-1e-5, -1, -1e-5],
            }

        # Based on the provided axis, choose the correct mapping
        thruster_direction = axis_map[axis]

        # Assign the directions in the VNB frame
        v = thruster_direction[0]
        self.thrusters[axis].SetField("ThrustDirection1", v)

        n = thruster_direction[1]
        self.thrusters[axis].SetField("ThrustDirection2", n)

        b = thruster_direction[2]
        self.thrusters[axis].SetField("ThrustDirection3", b)

    def _set_power_system(self, power_system_type: str="Nuclear", kw: float=20):
        """ Create the power supply for the spacecraft.
        
        While the power system type of spacecraft is commonly "Solar"
        power, this function sets the default type to "Nuclear". This
        was chosen because in GMAT there is no way to create a battery
        and maneuver a solar powered spacecraft while eclipsed by the
        Earth. To prevent missed maneuver opportunities, "Nuclear" was
        chosen to be the default power supply type. Future versions of
        this project will include an eclipse checker to verify viable
        maneuver windows.

        Parameters
        ----------
        power_system_type : str, default="Nuclear"
            Type of power supply for the spacecraft. GMAT only
            recognizes "Nuclear" or "Solar".
        kw : float, default=20 KW
            How much initial power the power supply will have at the
            spacecraft's epoch.
            
        Raises
        ------
        ValueError
            Check to make sure 1 of the 2 acceptable power supply types
            is provided.
        """

        if power_system_type != "Nuclear" and power_system_type != "Solar":
            raise ValueError(power_system_type + " is not a valid power "
                             + "system type in GMAT. Please select from "
                             + "either 'Nuclear' or 'Solar'.")
        power_system = gmat.Construct(power_system_type + "PowerSystem",
                                          self.sat.GetName() + "_"
                                          + power_system_type + "Power")

        power_system.SetField("InitialMaxPower", kw)
        power_system.SetField("InitialEpoch", self.epoch)

        self.sat.SetField("PowerSystem", power_system.GetName())
