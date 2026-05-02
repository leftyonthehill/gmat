from load_gmat import *

import datetime as dt
import numpy as np

class Satellite:
    """
    This class is a wrapper for a Spacecraft object in GMAT. If this is for the 
    reference satellite, no other work is required. If this is for the truth 
    satellite, make sure to call the setManeuverable() function to create thrusters 
    along the +/-R, +/-I, +/-C axes in the RIC reference frame. If a different 
    combination of thrusters are required, make sure to call setEThruster(axis, 
    engineSpecs) before calling setManeuverable().
    
    Inputs during intitialization:
        - satName (str): Any name the user would like to call their satellite
    
      Variables:
        - sat: GMAT Spacecraft Object
        - epoch: string of today's date set to 1200 UTC
        - thrusters: dict containing any and all of the thrusters assigned to 
          this satellite
        - etank: GMAT Electric Tank object
        - powerSystem: GMAT Solar/Nuclear Power System
        - mu : Earth's GM
    """
    
    def __init__(self, satName: str):
        """Initialize the Satellite wrapper"""
        self.sat = None
        self.epoch = None
        self.thrusters = {}
        self.etank = None
        self.powerSystem = None

        self.mu = 3.986e5

        self._setSatParam(satName)
    
    def _setSatParam(self, satName: str):
        """ ** Internal function**
        Create a satellite object using the following baseline parameters:
        - SRPArea: Area of the spacecraft exposed to the solar radiation pressure
        - Cr: Spacecraft's coefficient of reflectivity
        - DragArea: Area of the spacecraft exposed to atmospheric drag
        - Cd: Spacecraft's coefficient of drag
        - Mass
        - Epoch: Date and time associated with state vector
        
        """
        self.sat = gmat.Construct("Spacecraft", satName)
        
        # Spacecraft physical parameters
        self.sat.SetField("SRPArea", 6)
        self.sat.SetField("Cr", 1.8)
        self.sat.SetField("DragArea", 9)
        self.sat.SetField("Cd", 2.2)
        self.sat.SetField("DryMass", 800)

        # Spacecraft time reference
        today = dt.datetime.today()
        self.epoch = today.strftime("%d %b %Y 12:00:00.000")
        self.sat.SetField("DateFormat", "UTCGregorian")
        self.sat.SetField("Epoch", self.epoch)
        self.sat.SetField("CoordinateSystem", "EarthMJ2000Eq")
        self.sat.SetField("DisplayStateType", "Keplerian")

    def getSat(self):
        """Returns the GMAT satellite object"""
        return self.sat
    
    
    def getCartesianState(self):
        """Returns the cartesian state vector of the spacecraft"""
        return self.sat.GetCartesianState()
    
    def getKeplerianState(self):
        """Return the keplerian state vector of the satellite"""
        return self.sat.GetKeplerianState()
    
    def getSMAFromEnergy(self):
        """Returns the semi-major axis based on the current position and velocity vectors"""
        rv = self.getCartesianState()
        r = np.linalg.norm(rv[:3])
        v = np.linalg.norm(rv[3:])

        specificEnergy = v**2 / 2 - self.mu / r
        sma = -self.mu / (2 * specificEnergy)
        return sma

    def setOrbitElements(self, coes: list):
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

        Raises:
            - ValueError: If the provided list is not exactly 6 elements long.
        """
        if len(coes) != 6:
            raise ValueError("Incorrect amount of orbital elements passed. There needs to be exactly 6")
        
        a, e, i, raan, aop, f = coes

        self.sat.SetField("SMA", a)
        self.sat.SetField("ECC", e)
        self.sat.SetField("INC", i)
        self.sat.SetField("RAAN", raan)
        self.sat.SetField("AOP", aop)
        self.sat.SetField("TA", f) 
    
    def setCartesianState(self, xyz: list):
        """Provided a 6-element list of cartesian elements, assign the state vector to the spacecraft
        
        Inputs:
            - coes (list):  [
                            X, 
                            Y, 
                            Z, 
                            X_dot, 
                            Y_dot, 
                            Z_dot
                            ]

        Raises:
            - ValueError: If the provided list is not exactly 6 elements long.
        """
        if len(xyz) != 6:
            raise ValueError("Incorrect amount of cartesian elements passed. There needs to be exactly 6")
        
        x, y, z, xdot, ydot, zdot = xyz

        self.sat.SetField("X", x)
        self.sat.SetField("Y", y)
        self.sat.SetField("Z", z)
        self.sat.SetField("VX", xdot)
        self.sat.SetField("VY", ydot)
        self.sat.SetField("VZ", zdot)
        
        self.sat.SetField("DisplayStateType", "Cartesian")
        self.sat.SetField("DisplayStateType", "Keplerian") 


    def setETank(self, mass: float = 200):
        """Create the fuel tank that the onboard electric thrusters will use.
        
        Inputs:
            - mass (float): How much gas is in the tank
        """

        self.etank = gmat.Construct("ElectricTank", f"{self.sat.GetName()}_tank")
        self.etank.SetField("FuelMass", mass)
        
        # Assign the tank to the spacecraft
        self.sat.SetField("Tanks", self.etank.GetName())
    
    def setEThruster(self, axis:str = "I+", engineSpecs: tuple = (0.2, 3000)):
        """Create the thrusters to go onto the spacecraft.
        
        Inputs:
            - axis (str): Which axis in the Radial/In-track/Cross-Track (RIC) will this thruster fire
            - engineSpecs (tuple): Two element tuple containing the engine force and ISP, respectively
        
        Raises:
            - ValueError: Checks to see if the provided axis is valid in the RIC frame
        """


        if axis not in ("R+", "R-", "I+", "I-", "C+", "C-"):
            raise ValueError(f"{axis} axis not found. Acceptable values are: R+, R-, I+, I-, C+, C-")
        
        thrust = engineSpecs[0]
        isp = engineSpecs[1]

        ethruster = gmat.Construct("ElectricThruster",f"{self.sat.GetName()}_electric_thruster_{axis}")
        ethruster.SetField("Isp", isp)
        ethruster.SetField("ConstantThrust", thrust)
        
        # To simplify the model, assume a constant thrust and no fuel depletion
        ethruster.SetField("ThrustModel", "ConstantThrustAndIsp")
        ethruster.SetField("DecrementMass", False)

        # Add to the wrapper's dict of thrusters
        self.thrusters[axis] = ethruster

        # Based on the thruster axis, assign its corresponding axis
        self.setEThrusterDirection(axis)

    def setEThrusterDirection(self, axis):
        """Assign the thruster's direction
        
        Inputs:
            - axis (str): Which axis in the RIC will this thruster fire
        
        Raises:
            - ValueError: Checks to see if the provided axis is valid in the RIC frame
        """
        if axis not in ("R+", "R-", "I+", "I-", "C+", "C-"):
            raise ValueError(f"{axis} axis not found. Acceptable values are: R+, R-, I+, I-, C+, C-")
        
        # The spacecraft's thrusters are created referencing the spacecraft's Velocity/Normal/Bi-normal (VNB) reference frame
        # so a map is needed to correlate the RIC frame to the VNB frame
        axisMap = {
            "R+": [0, 0, -1], 
            "R-": [0, 0, 1], 
            "I+": [1, 0, 0], 
            "I-": [-1, 0, 0], 
            "C+": [1e-5, 1, 1e-5], # This direction array should read [0, 1, 0], but GMAT will not compile this thruster. 1e-5 is the smallest value to compile the scenario
            "C-": [-1e-5, -1, -1e-5]} # This direction array should read [0, -1, 0], but GMAT will not compile this thruster. -1e-5 is the smallest value to compile the scenario
        
        # Based on the provided axis, choose the correct mapping
        mags = axisMap[axis]
        
        
        # Assign the directions in the VNB frame
        v = mags[0]
        self.thrusters[axis].SetField("ThrustDirection1", v)

        n = mags[1]
        self.thrusters[axis].SetField("ThrustDirection2", n)
        
        b = mags[2]
        self.thrusters[axis].SetField("ThrustDirection3", b)

    def setPowerSystem(self, powerSystemType: str = "Nuclear", kw:float = 20):
        """Create the power supply for the spacecraft
        
        Inputs:
            - powerSystemType (str): Type of power supply for the spacecraft. GMAT only recognizes 'Nuclear' or 'Solar'
            - kw (float): How much initial power the power supply will have at the spacecraft's epoch
            
        Raises:
            - ValueError: Check to make sure 1 of the 2 acceptable power supply types is provided.
        """
        
        if powerSystemType != "Nuclear" and powerSystemType != "Solar":
            raise ValueError(f"{powerSystemType} is not a valid power system type in GMAT. Please select from either 'Nuclear' or 'Solar'")
        self.powerSystem = gmat.Construct(f"{powerSystemType}PowerSystem", f"{self.sat.GetName()}_{powerSystemType}Power")
        
        self.powerSystem.SetField("InitialMaxPower", kw)
        self.powerSystem.SetField("InitialEpoch", self.epoch)

        self.sat.SetField("PowerSystem", self.powerSystem.GetName())
    
    def setManeuverable(self):
        """Creates the fuel tank, power supply, and thrusters for a maneuverable spacecraft.
        
        These components can be created individually with specific values first"""

        # Tank creation
        if self.etank is None:
            self.setETank()
    
        # Power supply creation
        if self.powerSystem is None:
            self.setPowerSystem()
    
        # Thruster creation
        if self.thrusters == {}:
            # Default thruster combination:
            # - 1 thruster in both R+/- directions
            # - 1 thruster in both I+/- directions
            # - 1 thruster in both C+/- directions
            thrusterAxes = {
                "R+": (0.2, 3000), 
                "R-": (0.2, 3000), 
                "I+": (0.2, 3000), 
                "I-": (0.2, 3000), 
                "C+": (0.2, 3000), 
                "C-": (0.2, 3000)}
            
            # Create a thruster for each thruster direction and assign it to the onboard tank 
            for ax, thrParam in thrusterAxes.items():
                self.setEThruster(ax, thrParam)
                self.thrusters[ax].SetField("Tank", self.etank.GetName())
            
            # String together a list of the thrusters' names to assign them all to the satellite
            thrusterNames = [i.GetName() for i in self.thrusters.values()]
            thrusterArray = "{" + ", ".join(thrusterNames) + "}"
            self.sat.SetField("Thrusters", thrusterArray)
        else:
            # If custom thrusters were used, string together their names and assign them all to the satellite
            thrusterNames = [i.GetName() for i in self.thrusters.values()]
            thrusterArray = "{" + ", ".join(thrusterNames) + "}"
            self.sat.SetField("Thrusters", thrusterArray)
            [i.SetField("Tank", self.etank.GetName()) for i in self.thrusters.values()]

        

    