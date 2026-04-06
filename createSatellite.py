from load_gmat import *

import datetime as dt
import numpy as np

class Satellite:
    """This class is a wrapper for a Spacecraft object in GMAT.

    
    Inputs during intitialization:
        - satName (str): Any name the user would like to call their satellite
    
    - If this is for the reference satellite, no other work is required.
    - If this is for the truth satellite, make sure to call the 
      setManeuverable() function to create thrusters along the +/-R, +/-I, +/-C 
      axes in the RIC reference frame. If a different combination of thrusters 
      are required, make sure to call setEThruster(axis, engineSpecs) before 
      calling setManeuverable().
    
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
        """Initialize the Satellite wrapper
        
        Inputs:
            - satName (str): Can be any name
        """

        self.sat = None
        self.epoch = None
        self.thrusters = {}
        self.etank = None
        self.powerSystem = None

        self.mu = 3.986e5

        self._setSatParam(satName)
    
    def _setSatParam(self, satName: str):
        """Local function to create and assign GMAt spacecraft parameters. SHOULD NOT BE CALLED ANYWHERE ELSE.
        
        Inputs:
            - satName (str): Can be any name
        """

        self.sat = gmat.Construct("Spacecraft", satName)

        # Default epoch for all satellites will be 1200 UTC of the day the code is run
        today = dt.datetime.today()
        self.epoch = today.strftime("%d %b %Y 12:00:00.000")
        self.sat.SetField("DateFormat", "UTCGregorian")
        self.sat.SetField("Epoch", self.epoch)

        # Default satellite areas
        self.sat.SetField("SRPArea", 6)
        self.sat.SetField("DragArea", 9)

        # Default reflective and drag coefficients
        self.sat.SetField("Cr", 1.8)
        self.sat.SetField("Cd", 2.2)

        # Default mass of the satellite
        self.sat.SetField("DryMass", 800)
        
        self.sat.SetField("CoordinateSystem", "EarthMJ2000Eq")
        self.sat.SetField("DisplayStateType", "Keplerian")

    def getSat(self):
        """Return the satellite
        
        Returns:
            - GMAT Spacecraft object
        """

        return self.sat
    
    def getCartesianState(self):
        """Return the satellite's current cartesian state vector
        
        Returns:
            - Cartesian State (list() with length 6)
                + rX
                + rY
                + rZ
                + vX
                + vY
                + vZ
        """

        return self.sat.GetCartesianState()
    
    def getKeplerianState(self):
        """Return the satellite's current keplerian state vector
        
        Returns:
            - Keplerian State (list() with length 6)
                + Semi-major axis
                + Eccentricity
                + Inclination
                + Right Ascension of the Ascending Node
                + Argument of Periapsis
                + True Anomaly
        """

        return self.sat.GetKeplerianState()
    
    def getSMAFromEnergy(self):
        """Return current semi-major axis based on position and velocity vectors"""
        rv = self.getCartesianState()
        r = np.linalg.norm(rv[:3])
        v = np.linalg.norm(rv[3:])

        specificEnergy = v**2 / 2 - self.mu / r
        sma = -self.mu / (2 * specificEnergy)
        return sma

    def setOrbitElements(self, coes: list):
        """
        Provided a list of classical orbital elements, this method assigns the state vector to the satellite
        coes: [
                a =      Semi-major Axis,
                e =     Eccentricity
                i =     Inclination
                raan =  Right Ascention of the Ascending Node
                aop =   Argument of Periapsis
                f =     True anomaly
            ]
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
        """Provided a list containing the cartesian position and velocity vectors, 
        assign the corresponding values to the GMAT Spacecraft object
        
        xyz = [
                rX,
                rY,
                rZ,
                vX,
                vY,
                vZ
            ]
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
        """Create electric fuel tank and attach it to the Spacecraft object"""

        self.etank = gmat.Construct("ElectricTank", f"{self.sat.GetName()}_tank")
        self.etank.SetField("FuelMass", 200.0)
        self.sat.SetField("Tanks", self.etank.GetName())
    
    def setEThruster(self, axis:str = "I+", engineSpecs: tuple = (0.6, 3000)):
        """Given a direction in the RIC, create a thruster to be attached to the Spacecraft object later"""

        if axis not in ("R+", "R-", "I+", "I-", "C+", "C-"):
            raise ValueError(f"{axis} axis not found. Acceptable values are: R+, R-, I+, I-, C+, C-")
        
        thrust = engineSpecs[0]
        isp = engineSpecs[1]
        ethruster = gmat.Construct("ElectricThruster",f"{self.sat.GetName()}_electric_thruster_{axis}")
        ethruster.SetField("Isp", isp)
        ethruster.SetField("ConstantThrust", thrust)
        ethruster.SetField("ThrustModel", "ConstantThrustAndIsp")
        ethruster.SetField("DecrementMass", False)

        self.thrusters[axis] = ethruster

        self.setEThrusterDirection(axis)

    def setEThrusterDirection(self, axis: str):
        """Assign the thruster direction for the provided thruster axis"""

        if axis not in ("R+", "R-", "I+", "I-", "C+", "C-"):
            raise ValueError(f"{axis} axis not found. Acceptable values are: R+, R-, I+, I-, C+, C-")
        
        # Thruster is default created using VNB reference frame where V -> I, N -> C, and B => -R
        axisMap = {
            "R+": [0, 0, -1], 
            "R-": [0, 0, 1], 
            "I+": [1, 0, 0], 
            "I-": [-1, 0, 0], 
            "C+": [1e-5, 1, 1e-5], # Thruster cannot be initialized with any smaller V and B components
            "C-": [-1e-5, -1, -1e-5]}
        mags = axisMap[axis]

        v = mags[0]
        self.thrusters[axis].SetField("ThrustDirection1", v)

        n = mags[1]
        self.thrusters[axis].SetField("ThrustDirection2", n)
        
        b = mags[2]
        self.thrusters[axis].SetField("ThrustDirection3", b)

    def setPowerSystem(self, powerSystemType: str = "Nuclear", kw: float = 20):
        """Create power source for electric the onboard engines and attach it to the satellite."""

        if powerSystemType != "Nuclear" and powerSystemType != "Solar":
            raise TypeError(f"{powerSystemType} is not a valid power system type in GMAT. Please select from either 'Nuclear' or 'Solar'")
        self.powerSystem = gmat.Construct(f"{powerSystemType}PowerSystem", f"{self.sat.GetName()}_{powerSystemType}Power")
        self.powerSystem.SetField("InitialMaxPower", kw)
        self.powerSystem.SetField("InitialEpoch", self.epoch)
        self.sat.SetField("PowerSystem", self.powerSystem.GetName())

    def setManeuverable(self):
        """Create a tank, power system, and thrusters (if not created already) then assign them to the Spacecraft object"""
        
        if self.etank is None:
            self.setETank()

        if self.powerSystem is None:
            self.setPowerSystem()

        if self.thrusters == {}:
            thrusterAxes = {
                "R+": (0.2, 3000), 
                "R-": (0.2, 3000), 
                "I+": (0.2, 3000), 
                "I-": (0.2, 3000), 
                "C+": (0.6, 3000), 
                "C-": (0.6, 3000)}
            for ax, thrParam in thrusterAxes.items():
                self.setEThruster(ax, thrParam)
                self.thrusters[ax].SetField("Tank", self.etank.GetName())
            
            thrusterNames = [i.GetName() for i in self.thrusters.values()]
            thrusterArray = "{" + ", ".join(thrusterNames) + "}"
            self.sat.SetField("Thrusters", thrusterArray)
        else:
            thrusterNames = [i.GetName() for i in self.thrusters.values()]
            thrusterArray = "{" + ", ".join(thrusterNames) + "}"
            self.sat.SetField("Thrusters", thrusterArray)
            [i.SetField("Tank", self.etank.GetName()) for i in self.thrusters.values()]

        
        

    