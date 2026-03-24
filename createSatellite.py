from load_gmat import *

import datetime as dt
import numpy as np


class Satellite:
    """
    Create a GMAT Satellite object using the createSatellite wrapper.

    To create this wrapper, provide a name for this satellite as a string. 
    This object's name must not be exist in the GMAT scenario otherwise an 
    error will be raised.
    """
    def __init__(self, satName: str):
        self.sat = None
        self.epoch = None
        self.ethruster = None
        self.etank = None
        self.powerSystem = None
        self.maneuverable = False

        self.__setSatParam__(satName)
    
    """
    This class is private and designed to only be called from __init__(). 
    This class establishes the parameters needed for the satellite objects. 
    Future methods will be built to provide an override all parameters below.
    """
    def __setSatParam__(self, satName: str):
        self.sat = gmat.Construct("Spacecraft", satName)
        self.sat.SetField("DateFormat", "UTCGregorian")

        today = dt.datetime.today()
        self.epoch = today.strftime("%d %b %Y 12:00:00.000")

        self.sat.SetField("Epoch", self.epoch)
        self.sat.SetField("CoordinateSystem", "EarthMJ2000Eq")
        self.sat.SetField("DisplayStateType", "Keplerian")

        self.sat.SetField("SRPArea", 6)
        self.sat.SetField("Cr", 1.8)
        self.sat.SetField("DragArea", 9)
        self.sat.SetField("Cd", 2.2)

    def __repr__(self):
        return f"{self.sat}, {self.etank}, {self.ethruster}, {self.powerSystem}"

    """
    Return the GMAT object contained in this wrapper
    """
    def getSat(self):
        return self.sat
    
    """
    Return the cartesian state vector of the satellite
    """
    def getCartesianState(self):
        return self.sat.GetCartesianState()
    
    """
    Return the keplerian state vector of the satellite
    """
    def getKeplerianState(self):
        return self.sat.GetKeplerianState()
    
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
    def setOrbitElements(self, coes: list):
        if len(coes) != 6:
            raise ValueError("Incorrect amount of orbital elements passed. There needs to be exactly 6")
        a, e, i, raan, aop, f = coes

        self.sat.SetField("SMA", a)
        self.sat.SetField("ECC", e)
        self.sat.SetField("INC", i)
        self.sat.SetField("RAAN", raan)
        self.sat.SetField("AOP", aop)
        self.sat.SetField("TA", f) 
    
    def setETank(self, mass: float = 200):
        self.etank = gmat.Construct("ElectricTank", f"{self.sat.GetName()}_tank")
        self.etank.SetField("FuelMass", 200.0)
        self.sat.SetField("Tanks", self.etank.GetName())
    
    def setEThruster(self, engineSpecs: tuple = (.1, 3000)):
        thrust = engineSpecs[0]
        isp = engineSpecs[1]
        self.ethruster = gmat.Construct("ElectricThruster",f"{self.sat.GetName()}_electric_thruster")
        self.ethruster.SetField("Isp", isp)
        self.ethruster.SetField("ConstantThrust", thrust)
        self.ethruster.SetField("ThrustModel", "ConstantThrustAndIsp")
        self.ethruster.SetField("DecrementMass", False)

        self.setEThrusterDirection()

    def setEThrusterDirection(self, mags: list[float] = [1, 0, 0]):
        v = mags[0]
        self.ethruster.SetField("ThrustDirection1", v)

        n = mags[1]
        self.ethruster.SetField("ThrustDirection2", n)
        
        b = mags[2]
        self.ethruster.SetField("ThrustDirection3", b)

    def setPowerSystem(self, powerSystemType: str = "Nuclear"):
        if powerSystemType != "Nuclear" and powerSystemType != "Solar":
            raise TypeError(f"{powerSystemType} is not a valid power system type in GMAT. Please select from either 'Nuclear' or 'Solar'")
        self.powerSystem = gmat.Construct(f"{powerSystemType}PowerSystem", f"{self.sat.GetName()}_{powerSystemType}Power")
        self.powerSystem.SetField("InitialMaxPower", 20)
        self.powerSystem.SetField("InitialEpoch", self.epoch)

    def setPowerSystemWattage(self, kw: float = 20):
        self.powerSystem.SetField("InitialMaxPower", kw)
    
    def setManeuverable(self):
        self.maneuverable = True

        if self.etank is None:
            self.setETank()
        self.sat.SetField("Tanks", self.etank.GetName())

        if self.ethruster is None:
            self.setEThruster()
        self.ethruster.SetField("Tank", self.etank.GetName())
        self.sat.SetField("Thrusters", self.ethruster.GetName())

        if self.powerSystem is None:
            self.setPowerSystem()
        self.sat.SetField("PowerSystem", self.powerSystem.GetName())
        

    