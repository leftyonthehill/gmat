from load_gmat import *

import datetime as dt
import numpy as np

class createSatellite:
    def __init__(self, satType: str):
        if satType.lower() != "reference" and satType.lower() != "truth":
            raise ValueError(f"Incorrect satellite type was chosen (Provided: {satType}). The satellite type must only be 'reference' or 'truth'.")
        
        self.sat = None
        self.createSat(satType)
    
    def getSat(self):
        return self.sat
    
    def getCartesianState(self):
        return self.sat.GetCartesianState()
    
    def getKeplerianState(self):
        return self.sat.GetKeplerianState()

    def createSat(self, satType: str):
        self.sat = gmat.Construct("Spacecraft", f"{satType.lower()}Sat")
        self.sat.SetField("DateFormat", "UTCGregorian")

        today = dt.datetime.today()
        today = today.strftime("%d %b %Y 12:00:00.000")

        self.sat.SetField("Epoch", today)
        self.sat.SetField("CoordinateSystem", "EarthMJ2000Eq")
        self.sat.SetField("DisplayStateType", "Keplerian")

        
        self.sat.SetField("DryMass", 500)
        if satType.lower() == "truth":
            self.sat.SetField("SRPArea", 6)
            self.sat.SetField("Cr", 1.8)
            self.sat.SetField("DragArea", 9)
            self.sat.SetField("Cd", 2.2)
        else:
            self.sat.SetField("SRPArea", 0)
            self.sat.SetField("Cr", 0)
            self.sat.SetField("DragArea", 0)
            self.sat.SetField("Cd", 0)
    
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
    
    def assignTank(self, tank):
        self.sat.SetField("Tanks", tank)
    
    def assignThruster(self, thruster):
        self.sat.SetField("Thrusters", thruster)

    def assignPower(self, power):
        self.sat.SetField("PowerSystem", power)
    
    def setManeuvering(self, logic:bool):
        self.sat.IsManeuvering(logic)