import gmatpyplus as gmat

class createThruster:
    def __init__(self, thursterName: str):
        self.thruster = gmat.Construct("ElectricThruster", thursterName)
    
    def getThruster(self):
        return self.thruster

    def assignTank(self, tank):
        self.thruster.SetField("Tank", tank)

    def assignForce(self, f=0.1):
        self.thruster.SetField("ThrustModel", "ConstantThrustAndIsp")
        self.thruster.SetField("Isp", 4200)
        self.thruster.SetField("ConstantThrust", f)

    def assignDirection(self, axis, mag):
        if abs(mag) > 1:
            raise ValueError("The magnitude of the thrust is too large. -1 <= mag <= 1")

        if axis.lower() == "v":
            self.thruster.SetField("ThrustDirection1", mag)
        elif axis.lower() == "n":
            self.thruster.SetField("ThrustDirection2", mag)
        elif axis.lower() == "b":
            self.thruster.SetField("ThrustDirection3", mag)
        else:
            raise ValueError("Incorrect thruster direction. Must either be V, N, or B")