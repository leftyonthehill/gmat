from load_gmat import *
# import gmatpyplus as gmat
class Propagator:
    def __init__(self, propName: str):

        self.prop = gmat.Construct("Propagator", f"{propName}_Prop")
        self.integrator = None
  
    def getPropagator(self):
        return self.prop
    
    def getIntegrator(self):
        return self.prop.GetPropagator()

    def setIntegrator(self, propType: str  = ""):
        self.integrator = gmat.Construct("RungeKutta89", "Gator")
        self.prop.SetReference(self.integrator)
        
        self.prop.SetField("InitialStepSize", 60)
        self.prop.SetField("MinStep", 1e-3)
        self.prop.SetField("Accuracy", 1e-10)

        if propType.lower() == "truth":
            self.prop.SetField("MaxStep", 20)
            self.prop.SetField("MaxStepAttempts", 10000)
        else:
            self.prop.SetField("MaxStep", 1200)
            self.prop.SetField("MaxStepAttempts", 100)
    
    def setFM(self, fm):
        self.prop.SetReference(fm)
    
    def setSat(self, sat):
        self.prop.AddPropObject(sat)
        # self.prop.PrepareInternals()
    
    def prepareInternals(self):
        self.prop.PrepareInternals()