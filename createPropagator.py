from load_gmat import *
# import gmatpyplus as gmat
class createPropagator:
    def __init__(self, propType: str):
        if propType.lower() != "reference" and propType.lower() != "truth":
            raise ValueError(f"Incorrect propagation type was chosen (Provided: {propType}). The propagator type must only be 'reference' or 'truth'.")
        
        self.prop = None
        self.createProp(propType)
  
    def getPropagator(self):
        return self.prop
    
    def getIntegrator(self):
        return self.prop.GetPropagator()

    def createProp(self, propType: str):
        self.prop = gmat.Construct("Propagator", f"{propType.lower()}Propagator")
        integrator = gmat.Construct("RungeKutta89", "Gator")
        self.prop.SetReference(integrator)
        
        self.prop.SetField("InitialStepSize", 60)
        self.prop.SetField("MinStep", 1e-3)
        self.prop.SetField("Accuracy", 1e-10)

        if propType.lower() == "truth":
            self.prop.SetField("MaxStep", 20)
            self.prop.SetField("MaxStepAttempts", 10000)
        else:
            self.prop.SetField("MaxStep", 1200)
            self.prop.SetField("MaxStepAttempts", 100)
    
    def assignFM(self, fm):
        self.prop.SetReference(fm)
    
    def assignSat(self, sat):
        self.prop.AddPropObject(sat)
        self.prop.PrepareInternals()