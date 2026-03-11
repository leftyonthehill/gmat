# from load_gmat import *
import gmatpyplus as gmat
class createPropagator:
    def __init__(self, propType: str):
        if propType.lower() != "reference" and propType.lower() != "truth":
            raise ValueError(f"Incorrect propagation type was chosen (Provided: {propType}). The propagator type must only be 'reference' or 'truth'.")
        
        self.prop = self.createFM(propType)

        def getProp(self):
            return self.Prop

        def createProp(self, propType: str):
            self.propagator = gmat.Construct("Propagator", f"{propType.lower()}Propagator")
            integrator = gmat.Construct("RungeKutta89", "Gator")
            self.propagator.SetReference(integrator)
            
            self.propagator.SetField("InitialStepSize", 60)
            self.propagator.SetField("MinStep", 1e-3)
            self.propagator.SetField("Accuracy", 1e-10)
        
        def assignFM(self, fm):
            self.propagator.SetReference(fm)
            gp.cons