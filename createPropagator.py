from load_gmat import *

class Propagator:
    """
    This class is a wrapper for a Propagator object in GMAT.
    
    This propagator wrapper is to support the propagation of the reference and truth satellites. Depending on the inputs,
    this class will assign the corresponding propagator parameters. The main differences between the two modes are:
        - Maximum step size
        - Max step attemps

    Inputs during intiialization:
        - propName: Name of the propagator to be created (does not need to include "reference" or "truth")

    Variables:
        - prop: GMAT Propagator object (object descibing what is being numerical integrated)
        - integrator: GMAT RungeKutta89 object (object containing the numerical integration parameters)
    """
    
    def __init__(self, propName: str):
        """Initialize the Propagator wrapper and create the underlying Propagator object"""
        self.prop = gmat.Construct("Propagator", f"{propName}_Prop")
        self.integrator = None
  
    def getPropagator(self):
        """Returns the propagator
        
        Returns:
            - GMAT Propagator object
        """
        return self.prop
    
    def getIntegrator(self):
        """Return the numerical integrator
        
        Returns:
            - GMAT RungeKutta89 object
        """
        return self.prop.GetPropagator()

    def setIntegrator(self, propType: str  = ""):
        """Creates numerical integrator
        
        Inputs:
            - propType (str): The maneuverable satellite should request the "truth" parameters while all else can request 
                              the "reference" or nothing specific parameters (default case if propType not provided at call)
        """
        
        # Create the numerical integrator and assign it to propagator
        self.integrator = gmat.Construct("RungeKutta89", "Gator")
        self.prop.SetReference(self.integrator)
        
        # Shared integratation parameters
        self.prop.SetField("InitialStepSize", 60)
        self.prop.SetField("MinStep", 1e-4)
        self.prop.SetField("Accuracy", 1e-10)

        # Case-specific parameters
        if propType.lower() == "truth":
            self.prop.SetField("MaxStep", 20)
            self.prop.SetField("MaxStepAttempts", 10000)
        else:
            self.prop.SetField("MaxStep", 1200)
            self.prop.SetField("MaxStepAttempts", 100)
    
    def setFM(self, fm):
        """Assign a ForceModel object to the propagator
        
        Inputs:
            - GMAT ForceModel object
        """
        self.prop.SetReference(fm)
    
    def setSat(self, sat):
        """Assign the satellite to propagate to the propagator
        
        Inputs:
            - GMAT Spacecraft object
        """
        self.prop.AddPropObject(sat)
        # self.prop.PrepareInternals()
    
    def prepareInternals(self):
        """After any changes to the force model or updates to the satellite being propagated, prepare the propagator 
        internals for future integratiom
        """
        self.prop.PrepareInternals()