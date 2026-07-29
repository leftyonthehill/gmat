""" Support class that creates the propagators for the station keeping
scenario. """

from load_gmat import gmat

class Propagator:
    """ Wrapper for a Propagator object in GMAT.
    
    This wrapper supports the forwards and backwards propagation of
    high-fidelity orbital dynamics.

    Attributes
    ----------
    prop : gmat.Propagator
        GMAT Propagator object (object descibing what is being
        numerical integrated)
    integrator : gmat.RungeKutta89
        GMAT RungeKutta89 object (object containing the numerical
        integration parameters)
    """
    
    def __init__(self, propName: str):
        """ Initialize the Propagator wrapper.
        
        Parameters
        ----------
        propName : str
            A unique name to be recognized in GMAT for simulation
        """

        self.prop_gmat = gmat.Construct("Propagator", f"{propName}_Prop")
        self.integrator = None

    def setIntegrator(self):
        """ Creates numerical integrator. """
        
        # Create the numerical integrator and assign it to propagator
        self.integrator = gmat.Construct("RungeKutta89", "Gator")
        self.prop_gmat.SetReference(self.integrator)
        
        # Shared integratation parameters
        self.prop_gmat.SetField("InitialStepSize", 5)
        self.prop_gmat.SetField("MinStep", 1e-5)
        self.prop_gmat.SetField("Accuracy", 1e-10)

        self.prop_gmat.SetField("MaxStep", 120)
        self.prop_gmat.SetField("MaxStepAttempts", 2.5e4)
    
    def setFM(self, fm: gmat.ODEModel):
        """ Assign a ForceModel object to the propagator.
        
        Parameters
        ----------
        fm : gmat.ODEModel
        """

        self.prop_gmat.SetReference(fm)
    
    def setSat(self, sat: gmat.Spacecraft):
        """ Assign a satellite to the propagator.
        
        Parameters
        ----------
        sat : gmat.Spacecraft
        """

        self.prop_gmat.AddPropObject(sat)
