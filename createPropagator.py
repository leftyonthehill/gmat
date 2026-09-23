""" Support class that creates the propagators for the station keeping
scenario. """

from load_gmat import gmat

class Propagator:
    """ Wrapper for a Propagator object in GMAT.
    
    This wrapper supports the forwards and backwards propagation of
    high-fidelity orbital dynamics.

    Attributes
    ----------
    prop_gmat : gmat.Propagator
        GMAT Propagator object (object describing what is being
        numerical integrated)
    integrator : gmat.RungeKutta89
        GMAT RungeKutta89 object (object containing the numerical
        integration parameters)
    """

    def __init__(self, prop_name: str):
        """ Initialize the Propagator wrapper.
        
        Parameters
        ----------
        prop_name : str
            A unique name to be recognized in GMAT for simulation
        """

        self.prop_gmat = gmat.Construct("Propagator", f"{prop_name}_Prop")
        self.integrator = None

    def set_integrator(
            self,
            init_step_size: int = 5,
            min_step_size: float = 1e-5,
            max_step_size: float = 120,
            max_step_attempts: float = 2.5e4,
            accuracy: float = 1e-10,
        ):
        """ Creates numerical integrator. 
        
        To support the orbital propagators, GMAT needs a numerical
        integrator. This function creates an RK89 integrator GMAT
        object.

        Parameters
        ----------
        init_step_size : int, default = 5
            Initial time step, in seconds, to use in the integrator.
        min_step_size : float, default = 1e-5
            Smallest time step, in seconds, to use in the integrator.
        max_step_size : float, default = 120
            Largest time step, in seconds, to use in the integrator.
        max_step_attempts : float, default = 2.5e4
            Upper limit as to how many max steps to be attempted during
            each propagator step.
        accuracy : float, default = 1e-10
            Error tolerance used in RK89's adaptive step control.
        """

        # Create the numerical integrator and assign it to propagator
        self.integrator = gmat.Construct("RungeKutta89", "Integrator")
        self.prop_gmat.SetReference(self.integrator)

        # Shared integratation parameters
        self.prop_gmat.SetField("InitialStepSize", init_step_size)
        self.prop_gmat.SetField("MinStep", min_step_size)
        self.prop_gmat.SetField("MaxStep", max_step_size)
        self.prop_gmat.SetField("MaxStepAttempts", max_step_attempts)
        self.prop_gmat.SetField("Accuracy", accuracy)

    def set_fm(self, fm: gmat.ODEModel):
        """ Assign a ForceModel object to the propagator.
        
        Parameters
        ----------
        fm : gmat.ODEModel
            GMAT Force Model object that defines the external forces
            effecting the spacecraft.
        """

        self.prop_gmat.SetReference(fm)

    def set_sat(self, sat: gmat.Spacecraft):
        """ Assign a satellite to the propagator.
        
        Parameters
        ----------
        sat : gmat.Spacecraft
            GMAT Spacecraft object to propagate.
        """

        self.prop_gmat.AddPropObject(sat)
