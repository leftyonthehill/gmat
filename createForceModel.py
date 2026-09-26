""" Support class that creates the dynamics and force model for the
station keeping scenario. """

from createSatellite import Satellite
from load_gmat import gmat


class ForceModel:
    """
    Wrapper for a ForceModel object in GMAT designed to model the forces
    experienced during a station keeping scenario.

    This wrapper supports two types of force models:
    - "reference": Includes the minimum perturbation load (16x16 Earth
      geopotential model) for an ideal reference trajectory.
    - "truth": High fidelity dynamics model to include atmospheric
      drag, 3rd body effects (Sun/Moon), and solar radiation pressure.

    Attributes
    ----------
    fm_type : str
        The force load this model carries.
        - "reference" = gravity only
        - "truth" = full force load
    fm : gmat.ODEModel
        GMAT object holding the list of forces that contribute to the
        spacecraft's acceleration.
    burn : {str: gmat.FiniteBurn}
        dict of GMAT objects describing the configuration of the
        thrusters and their axes.
    burn_force : {str: gmat.FiniteThrust}
        dict of GMAT forces to be applied to the gmat.PhysicalModel to
        simulate continuous-thrust acceleration for each corresponding
        thruster axis.
    """

    def __init__(self, fm_type: str):
        """ Initialize the ForceModel wrapper.
        
        Parameters
        ----------
        fm_type : str
            Type of ForceModel to produce.
        """

        self.fm_type = fm_type
        self.fm = gmat.Construct("ForceModel", f"{self.fm_type}_Forces")
        self.burn = {}
        self.burn_force = {}

    def set_forces_to_propagate(self):
        """ Assign the corresponding forces for a given force model.
        
        Raises
        ------
        ValueError
            `self.fm_type` does not start with one of the following:
            "reference" or "truth".
        """

        if (not self.fm_type.startswith("reference")
            and not self.fm_type.startswith("truth")):
            raise ValueError(
                "Incorrect force-model type was chosen (Provided: "
                + self.fm_type + "). The force model type must start with "
                + "'reference' or 'truth'.")

        if self.fm_type.startswith("reference"):
            self._set_forces(
                degree=16,
                order=16
            )
        else:
            self._set_forces(
                order=16,
                degree=16,
                thirdBodyEffects=True,
                atmDrag=True,
                srp=True
            )

    def _set_forces(self, degree: int, order: int, **kwargs):
        """ Assign the desired forces to the ForceModel.

        Parameters
        ----------
        degree : int
            Sets how finely latitude / radial structure is resolved
            in Earth's gravity spherical-harmonic expansion. All
            harmonics with degree n <= 70 are eligible.
        order : int
            Sets how finely longitude structure is resolved in Earth's
            gravity spherical-harmonic expansion. For each degree n,
            only terms with order m <= n are used.
        thirdBodyEffects : bool, optional
            Enables the gravitational effects of the Moon and Sun.
        atmDrag : bool, optional
            Enables atmospheric effects.
        srp : bool, optional
            Enables the effects of solar radiation pressure.
        """

        # Assign Earth's Geopotential model
        self.fm.SetField("CentralBody", "Earth")
        earth_grav = gmat.Construct("GravityField",
                                   f"{self.fm.GetName()}_Earth_Geopotential")
        earth_grav.SetField("BodyName", "Earth")
        earth_grav.SetField("Order", order)
        earth_grav.SetField("Degree", degree)
        earth_grav.SetField("PotentialFile", "JGM2.cof")
        earth_grav.SetField("StmLimit", 100)
        earth_grav.SetField("TideModel", "None")
        self.fm.AddForce(earth_grav)

        # If enabled, add solar and lunar third body effects
        if kwargs.get("thirdBodyEffects", False):
            solar = gmat.GetSolarSystem()
            self.fm.SetSolarSystem(solar)

            # Lunar attraction
            moon_grav = gmat.Construct("PointMassForce",
                                      f"{self.fm.GetName()}_Lunar_Grav")
            moon_grav.SetField("BodyName", "Luna")
            self.fm.AddForce(moon_grav)

            # Solar attraction
            sun_grav = gmat.Construct("PointMassForce",
                                     f"{self.fm.GetName()}_Solar_Grav")
            sun_grav.SetField("BodyName", "Sun")
            self.fm.AddForce(sun_grav)

        # If enabled, add atmospheric drag effects
        if kwargs.get("atmDrag", False):
            drag = gmat.Construct("DragForce", f"{self.fm.GetName()}_atmDrag")
            drag.SetField("AtmosphereModel", "JacchiaRoberts")
            atmosphere = gmat.Construct("JacchiaRoberts")
            drag.SetReference(atmosphere)

            # Values below represent a quiet, mid-cycle solar climate
            drag.SetField("F107", 120.0)
            drag.SetField("F107A", 120.0)

            # Jacchia Roberts magnetic index is Kp
            drag.SetField("MagneticIndex", 2)

            self.fm.AddForce(drag)

        # If enabled, add solar radiation pressure effects
        if kwargs.get("srp", False):
            srp = gmat.Construct(
                "SolarRadiationPressure",
                f"{self.fm.GetName()}_srp"
            )
            self.fm.AddForce(srp)

    def create_burn_forces(self, sat_obj: Satellite, ax: str):
        """
        For a given thruster axis, add its dynamics to the scenario's
        physical model.
        
        Parameters
        ----------
        sat_obj : Satellite
            Contains the thrusters to be added to the physics model.
        ax : str
            Thruster axis to create a BurnForce for.

        Raises
        ------
        RuntimeError
            FiniteBurn and BurnForce objects were attempted to be
            created a second time.
        """

        # Check to see if burn has already been created
        if ax in self.burn:
            raise RuntimeError("Thrust profiles have already been produced")

        # Create the FiniteBurn for the thruster
        thr = sat_obj.thrusters[ax]
        self.burn[ax] = gmat.Construct(
            "FiniteBurn",
            f"{self.fm.GetName()}_{ax}_Burn"
        )
        self.burn[ax].SetField("Thrusters", thr.GetName())
        self.burn[ax].SetRefObject(thr, gmat.THRUSTER, thr.GetName())
        self.burn[ax].SetSolarSystem(gmat.GetSolarSystem())
        self.burn[ax].SetSpacecraftToManeuver(sat_obj.sat)
        self.burn[ax].SetRefObject(
            sat_obj.sat,
            gmat.SPACECRAFT,
            sat_obj.sat.GetName()
        )

        # Create the BurnForce for the FiniteBurn
        self.burn_force[ax] = gmat.FiniteThrust(
            f"{self.fm.GetName()}_{ax}_Thrust"
        )
        self.burn_force[ax].SetRefObjectName(
            gmat.SPACECRAFT, sat_obj.sat.GetName()
        )
        self.burn_force[ax].SetReference(self.burn[ax])

        # Assign the BurnForce to the GMAT table of physical models
        gmat.ConfigManager.Instance().AddPhysicalModel(self.burn_force[ax])
