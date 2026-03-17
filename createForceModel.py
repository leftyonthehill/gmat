from load_gmat import *

class createForceModel:
    def __init__(self, fmType: str):
        if fmType.lower() != "reference" and fmType.lower() != "truth":
            raise ValueError(f"Incorrect propagation type was chosen (Provided: {fmType}). The propagator type must only be 'reference' or 'truth'.")
        
        self.fm = None
        self.createFM(fmType)

    def getFM(self):
        return self.fm

    def createFM(self, propType: str):
        self.fm = gmat.Construct("ForceModel", f"{propType.lower()}Forces")
        if propType.lower() == "reference":
            self.assignForces()
        else:
            self.assignForces(
                # thirdBodyEffects=True,
                # atmDrag=True,
                # srp=True
            )

    def assignForces(self, **kwargs):
        solar = gmat.GetSolarSystem()
        self.fm.SetSolarSystem(solar)
        self.fm.SetField("CentralBody", "Earth")
        earthGrav = gmat.Construct("GravityField")
        earthGrav.SetField("BodyName", "Earth")
        earthGrav.SetField("Degree", 0)
        earthGrav.SetField("Order", 0) # 4
        earthGrav.SetField("PotentialFile", "JGM2.cof")
        earthGrav.SetField("StmLimit", 100)
        earthGrav.SetField("TideModel", "None")
        self.fm.AddForce(earthGrav)
        
        if "thirdBodyEffects" in kwargs.keys() and kwargs["thirdBodyEffects"]:
            moonGrav = gmat.Construct("PointMassForce")
            moonGrav.SetField("BodyName", "Luna")
            self.fm.AddForce(moonGrav)

            sunGrav = gmat.Construct("PointMassForce")
            sunGrav.SetField("BodyName", "Sun")
            self.fm.AddForce(sunGrav)

        if "atmDrag" in kwargs.keys() and kwargs["atmDrag"]:
            drag = gmat.Construct("DragForce", "atmDrag")
            drag.SetField("AtmosphereModel", "JacchiaRoberts")
            atmosphere = gmat.Construct("JacchiaRoberts")
            drag.SetReference(atmosphere)
            
            """drag.SetField("HistoricWeatherSource", "ConstantFluxAndGeoMag")
            drag.SetField("PredictedWeatherSource", "ConstantFluxAndGeoMag")
            drag.SetField("F107", 150.0)
            drag.SetField("F107A", 150.0)
            drag.SetField("MagneticIndex", 3.0)

            drag.SetField("CSSISpaceWeatherFile", "SpaceWeather-All-v1.2.txt")
            drag.SetField("SchattenFile", "SchattenPredict.txt")

            drag.SetField("SchattenErrorModel", "Nominal")
            drag.SetField("SchattenTimingModel", "NominalCycle")"""

            self.fm.AddForce(drag)

        if "srp" in kwargs.keys() and kwargs["srp"]:
            srp = gmat.Construct("SolarRadiationPressure")
            self.fm.AddForce(srp)

    def assignSats(self, sat):
        psm = gmat.PropagationStateManager()
        psm.SetObject(sat)
        psm.BuildState()

        self.fm.SetPropStateManager(psm)
        self.fm.SetState(psm.GetState())
