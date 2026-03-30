from createForceModel import ForceModel
from createPropagator import Propagator
from createSatellite import Satellite
from load_gmat import *

class StationKeepingObjects:
    def __init__(self, objectType: str):
        if objectType.lower() != "truth" and objectType.lower() != "reference":
            raise ValueError("Object typing can only be 'Truth' or 'Reference'.")
        self.objType = objectType
        self.coes = []
        self.sat_wrap    = Satellite(f"{objectType}_Sat")
        self.fm_wrap     = {"coast": ForceModel(objectType)}
        self.prop_wrap   = {"coast": Propagator(objectType)}

        self.fm_wrap["coast"].setDynamics(objectType)

        self.prop_wrap["coast"].setIntegrator(objectType)
        self.prop_wrap["coast"].setFM(self.fm_wrap["coast"].getFM())
        self.prop_wrap["coast"].setSat(self.sat_wrap.getSat())

    def setSatCOEs(self, coes: list):
        if len(coes) != 6:
            raise ValueError("Incorrect amount of orbital elements passed. There needs to be exactly 6")
        self.coes = coes
        self.sat_wrap.setOrbitElements(coes)
    
    def setManeuverable(self):
        self.sat_wrap.setManeuverable()
        
        for ax in self.sat_wrap.thrusters.keys():
            self.fm_wrap[ax]     = ForceModel(f"{self.objType}_{ax}")
            self.fm_wrap[ax].setDynamics(self.objType)

            self.prop_wrap[ax]   = Propagator(f"{self.objType}_{ax}")
            self.prop_wrap[ax].setIntegrator(self.objType)
            self.prop_wrap[ax].setFM(self.fm_wrap[ax].getFM())
            self.prop_wrap[ax].setSat(self.sat_wrap.getSat())
    
    def setBurnForces(self):
        for ax in self.sat_wrap.thrusters.keys(): # self.sat_wrap.items():
            self.fm_wrap[ax].createBurnForces(self.sat_wrap, ax)
            self.prop_wrap[ax].prepareInternals()

    def preparePropInternal(self):
        for prop in self.prop_wrap.values():
            prop.prepareInternals()

    def dispObjects(self):
        print(f"{self.objType} objects:")
        print("\nsatellites:")
        print(self.sat_wrap.getSat().GetName())
        
        print("\nforce models:")
        for key, item in self.fm_wrap.items():
            print(f"\t- {item.getFM().GetName()} ({key})")
        
        print("\npropagators:")
        for key, item in self.prop_wrap.items():
            print(f"\t- {item.getPropagator().GetName()} ({key})")