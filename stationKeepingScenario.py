
from createForceModel import createForceModel
from createPropagator import createPropagator
from createSatellite import createSatellite
from createThruster import createThruster
from load_gmat import *
from matplotlib import pyplot as plt

import datetime as dt
import numpy as np

sat = createSatellite("truth")

fm = createForceModel("truth")

prop = createPropagator("truth")
prop.assignFM(fm.getFM())

tank = gmat.Construct("ElectricTank", "eTank")
tank.SetField("FuelMass", 150)

thruster = createThruster("thruster")
thruster.assignForce(10000)
thruster.assignTank(tank.GetName())

powerSystem = gmat.Construct("NuclearPowerSystem", "EPS")# gmat.Construct("SolarPowerSystem", "EPS")
powerSystem.SetField("InitialMaxPower", 1e7)
today = dt.datetime.today()
today = today.strftime("%d %b %Y 00:00:00.000")
powerSystem.SetField("InitialEpoch", today)
sat.assignTank(tank.GetName())
sat.assignThruster(thruster.getThruster().GetName())
sat.assignPower(powerSystem.GetName())

burn = gmat.Construct("FiniteBurn", "finBurn")
burnForce = gmat.FiniteThrust("burnForce")

gmat.Initialize()
# print(sat.getSat().GetGeneratingString(0))

burn.SetField("Thrusters", thruster.getThruster().GetName())
burn.SetRefObject(thruster.getThruster(), gmat.THRUSTER, thruster.getThruster().GetName())
burn.SetSolarSystem(gmat.GetSolarSystem())
burn.SetSpacecraftToManeuver(sat.getSat())
burn.SetRefObject(sat.getSat(), gmat.SPACECRAFT, sat.getSat().GetName())

burnForce.SetRefObjectName(gmat.SPACECRAFT, sat.getSat().GetName())
burnForce.SetReference(burn)
burnForce.SetSolarSystem(gmat.GetSolarSystem())
# gmat.ConfigManager.Instance().AddPhysicalModel(burnForce)
fm.getFM().AddForce(burnForce)
prop.assignFM(fm.getFM())
gmat.Initialize()

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
elapsed = 0
t = []
states = [[], [], []]
dt = 60
burnMinutes = 80
coastMinutes = 100
mu = 3.986e5

# integrator = prop.getIntegrator()
# gmat.Initialize()
# print(sat.getCartesianState())
for count in range(1):
    integrator = prop.getIntegrator()
    gmat.Initialize()

    prop.assignSat(sat.getSat())
    theThruster = thruster.getThruster()
    theThruster.SetField("IsFiring", True)
    sat.getSat().IsManeuvering(True)
    burn.SetSpacecraftToManeuver(sat.getSat())
    # prop.getPropagator().AddForce(burnForce)    
    # prop.getPropagator().PrepareInternals()
    psm = prop.getPropagator().GetPropStateManager()
    psm.SetProperty("MassFlow")
    

    prop.getPropagator().PrepareInternals()
    integrator = prop.getIntegrator()
    fmData = prop.getPropagator().GetODEModel()
    for j in range(burnMinutes):
        integrator.Step(dt)
        elapsed += dt
        rv = integrator.GetState()
        states[0].append(rv[0])
        states[1].append(rv[1])
        states[2].append(rv[2])
        r = np.linalg.norm(rv[:3])
        accelActual = fmData.GetDerivativesForSpacecraft(sat.getSat())
        # print(accelActual[3:])
        accel = -mu / r**3 * np.array(rv[:3])
        # print(accel)
        diff = np.linalg.norm(accelActual[3:] - accel)
        # print(diff)
        print(f"Step {j:2d} | diff = {diff:.10f} km/s²  | "
            f"Commanded thrust = {theThruster.GetField('ConstantThrust')} N  | "
            f"Scale factor     = {theThruster.GetField('ThrustScaleFactor')}  | "
            f"IsFiring         = {theThruster.GetField('IsFiring')}  | "
            f"Total spacecraft mass = {sat.getSat().GetField('TotalMass')} kg | "
            f"ThrustCoeff1     = {theThruster.GetField('ThrustCoeff1')} | ")
        
        integrator.UpdateSpaceObject()

    # prop.getPropagator().GetODEModel().DeleteForce(burnForce)
    theThruster.SetField("IsFiring", False)
    sat.getSat().IsManeuvering(False)
    
    
    prop.getPropagator().PrepareInternals()
    gator = prop.getIntegrator()
    # gmat.Initialize()

    ax.plot(states[0], states[1], states[2], 'r')
    states = [[], [], []]

    for j in range(coastMinutes):
        integrator.Step(dt)
        elapsed += dt
        rv = integrator.GetState()
        
        states[0].append(rv[0])
        states[1].append(rv[1])
        states[2].append(rv[2])
        integrator.UpdateSpaceObject()
    
    ax.plot(states[0], states[1], states[2], 'b')
    states = [[], [], []]
    # print(sat.getCartesianState())


ax.set_xlabel('X (km)')
ax.set_ylabel('Y (km)')
ax.set_zlabel('Z (km)')
ax.set_title('3D Trajectory of Earth Orbiter')
ax.axis('equal')



plt.show()