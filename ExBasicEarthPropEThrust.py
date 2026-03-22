################################################################
# Credit: Hunter Quebedeaux
# Link: https://github.com/UCF-ASRL/GMAT-API-Tutorial/blob/main/scripts/python/ExBasicEarthPropEThrust.py

from load_gmat import *
import numpy as np  
from matplotlib import pyplot as plt

problemDim = 6
numRandSys = 10
mu = 398600  # Earth’s mu in km^3/s^2
R = 6371 # radius of earth in km
numMinProp = 60 * 24 # take a step 60 times in an hour and for 24 hours
numMinProp = 80 # take a step 60 times in an hour and for 24 hours
dt = 60.0 # step every 60 secs
elapsed = 0.0

# -----------configuration preliminaries----------------------------

mu = 398600

# Spacecraft
earthorb = gmat.Construct("Spacecraft", "EarthOrbiter") # create a spacecraft object named EarthOrbiter
earthorb.SetField("DateFormat", "UTCGregorian")
earthorb.SetField("Epoch", "20 Jul 2020 12:00:00.000") # set the epoch of the spacecraft

# Set the coordinate system and display state type
earthorb.SetField("CoordinateSystem", "EarthMJ2000Eq")
earthorb.SetField("DisplayStateType", "Keplerian")

# Spacecraft ballistic properties for the SRP and Drag models
earthorb.SetField("SRPArea", 2.5)
earthorb.SetField("Cr", 1.75)
earthorb.SetField("DragArea", 1.8)
earthorb.SetField("Cd", 2.1)
earthorb.SetField("DryMass", 80)

# Force model settings
fm = gmat.Construct("ForceModel", "FM")
fm.SetField("CentralBody", "Earth")

earthgrav = gmat.Construct("GravityField")
earthgrav.SetField("BodyName","Earth")
earthgrav.SetField("PotentialFile", 'JGM2.cof')
earthgrav.SetField("Degree",0)
earthgrav.SetField("Order",0) # 8


# Drag using Jacchia-Roberts
jrdrag = gmat.Construct("DragForce", "JRDrag")
jrdrag.SetField("AtmosphereModel","JacchiaRoberts")

# Build and set the atmosphere for the model
atmos = gmat.Construct("JacchiaRoberts", "Atmos")
jrdrag.SetReference(atmos)

# Construct Solar Radiation Pressure model
srp = gmat.Construct("SolarRadiationPressure", "SRP")

# Add forces into the ODEModel container
fm.AddForce(earthgrav)
# fm.AddForce(jrdrag)
# fm.AddForce(srp)

# Initialize propagator object
pdprop = gmat.Construct("Propagator","PDProp")

# Create and assign a numerical integrator for use in the propagation
gator = gmat.Construct("RungeKutta89", "Gator")
pdprop.SetReference(gator)

# Assign the force model contructed above
pdprop.SetReference(fm)

# Set some of the fields for the integration
pdprop.SetField("InitialStepSize", 60.0)
pdprop.SetField("Accuracy", 1.0e-12)
pdprop.SetField("MinStep", 0.0)

nonthrust = gmat.Construct("Propagator","noThrust")
nonthrust.Copy(pdprop)

rng = np.random.default_rng()
rng.random()



statesArrayElectric = np.zeros((numRandSys,numMinProp,problemDim))
statesArrayElectric2 = np.zeros((numRandSys,numMinProp,problemDim))
statesArrayCoast = np.zeros((numRandSys,numMinProp,problemDim))

ETank = gmat.Construct("ElectricTank", "EFuel") # create an electric tank with the name "EFuel"
EThruster = gmat.Construct("ElectricThruster", "EThruster") # create an electric thruster with the name "EThruster"
EThruster.SetField("ThrustModel", "ConstantThrustAndIsp")
EThruster.SetField("Isp", 3000)
EThruster.SetField("ConstantThrust", .1)

powerSystem = gmat.Construct("NuclearPowerSystem", "EPS") # create a power system with the name "EPS"
powerSystem.SetField("InitialMaxPower", 20)
powerSystem.SetField("InitialEpoch", "20 Jul 2020 00:00:00.000")

EThruster.SetField("DecrementMass", False)
EThruster.SetField("Tank", "EFuel") # set the tank for the "EThruster" to use the "EFuel" object
earthorb.SetField("Tanks", "EFuel") # set possible tanks for the "EThruster" to use the "Fuel" object
earthorb.SetField("Thrusters", "EThruster") # set possible thrusters to use the "EThruster" object
earthorb.SetField("PowerSystem", "EPS") # set the power system to use the "EPS" object
gmat.Initialize()

# construct the burn force model
def setThrust(s, b):
    bf = gmat.FiniteThrust("Thrust")
    bf.SetRefObjectName(gmat.SPACECRAFT, s.GetName())
    bf.SetReference(b)
    # gmat.ConfigManager.Instance().AddPhysicalModel(bf)
    fm.AddForce(bf)
    pdprop.SetReference(fm)
    return bf


burn = gmat.Construct("FiniteBurn", "TheEBurn")
burn.SetField("Thrusters", "EThruster")
burn.SetRefObject(EThruster, gmat.THRUSTER, EThruster.GetName())
burn.SetSolarSystem(gmat.GetSolarSystem())
burn.SetSpacecraftToManeuver(earthorb)
burn.SetRefObject(earthorb, gmat.SPACECRAFT, earthorb.GetName())

burnForce = setThrust(earthorb, burn)

gmat.Initialize()

print("Forces in FM:")
for i in range(fm.GetNumForces()):
    f = fm.GetForce(i)
    print(" -", f.GetName(), f.GetTypeName())
for i in range(1): # numRandSys):
    earthorb.SetField("SMA", 7000) # km
    earthorb.SetField("ECC", 0.05)
    earthorb.SetField("INC", 10) # deg
    earthorb.SetField("RAAN", 0) # deg
    earthorb.SetField("AOP", 0) # deg
    earthorb.SetField("TA", 0) # deg

    ETank.SetField("FuelMass", 210.0)

    # Perform initializations
    gmat.Initialize()

    # Refresh the 'gator reference
    gator = pdprop.GetPropagator()

    gmat.Initialize()
    
    pdprop.AddPropObject(earthorb)
    # pdprop.PrepareInternals()

    theThruster = earthorb.GetRefObject(gmat.THRUSTER, "EThruster")

    # -----------------------------
    # Finite Burn Specific Settings
    # -----------------------------
    # Turn on the thruster
    theThruster.SetField("IsFiring", True)
    earthorb.IsManeuvering(True)
    # burn.SetSpacecraftToManeuver(earthorb)
    # # Add the thrust to the force model
    # pdprop.AddForce(burnForce)
    # pdprop.GetODEModel().AddForce(burnForce)
    # psm = pdprop.GetPropStateManager()
    # psm.SetProperty("MassFlow")
    # -----------------------------
    pdprop.PrepareInternals()
    gator = pdprop.GetPropagator()

    for j in range(numMinProp):
        gator.Step(dt)
        elapsed = elapsed + dt
        state = gator.GetState()
        statesArrayElectric[i,j,:] = state[0:6]
        gator.UpdateSpaceObject()
        rv = gator.GetState()
        r = np.linalg.norm(rv[:3])
        accelActual = fm.GetDerivativesForSpacecraft(earthorb)
        # print(accelActual[3:])
        accel = -mu / r**3 * np.array(rv[:3])
        # print(accel)
        diff = np.linalg.norm(accelActual[3:] - accel)
        # print(diff)
        print(f"Step {j:2d} | diff = {diff:.10f} km/s²  | "
            f"IsFiring         = {theThruster.GetField('IsFiring')}  | ")

    fm = pdprop.GetODEModel()
    fm.DeleteForce(burnForce)
    pdprop.PrepareInternals()
    theThruster.SetField("IsFiring", False)
    earthorb.IsManeuvering(False)
    
    print("end-thrust state: ", gator.GetState())

    nonthrust.AddPropObject(earthorb)
    nonthrust.PrepareInternals()
    fm = nonthrust.GetODEModel()
    gator = nonthrust.GetPropagator()
    print("start-nonthrust state: ", gator.GetState())
    
    for j in range(numMinProp):
        gator.Step(dt)
        elapsed = elapsed + dt
        state = gator.GetState()
        statesArrayCoast[i,j,:] = state[0:6]
        gator.UpdateSpaceObject()
        rv = gator.GetState()
        r = np.linalg.norm(rv[:3])
        accelActual = fm.GetDerivativesForSpacecraft(earthorb)
        # print(accelActual[3:])
        accel = -mu / r**3 * np.array(rv[:3])
        # print(accel)
        diff = np.linalg.norm(accelActual[3:] - accel)
        # print(diff)
        print(f"Step {j:2d} | diff = {diff:.10f} km/s²  | "
            f"IsFiring         = {theThruster.GetField('IsFiring')}  | ")

    # Turn on the thruster
    theThruster.SetField("IsFiring", True)
    earthorb.IsManeuvering(True)
    # burn.SetSpacecraftToManeuver(earthorb)
    # # Add the thrust to the force model
    pdprop.AddForce(burnForce)
    # pdprop.GetODEModel().AddForce(burnForce)
    # psm = pdprop.GetPropStateManager()
    # psm.SetProperty("MassFlow")
    # -----------------------------
    
    print("end-nonthrust state: ", gator.GetState())
    pdprop.AddPropObject(earthorb)
    pdprop.PrepareInternals()
    gator = pdprop.GetPropagator()
    print("start-thrust state: ", gator.GetState())
    print("Forces in FM:")
    fm = pdprop.GetODEModel()
    for j in range(fm.GetNumForces()):
        f = fm.GetForce(j)
        print(" -", f.GetName(), f.GetTypeName())

    for j in range(numMinProp):
        gator.Step(dt)
        elapsed = elapsed + dt
        state = gator.GetState()
        statesArrayElectric2[i,j,:] = state[0:6]
        gator.UpdateSpaceObject()
        rv = gator.GetState()
        r = np.linalg.norm(rv[:3])
        accelActual = fm.GetDerivativesForSpacecraft(earthorb)
        # print(accelActual[3:])
        accel = -mu / r**3 * np.array(rv[:3])
        # print(accel)
        diff = np.linalg.norm(accelActual[3:] - accel)
        # print(diff)
        print(f"Step {j:2d} | diff = {diff:.10f} km/s²  | "
            f"IsFiring         = {theThruster.GetField('IsFiring')}  | ")

    fm = pdprop.GetODEModel()
    fm.DeleteForce(burnForce)
    pdprop.PrepareInternals()
    
    # pdprop.GetODEModel().DeleteForce(burnForce)
    theThruster.SetField("IsFiring", False)
    earthorb.IsManeuvering(False)
    
    nonthrust.AddPropObject(earthorb)
    nonthrust.PrepareInternals()
    gator = nonthrust.GetPropagator()
    print("Forces in FM:")
    fm = nonthrust.GetODEModel()
    for j in range(fm.GetNumForces()):
        f = fm.GetForce(j)
        print(" -", f.GetName(), f.GetTypeName())
    for j in range(numMinProp):
        gator.Step(dt)
        elapsed = elapsed + dt
        state = gator.GetState()
        statesArrayCoast[i,j,:] = state[0:6]
        gator.UpdateSpaceObject()
        rv = gator.GetState()
        r = np.linalg.norm(rv[:3])
        accelActual = fm.GetDerivativesForSpacecraft(earthorb)
        # print(accelActual[3:])
        accel = -mu / r**3 * np.array(rv[:3])
        # print(accel)
        diff = np.linalg.norm(accelActual[3:] - accel)
        # print(diff)
        print(f"Step {j:2d} | diff = {diff:.10f} km/s²  | "
            f"IsFiring         = {theThruster.GetField('IsFiring')}  | ")

    # Turn on the thruster
    theThruster.SetField("IsFiring", True)
    earthorb.IsManeuvering(True)
    # burn.SetSpacecraftToManeuver(earthorb)
    # # Add the thrust to the force model
    pdprop.AddForce(burnForce)
    # psm = pdprop.GetPropStateManager()
    # psm.SetProperty("MassFlow")
    # -----------------------------
    pdprop.AddPropObject(earthorb)
    pdprop.PrepareInternals()
    gator = pdprop.GetPropagator()
    print("Forces in FM:")
    fm = pdprop.GetODEModel()
    for j in range(fm.GetNumForces()):
        f = fm.GetForce(j)
        print(" -", f.GetName(), f.GetTypeName())

    for j in range(numMinProp):
        gator.Step(dt)
        elapsed = elapsed + dt
        state = gator.GetState()
        statesArrayElectric2[i,j,:] = state[0:6]
        gator.UpdateSpaceObject()
        rv = gator.GetState()
        r = np.linalg.norm(rv[:3])
        accelActual = fm.GetDerivativesForSpacecraft(earthorb)
        # print(accelActual[3:])
        accel = -mu / r**3 * np.array(rv[:3])
        # print(accel)
        diff = np.linalg.norm(accelActual[3:] - accel)
        # print(diff)
        print(f"Step {j:2d} | diff = {diff:.10f} km/s²  | "
            f"IsFiring         = {theThruster.GetField('IsFiring')}  | ")

        
    

t = np.linspace(0,numMinProp*dt,len(statesArrayElectric[0,:,0]))


fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.plot(statesArrayElectric[0,:,0],statesArrayElectric[0,:,1],statesArrayElectric[0,:,2],label='Electric',c='r')
ax.plot(statesArrayCoast[0,:,0],statesArrayCoast[0,:,1],statesArrayCoast[0,:,2],label='Electric',c='b')
ax.plot(statesArrayElectric2[0,:,0],statesArrayElectric2[0,:,1],statesArrayElectric2[0,:,2],label='Electric',c='r')
ax.set_xlabel('X (km)')
ax.set_ylabel('Y (km)')
ax.set_zlabel('Z (km)')
ax.set_title('3D Trajectory of Earth Orbiter')
ax.legend()
ax.axis('equal')

plt.show()