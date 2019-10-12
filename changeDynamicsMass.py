
# Imports
import pybullet as p
import time
import pybullet_data

# Connection to physics with GUI
p.connect(p.GUI)

# Add a path where we want to search for data
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load cubes from URDF
cube2 = p.loadURDF("cube.urdf", [0, 0, 3], useFixedBase=True)
cube = p.loadURDF("cube.urdf", useFixedBase=True)

# Change gravity
p.setGravity(0, 0, -10)

# Change time step
timeStep = 1. / 240.
p.setTimeStep(timeStep)

# Change cube2's dynamics
# p.changeDynamics(cube, -1, mass=1)
p.changeDynamics(cube2, -1, mass=1)

# Run while connected
while p.isConnected():
  p.stepSimulation()
  time.sleep(timeStep)
# end while
