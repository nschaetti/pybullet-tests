#
# Load soft body
#

# Imports
import pybullet as p
from time import sleep
import pybullet_data

# Connection
physics_client = p.connect(p.GUI)

# Path to search for models
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity
p.setGravity(0, 0, -10)

# Load plane from URDF file
plane_id = p.loadURDF("plane.urdf")

# Load cube from URDF
box_id = p.loadURDF("cube.urdf", useMaximalCoordinates=True)

# Load soft body
bunny_id = p.loadSoftBody("bunny.obj")

# Load small cube
p.loadURDF("cube_small.urdf", [1, 0, 1])

# Use real time simulation
use_realtime_simulation = True
if use_realtime_simulation:
    p.setRealTimeSimulation(True)
# end if

# Print informations
print(p.getDynamicsInfo(plane_id, -1))
print(p.getDynamicsInfo(bunny_id, 0))
print(p.getDynamicsInfo(box_id, -1))

# While connected
while p.isConnected():
    # Set gravity
    p.setGravity(0, 0, -10)

    # If real time simulation
    if use_realtime_simulation:
        sleep(0.01)
    else:
        p.stepSimulation()
    # end if
# end while

