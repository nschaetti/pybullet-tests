#
# Many spheres
#

# Imports
import pybullet as p
import pybullet_data
import time

# Connection with shared memory
connection_id = p.connect(p.SHARED_MEMORY)

# With GUI if error
if connection_id < 0:
    p.connect(p.GUI)
# end if

# Path to search for models
p.setAdditionalSearchPath(pybullet_data.getDataPath())
print(pybullet_data.getDataPath())

# Set internal simulation flags
p.setInternalSimFlags(0)

# Reset the simulation
p.resetSimulation()

# Load plane and traybox
p.loadURDF("plane.urdf", useMaximalCoordinates=True)
p.loadURDF("tray/traybox.urdf", useMaximalCoordinates=True)

# Add user debugging parameters
gravX_id = p.addUserDebugParameter("gravityX", -10, 10, 0)
gravY_id = p.addUserDebugParameter("gravityY", -10, 10, 0)
gravZ_id = p.addUserDebugParameter("gravityZ", -10, 10, -10)

# Set physics parameter
p.setPhysicsEngineParameter(numSolverIterations=10)
p.setPhysicsEngineParameter(contactBreakingThreshold=0.001)

# Configure debug visualizer
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING)

# Load 1000 spheres
for i in range(10):
    for j in range(10):
        for k in range(10):
            ob = p.loadURDF(
                "sphere_1cm.urdf",
                [0.02 * i, 0.02 * j, 0.2 + 0.02 * k],
                useMaximalCoordinates=True
            )
        # end for
    # end for
# end for

# Configure debug visualizer
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

# Change gravity
p.setGravity(0, 0, -10)

# Realtime simulation
p.setRealTimeSimulation(True)

# Infinite loop
while True:
    # Read debugging parameters
    gravX = p.readUserDebugParameter(gravX_id)
    gravY = p.readUserDebugParameter(gravY_id)
    gravZ = p.readUserDebugParameter(gravZ_id)

    # Change gravity
    p.setGravity(gravX, gravY, gravZ)

    # Sleep
    time.sleep(0.01)
# end while
