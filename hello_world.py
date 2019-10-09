
# Imports
import pybullet as pb
import time
import pybullet_data


# Client
physicsClient = pb.connect(pb.DIRECT)

# ??
pb.setAdditionalSearchPath(pybullet_data.getDataPath())

# Gravity
pb.setGravity(0, 0, -10)

# Load URDF
planeId = pb.loadURDF("plane.urdf")

# Start setting for the cube
cubeStartPos = [0, 0, 1]
cubeStartOrientation = pb.getQuaternionFromEuler([0, 0, 0])

# Load box
boxId = pb.loadURDF("r2d2.urdf", cubeStartPos, cubeStartOrientation)

# 10'000 steps
for i in range(10000):
    pb.stepSimulation()
    time.sleep(1./240.)
# end for

# Get cube's settings
cubePos, cubeOrn = pb.getBasePositionAndOrientation(boxId)

# Show
print(cubePos, cubeOrn)

# Disconnect
pb.disconnect()

