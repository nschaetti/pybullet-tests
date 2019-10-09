
# Imports
import pybullet as p
import pybullet_data


# Connection
p.connect(p.GUI)

# Path to search for models
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load plane
p.loadURDF("plane.urdf")

# Change gravity
p.setGravity(
    0,
    0,
    -10
)

# Husky position
husky_pos = [0, 0, 0.1]

# Load husky
husky = p.loadURDF(
    "husky/husky.urdf",
    husky_pos[0],
    husky_pos[1],
    husky_pos[2]
)

# Number of joints
num_joints = p.getNumJoints(husky)

# For each joint
for joint in range(num_joints):
    print(p.getJointInfo(husky, joint))
# end for

# Target velocity
target_velocity = 10
max_force = 100

# for join 2 to 5
for joint in range(2, 6):
    p.setJointMotorControl(
        husky,
        joint,
        p.VELOCITY_CONTROL,
        target_velocity,
        max_force
    )
# end for

# Simulate for 300 steps
for step in range(300):
    p.stepSimulation()
# end for

# Target velocity
target_velocity = -10

# For join 2 to 5
for step in range(2, 6):
    p.setJointMotorControl(
        husky,
        joint,
        p.VELOCITY_CONTROL,
        target_velocity,
        max_force
    )
# end for

# Simulate for 400 steps
for step in range(400):
    p.stepSimulation()
# end for

# Get contact points ?
p.getContactPoints()

# Disconnect
p.disconnect()
