#
# Test with mini cheetah
#

# Imports
import pybullet as p
import pybullet_data as pd
import time

# Connection with GUI
p.connect(p.GUI)

# Earth-like gravity
p.setGravity(0, 0, -9.8)

# Add path for data
p.setAdditionalSearchPath(pd.getDataPath())

# Load floor from URDF
floor = p.loadURDF("plane.urdf")

# Cheetah starting position
starting_position = [0, 0, 0.5]

# Load mini cheetah with starting position
robot = p.loadURDF("mini_cheetah/mini_cheetah.urdf", starting_position)

# How many joint
num_joints = p.getNumJoints(robot)

# Change Cheetah's color
p.changeVisualShape(robot, -1, rgbaColor=[1, 0, 0, 1])

# Informations
print("{} joints".format(num_joints))

# For each joints
for j in range(num_joints):
    # Change joint's color ?
    p.changeVisualShape(robot, j, rgbaColor=[1, 1, 0, 1])

    # Get joint info
    joint_info = p.getJointInfo(robot, j)

    # Print info
    print("Index : {}".format(joint_info[0]))
    print("Name : {}".format(joint_info[1]))
    print("Type : {}".format(joint_info[2]))
    print("qIndex : {}".format(joint_info[3]))
    print("uIndex : {}".format(joint_info[4]))
    print("Damping : {}".format(joint_info[6]))
    print("Friction : {}".format(joint_info[7]))
    print("")

    # Position and force
    pos = 0
    if j == 0:
        force = 200
    else:
        force = 0
    # end if

    # Change position and force
    p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, pos, force=force)
# end for

# Change timestep
dt = 1.0 / 240.0
p.setTimeStep(dt)

# Infinite
while True:
    # Run on step
    p.stepSimulation()

    # Sleep for one timestep
    time.sleep(dt)
# end while
