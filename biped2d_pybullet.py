
# Imports
import pybullet as p
import pybullet_data
import os
import time

# Parameters
GRAVITY = -9.8
dt = 1e-3
iters = 2000

# Physics client
physics_client = p.connect(p.GUI)

# Add a path where we want to search for data
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Reset the simulation
p.resetSimulation()

# Change the gravity
p.setGravity(0, 0, GRAVITY)

# Set timestep
p.setTimeStep(dt)

# Load the plane from URDF file
plane_id = p.loadURDF("plane.urdf")

# Starting position of the cube
cube_start_pos = [0, 0, 1.13]

# Starting orientation of the cube
cube_start_orientation = p.getQuaternionFromEuler([0., 0, 0])

# Load the biped from URDF file
bot_id = p.loadURDF("biped/biped2d_pybullet.urdf", cube_start_pos, cube_start_orientation)

# Disable the default velocity motors
# and set some position control with small force to emulate
# joint friction/return to a rest pose.
joint_friction_force = 1

# For each join
for joint in range(p.getNumJoints(bot_id)):
    p.setJointMotorControl2(bot_id, joint, p.POSITION_CONTROL, force=joint_friction_force)
# end for

p.setRealTimeSimulation(1)

while(True):
    p.setGravity(0, 0, GRAVITY)
    time.sleep(1 / 240.0)
# end while

time.sleep(1000)
