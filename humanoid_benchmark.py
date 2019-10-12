
# Import
import pybullet as p

# Connection to physics server
p.connect(p.DIRECT)

# Set gravity
p.setGravity(0, 0, -10)

# Set engine parameters
p.setPhysicsEngineParameter(numSolverIterations=5)
p.setPhysicsEngineParameter(fixedTimeStep=1.0 / 240.0)
p.setPhysicsEngineParameter(numSubSteps=1)

# Load plane from URDF
p.loadURDF("plane.urdf")

# Load humanoid from synmmetric
objects = p.loadMJCF("mjcf/humanoid_symmetric.xml")

# Get the object
ob = objects[0]

# Base position and orientation
p.resetBasePositionAndOrientation(
    ob,
    [0.789351, 0.962124, 0.113124],
    [0.710965, 0.218117, 0.519402, -0.420923]
)

# Position of the joints
joint_positions = [
    -0.200226, 0.123925, 0.000000, -0.224016, 0.000000, -0.022247, 0.099119, -0.041829, 0.000000,
    -0.344372, 0.000000, 0.000000, 0.090687, -0.578698, 0.044461, 0.000000, -0.185004, 0.000000,
    0.000000, 0.039517, -0.131217, 0.000000, 0.083382, 0.000000, -0.165303, -0.140802, 0.000000,
    -0.007374, 0.000000
]

# For each joint
for joint_index in range(p.getNumJoints(ob)):
    # Change joint positions
    p.resetJointState(ob, joint_index, joint_positions[joint_index])
# end for

# First let the humanoid fall
#p.setRealTimeSimulation(1)
#time.sleep(5)
p.setRealTimeSimulation(False)
#p.saveWorld("lyiing.py")

# Now do a benchmark and save
# the result to a file
print("Starting benchmark")
file_name = "pybullet_humanoid_timings.json"

# State logger
log_id = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, file_name)
for i in range(1000):
  p.stepSimulation()
p.stopStateLogging(logId)

print("ended benchmark")
print("Use Chrome browser, visit about://tracing, and load the %s file" % fileName)