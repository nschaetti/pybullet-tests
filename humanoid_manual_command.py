
# Imports
import pybullet as p
import time
import pybullet_data

# Connection
p.connect(p.GUI)

# ??
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load humanoid
ob_uids = p.loadMJCF("mjcf/humanoid.xml")
humanoid = ob_uids[1]

gravity_id = p.addUserDebugParameter("gravity", -10, 10, -10)

joint_ids = []
param_ids = []

p.setPhysicsEngineParameter(numSolverIterations=10)

p.changeDynamics(
    humanoid,
    -1,
    linearDamping=0,
    angularDamping=0
)

# For each joint
for j in range(p.getNumJoints(humanoid)):
    p.changeDynamics(
        humanoid,
        j,
        linearDamping=0,
        angularDamping=0
    )
    info = p.getJointInfo(humanoid, j)
    joint_name = info[1]
    joint_type = info[2]
    if joint_type == p.JOINT_PRISMATIC or joint_type == p.JOINT_REVOLUTE:
        joint_ids.append(j)
        param_ids.append(
            p.addUserDebugParameter(
                joint_name.decode("utf-8"),
                -4,
                4,
                0
            )
        )
    # end if
# end for

# Set real time simulation
p.setRealTimeSimulation(1)

while(True):
    p.setGravity(
        0,
        0,
        p.readUserDebugParameter(gravity_id)
    )
    for i in range(len(param_ids)):
        c = param_ids[i]
        target_pos = p.readUserDebugParameter(c)
        p.setJointMotorControl2(
            humanoid,
            joint_ids[i],
            p.POSITION_CONTROL,
            target_pos,
            force=5 * 240.0
        )
    # end for
    time.sleep(0.01)
# end while
