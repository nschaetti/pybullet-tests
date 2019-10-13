import pybullet_data as pd
import pybullet_utils as pu
import pybullet
import pybullet_utils.bullet_client as bc
import time

# Connection to bullet with GUI
p = bc.BulletClient(connection_mode=pybullet.GUI)

# Add path to data
p.setAdditionalSearchPath(pd.getDataPath())

# Add a transparent plane
p.loadURDF("plane_transparent.urdf", useMaximalCoordinates=True)
# p.setPhysicsEngineParameter(numSolverIterations=10, fixedTimeStep=0.01)

# Enable planar reflection (?) but not rendering
p.configureDebugVisualizer(p.COV_ENABLE_PLANAR_REFLECTION, True)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, False)

# ??
y2z = p.getQuaternionFromEuler([0, 0, 1.57])

# Mesh scale for x, y and z
meshScale = [1, 1, 1]

# Create visual shape from OBJ with specific color
visualShapeId = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName="domino/domino.obj",
    rgbaColor=[1, 1, 1, 1],
    specularColor=[0.4, .4, 0],
    visualFrameOrientation=y2z,
    meshScale=meshScale
)

# Box dimensions
box_dimensions = [0.5 * 0.00635, 0.5 * 0.0254, 0.5 * 0.0508]

# Create collision box from dimensions
collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=box_dimensions)

# Create all boxes
for j in range(12):
    print("j=", j)
    for i in range(35):
        # Create body with mass, shape and collision box
        p.createMultiBody(
            baseMass=0.025,
            baseCollisionShapeIndex=collision_shape_id,
            baseVisualShapeIndex=visualShapeId,
            basePosition=[i * 0.04, j * 0.05, 0.06],
            useMaximalCoordinates=True
        )
    # end for
# end for

# Enable rendering
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, True)

# Set Earth like gravity
p.setGravity(0, 0, -9.8)

# Real time simulation on
p.setRealTimeSimulation(True)

# Infinite loop
while True:
    # Set gravity
    p.setGravity(0, 0, -9.8)
    #p.stepSimulation(1./100.)
    time.sleep(1. / 240.)
# end while
