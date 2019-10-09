
# Imports
import pybullet as p
import time
import math

# Connection
p.connect(p.GUI)

# Reset -> no ground plane
p.resetSimulation()

# Reset camera position
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING)

#
# 1.
# Create a collision sphere directly from
# radius property.
#
sphere_radius = 0.05
col_sphere_id = p.createCollisionShape(p.GEOM_SPHERE, radius=sphere_radius)

#
# 2.
# Create a mesh directly from meshes and indices
#
vertices = [[-0.246350, -0.246483, -0.000624], [-0.151407, -0.176325, 0.172867],
            [-0.246350, 0.249205, -0.000624], [-0.151407, 0.129477, 0.172867],
            [0.249338, -0.246483, -0.000624], [0.154395, -0.176325, 0.172867],
            [0.249338, 0.249205, -0.000624], [0.154395, 0.129477, 0.172867]]
indices = [
    0, 3, 2, 3, 6, 2, 7, 4, 6, 5, 0, 4, 6, 0, 2, 3, 5, 7, 0, 1, 3, 3, 7, 6, 7, 5, 4, 5, 1, 0, 6, 4,
    0, 3, 1, 5
]
stone_id = p.createCollisionShape(
    p.GEOM_MESH,
    vertices=vertices,
    indices=indices
)

#
# 3.
# Create a box directly from
# Lengths, heights and widths
#
box_half_length = 0.5
box_half_width = 2.5
box_half_height = 0.1
segment_length = 5
col_box_id = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[box_half_length, box_half_width, box_half_height]
)

mass = 1
visual_shape_id = -1
segment_start = 0

# 4.
# Create boxes as a multiple body
#

# For each segment
for i in range(segment_length):
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col_box_id,
        basePosition=[segment_start, 0, -0.1]
    )
    segment_start -= 1
# end for

# For each segment
for i in range(segment_length):
    height = 0
    if i % 2:
        height = 1
    # end if
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col_box_id,
        basePosition=[segment_start, 0, -0.1 + height]
    )
    segment_start -= 1
# end for

base_orientation = p.getQuaternionFromEuler(
    [
        math.pi / 2.0,
        0,
        math.pi / 2.0
    ]
)

# For each segment
for i in range(segment_length):
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col_box_id,
        basePosition=[segment_start, 0, -0.1]
    )
    segment_start -= segment_start
    if i % 2:
        p.createMultiBody(
            baseMass=0,
            basePosition=[segment_start, i % 3, -0.1 + height + box_half_width],
            baseOrientation=base_orientation
        )
    # end if
# end for

# For each segment
for i in range(segment_length):
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col_box_id,
        basePosition=[segment_start, 0, -0.1]
    )
    width = 4
    for j in range(width):
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=stone_id,
            basePosition=[segment_start, 0.5 * (i % 2) + j - width / 2., 0]
        )
    # end for
    segment_start = segment_start - 1
# end for

link_masses = [1]
link_collision_shape_indices = [col_box_id]
link_visual_shape_indices = [-1]
link_positions = [[0, 0, 0]]
link_orientations = [[0, 0, 0, 1]]
link_inertial_frame_positions = [[0, 0, 0]]
link_inertial_frame_orientations = [[0, 0, 0, 1]]
indices = [0]
joint_types = [p.JOINT_REVOLUTE]
axis = [[1, 0, 0]]

base_orientation = [0, 0, 0, 1]

# For each segment
for i in range(segment_length):
    box_id = p.createMultiBody(
        0,
        col_sphere_id,
        -1,
        [segment_start, 0, -0.1],
        base_orientation,
        linkMasses=link_masses,
        linkCollisionShapeIndices=link_collision_shape_indices,
        linkVisualShapeIndices=link_visual_shape_indices,
        linkPositions=link_positions,
        linkOrientations=link_orientations,
        linkInertialFramePositions=link_inertial_frame_positions,
        linkInertialFrameOrientations=link_inertial_frame_orientations,
        linkParentIndices=indices,
        linkJointTypes=joint_types,
        linkJointAxis=axis
    )
    p.changeDynamics(
        box_id,
        -1,
        spinningFriction=0.001,
        rollingFriction=0.001,
        linearDamping=0.0
    )

    print(p.getNumJoints(box_id))

    for joint in range(p.getNumJoints(box_id)):
        target_velocity = 10
        if i % 2:
            target_velocity = -10
        # end if
        p.setJointMotorControl2(
            box_id,
            joint,
            p.VELOCITY_CONTROL,
            targetVelocity=target_velocity,
            force=100
        )
    # end for

    # Segment start
    segment_start = segment_start - 1.0
# end for

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

# Continue
while(True):
    cam_data = p.getDebugVisualizerCamera()
    view_mat = cam_data[2]
    proj_mat = cam_data[3]
    p.getCameraImage(
        256,
        256,
        viewMatrix=view_mat,
        projectionMatrix=proj_mat,
        renderer=p.ER_BULLET_HARDWARE_OPENGL
    )
    keys = p.getKeyboardEvents()
    print(keys)
    p.stepSimulation()
    time.sleep(0.01)
# end while
