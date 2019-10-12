
# Imports
import pybullet as p
import pybullet_data
import time

# Connection to physics
rep = p.connect(p.GUI)

# Add a path where we want to search for data
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a plane and a cube from URDF
plane_id = p.loadURDF("plane.urdf", useMaximalCoordinates=False)
cube_id = p.loadURDF("cube_collisionfilter.urdf", [0, 0, 3], useMaximalCoordinates=False)

# Filter group and mask
collision_filter_group = 0
collision_filter_mask = 0

# Set filter group and mask
p.setCollisionFilterGroupMask(cube_id, -1, collision_filter_group, collision_filter_mask)

# Collision enabled
enable_collision = True

# Collision between plane and cube enabled
p.setCollisionFilterPair(plane_id, cube_id, -1, -1, enable_collision)

# Set real time
p.setRealTimeSimulation(True)

# Set gravity
# p.setGravity(0, 0, -9,8)

# Run while connected
while p.isConnected():
    # Sleep timestep
    time.sleep(1. / 240.)
    p.setGravity(0, 0, -9.8)
# end while
