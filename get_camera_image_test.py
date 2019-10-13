#
# get_camera_image_test.py
# Get image from camera
#

# Imports
import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import pybullet_data
import time

# Connection to physics
direct = p.connect(p.GUI, options="--window scale=2")
# direct = p.connect(p.GUI)
#, options="--window_backend=2 --render_device=0")
#egl = p.loadPlugin("eglRendererPlugin")

# Add a path where we want to search for data
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load plane, R2D2 and two small cubes from URDF
p.loadURDF('plane.urdf')
p.loadURDF("r2d2.urdf", [0, 0, 1])
p.loadURDF('cube_small.urdf', basePosition=[0.0, 0.0, 0.025])
cube_trans = p.loadURDF('cube_small.urdf', basePosition=[0.0, 0.1, 0.025])

# Change color of the second cube
p.changeVisualShape(cube_trans, -1, rgbaColor=[1, 1, 1, 0.1])

# Height and width
width = 512
height = 512

# Rendering parameters
fov = 60
aspect = width / height
near = 0.02
far = 1

# Compute view matrix and project matrix
view_matrix = p.computeViewMatrix([0, 0, 0.5], [0, 0, 0], [1, 0, 0])
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

# Get depth values using the OpenGL renderer
images = p.getCameraImage(
    width,
    height,
    view_matrix,
    projection_matrix,
    shadow=True,
    renderer=p.ER_BULLET_HARDWARE_OPENGL
)

# Print information
print(len(images))

# Reshape image
rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.

# Reshape depth buffer
depth_buffer_opengl = np.reshape(images[3], [width, height])

# Reshape i don t know what
depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)
seg_opengl = np.reshape(images[4], [width, height]) * 1. / 255.

# Sleep 1 millisecond
time.sleep(1)

# Get depth values using Tiny renderer
images = p.getCameraImage(
    width,
    height,
    view_matrix,
    projection_matrix,
    shadow=True,
    renderer=p.ER_TINY_RENDERER
)

# Reshape depth buffer
depth_buffer_tiny = np.reshape(images[3], [width, height])
depth_tiny = far * near / (far - (far - near) * depth_buffer_tiny)

# Reshape image
rgb_tiny = np.reshape(images[2], (height, width, 4)) * 1. / 255.

# Reshape I don't know what
seg_tiny = np.reshape(images[4], [width, height]) * 1. / 255.

# Start orientation and position of the teddy bear
bear_start_pos1 = [-3.3, 0, 0]
bear_start_orientation1 = p.getQuaternionFromEuler([0, 0, 0])

# Load a new plane from URDV
bear_id_1 = p.loadURDF("plane.urdf", bear_start_pos1, bear_start_orientation1)

# Start orientation and position of the teddy bear
bear_start_pos2 = [0, 0, 0]
bear_start_orientation2 = p.getQuaternionFromEuler([0, 0, 0])

# Load teddy large object from URDF
bear_id_2 = p.loadURDF("teddy_large.urdf", bear_start_pos2, bear_start_orientation2)

# Load texture "checker"
texture_id = p.loadTexture("checker_grid.jpg")

# For each body in the scene
for b in range(p.getNumBodies()):
    # Change texture
    p.changeVisualShape(b, linkIndex=-1, textureUniqueId=texture_id)

    # For each joint
    for j in range(p.getNumJoints(b)):
        # Change texture of the link ?
        p.changeVisualShape(b, linkIndex=j, textureUniqueId=texture_id)
    # end for
# end for

# View matrix
viewMat = [
    0.642787516117096, -0.4393851161003113, 0.6275069713592529, 0.0, 0.766044557094574,
    0.36868777871131897, -0.5265407562255859, 0.0, -0.0, 0.8191521167755127, 0.5735764503479004,
    0.0, 2.384185791015625e-07, 2.384185791015625e-07, -5.000000476837158, 1.0
]

# Projection matrix
projMat = [
    0.7499999403953552, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0,
    0.0, 0.0, -0.02000020071864128, 0.0
]

# Get image from camera (from OpenGL)
images = p.getCameraImage(
    width,
    height,
    viewMatrix=viewMat,
    projectionMatrix=projMat,
    renderer=p.ER_BULLET_HARDWARE_OPENGL,
    flags=p.ER_USE_PROJECTIVE_TEXTURE,
    projectiveTextureView=viewMat,
    projectiveTextureProj=projMat
)

# Reshape image
proj_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
time.sleep(1)

# Get image from camera (from Tiny Renderer)
images = p.getCameraImage(
    width=width,
    height=height,
    viewMatrix=viewMat,
    projectionMatrix=projMat,
    renderer=p.ER_TINY_RENDERER,
    flags=p.ER_USE_PROJECTIVE_TEXTURE,
    projectiveTextureView=viewMat,
    projectiveTextureProj=projMat
)

# Reshape image
proj_tiny = np.reshape(images[2], (height, width, 4)) * 1. / 255.

# Plot depth
plt.subplot(1, 2, 1)
plt.imshow(depth_opengl, cmap='gray', vmin=0, vmax=1)
plt.title('Depth OpenGL3')
plt.subplot(1, 2, 2)
plt.imshow(depth_tiny, cmap='gray', vmin=0, vmax=1)
plt.title('Depth TinyRenderer')
plt.show()

# Plot RGB images
plt.subplot(1, 2, 1)
plt.imshow(rgb_opengl)
plt.title('RGB OpenGL3')
plt.subplot(1, 2, 2)
plt.imshow(rgb_tiny)
plt.title('RGB Tiny')
plt.show()

# Plot segmentation images
plt.subplot(1, 2, 1)
plt.imshow(seg_opengl)
plt.title('Seg OpenGL3')
plt.subplot(1, 2, 2)
plt.imshow(seg_tiny)
plt.title('Seg Tiny')
plt.show()

# Plot projection from OpenGL and Tiny
plt.subplot(1, 2, 1)
plt.imshow(proj_opengl)
plt.title('Proj OpenGL')
plt.subplot(1, 2, 2)
plt.imshow(proj_tiny)
plt.title('Proj Tiny')
plt.subplots_adjust(hspace=0.7)
plt.show()
