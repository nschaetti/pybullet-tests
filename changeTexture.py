
# Imports
import pybullet as p
import time
import pybullet_data

# Connection to Physics
p.connect(p.GUI)

# Add a path where we want to search for data
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load to transparent plane from URDF file
# plane_uid_A = p.loadURDF("plane_transparent.urdf", [0, 0, 0])
plane_uid_B = p.loadURDF("plane_transparent.urdf", [0, 0, -1])

# Load texture from PNG
text_uid = p.loadTexture("tex256.png")

# Change color and textures
# p.changeVisualShape(plane_uid_A, -1, rgbaColor=[1, 1, 1, 0.5])
p.changeVisualShape(plane_uid_B, -1, rgbaColor=[1, 1, 1, 0.5])
p.changeVisualShape(plane_uid_B, -1, textureUniqueId=text_uid)

# Width & height
width = 256
height = 256

# A white image
image = [255] * width * height * 3

# Color values
colorR = 0
colorG = 0
colorB = 0

# ??
blue = 0

# Log states
log_id = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "renderbench.json")

# For 100k steps
for i in range(100000):
    # One step
    p.stepSimulation()

    # Width
    for i in range(width):
        # Height
        for j in range(height):
            # Change color
            image[(i + j * width) * 3 + 0] = i
            image[(i + j * width) * 3 + 1] = (j + blue) % 256
            image[(i + j * width) * 3 + 2] = blue
        # end for
    # end for

    # Increase blue component
    blue = blue + 1

    # Change texture
    p.changeTexture(text_uid, image, width, height)

    # Start measuring time
    start = time.time()

    # Get image from camera
    p.getCameraImage(300, 300, renderer=p.ER_BULLET_HARDWARE_OPENGL)

    end = time.time()

    # Print time for rendering
    print("rendering duration")
    print(end - start)
# end for

# Stop state logging
p.stopStateLogging(log_id)
