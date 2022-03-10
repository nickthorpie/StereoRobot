### DOCS ###
"""
- Creates a pyBullet GUI Server
- initializes objects (Racecar,ground plane)
- Initializes stereo_camera
    - Writes stereo_camera to a shared memory buffer
- writes a .json file to ./env_variables.json containing:
    - the racecar ID
    - link ID of steering and wheels
    - The shared memory buffer

- Finally, runs a continuous loop to run the server
"""

### SETUP ENGINE ###
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print("current_dir=" + currentdir)
parentdir = os.path.join(currentdir, "../gym")
os.sys.path.insert(0, parentdir)

import pybullet as p
import pybullet_data
import time
CID = p.connect(p.GUI_SERVER)
if CID!=0:
    print("WARNING: CID SHOULD BE 0. ANOTHER PYBULLET SERVER MAY BE RUNNING")

### SETUP ENV ###
p.resetSimulation()
p.setGravity(0, 0, -10)
useRealTimeSim = 1
p.setRealTimeSimulation(useRealTimeSim)
p.loadSDF("resources/grey_field.sdf")

### SETUP CAR ###
car = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "racecar/racecar.urdf"))

inactive_wheels = [3, 5, 7]
wheels = [2]

for wheel in inactive_wheels:
    p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

steering = [4, 6]

### SETUP CAMERA ###
import numpy as np

# Initializes cam specs #
width = 128
height = 128
fov = 60
aspect = width / height
near = 0.02
far = 1

# Write a function that moves stereo_camera to the position and orientation, and captures.
def get_cam_image():
    cam = np.array(p.getLinkState(car, 11))
    cam_mtx = np.array(p.getMatrixFromQuaternion(cam[1])).reshape((3, 3)).T
    pos_vec = cam[0]
    fwd_vec = cam_mtx[0]
    fwd_vec = fwd_vec/np.linalg.norm(fwd_vec)
    up_vec = cam_mtx[2]
    cam_position = pos_vec + fwd_vec*0
    cam_target = cam_position + fwd_vec*10
    cam_up_vector = up_vec

    view_matrix = p.computeViewMatrix(cam_position, cam_target,cam_up_vector)
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    print(projection_matrix)
    # Get depth values using the OpenGL renderer
    images = p.getCameraImage(width,
                              height,
                              view_matrix,
                              projection_matrix,
                              shadow=True,
                              renderer=p.ER_BULLET_HARDWARE_OPENGL)
    #
    rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
    depth_buffer_opengl = np.reshape(images[3], [width, height])
    depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)
    # seg_opengl = np.reshape(images[4], [width, height]) * 1. / 255.
    # RGB_DEPTH = np.array([rgb_opengl, depth_opengl])
    return np.dstack((rgb_opengl,depth_opengl))


images = get_cam_image()

## Initialize shared memory
from multiprocessing import shared_memory
shm = shared_memory.SharedMemory(name='pyBullet_camera_shm',create=True, size=images.nbytes)
buff = np.ndarray(images.shape, dtype=images.dtype, buffer=shm.buf)
buff[:] = images[:]

## Write JSON File ##
import json

print("CURRENT PATH:", currentdir)
json_dict = {
    "car": car,
    "wheels": wheels,
    "steering": steering,
    "shm_props":{
        "name": shm.name,
        "shape":images.shape,
        "dtype":str(images.dtype)
    },
    "cam_props":{
        "width" : width,
        "height" : height,
        "fov" : fov,
        "aspect" : aspect,
        "near" : near,
        "far" : far
    }
}
for k,v in json_dict.items():
    print(f"{k}:{type(v)}")
    if type(v) is dict:
        for k1,v1 in v.items():
            print(f"\t{k1}:{type(v1)}")

with open(os.path.join(currentdir, "env_variables.json"), "w") as f:
    json.dump(json_dict, f)

## MAIN LOOP ##
while (True):
    images = get_cam_image()
    buff[:] = images[:]

    if (useRealTimeSim == 0):
        p.stepSimulation()
    time.sleep(0.01)




