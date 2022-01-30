import json as JSON
# import pybullet as p
import numpy as np

with open('./env_variables.json', 'r') as f:
    dic = JSON.loads(f.read())

shm_props = dic['shm_props']

car= dic['car']
wheels = dic['wheels']
steering= dic['steering']

## CONFIG env_variables
default_force = 10
lateral_constant = 2
vertical_constant = 2

def pyBullet_Controller(p,lateral,vertical):
    steeringAngle = np.atan2(lateral/vertical)
    targetVelocity = (lateral**2+vertical**2)**0.5
    for wheel in wheels:
        p.setJointMotorControl2(car,
                                wheel,
                                p.VELOCITY_CONTROL,
                                targetVelocity=-targetVelocity,
                                force=default_force)
    for steer in steering:
        p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steeringAngle)

import pybullet as p
p.connect(p.SHARED_MEMORY)
pyBullet_Controller(p,10,0.2)