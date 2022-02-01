import json as JSON
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


def pybullet_controller(p,lateral,vertical):
    target_velocity = 1
    steering_angle = np.arctan2(lateral,vertical)
    if steering_angle > np.pi/2:
        steering_angle = steering_angle-np.pi
        target_velocity = -1
    elif steering_angle < -np.pi/2:
        steering_angle = steering_angle+np.pi
        target_velocity = -1

    target_velocity *= (lateral**2+vertical**2)**0.5
    for wheel in wheels:
        p.setJointMotorControl2(car,
                                wheel,
                                p.VELOCITY_CONTROL,
                                targetVelocity=target_velocity,
                                force=default_force)
    for steer in steering:
        p.setJointMotorControl2(car,
                                steer,
                                p.POSITION_CONTROL,
                                targetPosition=steering_angle)


import pybullet as p

p.connect(p.SHARED_MEMORY)
pybullet_controller(p,
                    0.,0)