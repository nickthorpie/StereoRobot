# import pybullet as p
# p.connect(p.SHARED_MEMORY)

import json as JSON
from multiprocessing import shared_memory
import numpy as np
import matplotlib.pyplot as plt
from resources.PyBullet.mprt_monkeypatch import remove_shm_from_resource_tracker as patch
patch()

class Camera:
    def __init__(self):
        self.init()
    def init(self):

        with open('env_variables.json', 'r') as f:
            dic = JSON.loads(f.read())

        # car= dic['car']
        # wheels = dic['wheels']
        # steering= dic['steering']
        #
        # cam_props=dic['cam_props']
        # width=cam_props['width']
        # height=cam_props['height']
        # fov=cam_props['fov']
        # aspect=cam_props['aspect']
        # near=cam_props['near']
        # far=cam_props['far']

        shm_props = dic['shm_props']
        self.shm_dtype = shm_props['dtype']
        self.shm_shape = shm_props['shape']
        self.existing_shm = shared_memory.SharedMemory(name=shm_props['name'])

    def get_image(self):
        images = np.ndarray(self.shm_shape, dtype=self.shm_dtype, buffer=self.existing_shm.buf).copy()

        rgb_opengl=images[:,:,:4]
        RGB_DEPTH=images[:,:,4]
        return rgb_opengl, RGB_DEPTH

    def show_image(self):
        rgb_opengl, RGB_DEPTH = self.get_image()
        plt.imshow(rgb_opengl)
        plt.show()
        return self

# rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
# depth_buffer_opengl = np.reshape(images[3], [width, height])
# depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)
# seg_opengl = np.reshape(images[4], [width, height]) * 1. / 255.
# RGB_DEPTH = np.array([rgb_opengl, depth_opengl])