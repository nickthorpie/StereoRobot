import sys
import os
sys.path.append('../src')

# import os
# os.chdir(os.path.join(os.path.abspath(os.path.dirname(__file__)),"..","src"))
# print(os.path.abspath("./"))

import stereo_robot as robot
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import proj3d
with robot.StereoCamera() as cam:
    frame = cam.get_frame()
    rectPair = cam.to_rectified_pair(frame)
    disp = cam.to_disparity(rectPair)
    pcl = cam.to_pointcloud(disp)
    print(pcl.shape)
    fig = plt.figure(figsize=(8,8))
    ax = fig.add_subplot(111,projection='3d')
    ax.view_init(azim=0,elev=20)
    ax.scatter(pcl[:,:,0],pcl[:,:,1],pcl[:,:,2])

    plt.show()