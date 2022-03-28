import sys
import os
sys.path.append('../src')

import stereo_robot as robot

if __name__=="__main__":
    with robot.StereoCamera(upsidedown=True) as camera:
        camera.show_windows()