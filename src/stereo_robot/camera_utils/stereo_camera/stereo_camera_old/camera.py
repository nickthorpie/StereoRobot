#
# # Copyright (C) 2019 Eugene Pomazov, <stereopi.com>, virt2real team
# #
# # This file is part of StereoPi tutorial scripts.
# #
# # StereoPi tutorial is free software: you can redistribute it
# # and/or modify it under the terms of the GNU General Public License
# # as published by the Free Software Foundation, either version 3 of the
# # License, or (at your option) any later version.
# #
# # StereoPi tutorial is distributed in the hope that it will be useful,
# # but WITHOUT ANY WARRANTY; without even the implied warranty of
# # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# # GNU General Public License for more details.
# #
# # You should have received a copy of the GNU General Public License
# # along with StereoPi tutorial.
# # If not, see <http://www.gnu.org/licenses/>.
# #
# # Most of this code is updated version of 3dberry.org project by virt2real
# #
# # Thanks to Adrian and http://pyimagesearch.com, as there are lot of
# # code in this tutorial was taken from his lessons.
# #
#
#
# import os
# print("CWD: ", os.path.abspath("./"))

from .config import _load_config as config
from . import arducam_mipicamera as arducam
from stereovision.calibration import StereoCalibration
import cv2
import warnings
import traceback


camera_params = config.load_camera_params()

def align_down(size, align):
    return (size & ~((align ) -1))

def align_up(size, align):
    return align_down(size + align - 1, align)

class StereoCamera:
    """
    Stereo camera handler. This should always be called from in context, and the cameras will not be initialized
    otherwise. Usage is as follows:
    with StereoCamera() as camera:
        all camera related activities.
    """
    def __init__(self,upsidedown = False):
        self.camera_params = config.load_camera_params()
        self.sbm_settings = config.load_sbm_config()
        self.upsidedown = upsidedown
        self.in_context = False
        # self.init_camera()
        # self.init_block_matcher()
    def __enter__(self):
        self.in_context = True
        self._init_camera()
        self._init_block_matcher()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()
        if exc_type is not None:
            traceback.print_exception(exc_type,exc_val,exc_tb)
            return False
        return True

    def cleanup(self):
        """cleanup method to ensure camera gets closed properly. Not properly closing the camera may lead to a non-functional
        driver, which can be solved by a system reboot.
        """
        self.camera.close_camera()
        del self.camera
        print("(hopefully) successful MIPI Cleanup")


    def _init_camera(self):
        assert self.in_context == True, "Must be called with context manager. See help for more"
        self.camera = arducam.mipi_camera()
        print("Open stereo_camera_old...")
        self.camera.init_camera()
        mode = self.camera_params['mode']
        self.camera.set_mode(mode)
        fmt = self.camera.get_format()
        print("Current mode: {},resolution: {}x{}".format(fmt['mode'], fmt['width'], fmt['height']))

        self.img_width= camera_params['width']
        self.img_height = camera_params['height']

        print ("Scaled image resolution:  " +str(self.img_width ) +" x  " +str(self.img_height))

        # Implementing calibration data
        print('Read calibration data and rectifying stereo pair...')
        self.calibration = StereoCalibration(input_folder='calib_result')

        print("Camera Successfully Initialized\n")

    def _init_block_matcher(self,
                           setLambda = 8000,
                           setSigmaColor = 1):
        data = self.sbm_settings

        left_matcher = cv2.StereoBM_create(numDisparities=0, blockSize=21)
        left_matcher.setPreFilterType(1)
        left_matcher.setPreFilterSize(data['preFilterSize'])
        left_matcher.setPreFilterCap(data['preFilterCap'])
        left_matcher.setMinDisparity(data['minDisparity'])
        left_matcher.setNumDisparities(data['numberOfDisparities'])
        left_matcher.setTextureThreshold(data['textureThreshold'])
        left_matcher.setUniquenessRatio(data['uniquenessRatio'])
        left_matcher.setSpeckleRange(data['speckleRange'])
        left_matcher.setSpeckleWindowSize(data['speckleWindowSize'])

        right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

        wls_filter = cv2.ximgproc.createDisparityWLSFilterGeneric(False)
        wls_filter.setLambda(setLambda);
        wls_filter.setSigmaColor(setSigmaColor)  # between 0.8 to 2.0

        self.right_matcher = right_matcher
        self.left_matcher  = left_matcher
        self.wls_filter = wls_filter
        print("Block Matcher Successfully Initialized. \n")

    def get_frame(self):
        """captures and returns an unsplit image pair"""
        frame = self.camera.capture(encoding = 'i420')
        fmt = self.camera.get_format()

        height = int(align_up(fmt['height'], 16))
        width = int(align_up(fmt['width'], 32))

        img_pair = frame.as_array.reshape(int(height * 1.5), width)
        img_pair = cv2.cvtColor(img_pair, cv2.COLOR_YUV2BGR_I420)
        img_pair = img_pair[:fmt['width'], : fmt['height']]
        if self.upsidedown == True:
            return cv2.rotate(img_pair,cv2.ROTATE_180)
        return img_pair

    def split_frame(self,img_pair):
        """splits an image pair"""
        img_height,img_width = (self.img_height,self.img_width)
        img_left = img_pair [0:img_height,0:int(img_width/2)]
        img_right = img_pair[0:img_height, int(img_width / 2):img_width]
        return img_left, img_left

    def to_rectified_pair(self,frame):
        """takes a image pair, splits it and rectifies each frame"""
        img_width = self.img_width
        img_height = self.img_height

        frame = cv2.resize(frame,(img_width,img_height))
        pair_img = cv2.cvtColor (frame, cv2.COLOR_BGR2GRAY)

        img_left = pair_img[0:img_height, 0:int(img_width / 2)]  # Y+H and X+W
        img_right = pair_img[0:img_height, int(img_width / 2):img_width]  # Y+H and X+W

        rectified_pair = self.calibration.rectify((img_left, img_right))
        return rectified_pair
    
    def get_rectified_pair(self):
        frame = self.get_frame()
        return self.to_rectified_pair(frame)

    def block_matcher_wls(self,rectified_pair):
        """Block matcher. Could be implemented in to_disparity, but keeping block_matcher
        methods separate to make implementations easier
        """
        dmLeft = rectified_pair[0]
        dmRight = rectified_pair[1]

        disparityL = self.left_matcher.compute(dmLeft, dmRight)
        disparityR = self.right_matcher.compute(dmLeft, dmRight)

        disparity = self.wls_filter.filter(disparityL, dmLeft, disparity_map_right=disparityR)

        return disparity

    def to_disparity(self,rectified_pair,block_matcher = None):
        """takes a rectified pair and applies a block_matcher
            can pass an optional block_matcher, but defaults to the block_matcher_wls methods
            block_matcher can any function of type block_matcher(rectified_pair=(imgL,imgR)) -> disparity
        """
        if block_matcher is None:
            block_matcher = self.block_matcher_wls
        disparity = block_matcher(rectified_pair)

        local_max = disparity.max()
        local_min = 0  # disparity.min()
        # 65535.0 = 2^16-1
        disparity_grayscale = (disparity - local_min) * (65535.0 / (local_max - local_min))
        disparity_fixtype = cv2.convertScaleAbs(disparity_grayscale, alpha=(255.0 / 65535.0))
        disparity_fixtype = cv2.flip(disparity_fixtype, 1)
        # disparity_color = cv2.applyColorMap(disparity_fixtype, cv2.COLORMAP_JET)
        return disparity_fixtype


    def to_colormap(self,disparity):
        disparity_color = cv2.applyColorMap(disparity,cv2.COLORMAP_JET)
        return disparity_color

    def to_pointcloud(self,disparity):
        disp_to_depth_mat = self.calibration.disp_to_depth_mat
        return cv2.reprojectImageTo3D(disparity,disp_to_depth_mat)

    def show_windows(self):
        # Initialize interface windows
        try:
            print("Opening Windows. Press Q to exit")
            cv2.namedWindow("disp_map")
            cv2.moveWindow("disp_map", 50, 100)
            cv2.namedWindow("left")
            cv2.moveWindow("left", 450, 100)
            cv2.namedWindow("right")
            cv2.moveWindow("right", 850, 100)
            while True:
                frame = self.get_frame()
                rectified_pair = self.to_rectified_pair(frame)
                disparity = self.to_disparity(rectified_pair)
                colormap = self.to_colormap(disparity)
                cv2.imshow("left", rectified_pair[0])
                cv2.imshow("right", rectified_pair[1])
                cv2.imshow("disp_map",colormap)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    raise Exception("User Exited windows")
        except Exception as E:
            print(E)
            raise(E)
        finally:
            cv2.destroyAllWindows()

<<<<<<< HEAD:src/stereo_robot/stereo_camera/camera.py
=======

    # def align_down(size, align):
#     return (size & ~((align ) -1))
#
# def align_up(size, align):
#     return align_down(size + align - 1, align)
#
# def get_frame(camera):
#     frame = camera.capture(encoding = 'i420')
#     fmt = camera.get_format()
#     height = int(align_up(fmt['height'], 16))
#     width = int(align_up(fmt['width'], 32))
#     image = frame.as_array.reshape(int(height * 1.5), width)
#     image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
#     image = image[:fmt['height'], :fmt['width']]
#     return image
#
# # Initialize the stereo_camera_old
# camera = arducam.mipi_camera()
# print("Open stereo_camera_old...")
# camera.init_camera()
# mode = camera_params['mode']
# camera.set_mode(mode)
# fmt = camera.get_format()
# print("Current mode: {},resolution: {}x{}".format(fmt['mode'], fmt['width'], fmt['height']))
#
# # Camera settimgs
# cam_width = fmt['width']
# cam_height = fmt['height']
#
# img_width= camera_params['width']
# img_height = camera_params['height']
# print ("Scaled image resolution:  " +str(img_width ) +" x  " +str(img_height))
#
# # stereo_camera_old.set_control(0x00980911, 1000)
# # Implementing calibration data
# print('Read calibration data and rectifying stereo pair...')
# calibration = StereoCalibration(input_folder='calib_result')
#
# # Initialize interface windows
# cv2.namedWindow("Image")
# cv2.moveWindow("Image", 50 ,100)
# cv2.namedWindow("left")
# cv2.moveWindow("left", 450 ,100)
# cv2.namedWindow("right")
# cv2.moveWindow("right", 850 ,100)
#
#
# disparity = np.zeros((img_width, img_height), np.uint8)
# left_matcher = cv2.StereoBM_create(numDisparities=0, blockSize=21)
#
# def stereo_depth_map(rectified_pair):
#     dmLeft = rectified_pair[0]
#     dmRight = rectified_pair[1]
#
#     disparityL = left_matcher.compute(dmLeft, dmRight)
#     disparityR = right_matcher.compute(dmLeft ,dmRight)
#
#     disparity = wls_filter.filter(disparityL ,dmLeft ,disparity_map_right=disparityR)
#
#     local_max = disparity.max()
#     local_min =  0  # disparity.min()
#     # confidence = wls_filter.getConfidenceMap()
#     # disparity[confidence==0] = 0
#
#     disparity_grayscale = (disparity - local_min) * (65535.0 / (local_max - local_min))
#     disparity_fixtype = cv2.convertScaleAbs(disparity_grayscale, alpha=(255.0 / 65535.0))
#     disparity_fixtype = cv2.flip(disparity_fixtype, 1)
#     disparity_color = cv2.applyColorMap(disparity_fixtype, cv2.COLORMAP_JET)
#
#     cv2.imshow("Image", disparity_color)
#     key = cv2.waitKey(1) & 0xFF
#     if key == ord("q"):
#         quit();
#     return disparity_color
#
#
# def load_map_settings(fName):
#     global SWS, PFS, PFC, MDS, NOD, TTH, UR, SR, SPWS, loading_settings
#     print('Loading parameters from file...')
#     f = open(fName, 'r')
#     data = json.load(f)
#     SWS = data['SADWindowSize']
#     PFS = data['preFilterSize']
#     PFC = data['preFilterCap']
#     MDS = data['minDisparity']
#     NOD = data['numberOfDisparities']
#     TTH = data['textureThreshold']
#     UR = data['uniquenessRatio']
#     SR = data['speckleRange']
#     SPWS = data['speckleWindowSize']
#     # sbm.setSADWindowSize(SWS)
#     left_matcher.setPreFilterType(1)
#     left_matcher.setPreFilterSize(PFS)
#     left_matcher.setPreFilterCap(PFC)
#     left_matcher.setMinDisparity(MDS)
#     left_matcher.setNumDisparities(NOD)
#     left_matcher.setTextureThreshold(TTH)
#     left_matcher.setUniquenessRatio(UR)
#     left_matcher.setSpeckleRange(SR)
#     left_matcher.setSpeckleWindowSize(SPWS)
#     f.close()
#     print('Parameters loaded from file ' + fName)
#
#
# load_map_settings("3dmap_set.txt")
#
# right_matcher = cv2.ximgproc.createRightMatcher(left_matcher);
#
# wls_filter = cv2.ximgproc.createDisparityWLSFilterGeneric(False)
# wls_filter.setLambda(8000);
# wls_filter.setSigmaColor(1)  # between 0.8 to 2.0
#
# # capture frames from the stereo_camera_old
# # for frame in stereo_camera_old.capture_continuous(capture, format="bgra", use_video_port=True, resize=(img_width,img_height)):
# while True:
#     frame = get_frame(camera)
#     frame = cv2.resize(frame, (img_width, img_height))
#
#     t1 = datetime.now()
#     pair_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     imgLeft = pair_img[0:img_height, 0:int(img_width / 2)]  # Y+H and X+W
#     imgRight = pair_img[0:img_height, int(img_width / 2):img_width]  # Y+H and X+W
#     rectified_pair = calibration.rectify((imgLeft, imgRight))
#     disparity = stereo_depth_map(rectified_pair)
#
#     # show the frame
#     cv2.imshow("left", imgLeft)
#     cv2.imshow("right", imgRight)
#
#     t2 = datetime.now()
#     print("DM build time: " + str(t2 - t1))
#
#
# class stereo_camera_old:
#     def __init__(self):
>>>>>>> master_update:src/stereo_robot/camera_utils/stereo_camera/stereo_camera_old/camera.py
