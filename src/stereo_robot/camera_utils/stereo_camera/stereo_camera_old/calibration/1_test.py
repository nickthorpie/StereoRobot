import os

print("OY",os.path.dirname(__file__))

os.chdir(os.path.dirname(os.path.realpath(__file__)))
import arducam_mipicamera as arducam
import time
import cv2  # sudo apt-get install python-opencv
import json
import os
from datetime import datetime


def align_down(size, align):
    return (size & ~((align) - 1))


def align_up(size, align):
    return align_down(size + align - 1, align)


def input_number(num_range=None):
    loop = True
    while loop:
        try:
            input_str = input("Please enter the mode number:")
            num = int(input_str)
            if num_range != None and num not in num_range:
                print("Your input is out of range, please re-enter.")
                continue
            loop = False
        except:
            print("Please enter a number.")
            loop = True
    return num


def select_mode(camera):
    fmts = camera.get_support_formats()
    modes = []
    for fmt in fmts:
        desc = fmt['description']
<<<<<<< HEAD:src/stereo_robot/stereo_camera/calibration/1_test.py
        if desc.find(b'stereo camera') > 0 if desc != None else False:
=======
        if desc.find(b'stereo stereo_camera_old') > 0 if desc != None else False:
>>>>>>> master_update:src/stereo_robot/camera_utils/stereo_camera/stereo_camera_old/calibration/1_test.py
            modes.append(fmt['mode'])
            print("mode: {}, width: {}, height: {}".format(fmt['mode'], fmt['width'], fmt['height']))
    mode = input_number(modes)

    camera.set_mode(mode)

    fmt = camera.get_format()
    print("Current mode: {},resolution: {}x{}".format(fmt['mode'], fmt['width'], fmt['height']))


def get_frame(camera):
    frame = camera.capture(encoding='i420')
    fmt = camera.get_format()
    height = int(align_up(fmt['height'], 16))
    width = int(align_up(fmt['width'], 32))
    image = frame.as_array.reshape(int(height * 1.5), width)
    image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
    image = image[:fmt['height'], :fmt['width']]
    return image


def write_camera_params(fmt):
    result = json.dumps({'mode': fmt['mode'], 'width': fmt['width'], 'height': fmt['height'], }, \
                        sort_keys=True, indent=4, separators=(',', ':'))
    fName = 'camera_params.txt'
    f = open(str(fName), 'w')
    f.write(result)
    f.close()


# File for captured image
filename = './scenes/photo.png'
if __name__ == "__main__":
    try:
        camera = arducam.mipi_camera()
<<<<<<< HEAD:src/stereo_robot/stereo_camera/calibration/1_test.py
        print("Open camera...")
=======
        print("Open stereo_camera_old...")
>>>>>>> master_update:src/stereo_robot/camera_utils/stereo_camera/stereo_camera_old/calibration/1_test.py
        camera.init_camera()
        select_mode(camera)

        fmt = camera.get_format()
<<<<<<< HEAD:src/stereo_robot/stereo_camera/calibration/1_test.py
        # camera.set_control(0x00980911, 1000)
=======
        # stereo_camera_old.set_control(0x00980911, 1000)
>>>>>>> master_update:src/stereo_robot/camera_utils/stereo_camera/stereo_camera_old/calibration/1_test.py
        frame = None
        t2 = datetime.now()
        counter = 0
        avgtime = 0

        scale = 1280.0 / fmt['width']
        image_width = int(fmt['width'] * scale)
        image_height = int(fmt['height'] * scale)

        while cv2.waitKey(10) != ord('q'):
            counter += 1
            frame = get_frame(camera)
            frame = cv2.resize(frame, (image_width, image_height))
            cv2.imshow("Arducam", frame)

        t1 = datetime.now()
        timediff = t1 - t2
        avgtime = avgtime + (timediff.total_seconds())
<<<<<<< HEAD:src/stereo_robot/stereo_camera/calibration/1_test.py
        avgtime = avgtime / counter
        print("Average time between frames: " + str(avgtime))
        print("Average FPS: " + str(1 / avgtime))
        if (os.path.isdir("./scenes") == False):
            os.makedirs("./scenes")

=======
        avgtime = avgtime/counter
        print ("Average time between frames: " + str(avgtime))
        print ("Average FPS: " + str(1/avgtime))
        if (os.path.isdir("scenes")==False):
            os.makedirs("scenes")
        
>>>>>>> master_update:src/stereo_robot/camera_utils/stereo_camera/stereo_camera_old/calibration/1_test.py
        fmt['width'] = image_width
        fmt['height'] = image_height
        write_camera_params(fmt)
        cv2.imwrite(filename, frame)
<<<<<<< HEAD:src/stereo_robot/stereo_camera/calibration/1_test.py
        print("Close camera...")
=======
        print("Close stereo_camera_old...")
>>>>>>> master_update:src/stereo_robot/camera_utils/stereo_camera/stereo_camera_old/calibration/1_test.py
        camera.close_camera()
    except Exception as e:
        print(e)

