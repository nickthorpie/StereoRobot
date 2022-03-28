import sys,os
sys.path.append("/home/pi/PycharmProjects/StereoRobot/src")

# import stereo_robot
# from stereo_robot import StereoCamera
import cv2, json
import numpy as np

os.chdir(os.path.dirname(os.path.realpath(__file__)))

cam = cv2.VideoCapture(0)
cv2.namedWindow("test")


## PART 1: Capture image for calibration
while True:
    ret,frame=cam.read()
    if not ret:
        print("failed to grab frame")
        break
    cv2.imshow("test",frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    
cam.release()
cv2.destroyAllWindows()

## Part 2: Select colors that fit

saved_colors = []
HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
def mouse_event(event,x,y,flags,param):
    global mouseX,mouseY
    if event == cv2.EVENT_LBUTTONDBLCLK:
#         cv2.circle(frame,(x,y),3,(0,255,255),-1)
        saved_colors.append(HSV[y,x])
        mouseX,mouseY = x,y

cv2.namedWindow('image')
cv2.setMouseCallback('image',mouse_event)

while(1):
    cv2.imshow('image',frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cv2.destroyAllWindows()


## part 3: allow user to adjust with sliders
saved_colors = np.array(saved_colors)
R_max = np.max(saved_colors[:,0])
G_max = np.max(saved_colors[:,1])
B_max = np.max(saved_colors[:,2])

R_min = np.min(saved_colors[:,0])
G_min = np.min(saved_colors[:,1])
B_min = np.min(saved_colors[:,2])

def nothing(x):
    pass
cv2.namedWindow('threshold')
cv2.createTrackbar('R_HI','threshold',R_max,255,nothing)
cv2.createTrackbar('G_HI','threshold',G_max,255,nothing)
cv2.createTrackbar('B_HI','threshold',B_max,255,nothing)
cv2.createTrackbar('R_LO','threshold',R_min,255,nothing)
cv2.createTrackbar('G_LO','threshold',G_min,255,nothing)
cv2.createTrackbar('B_LO','threshold',B_min,255,nothing)

while True:
    R_HI = cv2.getTrackbarPos('R_HI','threshold')
    G_HI = cv2.getTrackbarPos('G_HI','threshold')
    B_HI = cv2.getTrackbarPos('B_HI','threshold')
    R_LO = cv2.getTrackbarPos('R_LO','threshold')
    G_LO = cv2.getTrackbarPos('G_LO','threshold')
    B_LO = cv2.getTrackbarPos('B_LO','threshold')
    
    bounds_HI = (R_HI,G_HI,B_HI)
    bounds_LO = (R_LO,G_LO,B_LO)
    thresh_mask = cv2.inRange(HSV,bounds_LO,bounds_HI)
    thresh_mask = cv2.cvtColor(thresh_mask,cv2.COLOR_GRAY2BGR)
    thresh_frame = HSV & thresh_mask
    thresh_frame_BGR = cv2.cvtColor(thresh_frame,cv2.COLOR_HSV2BGR)
    cv2.imshow("threshold",thresh_frame_BGR)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    
    if key == ord("s"):
        bounds_json = {
            'bounds_HI':bounds_HI,
            'bounds_LO':bounds_LO
            }
        with open('./thresh_data.json','w') as f:
            json.dump(bounds_json,f)
        print(f"SAVED TO {os.path.abspath('./')}/thresh_data.json")
cv2.destroyAllWindows()