
import cv2, json, os
import numpy as np

os.chdir(os.path.dirname(os.path.realpath(__file__)))


def cam_read(camera):
    ret,frame = camera.read()
    if not ret:
        raise Exception("Could not read camera. Check cables. Might need to use legacy camera drivers")
    h,w,c = frame.shape
    print(frame.shape)
    cut_left = frame[:,:int(w/2)]
    cut_right = frame[:,int(w/2):]
    
    left = cv2.resize(cut_left,(w,h))
    right=cv2.resize(cut_right,(w,h))
    
    return ret,left

## Part 2: Select colors that fit

ROI_POINTS = []
def mouse_event(event,x,y,flags,param):
    global mouseX,mouseY
    if event == cv2.EVENT_LBUTTONDBLCLK:
#         cv2.circle(frame,(x,y),3,(0,255,255),-1)
        ROI_POINTS.append([x,y])
        mouseX,mouseY = x,y
        
cam = cv2.VideoCapture(0)
ret,frame=cam_read(cam)

cv2.namedWindow("orig_frame")
cv2.setMouseCallback('orig_frame',mouse_event)

while True:
    ret,frame=cam_read(cam)
    
    if not ret:
        print("failed to grab frame")
        break
    
    shapes = np.zeros_like(frame)
    out = frame.copy()

    for ROI in ROI_POINTS:
        cv2.circle(out,(ROI),2,(0,255,255),-1)
    if len(ROI_POINTS)==4:
        cv2.fillPoly(shapes, np.int_([ROI_POINTS]), (0,255, 0))
    mask = shapes.astype(bool)
    out[mask] = cv2.addWeighted(frame,0.5,shapes,0.5,0)[mask]

    cv2.imshow("orig_frame",out)
    
    
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord("q"):
        break
    
    elif key == ord("z"):
        if len(ROI_POINTS)!=0:
            ROI_POINTS.pop(-1)
        
    elif key == ord("s"):
        ROI_json = {
            'ROI_POINTS':ROI_POINTS
            }
        with open('ROI_JSON.json', 'w') as f:
            json.dump(ROI_json,f)
        print(f"SAVED TO {os.path.abspath('/')}/ROI_POINTS.json")

cam.release()
cv2.destroyAllWindows()
