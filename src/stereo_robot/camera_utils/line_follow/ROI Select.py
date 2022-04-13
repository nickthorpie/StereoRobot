
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

def perspective_transform(frame,orig_frame_shape,roi_points,desired_roi_points):
    """
    Perform the perspective transform.
    :param: frame Current frame
    :param: plot Plot the warped image if True
    :return: Bird's eye view of the current lane
    """
    # Calculate the transformation matrix
    transformation_matrix = cv2.getPerspectiveTransform(
      roi_points, desired_roi_points)

    # Calculate the inverse transformation matrix           
    inv_transformation_matrix = cv2.getPerspectiveTransform(
      desired_roi_points, roi_points)

    # Perform the transform using the transformation matrix
    warped_frame = cv2.warpPerspective(
      frame, transformation_matrix, orig_frame_shape, flags=(
     cv2.INTER_LINEAR))
    
#     (thresh, binary_warped) = cv2.threshold(
#       warped_frame, 127, 255, cv2.THRESH_BINARY)           
#     warped_frame = binary_warped
    
    return warped_frame,inv_transformation_matrix     

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
cv2.namedWindow("warped frame")
cv2.setMouseCallback('orig_frame',mouse_event)
top_adj=0
bot_adj=0
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
        
        BL,TL,TR,BR = ROI_POINTS
        
        
        
        
        width,height = frame.shape[::-1][1:]
        padding = int(0.25 * width)
        DESIRED_ROI_POINTS = np.float32([
          [padding, 0], # Top-left corner
          [padding, height], # Bottom-left corner         
          [width-padding, height], # Bottom-right corner
          [width-padding, 0] # Top-right corner
        ])
        np_ROI_POINTS = np.float32(ROI_POINTS)
        np_ROI_POINTS[0]-=np.array([bot_adj,0])
        np_ROI_POINTS[1]-=np.array([top_adj,0])
        np_ROI_POINTS[2]+=np.array([top_adj,0])
        np_ROI_POINTS[3]+=np.array([bot_adj,0])
        warped_frame,inv_mat = perspective_transform(
            frame,
            (width,height),
            np_ROI_POINTS,
            DESIRED_ROI_POINTS)
        cv2.imshow("warped frame",warped_frame)
    mask = shapes.astype(bool)
    out[mask] = cv2.addWeighted(frame,0.5,shapes,0.5,0)[mask]

    cv2.imshow("orig_frame",out)
    
    
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord("q"):
        break
    if key == ord("i"):
        top_adj+=1
    if key == ord("k"):
        top_adj-=1
    if key == ord("j"):
        bot_adj-=1
    if key == ord("l"):
        bot_adj+=1
    elif key == ord("z"):
        if len(ROI_POINTS)!=0:
            ROI_POINTS.pop(-1)
        
    elif key == ord("s"):
        ROI_json = {
            'ROI_POINTS':np_ROI_POINTS
            }
        with open('ROI_JSON.json', 'w') as f:
            json.dump(ROI_json,f)
        print(f"SAVED TO {os.path.abspath('/')}/ROI_POINTS.json")

cam.release()
cv2.destroyAllWindows()
