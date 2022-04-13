try:
    from .line_follow import LineFollow
except:
    print("Can't import .line_follow relatively, trying import linefollow")
    from line_follow import LineFollow
import cv2,time
import numpy as np

from pyzbar import pyzbar

def display(im, bbox):
    bbox = np.array(bbox,dtype=int)
    n = len(bbox)
    for j in range(n):
        print(j)
        arg1 = tuple(bbox[j][0])
        arg2 = tuple(bbox[ (j+1) % n][0])
        
        cv2.line(im, arg1, arg2, (255,0,0), 3)

    # Display results
    cv2.imshow("Results", im)



def cam_read(camera):
    ret,frame = camera.read()
    if not ret:
        errmsg = "Could not read camera. \n(1) try sudo pkill python\n"
        errmsg+= "(2) Check cables. \n(3) Might need to use legacy camera drivers"
        raise Exception(errmsg)
    h,w = frame.shape[:2]
    cut_left = frame[:,:int(w/2)]
    cut_right = frame[:,int(w/2):]
    
    left = cv2.resize(cut_left,(w,h))
    right=cv2.resize(cut_right,(w,h))
    
    return ret,left


def CameraClient(q_camera,cam):
    
    message = {"top":None,"bottom":None,"confidence":0,"barcode":None}
    q_camera.put(message)
#     cam = cv2.VideoCapture(0)
    ret,frame = cam_read(cam)
    
    line = LineFollow(frame)
    barcodeData = None
    
    cv2.namedWindow("lane_line_markings")
    cv2.namedWindow("result")
#     cv2.namedWindow("warped_frame")
    while True:
        ret,frame = cam_read(cam)
        
        (ret,   (p1,p2),   lane_line_markings,
         result,confidence,barcodeData) = line.find_line(frame,)
        capture_time = time.time()
        h,w = lane_line_markings.shape[:2]
        
        #TODO: This is a messy way of passing the QR_old code info.
        # The QR_old reader is burried in LineFollow(), but we need the
        # information in direction_control.py. It's path looks like
        # LineFollow.step() -> camera_client -> camera_queue -> direction_control
        # It is initiallized in camera_queue as None, then we update
        # to the QR_old value when it's first found. in direction_control
        # we check if the QR_old value is new by comparing it with most recent value
        
        if barcodeData is not None:           
            message["barcode"] = barcodeData
        if ret:
            message["top"] = p2
            message["bottom"] = p1
            message["confidence"] = max(min(confidence*100,100),0)
            message["message_time"]=capture_time
            q_camera.put(message)
            
        if ret:
            cv2.imshow("result",result)
        else:
            cv2.imshow("result",frame)
        cv2.imshow("lane_line_markings",lane_line_markings)
        cv2.waitKey(1)

if __name__=="__main__":
    from multiprocessing import Queue
    cam = cv2.VideoCapture(0)
    q = Queue()
    try:
        proc = CameraClient(q,cam)
    finally:
        cv2.destroyAllWindows()