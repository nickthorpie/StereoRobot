
import cv2, json, os
import numpy as np

os.chdir(os.path.dirname(os.path.realpath(__file__)))




## Part 2: Select colors that fit

ROI_POINTS = []
def mouse_event(event,x,y,flags,param):
    global mouseX,mouseY
    if event == cv2.EVENT_LBUTTONDBLCLK:
#         cv2.circle(frame,(x,y),3,(0,255,255),-1)
        if CORNER_SELECTOR != 0 and len(ROI_POINTS)<CORNER_SELECTOR:
            ROI_POINTS[CORNER_SELECTOR-1] = [x,y]
        elif len(ROI_POINTS)>=4:
            pass
        else:
            ROI_POINTS.append([x,y])
            mouseX,mouseY = x,y

            

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
     cv2.INTER_LANCZOS4))
    
#     (thresh, binary_warped) = cv2.threshold(
#       warped_frame, 127, 255, cv2.THRESH_BINARY)           
#     warped_frame = binary_warped
    
    return warped_frame,inv_transformation_matrix

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

def adaptiveThreshold(img):
    # convert img to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # do adaptive threshold on gray image
    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 31, 31)

    # make background of input white where thresh is white
    result = img.copy()
    result[thresh==255] = (255,255,255)
    return result

CORNER_SELECTOR = 0

from pyzbar import pyzbar
if __name__=="__main__":
    cv2.namedWindow("orig_frame")
    cv2.namedWindow("warped frame")
    cv2.setMouseCallback('orig_frame',mouse_event)
    cv2.namedWindow('Results')
    cv2.namedWindow("Rectified QRCode")
    
    cam = cv2.VideoCapture(0)
    ret,frame=cam.read()
    
    width,height = frame.shape[::-1][1:]
    padding = int(0.25 * width)
    DESIRED_ROI_POINTS = np.float32([
      [padding, 0], # Top-left corner
      [padding, height], # Bottom-left corner         
      [width-padding, height], # Bottom-right corner
      [width-padding, 0] # Top-right corner
    ])
    

    while True:
        ret,frame=cam.read()
        
        if not ret:
            print("failed to grab frame")
            break
        
        shapes = np.zeros_like(frame)
        out = frame.copy()

        for ROI in ROI_POINTS:
            cv2.circle(out,(ROI),2,(0,255,255),-1)
        if len(ROI_POINTS)==4:
            cv2.fillPoly(shapes, np.int_([ROI_POINTS]), (0,255, 0))
            warpedframe,invMat = perspective_transform(frame,(width,height),np.float32(ROI_POINTS),DESIRED_ROI_POINTS)
#             warpedframe = adaptiveThreshold(warpedframe)
            
            barcodes = pyzbar.decode(warpedframe)
            for barcode in barcodes:
                (x,y,w,h) = barcode.rect
                cv2.rectangle(warpedframe,(x,y),(x+w,y+h),(0,0,255),2)
                barcodeData = barcode.data.decode("utf-8")
                barcodeType = barcode.type
                text = "{} ({})".format(barcodeData, barcodeType)
                cv2.putText(warpedframe, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 0, 255), 2)
            cv2.imshow("warped frame",warpedframe)

        mask = shapes.astype(bool)
        out[mask] = cv2.addWeighted(frame,0.5,shapes,0.5,0)[mask]

        cv2.imshow("orig_frame",out)
        
        
        key = cv2.waitKey(1) & 0xFF
        
        
        # Key handlers
        if key == ord("1"):
            CORNER_SELECTOR = 1
        elif key == ord("2"):
            CORNER_SELECTOR = 2
        elif key == ord("3"):
            CORNER_SELECTOR = 3
        elif key == ord("4"):
            CORNER_SELECTOR = 4
        else:
            CORNER_SELECTOR = 0
        
        if key == ord("q"):
            break
        
        elif key == ord("z"):
            if len(ROI_POINTS)!=0:
                ROI_POINTS.pop(-1)
            
        elif key == ord("s"):
            ROI_json = {
                'ROI_POINTS':ROI_POINTS
                }
            with open('./ROI_JSON.json','w') as f:
                json.dump(ROI_json,f)
            print(f"SAVED TO {os.path.abspath('/')}/ROI_POINTS.json")

    cam.release()
    cv2.destroyAllWindows()
