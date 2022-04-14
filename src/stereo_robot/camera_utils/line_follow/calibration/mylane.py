
import cv2, json, os
import numpy as np
# from sklearn.linear_model import LinearRegression
os.chdir(os.path.dirname(os.path.realpath(__file__)))

with open("thresh_data.json", 'r') as f:
    thresh_data = json.load(f)
    color_thresh_HI = thresh_data['bounds_HI']
    color_thresh_LO = thresh_data['bounds_LO']

with open("ROI_JSON.json", 'r') as f:
    ROI_data = json.load(f)
    TL,BL,BR,TR = ROI_data['ROI_POINTS']
    ROI_POINTS = np.float32([TL,BL,BR,TR])

def filter_colors(image):
    """
    Filter the image to include only yellow and white pixels
    """

    # Filter yellow pixels
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array(color_thresh_LO)
    upper_yellow = np.array(color_thresh_HI)
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    yellow_image = cv2.bitwise_and(image, image, mask=yellow_mask)
    
    # Combine the two above images
    
    return yellow_image


def perspective_transform(frame,roi_points,desired_roi_points):
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
      frame, transformation_matrix, frame.shape[::-1][1:], flags=(
     cv2.INTER_LINEAR)) 

#     # Convert image to binary
#     (thresh, binary_warped) = cv2.threshold(
#       warped_frame, 127, 255, cv2.THRESH_BINARY)           
#     warped_frame = binary_warped

    return warped_frame,inv_transformation_matrix        
def gaussian_blur(img, kernel_size):
	"""Applies a Gaussian Noise kernel"""
	return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def houghlines(img):
    Rres = 1
    Thetares = 1*np.pi/180
    Threshold = 1
    minLineLength = 1
    maxLineGap = 100
    lines = cv2.HoughLinesP(img,Rres,Thetares,Threshold,minLineLength,maxLineGap)
    N = lines.shape[0]
    for i in range(N):
        x1 = lines[i][0][0]
        y1 = lines[i][0][1]    
        x2 = lines[i][0][2]
        y2 = lines[i][0][3]    
        cv2.line(I,(x1,y1),(x2,y2),(255,0,0),2)

def init(frame):
    orig_image_size = frame.shape[::-1][1:]
 
    width = orig_image_size[0]
    height = orig_image_size[1]
    
    padding = int(0.25 * width)
    DESIRED_ROI_POINTS = np.float32([
      [padding, 0], # Top-left corner
      [padding, orig_image_size[1]], # Bottom-left corner         
      [orig_image_size[
        0]-padding, orig_image_size[1]], # Bottom-right corner
      [orig_image_size[0]-padding, 0] # Top-right corner
    ])
    
    return orig_image_size,DESIRED_ROI_POINTS

def theta_to_Q1Q2(theta):
    while theta<0:
        theta+=np.pi*2
    while theta>np.pi*2:
        theta-=np.pi*2
    if theta>np.pi:
        theta-=np.pi
    return theta

def houghLines(warped_img):
    minLineLength = 100
    maxLineGap = 10

    edges = cv2.Canny(warped_img,50,150,apertureSize = 3)
    lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength,maxLineGap)
    thetas = []
    points = []
    if lines is None:
        return None,None
    for x1,y1,x2,y2 in lines[:,0]:
        cv2.line(warped_img,(x1,y1),(x2,y2),(0,0,255),2)
        theta = theta_to_Q1Q2(np.arctan2(y2-y1,x2-x1))
        # mags = ((x2-x1)**2+(y2-y1)**2)**0.5
        thetas.append(np.arctan2(y2-y1,x2-x1))
    return lines[:,0],thetas

def extractLines(img,lines,thetas):
    # Get most commonly found theta using a histogram
    thetas = np.array(thetas)
    count,bins = np.histogram(thetas,bins=20,range=(0,np.pi))
    ind= np.where(count==np.max(count))[0][0]
    theta1 = bins[ind]


    # Look for other values in the histogram (scrub out last max +- pi/3)
    LO = theta_to_Q1Q2(theta1 - np.pi/3)
    HI = theta_to_Q1Q2(theta1 + np.pi/3)
    theta_range = [np.min((LO,HI)),np.max((LO,HI))]
    trimmed_thetas = thetas[(thetas<theta_range[0])&(thetas>theta_range[1])]
    count2,bins2 = np.histogram(trimmed_thetas,bins=20,range=(0,np.pi))

    if any(count2>10):
        ind= np.where(count2==np.max(count))[0][0]
        theta2 = bins[ind]
    else:
        theta2 = None
    
    # If we find another theta, take theta 1 as one closest to dead ahead
    if theta2 is not None:
        if abs(theta2-np.pi/2) < abs(theta1 - np.pi/2):
            dummy = theta1
            theta1 = theta2
            theta2 = dummy
    
    # Grab all lines that are +- 0.1 theta and do a linreg
    relevant_lines = lines[(thetas>theta1-0.1) & (thetas<theta1+0.1)]
    if len(relevant_lines) >2:
        
        xs = np.hstack((relevant_lines[:,0],relevant_lines[:,2]))
        ys = np.hstack((relevant_lines[:,1],relevant_lines[:,3]))

        model = np.polyfit(ys,xs,1)

        top_of_line = model[1]      ## x value where interesects top of image
        slope = model[0]

        bottom_of_line = (top_of_line + slope*img.shape[1]).astype(int)

        line1 = (int(top_of_line),0,int(bottom_of_line),img.shape[1])
    else:
        line1 = None
            
    if theta2 is not None:
        relevant_lines = lines[:,0][(thetas>theta2-0.1) & (thetas<theta2+0.1)]
        if len(relevant_lines)>2:
            xs = np.hstack((relevant_lines[:,0],relevant_lines[:,2]))
            ys = np.hstack((relevant_lines[:,1],relevant_lines[:,3])).reshape((-1,1))

            model = np.polyfit(ys,xs,1)

            top_of_line = model[1]      ## x value where interesects top of image
            slope = model[0]
        
            bottom_of_line = (top_line + slope*img.shape[1]).astype(int)

            line2 = (int(top_of_line),0,int(bottom_of_line),im.shape[1])
        else:
            line2 = None
    else: 
        line2 = None
    
    return line1,line2

def split_pair(pair):
    ret,pair = cam.read()
    left_frame = cv2.resize(pair[:,:int(len(pair)/2)],pair.shape[:-1])
    right_frame = cv2.resize(pair[:,int(len(pair)/2):],pair.shape[:-1])
    return left_frame,right_frame
if __name__ == '__main__':
    cam = cv2.VideoCapture(0)
    ret,frame = cam.read()
#     left_frame,right_frame = split_pair(frame)
    
#     frame = left_frame
    
    cv2.namedWindow('original')
    cv2.namedWindow('warped')
    cv2.namedWindow('blurred')
    cv2.namedWindow('binary')
    
    img_size, desired_roi = init(frame)
    while True:
        ret,pair = cam.read()
#         left_frame = cv2.resize(pair[:,:int(len(pair)/2)],pair.shape[:-1])
#         right_frame = cv2.resize(pair[:,int(len(pair)/2):],pair.shape[:-1])
#         frame=left_frame
        filtered = filter_colors(pair)
        
        
        blurred = gaussian_blur(filtered,31)
        warped_frame,inv_mat = perspective_transform(
            blurred,
            ROI_POINTS,
            desired_roi)
        
        binary = cv2.cvtColor(warped_frame,cv2.COLOR_BGR2GRAY)
        binary[binary>50 ]=255
        
        lines,thetas = houghLines(binary)
        if lines is not None:
            line1,line2 = extractLines(binary,lines,thetas)
            if line1 is not None:
                cv2.line(warped_frame,line1[:2],line1[2:],(0,255,0),30);
            if line2 is not None:
                cv2.line(warped_frame,line2[:2],line2[2:],(0,255,0),30);
        
        cv2.imshow('original',frame)
        cv2.imshow('warped',warped_frame)
        cv2.imshow('blurred',blurred)
        cv2.imshow('binary',binary)
        key = cv2.waitKey(1)
        
        if key == ord('q'):
            break
        
    cv2.destroyAllWindows()