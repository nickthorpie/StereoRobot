import cv2, json, os
import numpy as np # Import the NumPy scientific computing library
os.chdir(os.path.dirname(os.path.realpath(__file__)))

# Author: Addison Sears-Collins
# https://automaticaddison.com
# Description: A collection of methods to detect help with edge detection
with open("thresh_data.json", 'r') as f:
    thresh_data = json.load(f)
    thresh_HI = thresh_data['bounds_HI']
    thresh_LO = thresh_data['bounds_LO']

with open("ROI_JSON.json", 'r') as f:
    ROI_data = json.load(f)
    TL,BL,BR,TR = ROI_data['ROI_POINTS']
    ROI_POINTS = np.float32([TL,BL,BR,TR])



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

def get_line_markings(orig_frame):
    """
    Isolates lane lines.
   
      :param frame: The camera frame that contains the lanes we want to detect
    :return: Binary (i.e. black and white) image containing the lane lines.
    """
    hsv = cv2.cvtColor(orig_frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array(thresh_LO)
    upper_yellow = np.array(thresh_HI)
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
#     yellow_image = cv2.bitwise_and(image, image, mask=yellow_mask)
    
    # Combine the two above images
    
    return yellow_mask

def calculate_histogram_vert(warped_frame):
    """
    Calculate the image histogram to find peaks in white pixel count
         
    :param frame: The warped image
    :param plot: Create a plot if True
    """
    frame = warped_frame
             
    # Generate the histogram
    histogram = np.sum(frame[int(
              frame.shape[0]/2):,:], axis=0)
        
    return histogram

def histogram_peak(warped_frame):
    histogram = calculate_histogram_vert(warped_frame)
    x_base = np.argmax(histogram)
    return x_base

def get_lane_line_indices_sliding_windows(warped_frame,MINPIX,MARGIN):
    """
    Get the indices of the lane line pixels using the 
    sliding windows technique.
         
    :param: plot Show plot or not
    :return: Best fit lines for the left and right lines of the current lane 
    """
    no_of_windows = 10
    minpix = MINPIX #????
    # Sliding window width is +/- margin
    margin = MARGIN #???
 
    frame_sliding_window = warped_frame.copy()
 
    # Set the height of the sliding windows
    window_height = np.int(warped_frame.shape[0]/no_of_windows)       
 
    # Find the x and y coordinates of all the nonzero 
    # (i.e. white) pixels in the frame. 
    nonzero = warped_frame.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1]) 
         
    # Store the pixel indices for the left and right lane lines
    lane_inds = []
         
    # Current positions for pixel indices for each window,
    # which we will continue to update
    x_base= histogram_peak(warped_frame)
    x_current = x_base
 
    # Go through one window at a time
         
    for window in range(no_of_windows):
       
      # Identify window boundaries in x and y (and right and left)
      win_y_low = warped_frame.shape[0] - (window + 1) * window_height
      win_y_high = warped_frame.shape[0] - window * window_height
      win_x_low = x_current - margin
      win_x_high = x_current + margin
      cv2.rectangle(frame_sliding_window,(win_x_low,win_y_low),(
        win_x_high,win_y_high), (255,255,255), 2)

 
      # Identify the nonzero pixels in x and y within the window
      good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                          (nonzerox >= win_x_low) & (
                           nonzerox < win_x_high)).nonzero()[0]
                           
      # Append these indices to the lists
      lane_inds.append(good_inds)
         
      # If you found > minpix pixels, recenter next window on mean position
      if len(good_inds) > minpix:
        x_current = np.int(np.mean(nonzerox[good_inds]))
                     
    # Concatenate the arrays of indices
    lane_inds = np.concatenate(lane_inds)
 
    # Extract the pixel coordinates for the left and right lane lines
    x = nonzerox[lane_inds]
    y = nonzeroy[lane_inds] 
    
    if len(x)<3 or len(y)<3:
        return None
    # Fit a second order polynomial curve to the pixel coordinates for
    # the left and right lane lines
    fit = np.polyfit(y, x, 1)
             
    return fit
def get_lane_line_previous_window(warped_frame, fit,MINPIX,MARGIN):
    """
    Use the lane line from the previous sliding window to get the parameters
    for the polynomial line for filling in the lane line
    :param: left_fit Polynomial function of the left lane line
    :param: right_fit Polynomial function of the right lane line
    :param: plot To display an image or not
    """
    # margin is a sliding window parameter
    margin = MARGIN
 
    # Find the x and y coordinates of all the nonzero 
    # (i.e. white) pixels in the frame.         
    nonzero = warped_frame.nonzero()  
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
         
    # Store left and right lane pixel indices
    lane_inds = ((nonzerox > (fit[0]*(
      nonzeroy) + fit[1] - margin)) & (
      nonzerox < (fit[0]*(
      nonzeroy) + fit[1] + margin))) 
 
    # Get the left and right lane line pixel locations  
    x = nonzerox[lane_inds]
    y = nonzeroy[lane_inds]  
 
     
    # Fit a second order polynomial curve to each lane line
    fit = np.polyfit(y, x, 1)
    
    # Create the x and y values to plot on the image
#     ploty = np.linspace(
#       0, warped_frame.shape[0]-1, warped_frame.shape[0])
    ploty = np.linspace(min(y),max(y),int(max(y)-min(y)),dtype=int)
    fitx = fit[0]*ploty + fit[1] 
    
    
    return ploty,fitx

def overlay_lane_lines(warped_frame,fitx,ploty):
    """
    Overlay lane lines on the original frame
    :param: Plot the lane lines if True
    :return: Lane with overlay
    """
    # Generate an image to draw the lane lines on 
    warp_zero = np.zeros_like(warped_frame).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))       
         
    # Recast the x and y points into usable format for cv2.fillPoly()
    pts = np.array([np.transpose(np.vstack([
                         fitx, ploty]))])
    pts_left  = pts - np.array([2,0])
    pts_right = pts + np.array([2,0])
    pts = np.hstack((pts_left, pts_right))
         
    # Draw lane on the warped blank image
    cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))
 
    # Warp the blank back to original image space using inverse perspective 
    # matrix (Minv)
    return color_warp
     
    # Combine the result with the original image


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

class line_follow:
    CONFIG = {
        'padding_percentage':0.25
        }
    def __init__(self,init_frame):
        frame = init_frame
        
        width,height = frame.shape[::-1][1:]
        padding = int(0.25 * width)
        DESIRED_ROI_POINTS = np.float32([
          [padding, 0], # Top-left corner
          [padding, height], # Bottom-left corner         
          [width-padding, height], # Bottom-right corner
          [width-padding, 0] # Top-right corner
        ])
        
        self.width,self.height,self.DESIRED_ROI_POINTS = width,height,DESIRED_ROI_POINTS
        self.MARGIN = int((1/12) * width)  # Window width is +/- margin
        self.MINPIX = int((1/24) * width)
        
                      
    def find_line(self,frame):
        width,height,DESIRED_ROI_POINTS=self.width,self.height,self.DESIRED_ROI_POINTS
        MARGIN,MINPIX=self.MARGIN,self.MINPIX
        
        warped_frame,inv_mat = perspective_transform(
            frame,
            (width,height),
            ROI_POINTS,
            DESIRED_ROI_POINTS)
        
        lane_line_markings=get_line_markings(warped_frame)
        
        fit = get_lane_line_indices_sliding_windows(
            lane_line_markings,MINPIX,MARGIN)
        if fit is not None:
            ploty,fitx = get_lane_line_previous_window(
                lane_line_markings, fit,MINPIX,MARGIN)
            
            shape = overlay_lane_lines(
                lane_line_markings,
                fitx,ploty)
            
            unwarped = cv2.warpPerspective(
                shape,
                inv_mat,
                (frame.shape[1],
                 frame.shape[0])
                )
            
            result = cv2.addWeighted(frame, 1, unwarped, 0.3, 0)
            
            return True,(ploty,fitx),lane_line_markings,result
        else:
            return False,(None,None),lane_line_markings,None

if __name__ == '__main__':
    cv2.namedWindow('original')
    cv2.namedWindow('lane_line_markings')
    cv2.namedWindow('result')
    
    cam = cv2.VideoCapture(0)
    ret,frame = cam_read(cam)
    
    line = line_follow(frame)
    
    while True:
        ret,frame = cam_read(cam)
        
        ret,(ploty,fitx),lane_line_markings,result = line.find_line(frame,)
        
        cv2.imshow('original',frame)
        cv2.imshow('lane_line_markings',lane_line_markings)
        if ret:
            cv2.imshow('result',result)
        else:
            cv2.imshow('result',frame)
        
        key = cv2.waitKey(1)
        if key == ord('q'):
            cv2.destroyAllWindows()
            break
        
        
    
if False:#__name__ == '__main__':  overwriting this to use the find_line class
    cam = cv2.VideoCapture(0)
    ret,frame = cam_read(cam)
    result = frame
    
    
    cv2.namedWindow('original')
    cv2.namedWindow('result')
    
    
    width,height = frame.shape[::-1][1:]
    padding = int(0.25 * width)
    DESIRED_ROI_POINTS = np.float32([
      [padding, 0], # Top-left corner
      [padding, height], # Bottom-left corner         
      [width-padding, height], # Bottom-right corner
      [width-padding, 0] # Top-right corner
    ])
    
    MARGIN = int((1/12) * width)  # Window width is +/- margin
    MINPIX = int((1/24) * width)

    while True:
        ret,frame = cam_read(cam)
        cv2.imshow('original',frame)
        
        
        
        warped_frame,inv_mat = perspective_transform(
            frame,
            (width,height),
            ROI_POINTS,
            DESIRED_ROI_POINTS)
        
        lane_line_markings=get_line_markings(warped_frame)
        
        fit = get_lane_line_indices_sliding_windows(
            lane_line_markings)
        if fit is not None:
            ploty,fitx = get_lane_line_previous_window(
                lane_line_markings, fit)
            
            shape = overlay_lane_lines(
                lane_line_markings,
                fitx,ploty)
            
            unwarped = cv2.warpPerspective(
                shape,
                inv_mat,
                (frame.shape[1],
                 frame.shape[0])
                )
            
            result = cv2.addWeighted(frame, 1, unwarped, 0.3, 0)
     
                
        cv2.imshow('lane_line_markings',lane_line_markings)
        cv2.imshow('result',result)
        key = cv2.waitKey(1)
        if key == ord('q'):
            cv2.destroyAllWindows()
            break
        
        if key == ord('s'):
            cv2.imwrite('./QR_CLOSE.png',result)
        
        if key == ord('d'):
            cv2.imwrite('./QR_FAR.png',result)
    