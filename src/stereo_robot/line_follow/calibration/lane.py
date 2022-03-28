import cv2 # Import the OpenCV library to enable computer vision
import numpy as np # Import the NumPy scientific computing library
import edge_detection as edge # Handles the detection of lane lines
import matplotlib.pyplot as plt # Used for plotting and error checking
 
# Author: Addison Sears-Collins
# https://automaticaddison.com
# Description: Implementation of the Lane class 
 
filename = 'original_lane_detection_5.jpg'
 
class Lane:
  """
  Represents a lane on a road.
  """
  def __init__(self, orig_frame):
    """
      Default constructor
         
    :param orig_frame: Original camera image (i.e. frame)
    """
    self.orig_frame = orig_frame
 
    # This will hold an image with the lane lines       
    self.lane_line_markings = None
 
    # This will hold the image after perspective transformation
    self.warped_frame = None
    self.transformation_matrix = None
    self.inv_transformation_matrix = None
 
    # (Width, Height) of the original video frame (or image)
    self.orig_image_size = self.orig_frame.shape[::-1][1:]
 
    width = self.orig_image_size[0]
    height = self.orig_image_size[1]
    self.width = width
    self.height = height
     
    # Four corners of the trapezoid-shaped region of interest
    # You need to find these corners manually.
    self.roi_points = np.float32([
      (264, 239), # Top-left corner
      (13, 412), # Bottom-left corner            
      (637, 412), # Bottom-right corner
      (497, 247) # Top-right corner
    ])
         
    # The desired corner locations  of the region of interest
    # after we perform perspective transformation.
    # Assume image width of 600, padding == 150.
    self.padding = int(0.25 * width) # padding from side of the image in pixels
    self.desired_roi_points = np.float32([
      [self.padding, 0], # Top-vert corner
      [self.padding, self.orig_image_size[1]], # Bottom-vert corner         
      [self.orig_image_size[
        0]-self.padding, self.orig_image_size[1]], # Bottom-right corner
      [self.orig_image_size[0]-self.padding, 0] # Top-right corner
    ]) 
         
    # Histogram that shows the white pixel peaks for lane line detection
    self.histogram = None
         
    # Sliding window parameters
    self.no_of_windows = 10
    self.margin = int((1/12) * width)  # Window width is +/- margin
    self.minpix = int((1/24) * width)  # Min no. of pixels to recenter window
         
    # Best fit polynomial lines for vert line and right line of the lane
    self.vert_fit = None
    self.vert_lane_inds = None
    self.ploty = None
    self.vert_fitx = None
    self.vertx = None
    self.verty = None
         
    # Pixel parameters for x and y dimensions
    self.YM_PER_PIX = 10.0 / 1000 # meters per pixel in y dimension
    self.XM_PER_PIX = 3.7 / 781 # meters per pixel in x dimension
         
    # Radii of curvature and offset
    self.vert_curvem = None
    self.center_offset = None
 
  def calculate_car_position(self, print_to_terminal=False):
    """
    Calculate the position of the car relative to the center
         
    :param: print_to_terminal Display data to console if True       
    :return: Offset from the center of the lane
    """
    # Assume the camera is centered in the image.
    # Get position of car in centimeters
    car_location = self.orig_frame.shape[1] / 2
 
    # Fine the x coordinate of the lane line bottom
    height = self.orig_frame.shape[0]
    bottom_vert = self.vert_fit[0]*height**2 + self.vert_fit[
      1]*height + self.vert_fit[2]
 
    center_lane = bottom_vert 
    center_offset = (np.abs(car_location) - np.abs(
      center_lane)) * self.XM_PER_PIX * 100
 
    if print_to_terminal == True:
      print(str(center_offset) + 'cm')
             
    self.center_offset = center_offset
       
    return center_offset
 
  def calculate_curvature(self, print_to_terminal=False):
    """
    Calculate the road curvature in meters.
 
    :param: print_to_terminal Display data to console if True
    :return: Radii of curvature
    """
#     raise Exception("NOT IMPLEMENTED")
    # Set the y-value where we want to calculate the road curvature.
    # Select the maximum y-value, which is the bottom of the frame.
    y_eval = np.max(self.ploty)    
 
    # Fit polynomial curves to the real world environment
    vert_fit_cr = np.polyfit(self.verty * self.YM_PER_PIX, self.vertx * (
      self.XM_PER_PIX), 2)

             
    # Calculate the radii of curvature
    vert_curvem = ((1 + (2*vert_fit_cr[0]*y_eval*self.YM_PER_PIX + vert_fit_cr[
                    1])**2)**1.5) / np.absolute(2*vert_fit_cr[0])
    # Display on terminal window
    if print_to_terminal == True:
      print(vert_curvem, 'm',)
             
    self.vert_curvem = vert_curvem
 
    return vert_curvem        
         
  def calculate_histogram(self,frame=None,plot=True):
    """
    Calculate the image histogram to find peaks in white pixel count
         
    :param frame: The warped image
    :param plot: Create a plot if True
    """
    if frame is None:
      frame = self.warped_frame
             
    # Generate the histogram
    self.histogram = np.sum(frame[int(
              frame.shape[0]/2):,:], axis=0)
 
    if plot == True:
         
      # Draw both the image and the histogram
      figure, (ax1, ax2) = plt.subplots(2,1) # 2 row, 1 columns
      figure.set_size_inches(10, 5)
      ax1.imshow(frame, cmap='gray')
      ax1.set_title("Warped Binary Frame")
      ax2.plot(self.histogram)
      ax2.set_title("Histogram Peaks")
      plt.show()
             
    return self.histogram
 
  def display_curvature_offset(self, frame=None, plot=False):
    """
    Display curvature and offset statistics on the image
         
    :param: plot Display the plot if True
    :return: Image with lane lines and curvature
    """
    image_copy = None
    if frame is None:
      image_copy = self.orig_frame.copy()
    else:
      image_copy = frame

    cv2.putText(image_copy,'Center Offset: '+str(
      self.center_offset)[:7]+' cm', (int((
      5/600)*self.width), int((
      40/338)*self.height)), cv2.FONT_HERSHEY_SIMPLEX, (float((
      0.5/600)*self.width)),(
      255,255,255),2,cv2.LINE_AA)
             
    if plot==True:       
      cv2.imshow("Image with Curvature and Offset", image_copy)
 
    return image_copy
     
  def get_lane_line_previous_window(self, vert_fit, plot=False):
    """
    Use the lane line from the previous sliding window to get the parameters
    for the polynomial line for filling in the lane line
    :param: vert_fit Polynomial function of the vert lane line
    :param: right_fit Polynomial function of the right lane line
    :param: plot To display an image or not
    """
    # margin is a sliding window parameter
    margin = self.margin
 
    # Find the x and y coordinates of all the nonzero 
    # (i.e. white) pixels in the frame.         
    nonzero = self.warped_frame.nonzero()  
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
         
    # Store vert and right lane pixel indices
    vert_lane_inds = ((nonzerox > (vert_fit[0]*(
      nonzeroy**2) + vert_fit[1]*nonzeroy + vert_fit[2] - margin)) & (
      nonzerox < (vert_fit[0]*(
      nonzeroy**2) + vert_fit[1]*nonzeroy + vert_fit[2] + margin)))            
    self.vert_lane_inds = vert_lane_inds
 
    # Get the vert and right lane line pixel locations  
    vertx = nonzerox[vert_lane_inds]
    verty = nonzeroy[vert_lane_inds]
 
    self.vertx = vertx
    self.verty = verty
     
    # Fit a second order polynomial curve to each lane line
    vert_fit = np.polyfit(verty, vertx, 2)
    self.vert_fit = vert_fit
         
    # Create the x and y values to plot on the image
    ploty = np.linspace(
      0, self.warped_frame.shape[0]-1, self.warped_frame.shape[0]) 
    vert_fitx = vert_fit[0]*ploty**2 + vert_fit[1]*ploty + vert_fit[2]
    self.ploty = ploty
    self.vert_fitx = vert_fitx
         
    if plot==True:
         
      # Generate images to draw on
      out_img = np.dstack((self.warped_frame, self.warped_frame, (
                           self.warped_frame)))*255
      window_img = np.zeros_like(out_img)
             
      # Add color to the vert and right line pixels
      out_img[nonzeroy[vert_lane_inds], nonzerox[vert_lane_inds]] = [255, 0, 0]
      
      # Create a polygon to show the search window area, and recast 
      # the x and y points into a usable format for cv2.fillPoly()
      margin = self.margin
      vert_line_window1 = np.array([np.transpose(np.vstack([
                                    vert_fitx-margin, ploty]))])
      vert_line_window2 = np.array([np.flipud(np.transpose(np.vstack([
                                    vert_fitx+margin, ploty])))])
      vert_line_pts = np.hstack((vert_line_window1, vert_line_window2))
      # Draw the lane onto the warped blank image
      cv2.fillPoly(window_img, np.int_([vert_line_pts]), (0,255, 0))
      result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
       
      # Plot the figures 
      figure, (ax1, ax2, ax3) = plt.subplots(3,1) # 3 rows, 1 column
      figure.set_size_inches(10, 10)
      figure.tight_layout(pad=3.0)
      ax1.imshow(cv2.cvtColor(self.orig_frame, cv2.COLOR_BGR2RGB))
      ax2.imshow(self.warped_frame, cmap='gray')
      ax3.imshow(result)
      ax3.plot(vert_fitx, ploty, color='yellow')
      ax1.set_title("Original Frame")  
      ax2.set_title("Warped Frame")
      ax3.set_title("Warped Frame With Search Window")
      plt.show()
             
  def get_lane_line_indices_sliding_windows(self, plot=False):
    """
    Get the indices of the lane line pixels using the 
    sliding windows technique.
         
    :param: plot Show plot or not
    :return: Best fit lines for the vert and right lines of the current lane 
    """
    # Sliding window width is +/- margin
    margin = self.margin
 
    frame_sliding_window = self.warped_frame.copy()
 
    # Set the height of the sliding windows
    window_height = np.int(self.warped_frame.shape[0]/self.no_of_windows)       
 
    # Find the x and y coordinates of all the nonzero 
    # (i.e. white) pixels in the frame. 
    nonzero = self.warped_frame.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1]) 
         
    # Store the pixel indices for the vert and right lane lines
    vert_lane_inds = []
         
    # Current positions for pixel indices for each window,
    # which we will continue to update
    vertx_base = self.histogram_peak()
    vertx_current = vertx_base
 
    # Go through one window at a time
    no_of_windows = self.no_of_windows
         
    for window in range(no_of_windows):
       
      # Identify window boundaries in x and y (and right and vert)
      win_y_low = self.warped_frame.shape[0] - (window + 1) * window_height
      win_y_high = self.warped_frame.shape[0] - window * window_height
      win_xvert_low = vertx_current - margin
      win_xvert_high = vertx_current + margin
      cv2.rectangle(frame_sliding_window,(win_xvert_low,win_y_low),(
        win_xvert_high,win_y_high), (255,255,255), 2)
 
      # Identify the nonzero pixels in x and y within the window
      good_vert_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                          (nonzerox >= win_xvert_low) & (
                           nonzerox < win_xvert_high)).nonzero()[0]
      # Append these indices to the lists
      vert_lane_inds.append(good_vert_inds)
         
      # If you found > minpix pixels, recenter next window on mean position
      minpix = self.minpix
      if len(good_vert_inds) > minpix:
        vertx_current = np.int(np.mean(nonzerox[good_vert_inds]))
                     
    # Concatenate the arrays of indices
    vert_lane_inds = np.concatenate(vert_lane_inds)
 
    # Extract the pixel coordinates for the vert and right lane lines

    vertx = nonzerox[vert_lane_inds]
    verty = nonzeroy[vert_lane_inds]
    if len(vertx)==0:
        return None
    # Fit a second order polynomial curve to the pixel coordinates for
    # the vert and right lane lines
    vert_fit = np.polyfit(verty, vertx, 2)
         
    self.vert_fit = vert_fit
 
    if plot==True:
         
      # Create the x and y values to plot on the image  
      ploty = np.linspace(
        0, frame_sliding_window.shape[0]-1, frame_sliding_window.shape[0])
      vert_fitx = vert_fit[0]*ploty**2 + vert_fit[1]*ploty + vert_fit[2]
 
      # Generate an image to visualize the result
      out_img = np.dstack((
        frame_sliding_window, frame_sliding_window, (
        frame_sliding_window))) * 255
             
      # Add color to the vert line pixels and right line pixels
      out_img[nonzeroy[vert_lane_inds], nonzerox[vert_lane_inds]] = [255, 0, 0]

                 
      # Plot the figure with the sliding windows
      figure, (ax1, ax2, ax3) = plt.subplots(3,1) # 3 rows, 1 column
      figure.set_size_inches(10, 10)
      figure.tight_layout(pad=3.0)
      ax1.imshow(cv2.cvtColor(self.orig_frame, cv2.COLOR_BGR2RGB))
      ax2.imshow(frame_sliding_window, cmap='gray')
      ax3.imshow(out_img)
      ax3.plot(vert_fitx, ploty, color='yellow')
      ax1.set_title("Original Frame")  
      ax2.set_title("Warped Frame with Sliding Windows")
      ax3.set_title("Detected Lane Lines with Sliding Windows")
      plt.show()        
             
    return self.vert_fit
 
  def get_line_markings(self, frame=None):
    """
    Isolates lane lines.
   
      :param frame: The camera frame that contains the lanes we want to detect
    :return: Binary (i.e. black and white) image containing the lane lines.
    """
    if frame is None:
      frame = self.orig_frame
             
    # Convert the video frame from BGR (blue, green, red) 
    # color space to HLS (hue, saturation, lightness).
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
 
    ################### Isolate possible lane line edges ######################
         
    # Perform Sobel edge detection on the L (lightness) channel of 
    # the image to detect sharp discontinuities in the pixel intensities 
    # along the x and y axis of the video frame.             
    # sxbinary is a matrix full of 0s (black) and 255 (white) intensity values
    # Relatively light pixels get made white. Dark pixels get made black.
    _, sxbinary = edge.threshold(hls[:, :, 1], thresh=(120, 255))
    sxbinary = edge.blur_gaussian(sxbinary, ksize=3) # Reduce noise
         
    # 1s will be in the cells with the highest Sobel derivative values
    # (i.e. strongest lane line edges)
    sxbinary = edge.mag_thresh(sxbinary, sobel_kernel=3, thresh=(110, 255))
 
    ######################## Isolate possible lane lines ######################
   
    # Perform binary thresholding on the S (saturation) channel 
    # of the video frame. A high saturation value means the hue color is pure.
    # We expect lane lines to be nice, pure colors (i.e. solid white, yellow)
    # and have high saturation channel values.
    # s_binary is matrix full of 0s (black) and 255 (white) intensity values
    # White in the regions with the purest hue colors (e.g. >80...play with
    # this value for best results).
    s_channel = hls[:, :, 2] # use only the saturation channel data
    _, s_binary = edge.threshold(s_channel, (80, 255))
     
    # Perform binary thresholding on the R (red) channel of the 
        # original BGR video frame. 
    # r_thresh is a matrix full of 0s (black) and 255 (white) intensity values
    # White in the regions with the richest red channel values (e.g. >120).
    # Remember, pure white is bgr(255, 255, 255).
    # Pure yellow is bgr(0, 255, 255). Both have high red channel values.
    _, r_thresh = edge.threshold(frame[:, :, 2], thresh=(120, 255))
 
    # Lane lines should be pure in color and have high red channel values 
    # Bitwise AND operation to reduce noise and black-out any pixels that
    # don't appear to be nice, pure, solid colors (like white or yellow lane 
    # lines.)       
    rs_binary = cv2.bitwise_and(s_binary, r_thresh)
 
    ### Combine the possible lane lines with the possible lane line edges ##### 
    # If you show rs_binary visually, you'll see that it is not that different 
    # from this return value. The edges of lane lines are thin lines of pixels.
    self.lane_line_markings = cv2.bitwise_or(rs_binary, sxbinary.astype(
                              np.uint8))    
    return self.lane_line_markings
         
  def histogram_peak(self):
    """
    Get the vert and right peak of the histogram
 
    Return the x coordinate of the vert histogram peak and the right histogram
    peak.
    """
    midpoint = np.int(self.histogram.shape[0]/2)
    vertx_base = np.argmax(self.histogram)
 
    # (x coordinate of vert peak, x coordinate of right peak)
    return vertx_base
         
  def overlay_lane_lines(self, plot=False):
    """
    Overlay lane lines on the original frame
    :param: Plot the lane lines if True
    :return: Lane with overlay
    """
    # Generate an image to draw the lane lines on 
    warp_zero = np.zeros_like(self.warped_frame).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))       
         
    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_vert = np.array([np.transpose(np.vstack([
                         self.vert_fitx, self.ploty]))])
    pts = np.hstack((pts_vert+np.array([3,0]), pts_vert-np.array([3,0])))
         
    # Draw lane on the warped blank image
    cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))
 
    # Warp the blank back to original image space using inverse perspective 
    # matrix (Minv)
    newwarp = cv2.warpPerspective(color_warp, self.inv_transformation_matrix, (
                                  self.orig_frame.shape[
                                  1], self.orig_frame.shape[0]))
     
    # Combine the result with the original image
    result = cv2.addWeighted(self.orig_frame, 1, newwarp, 0.3, 0)
         
    if plot==True:
      
      # Plot the figures 
      figure, (ax1, ax2) = plt.subplots(2,1) # 2 rows, 1 column
      figure.set_size_inches(10, 10)
      figure.tight_layout(pad=3.0)
      ax1.imshow(cv2.cvtColor(self.orig_frame, cv2.COLOR_BGR2RGB))
      ax2.imshow(cv2.cvtColor(result, cv2.COLOR_BGR2RGB))
      ax1.set_title("Original Frame")  
      ax2.set_title("Original Frame With Lane Overlay")
      plt.show()   
 
    return result           
     
  def perspective_transform(self, frame=None, plot=False):
    """
    Perform the perspective transform.
    :param: frame Current frame
    :param: plot Plot the warped image if True
    :return: Bird's eye view of the current lane
    """
    if frame is None:
      frame = self.lane_line_markings
             
    # Calculate the transformation matrix
    self.transformation_matrix = cv2.getPerspectiveTransform(
      self.roi_points, self.desired_roi_points)
 
    # Calculate the inverse transformation matrix           
    self.inv_transformation_matrix = cv2.getPerspectiveTransform(
      self.desired_roi_points, self.roi_points)
 
    # Perform the transform using the transformation matrix
    self.warped_frame = cv2.warpPerspective(
      frame, self.transformation_matrix, self.orig_image_size, flags=(
     cv2.INTER_LINEAR)) 
 
    # Convert image to binary
    (thresh, binary_warped) = cv2.threshold(
      self.warped_frame, 127, 255, cv2.THRESH_BINARY)           
    self.warped_frame = binary_warped
 
    # Display the perspective transformed (i.e. warped) frame
    if plot == True:
      warped_copy = self.warped_frame.copy()
      warped_plot = cv2.polylines(warped_copy, np.int32([
                    self.desired_roi_points]), True, (147,20,255), 3)
 
      # Display the image
      while(1):
        cv2.imshow('Warped Image', warped_plot)
             
        # Press any key to stop
        if cv2.waitKey(0):
          break
 
      cv2.destroyAllWindows()   
             
    return self.warped_frame        
     
  def plot_roi(self, frame=None, plot=False):
    """
    Plot the region of interest on an image.
    :param: frame The current image frame
    :param: plot Plot the roi image if True
    """
    if plot == False:
      return
             
    if frame is None:
      frame = self.orig_frame.copy()
 
    # Overlay trapezoid on the frame
    this_image = cv2.polylines(frame, np.int32([
      self.roi_points]), True, (147,20,255), 3)
 
    # Display the image
    while(1):
      cv2.imshow('ROI Image', this_image)
             
      # Press any key to stop
      if cv2.waitKey(0):
        break
 
    cv2.destroyAllWindows()
  

def main():
  cam = cv2.VideoCapture(0)
  cv2.namedWindow('test')
  while True:
      
      ret,frame=cam.read()
      if not ret:
        print("failed to grab frame")
        return


      # Load a frame (or image)
      original_frame = frame
     
      # Create a Lane object
      lane_obj = Lane(orig_frame=original_frame)
     
      # Perform thresholding to isolate lane lines
      lane_line_markings = lane_obj.get_line_markings()
     
      # Plot the region of interest on the image
      lane_obj.plot_roi(plot=False)
     
      # Perform the perspective transform to generate a bird's eye view
      # If Plot == True, show image with new region of interest
      warped_frame = lane_obj.perspective_transform(plot=False)
     
      # Generate the image histogram to serve as a starting point
      # for finding lane line pixels
      histogram = lane_obj.calculate_histogram(plot=False)  
      # Find lane line pixels using the sliding window method 
      vert_fit = lane_obj.get_lane_line_indices_sliding_windows(
         plot=False)
#       # Fill in the lane line
      if vert_fit is not None:
          lane_obj.get_lane_line_previous_window(vert_fit, plot=False)
          
          # Overlay lines on the original frame
          frame_with_lane_lines = lane_obj.overlay_lane_lines(plot=False)
         
          # Calculate lane line curvature (vert and right lane lines)
#           lane_obj.calculate_curvature(print_to_terminal=False)
         
          # Calculate center offset                                                                 
          lane_obj.calculate_car_position(print_to_terminal=False)
             
          # Display curvature and center offset on image
          frame_with_lane_lines2 = lane_obj.display_curvature_offset(
            frame=frame_with_lane_lines, plot=False)
          cv2.imshow('test',frame_with_lane_lines2)
      # Create the output file name by removing the '.jpg' part
      
#       size = len(filename)
#       new_filename = filename[:size - 4]
#       new_filename = new_filename + '_thresholded.jpg'
      
      cv2.waitKey(1) 
     
  # Save the new image in the working directory
  #cv2.imwrite(new_filename, lane_line_markings)
 
  # Display the image 
  #cv2.imshow("Image", lane_line_markings) 
     
  # Display the window until any key is pressed
  
     
  # Close all windows
  cv2.destroyAllWindows() 
     
main()