#!/usr/bin/env python
'''
# /////////////////////////////////////////////////////////////////////////////////////////////////////
#                                           Camera Class
#
#     Engineer:  Josh Sackos
#                jsackos@pdx.edu
#                503.298.1820
#
#       School:  Portland State University
#       Course:  ECE 579
#      Project:  MCECS Jeeves
#  Description:  This Python module defines a "camera" class that is to be used with the OpenCV 2.4
#                library for computer vision applications that use a camera. The main target
#                application of this class is for the Portland State University MCECS Jeeves robot.
#                The class contains methods for calibrating a camera for use in OpenCV, detecting/
#                matching features in source and target images, calculating distance/angle to
#                an object, more.
#
#                Many of the methods make use of class data members, but in the future these methods
#                should be set to use local copies to avoid data corruption.
#
#        Notes:  The RQ-Decomposition Euler angles cannot be relied upon as
#                they are not consistent from request to request, and the fact that
#                the data measured so far does not agree with the camera pinhole model.
#
#    Revisions:
#
#                rev 0.0 - File created - 11/29/2015
#
# /////////////////////////////////////////////////////////////////////////////////////////////////////
'''

'''
# ---------------------------------------------------------------------
#                               Imports
# ---------------------------------------------------------------------
'''
import sys, os
import numpy as np
import cv2
import glob
import inspect
import time
import math
from qrcode_pos_service import DEV_ENV
# Only try to import the graphical libraries if code is NOT running on the robot.
if DEV_ENV:
   import matplotlib
   matplotlib.use('GTKAgg')
   print matplotlib.rcsetup.interactive_bk
   from matplotlib import pyplot as plt

# Get absolute path of where scripts/nodes are located
abs_node_path = sys.argv[0].split('/')
abs_node_path = '/'.join(abs_node_path[0:len(abs_node_path)-1]) + '/'

'''
# ---------------------------------------------------------------------
#                       Configuration Parameters
# ---------------------------------------------------------------------
'''

SHOW_CALIB_IMAGES = False                # When set to True calibration images are displayed. Hidden when False.

# Camera resolution
CAM_PIXEL_WIDTH              = 640       # Width in pixels of the camera being used.
CAM_PIXEL_HEIGHT             = 480       # Height in pixels of the camera being used.

# Configure "find_homogrpahy" function.
N_ORB_FEATURES               = 10000     # Maximum number of orb features to detect
FLANN_INDEX_LSH              = 6         # For FLANN matching when using OpenCV ORB
RATIO_TEST_PARAM             = 0.8       # Maximum distance point n can be from point m when determing good FLANN matches
MIN_MATCH_COUNT              = 150       # Minimum number of good matches detected by ratio test to determine if a good match
NUM_TREE_CHECKS              = 100       # Number of times trees are recursively checked. Higher value results in better results, but takes longer
MATCH_STD_DEVIATION_N_1      = 1.75      # Selects the 1st, 2nd, 3rd, ..., nth standard deviation to be used when filtering out outlier good matches for common feature points between source and target images.
                                         # Note that the std_deviation can be a floating point value, i.e. a fractional value
MATCH_STD_DEVIATION_N_2      = 1.75      # Selects the 1st, 2nd, 3rd, ..., nth standard deviation to be used when filtering out outlier good matches for common feature points between source and target images
                                         # Note that the std_deviation can be a floating point value, i.e. a fractional value
BORDER_KNOWN_DISTANCE        = 2.7708333 # Distance in feet from which BORDER_PIXELS_KNOWN_DISTANCE was calculated from.
BORDER_PIXLES_KNOWN_DISTANCE = 115.0     # Lenght in pixels of a side of the feature rich border at 2ft away

# Configure "calc_dist" function
#    *** Calcualte slope of mathematical function that describes distance as a function of pixel length ***
MEASUREMENT_PIXEL_1          = 312.0     # Lenght in pixels of longest rectangle side at known distance MEASUREMENT_DISTANCE_1
MEASUREMENT_DISTANCE_1       = 1.0       # Distance that MEASUREMENT_PIXEL_1 was taken act.
MEASUREMENT_PIXEL_2          = 83.0      # Lenght in pixels of longest rectangle side at known distance MEASUREMENT_DISTANCE_2
MEASUREMENT_DISTANCE_2       = 4.0       # Distance that MEASUREMENT_PIXEL_2 was taken act.
DIST_CALC_SLOPE              = (MEASUREMENT_DISTANCE_2-MEASUREMENT_DISTANCE_1)/(MEASUREMENT_PIXEL_2-MEASUREMENT_PIXEL_1);
DIST_CALC_Y_INTERCEPT        = 4.277

'''
=========================================================================================
                                    Camera Class
=========================================================================================
'''
class camera():
   
   '''
      --------------------------------------------
                     Class Members
      --------------------------------------------
   '''
   criteria       = None     # Termination criteria
   objp           = None     # Object points 
   objpoints      = []       # 3d point in real world space from all calibration images
   imgpoints      = []       # 2d points in image plane from all calibration images
   images         = None     # Stores list of ".jpg" image names found in images directory
   image_dir      = ''       # Image directory to look in for calibration jpegs
   camera_mtx     = None     # Calibrated camera matrix
   ret            = None     # Returned by OpenCV camera calibration function. Not sure what it is used for.
   dist           = None     # Returned by OpenCV camera calibration function.
   rvecs          = None     # Rotation vectors
   tvecs          = None     # Translation vectors
   corners        = None     # Corners of detected QR code object
   verbosity      = False    # Verbosity for displaying text/images
   
   h_src_pts      = None     # Source feature points
   h_tar_pts      = None     # Target feature points
   homography_mtx = None     # The homography matrix of the matched object surface
   h_mask         = None     # Homography matrix mask?
   
   rot_mtx        = None     # Rotation matrix
   q_mtx          = None     # Q Matrix
   qx_vec         = None     # X translation vector
   qy_vec         = None     # Y translation vector
   qz_vec         = None     # Z translation vector   
   euler_x_deg    = None     # X axis rotation angle (degrees)
   euler_y_deg    = None     # Y axis rotation angle (degrees)
   euler_z_deg    = None     # Z axis rotation angle (degrees)
   

   '''
      -------------------------------------------------------------------------------------------------------------
                                                   Constructor

         Description:  This constructor reads all of the files in the "image_dir" directory, and
                       usees all images that end with "_calib.jpg" for OpenCV camera calibration.
                       Additionally, the constructor creates/prepares class data members for use
                       in the class methods.
         
           Arguments:  image_dir - Relative path to directory containing the calibration images.
                       src_image - Path to image of object to be detected in picture.
           
             Returns:  N/A
             
      -------------------------------------------------------------------------------------------------------------
   '''
   def __init__(self, image_dir='../images', verbosity=False, src_image="../images/qrcode_query_image2.png"):
      
      # Store verbosity
      self.verbosity = verbosity
      self.objpoints = []
      self.imgpoints = []
      
      if self.verbosity:
         print "\n"
         print "//////////////////////////////////////////////////////////////"
         print "                     Camera: Constructor"
         print "//////////////////////////////////////////////////////////////"
      
      # Open template image and store data
      self.src_img = cv2.imread(abs_node_path + src_image,0)    # Template for detecting qr code in random image
      
      # Termination criteria
      self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
      
      # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
      self.objp = np.zeros((6*7,3), np.float32)
      self.objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
      
      # Get all image names ending with ".jpg" for calibration
      self.images = glob.glob(abs_node_path + image_dir + '/*.jpg')
      
      # Parse through image names and only keep the ones containing "_calib.jpg"
      tmp_images = []
      if self.verbosity:
         print "\nExtracting calibration images from jpegs..."
         print "--------------------------------------------"
      for fname in (self.images):
         # If the string "_calib.jpg" exists in the image name process it, otherwise skip
         if fname.find('_calib.jpg') != -1:
            if self.verbosity:
               print "\t   Found: %s" % str( fname.split('/')[len(fname.split('/'))-1] )
            tmp_images.append(fname)
         else:
            if self.verbosity:
               print "\tSkipping: %s" % str( fname.split('/')[len(fname.split('/'))-1] )
      if self.verbosity:
         print "--------------------------------------------"
      
      # Assign the actual calibration jpeg filenames
      self.images = tmp_images
      
      # If a calibration image was not found abort
      if len(self.images) == 0:
         try:
            raise Exception("No calibration images found in: %s" % (abs_node_path + image_dir))
         except Exception as e:
            print e
      # Calibrate camera
      else:
         self.calibrate_camera()
         
   '''
      -------------------------------------------------------------------------------------------------------------
   #                                                Destructor
   #
   #   Description:  Perform cleanup tasks. For now this method just prints a message indicating that the
   #                 destructor was called.
   #     
   #     Arguments:  N/A
   #       
   #       Returns:  N/A
   #          
      -------------------------------------------------------------------------------------------------------------
   '''
   def __del__(self):
      if self.verbosity:
         print "\n"
         print "//////////////////////////////////////////////////////////////"
         print "                    Camera: Destructor"
         print "//////////////////////////////////////////////////////////////"

   '''
   # -----------------------------------------------------------------------------------------------------
   #                                             dump_py_object()
   #
   #   Description:  Takes any Python object as an argument and prints out its data members, methods, etc.
   #
   #     Arguments:  obj - Any Python object
   #
   #       Returns:  N/A
   #
   # -----------------------------------------------------------------------------------------------------
   '''
   def dump_py_object(self, pyobj):
      # Loop through all attributes in the Python object
      for attribute in dir(pyobj):
         # If has attribute
         if hasattr( pyobj, attribute ):
            print( "pyobj.%s  =  %s" % (attribute, getattr(pyobj, attribute)))

   '''
   # -----------------------------------------------------------------------------------------------------
   #                                          calibrate_camera()
   #
   #   Description:  Uses either the "_calib.jpg" files found in the constructor, or a list supplied
   #                 by the developer to calibrate a camera in OpenCV. Camera calibration is critical
   #                 in OpenCV as it allows one to calculate things such as homographies, solvePnP, etc.
   #
   #     Arguments:  alt_calib_images - Optional user supplied list of images to be used for camera
   #                       (OPTIONAL)   calibration, instead of the images found in the constructor.
   #
   #                     bypass_print - Disables the printing of the camera matrix, etc., that takes
   #                       (OPTIONAL)   up a lot of vertical space to display. Useful when developing
   #                                    other methods. (OPTIONAL)
   #
   #       Returns:     Boolean value - True if camera calibration was successful. False if unsuccessful.
   #
   # -----------------------------------------------------------------------------------------------------
   '''
   def calibrate_camera(self, alt_calib_images=[],bypass_print=False):
      
      if self.verbosity:
         print "//////////////////////////////////////////////////////////////"
         print "               Camera: calibrate_camera()"
         print "//////////////////////////////////////////////////////////////"

      # If user specified calibration images use them instead of class member list
      if len(alt_calib_images) > 0:
         use_images = alt_calib_images;
      else:
         use_images = self.images
         
      # Process each filename in images
      for fname in (use_images):
         img  = cv2.imread(fname)
         gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

         # Find the chess board corners
         calib_found, self.corners = cv2.findChessboardCorners(gray, (7,6),None)
         
         # If found, add object points, image points (after refining them)
         if calib_found == True:
            self.objpoints.append(self.objp)
            cv2.cornerSubPix(gray,self.corners,(11,11),(-1,-1),self.criteria)
            self.imgpoints.append(self.corners)

            if DEV_ENV and SHOW_CALIB_IMAGES:
               # Draw and display the corners
               cv2.drawChessboardCorners(img, (7,6), self.corners,1);
               plt.imshow(img);
               plt.show();
         else:
            Pass
            
      # Calibrate the camera
      try:
         ret, camera_mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, gray.shape[::-1],None,None)

         # Store calibrated data for future use
         self.camera_mtx = camera_mtx
         self.dist       = dist
         self.rvecs      = rvecs
         self.tvecs      = tvecs
         
         # If verbose
         if self.verbosity and bypass_print == False:
            print "------ RET -----------"
            print np.shape(self.ret)
            print ret
            print "-------Camera Matrix----------"
            print np.shape(self.camera_mtx)
            print camera_mtx
            print "------- Dist ----------"
            print np.shape(self.dist)
            print dist
            print "------- rvecs ----------"
            print np.shape(self.rvecs)
            print rvecs
            print "------- tvecs ----------"
            print np.shape(self.tvecs)
            print tvecs
            
            print "\nCamera calibration successful!\n"
         return calib_found
      except Exception as e:
         if self.verbosity:
            print "\nFailed to calibrate camera. Chessboard pattern not detected...\n"
         return False

   '''
   # -----------------------------------------------------------------------------------------------------
   #                                          filter_matches()
   #
   #   Description:  Filters out outlier feature matches by using the mean and a user specified standard
   #                 deviation. First the mean is calcualted for the good matches, then the standard
   #                 deviation of the points is calculated. All of the (x,y) points that lie within +/-
   #                 the user specified standard deviation is kept. Note that the standard deviation
   #                 specified by the user can be a floating point number.
   #
   #     Arguments:  std_dev - The maximum standard deviation a point can lie away from the mean to be
   #                           considered a good match by the filter. Default standard deviation is 2.0
   #
   #       Returns:  N/A
   #
   # -----------------------------------------------------------------------------------------------------
   '''
   def filter_matches(self, std_dev=2.0):

      # Use mean and 2nd standard deviation to filter out outliers that corrupt distance calculations
      self.good_match_list_pts     = [list(self.tar_kp[m.trainIdx].pt) for m in self.good_matches];
      self.match_mean              = np.mean(self.good_match_list_pts, axis=0)             # Calculate the mean for X and Y separately
      self.match_std_deviation     = np.std(self.good_match_list_pts,axis=0)
      self.x_above_match_std_dev   = self.match_mean[0] + self.match_std_deviation[0]*std_dev
      self.x_below_match_std_dev   = self.match_mean[0] - self.match_std_deviation[0]*std_dev
      self.y_above_match_std_dev   = self.match_mean[1] + self.match_std_deviation[1]*std_dev
      self.y_below_match_std_dev   = self.match_mean[1] - self.match_std_deviation[1]*std_dev
      if self.verbosity:
         print "-------- Matched Point Stat Data --------"
         print "Standard Deviation:  %f" % std_dev
         print "Match Point Means: X = %f , Y = %f" % (self.match_mean[0], self.match_mean[1])
         print "Match Points Standard Deviations: X = %f , Y = %f" % (self.match_std_deviation[0], self.match_std_deviation[1])
         print "X %dth Standard Deviation Range: %f , %f" % (std_dev, self.x_below_match_std_dev, self.x_above_match_std_dev)
         print "Y %dth Standard Deviation Range: %f , %f" % (std_dev, self.y_below_match_std_dev, self.y_above_match_std_dev)
         print "\n"

      self.tar_good_point_keys = [self.tar_kp[m.trainIdx] for m in self.good_matches]
      # Filter out outlier points that were falsely identified as a "good match"
      # If point is not within the nth x and y standard deviation ranges
      self.good_matches[:] = [m for m in self.good_matches if(
         self.tar_kp[m.trainIdx].pt[0] >= self.x_below_match_std_dev and
         self.tar_kp[m.trainIdx].pt[0] <= self.x_above_match_std_dev and
         self.tar_kp[m.trainIdx].pt[1] >= self.y_below_match_std_dev and
         self.tar_kp[m.trainIdx].pt[1] <= self.y_above_match_std_dev
      )]

   '''
   # -----------------------------------------------------------------------------------------------------
   #                                         get_bounding_rect()
   #
   #   Description:  Creates an OpenCV bounding rectangle of the (x,y) points supplied by the user.
   #
   #     Arguments:  xy_pts - Numpy array of XY points.
   #
   #       Returns:       x - The X coordinate of the top left of the rectangle.
   #                      y - The Y cooridnate of the top left of the rectangle.
   #                      w - The width of the rectangle in pixels.
   #                      h - The height of the rectangle in pixels.
   #                    ltc - Left top corner list containing x and y coordinates.
   #                    lbc - Left bottom corner list containing x and y coordinates.
   #                    rtc - Right top corner list containing x and y coordinates.
   #                    rbc - Right bottom corner list containing x and y coordinates.
   #
   # -----------------------------------------------------------------------------------------------------
   '''
   def get_bounding_rect(self, xy_pts):
      x,y,w,h = cv2.boundingRect(xy_pts) # Create the rectangle that encompasses all xy points
      ltc     = [x,y]                    # Left top corner
      lbc     = [x, y+h]                 # Left bottom corner
      rtc     = [x+w, y]                 # Right top corner
      rbc     = [x+w,y+h]                # Right bottom corner
      return x,y,w,h,ltc,lbc,rtc,rbc

   '''
   # -----------------------------------------------------------------------------------------------------
   #                                         x_obj_center()
   #
   #   Description:  Calculates the center of an object relative to its leftmost point and its
   #                 rightmost point.
   #
   #     Arguments:   obj_left_edge - The leftmost point of the detected object.
   #                 obj_right_edge - The rightmost point of the detected object.
   #
   #       Returns:      obj_center - Floating point value that specifies the pixel center of the object.
   #
   # -----------------------------------------------------------------------------------------------------
   '''
   def x_obj_center(self, obj_left_edge, obj_right_edge):
      # Determine center of object and whether or not it resides in the left or right hemisphere of the image
      obj_center = (obj_left_edge + obj_right_edge)/2; # Calculate center of rectangle

      if self.verbosity:
         print "\n----------- Object Center --------------"
         print "Object Center      :  %f" % obj_center
      
      return obj_center

   '''
   # -----------------------------------------------------------------------------------------------------
   #                                         x_obj_hemisphere()
   #
   #   Description:  Determines what side of the image the detected object is on, or if it lies directly
   #                 on the origin/center of image.
   #
   #     Arguments:  obj_center - Floating point value that specifies the pixel center of the object.
   #
   #       Returns:  tar_img_hemisphere - A string describing the object's location hemisphere. Will
   #                                      either be "left", "right", or "origin".
   #
   # -----------------------------------------------------------------------------------------------------
   '''
   def x_obj_hemisphere(self, obj_center):
      tar_img_hemisphere = ""
      # If at the origin
      if (obj_center == 0):
         tar_img_hemisphere = "origin";
      # If object center is in the left side of the picture
      elif (obj_center < CAM_PIXEL_WIDTH/2.0):
         tar_img_hemisphere = "left";
      # If object center is in the right side of the picture
      else:
         tar_img_hemisphere = "right";
         
      if self.verbosity:
         print "\n----------- Object Center --------------"
         print "Object Center      :  %f" % obj_center
         print "Object Hemisphere  :  %s" % tar_img_hemisphere

      return tar_img_hemisphere

   '''
   # -----------------------------------------------------------------------------------------------------
   #                                           x_obj_angle()
   #
   #   Description:  Calculates the angle from the center of the camera to the object center using the
   #                 SOH part of SOH-CAH-TOA.
   #
   #     Arguments:  obj_center - Floating point value that specifies the pixel center of the object.
   #                          r - Distance from camera to center of object. This is the hypotenuse.
   #
   #       Returns:       theta - Angle in degrees from center of camera image to center of object.
   #
   # -----------------------------------------------------------------------------------------------------
   '''
   def x_obj_angle(self, obj_center, r):
      # Calculate angle of the center of the object relative to the center of the image.
      # Assume that the center of the image is the origin (x=0)
      theta = (-1.0)*math.sin((obj_center - (CAM_PIXEL_WIDTH/2.0))/(r*BORDER_PIXLES_KNOWN_DISTANCE))*360.0/(2*math.pi)
      
      if self.verbosity:         
         print "\n-----------DISTANCE Object Angle from Robot --------------"
         print "Object Center Angle:  %f" % theta
         
      return theta

   '''
   # -----------------------------------------------------------------------------------------------------
   #                                        decompose_homography()
   #
   #   Description:  Decomposes an OpenCV homography matrix into its rotation matrix, Q matrix, and 
   #                 (x,y,z) rotation Euler angles.
   #
   #     Arguments:  homography_mtx - Homography matrix of detected object calculated via OpenCV
   #                                  findHomography()
   #
   #       Returns:      rot_mtx - Rotation matrix 3x3
   #                 euler_x_deg - Camera rotation about the X axis (camera pinhole model)
   #                 euler_y_deg - Camera rotation about the Y axis (camera pinhole model)
   #                 euler_z_deg - Camera rotation about the Z axis (camera pinhole model)
   #                       q_mtx - Q Matrix 3x3
   #
   # -----------------------------------------------------------------------------------------------------
   '''
   def decompose_homography(self, homography_mtx):
      
      if self.verbosity:
         print "\n"
         print "//////////////////////////////////////////////////////////////"
         print "                Camera: decompose_homography()"
         print "//////////////////////////////////////////////////////////////"
      
      # Decompose homography matrix
      rqdecomp_euler_angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(homography_mtx)

      # Rotation matricies
      rot_mtx = mtxR
      if self.verbosity:
         print "---------- Rotation Matricies ------------"
         print rot_mtx

      # Extract euler angles
      euler_x_deg = rqdecomp_euler_angles[0]
      euler_y_deg = rqdecomp_euler_angles[1]
      euler_z_deg = rqdecomp_euler_angles[2]
      if self.verbosity:
         print "---------- Euler Angles ------------"
         print "Degrees:  X = %f" % euler_x_deg
         print "Degrees:  Y = %f" % euler_y_deg
         print "Degrees:  Z = %f" % euler_z_deg

      # Store Q matrix
      q_mtx = mtxQ
      if self.verbosity:
         print "---------- Q Matrix ------------"
         print q_mtx

      # Store translation vectors
      qx_vec = q_mtx[0]
      qy_vec = q_mtx[1]
      qz_vec = q_mtx[2]
      if self.verbosity:
         print "---------- Translation Vectors ------------"
         print qx_vec
         print qy_vec
         print qz_vec
         
      return rot_mtx,euler_x_deg,euler_y_deg,euler_z_deg,q_mtx
   
   '''
   # -----------------------------------------------------------------------------------------------------
   #                                        find_qr_homography()
   #
   #   Description:  Find the OpenCV homography matrix of the QR code template in the target.
   #
   #     Arguments:  target_cv2_image - OpenCV 8-bit RGB image
   #                              ltc - Left top corner list containing x and y coordinates.
   #                              lbc - Left bottom corner list containing x and y coordinates.
   #                              rtc - Right top corner list containing x and y coordinates.
   #                              rbc - Right bottom corner list containing x and y coordinates.
   #
   #       Returns:             valid - Boolean value that states whether or not the attempt to
   #                                    find the homography matrix succeeded or not. True means
   #                                    the attempt was a success, and False is failure.
   #                   homography_mtx - Homography matrix of detected object calculated via OpenCV
   #                                    findHomography()
   #                         ltc_filt - Left top corner list containing x and y coordinates.
   #                         lbc_filt - Left bottom corner list containing x and y coordinates.
   #                         rtc_filt - Right top corner list containing x and y coordinates.
   #                         rbc_filt - Right bottom corner list containing x and y coordinates.
   #
   # -----------------------------------------------------------------------------------------------------
   '''
   def find_qr_homography(self, target_cv2_image, ltc, lbc, rbc, rtc):
      
      if self.verbosity:
         print "\n"
         print "//////////////////////////////////////////////////////////////"
         print "                Camera: find_qr_homography()"
         print "//////////////////////////////////////////////////////////////"
         
      # Initiate STAR detector for source and target images, detect up N_ORB_FEATURES
      src_orb = cv2.ORB(N_ORB_FEATURES)
      tar_orb = cv2.ORB(N_ORB_FEATURES)

      # Compute ORB descriptors for source image. Convert memory locations to coordinates in Python list?
      src_kp, src_des = src_orb.detectAndCompute(self.src_img, None)
      self.tar_kp, tar_des = tar_orb.detectAndCompute(target_cv2_image, None)

      # -------------------   images without matching --------------
      self.plot_images(self.src_img, src_kp,src_des,False,target_cv2_image, self.tar_kp,tar_des,False)
      
      if self.verbosity:
         print "Source Features: %d" % (len(src_kp))
         print "Target Features: %d" % (len(self.tar_kp))

      # Setup FLANN matching algorithm for use with ORB
      index_params= dict(algorithm = FLANN_INDEX_LSH,
                         table_number = 6,      # 12  # 6
                         key_size = 12,         # 20  # 12
                         multi_probe_level = 1) # 2   # 1

      # Create search parameters dictionary.
      # Specifies number of times trees are recursively searched.
      # A higher value gives better results but takes longer.
      search_params = dict(checks=NUM_TREE_CHECKS)   # or pass empty dictionary

      # Create the flann matcher object
      flann = cv2.FlannBasedMatcher(index_params,search_params)

      # Match source descriptors with target descriptors
      matches = flann.knnMatch(src_des,tar_des,k=2)

      # -------------- images with matching but no ratio test ----------------
      # Keep track of all matches before applying ratio test
      self.good_matches = []
      for m, n in matches:
         self.good_matches.append(m)
         
      # Keep a copy of the original matched points via FLANN without additional filtering
      # This is for visually comparing the difference between the FLANN matches and the good matches that
      # had additional filterting applied to them.
      self.h_tar_pts_unfiltered = np.float32([ self.tar_kp[m.trainIdx].pt for m in self.good_matches ]).reshape(-1,1,2)
      
      # ******** NOTE: This ratio test performs horribly for the Jeeves project. Only use it if you absolutely have to! **********
      # Keep only good matches that pass the ratio test
      #
      #     Structure of a match in matches
      #             -> float distance
      #             -> int imgIdx       , Train image index
      #             -> int queryIdx     , Query descriptor index
      #             -> int trainIdx     , Train descriptor index
#      self.good_matches = []
#      for m, n in matches:
            # If the match passes the ratio test add it to the list of good matches
#         if m.distance < RATIO_TEST_PARAM*n.distance:
#            self.good_matches.append(m)
#      if self.verbosity:
#         print "Good Matches = %d" % len(self.good_matches)

      # If not enough good matches alert and abort
      if(not len(self.good_matches) >= MIN_MATCH_COUNT):
         if self.verbosity:
            print "The number of good matches found is less than %d, aborting" % MIN_MATCH_COUNT
         return False,False,False,False,False,False

      # ------------------ FLANN matching images ---------------------
#      self.h2_src_pts = np.float32([ src_kp[m.queryIdx].pt for m in self.good_matches ]).reshape(-1,1,2)
#      self.h2_tar_pts = np.float32([ self.tar_kp[m.trainIdx].pt for m in self.good_matches ]).reshape(-1,1,2)
#      self.tar2_filtered_good_point_keys = [self.tar_kp[m.trainIdx] for m in self.good_matches]
#      self.plot_images(self.src_img, src_kp,src_des,False,target_cv2_image, self.tar2_filtered_good_point_keys,self.h2_tar_pts,False)
      
      # Apply additional filtering to FLANN matches to get rid of outliers
      self.filter_matches(MATCH_STD_DEVIATION_N_1);
      
      # ------------------ Images with 1 iteration of addtional filtering ---------------------
#      self.h2_src_pts = np.float32([ src_kp[m.queryIdx].pt for m in self.good_matches ]).reshape(-1,1,2)
#      self.h2_tar_pts = np.float32([ self.tar_kp[m.trainIdx].pt for m in self.good_matches ]).reshape(-1,1,2)
#      self.tar2_filtered_good_point_keys = [self.tar_kp[m.trainIdx] for m in self.good_matches]
#      self.plot_images(self.src_img, src_kp,src_des,False,target_cv2_image, self.tar2_filtered_good_point_keys,self.h2_tar_pts,False)

      # Apply second round of additional filtering to increase accuracy
      self.filter_matches(MATCH_STD_DEVIATION_N_2);
      
      # ------------------ Images with 2 iterations of addtional filtering, and with/without bounding rectangles ---------------------
      # Create list of source points and list of target/destination points from good matches
      self.h_src_pts = np.float32([ src_kp[m.queryIdx].pt for m in self.good_matches ]).reshape(-1,1,2)
      self.h_tar_pts = np.float32([ self.tar_kp[m.trainIdx].pt for m in self.good_matches ]).reshape(-1,1,2)
      self.tar_filtered_good_point_keys = [self.tar_kp[m.trainIdx] for m in self.good_matches]
      
      self.plot_images(self.src_img, src_kp,src_des,False,target_cv2_image, self.tar_filtered_good_point_keys,self.h_tar_pts,False)   # Features before/after no rectangle
      self.plot_images(self.src_img, src_kp,src_des,False,target_cv2_image, self.tar_filtered_good_point_keys,self.h_tar_pts,True)    # Features before/after, with rectangle
      self.plot_images(target_cv2_image, self.tar_filtered_good_point_keys,self.h_tar_pts,True)                                       # Target image only with features and rectangle
      
      # Find homography
      homography_mtx, h_mask = cv2.findHomography(self.h_src_pts, self.h_tar_pts, cv2.RANSAC,100.0)
      
      if self.verbosity:
         print "\nHomography Matrix:"
         print "-------------------------"
         print homography_mtx
         print "-------------------------\n"
         
      # Create a bounding rectangle and extract corner points for a rectangle that encompasses all filtered good matches
      x_filt,y_filt,w_filt,h_filt,ltc_filt,lbc_filt,rtc_filt,rbc_filt = self.get_bounding_rect(self.h_tar_pts)
      
      # Show images of target with unfiltered features and filtered features
      self.plot_images(target_cv2_image, self.tar_good_point_keys,self.h_tar_pts_unfiltered,True,target_cv2_image, self.tar_filtered_good_point_keys,self.h_tar_pts,True)
         
      return True, homography_mtx,ltc_filt,lbc_filt,rtc_filt,rbc_filt

   '''
   # -----------------------------------------------------------------------------------------------------
   #                                        plot_images()
   #
   #   Description:  This function plots the OpenCV img1/img2 images with the supplied feature points
   #                 side by side if both are supplied. Each image has an additional option to plot
   #                 a bounding rectangle around the feature points. The second image img2 is optional
   #                 to the user and is not required.
   #
   #                 Note: If only img1 is supplied it will be plotted by itself.
   #
   #     Arguments:         img1 - OpenCV 8-bit RGB image.
   #                   img1_keys - Keypoints from ORB feature detection.
   #                    img1_pts - (x,y) poitns extracted from keypoints.
   #                   img1_rect - Boolean value: True = show bouding rectangle, False = No rectangle
   #       (OPTIONAL)       img2 - OpenCV 8-bit RGB image.
   #       (OPTIONAL)  img2_keys - Keypoints from ORB feature detection.
   #       (OPTIONAL)   img2_pts - (x,y) poitns extracted from keypoints.
   #       (OPTIONAL)  img2_rect - Boolean value: True = show bouding rectangle, False = No rectangle
   #
   #       Returns:  N/A
   #
   # -----------------------------------------------------------------------------------------------------
   '''
   def plot_images(self, img1, img1_keys,img1_pts, img1_rect=False, img2=[], img2_keys=[],img2_pts=False, img2_rect=False):
      
      # If in development mode, i.e. not running on Jeeves
      if DEV_ENV:
         # Create display images and display them for comparison
         if self.verbosity:
            # Draw keypoint locations, not size and orientation
            if len(img1) > 0:
               img1 = cv2.drawKeypoints(img1,img1_keys,color=(255,255,0), flags=0)
            if len(img2) > 0:
               img2 = cv2.drawKeypoints(img2,img2_keys,color=(255,255,0), flags=0)

            if img1_rect:
               # Create a bounding rectangle and extract corner points for a rectangle that encompasses all filtered good matches
               x_img1,y_img1,w_img1,h_img1,ltc_img1,lbc_img1,rtc_img1,rbc_img1 = self.get_bounding_rect(img1_pts)

            if img2_rect:
               # Create a bounding rectangle that encompasses all un-filtered good matches
               x_img2,y_img2,w_img2,h_img2,ltc_img2,lbc_img2,rtc_img2,rbc_img2 = self.get_bounding_rect(img2_pts)
               
            # Overlay the filtered and unfiltered rectangles
            if img1_rect:
               cv2.rectangle(img1,(x_img1,y_img1),(x_img1+w_img1,y_img1+h_img1),(0,255,0),2)

            if img2_rect:
               cv2.rectangle(img2,(x_img2,y_img2),(x_img2+w_img2,y_img2+h_img2),(0,255,0),2)

            # Concatenate images together for comparison
            if len(img1) > 0 and len(img2) > 0:
               self.vis = np.concatenate((img1, img2), axis=1);
            else:
               if len(img1) > 0:
                  self.vis = img1
               elif len(img2) > 0:
                  self.vis = img2

            # Show the images
            plt.imshow(self.vis)
            plt.show()
   
   '''
   # -----------------------------------------------------------------------------------------------------
   #                                      calc_distance()
   #
   #   Description:  Calculates the distance from the camera to the detected object using a pixel scaling
   #                 technique. Essentially the longest side of a bounding rectangle is used to determine
   #                 how far away the camera is from an object. The calculated distance accuracy decreases
   #                 as the camera views an object from an angle. There also appears to be issues when the
   #                 camera gets within 1ft of the object. Perhaps this is due to the camera's focal
   #                 length.
   #
   #     Arguments:  ltc - Left top corner list containing x and y coordinates.
   #                 lbc - Left bottom corner list containing x and y coordinates.
   #                 rtc - Right top corner list containing x and y coordinates.
   #                 rbc - Right bottom corner list containing x and y coordinates.
   #
   #       Returns:    r - Floating point distance value upon successful calcualtion, and -1 upon failure.
   #
   # -----------------------------------------------------------------------------------------------------
   '''
   def calc_distance(self, ltc, lbc, rbc, rtc):

      if self.verbosity:
         print "\n"
         print "//////////////////////////////////////////////////////////////"
         print "                  Camera: calc_distance()"
         print "//////////////////////////////////////////////////////////////"

      distance = 0
         
      if (ltc > -1 and lbc > -1 and rtc > -1 and rbc > -1):
         # Initialize variable
         distance = 0

         # Determine longest side of rectangle and use it
         lt_lb = float(lbc[1] - ltc[1])
         lt_rt = float(rtc[0] - ltc[0])
         rt_rb = float(rbc[1] - rtc[1])
         lb_rb = float(rbc[0] - lbc[0])
         
         if self.verbosity:
            print "Rectangle Left Side Length  :  %d" % lt_lb
            print "Rectangle Top Side Length   :  %d" % lt_rt
            print "Rectangle Right Side Length :  %d" % rt_rb
            print "Rectangle Bottom Side Length:  %d" % lb_rb

         # Use the longest side of the rectangle that encompasses all the filtered good match features
         # and subject from it the known pixel length of a border at the known distance
         delta_dist =  float(max([lt_lb,lt_rt,rt_rb,lb_rb])) - BORDER_PIXLES_KNOWN_DISTANCE

         distance = DIST_CALC_Y_INTERCEPT + float(max([lt_lb,lt_rt,rt_rb,lb_rb])) * DIST_CALC_SLOPE
   
         # Correct the error in the calculation, non-linear
         distance = self.correct_dist_err(distance);

         if self.verbosity:
            print "------------- calc_distance() -------------"
            print "Longest Rectangle Side = %d" % float(max([lt_lb,lt_rt,rt_rb,lb_rb]))
            print "Distance from Object:  %f ft." % distance
            print "-------------------------------------------"

         return distance
      else:
         if self.verbosity:
            print "Cannot calculate distance yet. Ensure that the left top, left bottom, right top, and right bottom corners have been initialized first."
         return -1

   '''
   # -----------------------------------------------------------------------------------------------------
   #                                        correct_dist_err()
   #
   #   Description:  This method removes the error from the calculated distance in calc_distance().
   #                 several distance calculations were performed with no error correction at known
   #                 distances, the error was calculated for each trial, and a 6th order polynomial
   #                 functiono describing the error at different distances was created using
   #                 Microsoft Excel.
   #
   #                 Note:  This 6th order polynomial works well for all distances of interest
   #                        for use with Jeeves except for at 1 ft.
   #
   #     Arguments:            distance - Floating point distance that contains error.
   #
   #       Returns:  corrected_distance - Floating point distance with error removed.
   #
   # -----------------------------------------------------------------------------------------------------
   '''
   def correct_dist_err(self, distance):

      corrected_distance = distance - (-0.0794*distance**6 + 0.8055*distance**5 - 3.195*distance**4 + 6.1133*distance**3 - 5.9012*distance**2 + 3.3285*distance - 1.3951);
      
      if self.verbosity:
         print "------------ correct_dist_err() ------------"
         print "Calculated Distance:  %f" % distance
         print "Corrected  Distance:  %f" % corrected_distance
         print "Calculated Error:     %f" % ((-0.0794*distance**6 + 0.8055*distance**5 - 3.195*distance**4 + 6.1133*distance**3 - 5.9012*distance**2 + 3.3285*distance - 1.3951))
         print "--------------------------------------------"
         
      return corrected_distance
      
   '''
   # -----------------------------------------------------------------------------------------------------
   #                                          solvePnP()
   #
   #   Description:  This method computes the OpenCV solvePnP() function. It is currently not used
   #                 and was left in the class for the sake of providing the essential building
   #                 blocks required for using OpenCV's solvePnP() function in the future.
   #
   #     Arguments:  N/A
   #
   #       Returns:  N/A
   #
   # -----------------------------------------------------------------------------------------------------
   '''
   def solvePnP(self):
      
      if self.verbosity:
         print "\n"
         print "//////////////////////////////////////////////////////////////"
         print "                     Camera: solvPnP()"
         print "//////////////////////////////////////////////////////////////"
      
      # Put matched source points into objPoints format for solvePnP()
      tmp_mtx = []
      i = 0
      for point in self.h_src_pts:
         tmp_mtx.append( np.append(point[0], np.float32( [0]), 1) )
         i += 1
      tmp_h_src_obj_pts = np.float32(tmp_mtx)
      
      # Put matched target points into imgPoints format for solvePnP()
      tmp_mtx = []
      i = 0
      for point in self.h_tar_pts:
         tmp_mtx.append( point[0] )
         i += 1
      tmp_h_tar_img_pts = np.float32(tmp_mtx)
   
      # solvePnP()
      self.img_retval, self.img_rvec, self.img_tvec = cv2.solvePnP(tmp_h_src_obj_pts, tmp_h_tar_img_pts, self.camera_mtx, self.dist, np.float32(self.rvecs), np.float32(self.tvecs),useExtrinsicGuess=1)
      
      # Make scaled version of camera matrix so we can multiply it with the scaled 3x4 identity matrix
      tmp_cam_mtx = np.concatenate((self.camera_mtx, [[0],[0],[0]]), axis=1)
      tmp_cam_mtx = np.append(  tmp_cam_mtx, [[0,0,0,0]] , axis=0  )
      
      print "------------ cam_mtx -----------"
      print tmp_cam_mtx
      
      
      # Compute rotation matrix from rotation vector
      self.rot_mtx, self.img_jacobian = cv2.Rodrigues(self.img_rvec)

      if self.verbosity:
         print "---------- solvePnP retVal -----------"
         print self.img_retval
         print "---------- solvePnP rvec -----------"
         print self.img_rvec
         print "---------- solvePnP tvec -----------"
         print self.img_tvec
         print "------------ Rotation Matrix ------------"
         print self.rot_mtx
      
      # Create 3x3 identity matrix with extra column for projection matrix calculation
      id_3x4 = np.zeros(shape=(4,4))
      id_3x4[0][0] = 1
      id_3x4[1][1] = 1
      id_3x4[2][2] = 1
      id_3x4 = np.float32(id_3x4)
      cam_id = tmp_cam_mtx*id_3x4
      
      if self.verbosity:
         print "--------------- Identity 3x4 matrix --------------"
         print np.float32(id_3x4)
         print "--------------- cam_mtx * id_3x4 -----------------"
         print cam_id
      
      # Concatenate rotation matrix with translation matrix, 0's, and a 1, yields a 4x4 matrix
      rot_tv = np.concatenate((self.rot_mtx, self.img_tvec), axis=1)
      rot_tv = np.append(  rot_tv, [[0,0,0,1]] , axis=0  )
      cam_rot_tv = tmp_cam_mtx*rot_tv
      if self.verbosity:
         print "----------- cam_rot_tv -----------"
         print cam_rot_tv

      
      















