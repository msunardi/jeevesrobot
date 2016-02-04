'''
# //////////////////////////////////////////////////////////////////////////////////////
#                        Camera Class for MCECS Jeeves Navigation
#
#     Engineer:  Josh Sackos
#                jsackos@pdx.edu
#                503.298.1820
#
#       School:  Portland State University
#       Course:  ECE 578
#      Project:  MCECS Jeeves
#  Description:
#
#        Notes:
#
#    Revisions:
#
#                rev 0.0 - File created - 11/29/2015
#
# //////////////////////////////////////////////////////////////////////////////////////
'''

import sys, os
import numpy as np
import cv2
import glob
import inspect
from qrcode_pos_service import DEV_ENV
if DEV_ENV:
   import matplotlib
   matplotlib.use('GTKAgg')
   print matplotlib.rcsetup.interactive_bk
   from matplotlib import pyplot as plt
import time
import math

SHOW_CALIB_IMAGES = False

# =======================================================================================
#                                 Global Variables
# =======================================================================================

# Get absolute path of where scripts/nodes are located
abs_node_path = sys.argv[0].split('/')
abs_node_path = '/'.join(abs_node_path[0:len(abs_node_path)-1]) + '/'

CAM_PIXEL_WIDTH              = 640      # Width in pixels of the camera being used.
CAM_PIXEL_HEIGHT             = 480      # Height in pixels of the camera being used.
N_ORB_FEATURES               = 10000    # Maximum number of orb features to detect
FLANN_INDEX_LSH              = 6        # For FLANN matching when using OpenCV ORB
RATIO_TEST_PARAM             = 0.8      # Maximum distance point n can be from point m when determing good FLANN matches
MIN_MATCH_COUNT              = 150      # Minimum number of good matches detected by ratio test to determine if a good match
NUM_TREE_CHECKS              = 100      # Number of times trees are recursively checked. Higher value results in better results, but takes longer
MATCH_STD_DEVIATION_N_1      = 1.75      # Selects the 1st, 2nd, 3rd, ..., nth standard deviation to be used when filtering out outlier good matches for common feature points between source and target images.
                                        # Note that the std_deviation can be a floating point value, i.e. a fractional value
MATCH_STD_DEVIATION_N_2      = 1.75      # Selects the 1st, 2nd, 3rd, ..., nth standard deviation to be used when filtering out outlier good matches for common feature points between source and target images
                                        # Note that the std_deviation can be a floating point value, i.e. a fractional value
BORDER_KNOWN_DISTANCE        = 1.5      # Distance in feet from which BORDER_PIXELS_KNOWN_DISTANCE was calculated from.
BORDER_PIXLES_KNOWN_DISTANCE = 123.0    # Lenght in pixels of a side of the feature rich border at 2ft away

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
   ret            = None
   dist           = None
   rvecs          = None
   tvecs          = None
   corners        = None
   verbosity      = False
   
   h_src_pts      = None
   h_tar_pts      = None
   homography_mtx = None
   h_mask         = None
   
   rot_mtx        = None
   q_mtx          = None
   qx_vec         = None
   qy_vec         = None
   qz_vec         = None
   
   euler_x_deg    = None
   euler_y_deg    = None
   euler_z_deg    = None
   
   

   '''
      --------------------------------------------
                      Constructor
      --------------------------------------------
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
      --------------------------------------------
                       Destructor
      --------------------------------------------
   '''
   def __del__(self):
      if self.verbosity:
         print "\n"
         print "//////////////////////////////////////////////////////////////"
         print "                    Camera: Destructor"
         print "//////////////////////////////////////////////////////////////"

      
   '''
      --------------------------------------------
                   calibrate_camera()
      --------------------------------------------
   '''
   def calibrate_camera(self):
      
      if self.verbosity:
         print "//////////////////////////////////////////////////////////////"
         print "               Camera: calibrate_camera()"
         print "//////////////////////////////////////////////////////////////"
      
      # Process each filename in images
      for fname in (self.images):
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
               cv2.drawChessboardCorners(img, (7,6), self.corners,self.ret);
               plt.imshow(img);
               plt.show();

            
      # Calibrate the camera
      try:
         ret, camera_mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, gray.shape[::-1],None,None)

         # Store calibrated data for future use
         self.camera_mtx = camera_mtx
         self.dist       = dist
         self.rvecs      = rvecs
         self.tvecs      = tvecs
         
         # If verbose
         if self.verbosity:
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
      except Exception as e:
         if self.verbosity:
            print "\nFailed to calibrate camera. Chessboard pattern not detected...\n"

   '''
      --------------------------------------------
                filter_ransac_matches()
      --------------------------------------------
   '''
   def filter_ransac_matches(self, std_dev=2.0):

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
      --------------------------------------------
                  get_bounding_rect()
      --------------------------------------------
   '''
   def get_bounding_rect(self, xy_pts):
      x,y,w,h = cv2.boundingRect(xy_pts) # Create the rectangle that encompasses all xy points
      ltc     = [x,y]                    # Left top corner
      lbc     = [x, y+h]                 # Left bottom corner
      rtc     = [x+w, y]                 # Right top corner
      rbc     = [x+w,y+h]                # Right bottom corner
      return x,y,w,h,ltc,lbc,rtc,rbc

   '''
      --------------------------------------------
                    x_obj_center()
      --------------------------------------------
   '''
   def x_obj_center(self, obj_left_edge, obj_right_edge):
      # Determine center of object and whether or not it resides in the left or right hemisphere of the image
      obj_center = (obj_left_edge + obj_right_edge)/2; # Calculate center of rectangle

      if self.verbosity:
         print "\n----------- Object Center --------------"
         print "Object Center      :  %f" % obj_center
      
      return obj_center

   '''
      --------------------------------------------
                  x_obj_hemisphere()
      --------------------------------------------
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
         print "\n----------- Object Hemisphere in Image --------------"
         print "Object Hemisphere  :  %s" % tar_img_hemisphere

      return tar_img_hemisphere

   '''
      --------------------------------------------
                     x_obj_angle()
      --------------------------------------------
   '''
   def x_obj_angle(self, obj_center, r):
      # Calculate angle of the center of the object relative to the center of the image.
      # Assume that the center of the image is the origin (x=0)
      obj_center_angle = (-1.0)*math.sin((obj_center - (CAM_PIXEL_WIDTH/2.0))/(r*BORDER_PIXLES_KNOWN_DISTANCE))*360.0/(2*math.pi)
      
      if self.verbosity:         
         print "\n----------- Object Angle from Robot --------------"
         print "Object Center      :  %f" % obj_center
         print "Object Center Angle:  %f" % obj_center_angle
         
      return obj_center_angle

   '''
      --------------------------------------------
                decompose_homography()
      --------------------------------------------
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
      --------------------------------------------
                  find_qr_homography()
      --------------------------------------------
   '''
   def find_qr_homography(self, target_cv2_image):
      
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

      # ------------------ with ratio test images ---------------------
      self.h2_src_pts = np.float32([ src_kp[m.queryIdx].pt for m in self.good_matches ]).reshape(-1,1,2)
      self.h2_tar_pts = np.float32([ self.tar_kp[m.trainIdx].pt for m in self.good_matches ]).reshape(-1,1,2)
      self.tar2_filtered_good_point_keys = [self.tar_kp[m.trainIdx] for m in self.good_matches]
      self.plot_images(self.src_img, src_kp,src_des,False,target_cv2_image, self.tar2_filtered_good_point_keys,self.h2_tar_pts,False)
      
      # Apply additional filtering to FLANN matches to get rid of outliers
      self.filter_ransac_matches(MATCH_STD_DEVIATION_N_1);
      
      # ------------------ with 1 iteration ---------------------
      self.h2_src_pts = np.float32([ src_kp[m.queryIdx].pt for m in self.good_matches ]).reshape(-1,1,2)
      self.h2_tar_pts = np.float32([ self.tar_kp[m.trainIdx].pt for m in self.good_matches ]).reshape(-1,1,2)
      self.tar2_filtered_good_point_keys = [self.tar_kp[m.trainIdx] for m in self.good_matches]
      self.plot_images(self.src_img, src_kp,src_des,False,target_cv2_image, self.tar2_filtered_good_point_keys,self.h2_tar_pts,False)
      
      self.filter_ransac_matches(MATCH_STD_DEVIATION_N_2);
      
      # Create list of source points and list of target/destination points from good matches
      self.h_src_pts = np.float32([ src_kp[m.queryIdx].pt for m in self.good_matches ]).reshape(-1,1,2)
      self.h_tar_pts = np.float32([ self.tar_kp[m.trainIdx].pt for m in self.good_matches ]).reshape(-1,1,2)
      self.tar_filtered_good_point_keys = [self.tar_kp[m.trainIdx] for m in self.good_matches]
      
      self.plot_images(self.src_img, src_kp,src_des,False,target_cv2_image, self.tar_filtered_good_point_keys,self.h_tar_pts,False)
      self.plot_images(self.src_img, src_kp,src_des,False,target_cv2_image, self.tar_filtered_good_point_keys,self.h_tar_pts,True)
      self.plot_images(target_cv2_image, self.tar_filtered_good_point_keys,self.h_tar_pts,True)
      
      # Find homography
      homography_mtx, h_mask = cv2.findHomography(self.h_src_pts, self.h_tar_pts, cv2.RANSAC,100.0)
      
      if self.verbosity:
         print "\nHomography Matrix:"
         print "-------------------------"
         print homography_mtx
         print "-------------------------\n"

      # Create a bounding rectangle and extract corner points for a rectangle that encompasses all filtered good matches
      x_filt,y_filt,w_filt,h_filt,ltc_filt,lbc_filt,rtc_filt,rbc_filt = self.get_bounding_rect(self.h_tar_pts)
      
      self.plot_images(target_cv2_image, self.tar_good_point_keys,self.h_tar_pts_unfiltered,True,target_cv2_image, self.tar_filtered_good_point_keys,self.h_tar_pts,True)
         
      return True, homography_mtx,ltc_filt,lbc_filt,rtc_filt,rbc_filt

   '''
      --------------------------------------------
                   plot_images()
      --------------------------------------------
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
      --------------------------------------------
                   calc_distance()
      --------------------------------------------
   '''
   def calc_distance(self, ltc, lbc, rtc, rbc):

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

         # Use the longest side of the rectangle that encompasses all the filtered good match features
         # and subject from it the known pixel length of a border at the known distance
         distance =  float(max([lt_lb,lt_rt,rt_rb,lb_rb])) - BORDER_PIXLES_KNOWN_DISTANCE
         print distance
         print float(max([lt_lb,lt_rt,rt_rb,lb_rb]))

         # If we moved closer to the object
         if distance > 0.0:
            distance = 2.0 - ( float( max([lt_lb,lt_rt,rt_rb,lb_rb])) - BORDER_PIXLES_KNOWN_DISTANCE )*( (BORDER_KNOWN_DISTANCE)/(BORDER_PIXLES_KNOWN_DISTANCE))
         elif distance < 0.0:
            distance = 2.0 + (BORDER_PIXLES_KNOWN_DISTANCE - float(max([lt_lb,lt_rt,rt_rb,lb_rb])))*((BORDER_KNOWN_DISTANCE)/(BORDER_PIXLES_KNOWN_DISTANCE))
         else:
            distance = 2.0
         print distance
         sys.exit()
         
         print float(max([lt_lb,lt_rt,rt_rb,lb_rb]))
         
         if self.verbosity:
            print "Distance from Object:  %f ft." % distance
         return distance
      else:
         if self.verbosity:
            print "Cannot calculate distance yet. Ensure that the left top, left bottom, right top, and right bottom corners have been initialized first."
         return -1
   
   '''
      --------------------------------------------
                      solvePnP()
      --------------------------------------------
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

      
      















