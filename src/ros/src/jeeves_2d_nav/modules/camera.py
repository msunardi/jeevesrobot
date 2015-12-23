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
import matplotlib
#matplotlib.use('GTKAgg')
print matplotlib.rcsetup.interactive_bk
from matplotlib import pyplot as plt
import time



# =======================================================================================
#                                 Global Variables
# =======================================================================================

# Get absolute path of where scripts/nodes are located
abs_node_path = sys.argv[0].split('/')
abs_node_path = '/'.join(abs_node_path[0:len(abs_node_path)-1]) + '/'

FLANN_INDEX_LSH  = 6        # For FLANN matching when using OpenCV ORB
RATIO_TEST_PARAM = 0.75     # Maximum distance point n can be from point m when determing good FLANN matches
MIN_MATCH_COUNT  = 35       # Minimum number of good matches detected by ratio test
NUM_TREE_CHECKS  = 50       # Number of times trees are recursively checked. Higher value results in better results, but takes longer

src_img = cv2.imread(abs_node_path + '../images/qrcode_query_image.jpg',0)    # Template for detecting qr code in random image
#tar_img = cv2.imread(abs_node_path + '../images/camera_image.jpg',0)

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
   camera_mtx     = None # Calibrated camera matrix
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
   def __init__(self, image_dir='../images', verbosity=False):

      # Store verbosity
      self.verbosity = verbosity
      
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
      None

      
   '''
      --------------------------------------------
                   calibrate_camera()
      --------------------------------------------
   '''
   def calibrate_camera(self):
      
      if self.verbosity:
         print "Calibrating camera..."
      
      # Process each filename in images
      for fname in (self.images):
         img  = cv2.imread(fname)
         gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

         # Find the chess board corners
         self.ret, self.corners = cv2.findChessboardCorners(gray, (7,6),None)

         # If found, add object points, image points (after refining them)
         if self.ret == True:
            self.objpoints.append(self.objp)
            cv2.cornerSubPix(gray,self.corners,(11,11),(-1,-1),self.criteria)
            self.imgpoints.append(self.corners)

            if self.verbosity:
               # Draw and display the corners
#               cv2.drawChessboardCorners(img, (7,6), self.corners,self.ret)
#               plt.imshow(img),plt.show();
               pass

            
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
         print "\nFailed to calibrate camera. Chessboard pattern not detected...\n"


   '''
      --------------------------------------------
                  find_qr_homography()
      --------------------------------------------
   '''
   def find_qr_homography(self, target_cv2_image):
      
      if self.verbosity:
         print "Detecting QR code homography..."
         
      # Initiate STAR detector for source and target images, detect up to 5000 features each
      src_orb = cv2.ORB(5000)
      tar_orb = cv2.ORB(5000)

      # Compute ORB descriptors for source image. Convert memory locations to coordinates in Python list?
      src_kp, src_des = src_orb.detectAndCompute(src_img, None)
      tar_kp, tar_des = tar_orb.detectAndCompute(target_cv2_image, None)
      
      if self.verbosity:
         print "Source Features: %d" % (len(src_kp))
         print "Target Features: %d" % (len(tar_kp))

      # Setup FLANN matching algorithm for use with ORB
      index_params= dict(algorithm = FLANN_INDEX_LSH,
                         table_number = 6,      # 12
                         key_size = 12,         # 20
                         multi_probe_level = 1) # 2

      # Create search parameters dictionary.
      # Specifies number of times trees are recursively searched.
      # A higher value gives better results but takes longer.
      search_params = dict(checks=NUM_TREE_CHECKS)   # or pass empty dictionary

      # Create the flann matcher object
      flann = cv2.FlannBasedMatcher(index_params,search_params)

      # Match source descriptors with target descriptors
      matches = flann.knnMatch(src_des,tar_des,k=2)

      # Keep only good matches that pass the ratio test
      #
      #     Structure of a match in matches
      #             -> float distance
      #             -> int imgIdx       , Train image index
      #             -> int queryIdx     , Query descriptor index
      #             -> int trainIdx     , Train descriptor index
      good_matches = []
      for m, n in matches:      
            # If the match passes the ratio test add it to the list of good matches
            if m.distance < RATIO_TEST_PARAM*n.distance:
               good_matches.append(m)
      if self.verbosity:
         print "Good Matches = %d" % len(good_matches)

      # If not enough good matches alert and abort
      if(not len(good_matches) >= MIN_MATCH_COUNT):
         if self.verbosity:
            print "The number of good matches found is less than %d, aborting" % MIN_MATCH_COUNT
         return False

      # Create list of source points and list of target/destination points from good matches
      self.h_src_pts = np.float32([ src_kp[m.queryIdx].pt for m in good_matches ]).reshape(-1,1,2)
      self.h_tar_pts = np.float32([ tar_kp[m.trainIdx].pt for m in good_matches ]).reshape(-1,1,2)
      
      # Extract corner points of the detected plane
      self.left_top_corner     = [65535,65535]
      self.left_bottom_corner  = [65535,0]
      self.right_top_corner    = [0,65535]
      self.right_bottom_corner = [0,0]
      for point in self.h_tar_pts:
         
         # Left top corner
         if(point[0][0] <= self.left_top_corner[0] and point[0][1] <= self.left_top_corner[1]):
            self.left_top_corner = point[0]
            
         # Left Bottom corner
         if(point[0][0] <= self.left_bottom_corner[0] and point[0][1] >= self.left_bottom_corner[1]):
            self.left_bottom_corner = point[0]

         # Right top corner
         if(point[0][0] >= self.right_top_corner[0] and point[0][1] <= self.right_top_corner[1]):
            self.right_top_corner = point[0]
            
         # Right Bottom corner
         if(point[0][0] >= self.right_bottom_corner[0] and point[0][1] >= self.right_bottom_corner[1]):
            self.right_bottom_corner = point[0]

      if self.verbosity:
         print self.left_top_corner
         print self.left_bottom_corner
         print self.right_top_corner
         print self.right_bottom_corner
         
         print "\n----------- Detected Plane Corners --------------"
         print "Left top corner    :  %f , %f" % (self.left_top_corner[0],self.left_top_corner[1])
         print "Left bottom corner :  %f , %f" % (self.left_bottom_corner[0],self.left_bottom_corner[1])
         print "Right top corner   :  %f , %f" % (self.right_top_corner[0],self.right_top_corner[1])
         print "Right bottom corner:  %f , %f" % (self.right_bottom_corner[0],self.right_bottom_corner[1])
      
      # Find homography
      self.homography_mtx, self.h_mask = cv2.findHomography(self.h_src_pts, self.h_tar_pts, cv2.RANSAC,5.0)

      if self.verbosity:
         print "\nHomography Matrix:"
         print "-------------------------"
         print self.homography_mtx
         print "-------------------------\n"

         # Draw keypoint locations, not size and orientation
         src_kp_img = cv2.drawKeypoints(src_img,src_kp,color=(0,255,0), flags=0)
         tar_kp_img = cv2.drawKeypoints(target_cv2_image,tar_kp,color=(0,255,0), flags=0)

         # Merge images for dual display
         self.vis = np.concatenate((src_kp_img, tar_kp_img), axis=1);
#         plt.imshow(self.vis)
#         plt.imshow(target_cv2_image)
#         plt.show()

      # Decompose homography matrix
      eulerAngles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(self.homography_mtx)

      # Rotation matricies
      self.rot_mtx = mtxR
      if self.verbosity:
         print "---------- Rotation Matricies ------------"
         print self.rot_mtx

      # Extract euler angles
      self.euler_x_deg = eulerAngles[0]
      self.euler_y_deg = eulerAngles[1]
      self.euler_z_deg = eulerAngles[2]
      if self.verbosity:
         print "---------- Euler Angles ------------"
         print "Degrees:  X = %f" % self.euler_x_deg
         print "Degrees:  Y = %f" % self.euler_y_deg
         print "Degrees:  Z = %f" % self.euler_z_deg

      # Store Q matrix
      self.q_mtx = mtxQ
      if self.verbosity:
         print "---------- Q Matrix ------------"
         print self.q_mtx

      # Store translation vectors
      self.qx_vec = self.q_mtx[0]
      self.qy_vec = self.q_mtx[1]
      self.qz_vec = self.q_mtx[2]
      if self.verbosity:
         print "---------- Translation Vectors ------------"
         print self.qx_vec
         print self.qy_vec
         print self.qz_vec

      self.calc_distance()
         
      return True

   '''
      --------------------------------------------
                      solvePnP()
      --------------------------------------------
   '''
   def solvePnP(self):
      
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
      print "--------------- Identity 3x4 matrix --------------"
      print np.float32(id_3x4)
      
      print "--------------- cam_mtx * id_3x4 -----------------"
      cam_id = tmp_cam_mtx*id_3x4
      print cam_id
      
      # Concatenate rotation matrix with translation matrix, 0's, and a 1, yields a 4x4 matrix
      rot_tv = np.concatenate((self.rot_mtx, self.img_tvec), axis=1)
      rot_tv = np.append(  rot_tv, [[0,0,0,1]] , axis=0  )
      
      print rot_tv
      cam_rot_tv = tmp_cam_mtx*rot_tv
      print "----------- cam_rot_tv -----------"
      print cam_rot_tv
      
      # Expand translation vector

      
   '''
      --------------------------------------------
                   calc_distance()
      --------------------------------------------
   '''
   def calc_distance(self):
      
      # Determine longest side of rectangle and use it
      lt_lb = float(self.left_bottom_corner[1]  - self.left_top_corner[1])
      lt_rt = float(self.right_top_corner[0]    - self.left_top_corner[0])
      rt_rb = float(self.right_bottom_corner[1] - self.right_top_corner[1])
      lb_rb = float(self.right_bottom_corner[0] - self.left_bottom_corner[0])
      
      self.distance = float(max([lt_lb,lt_rt,rt_rb,lb_rb]))*((3.0)/(117.878))
      
      print lt_lb
      print lt_rt
      print rt_rb
      print lb_rb
      print "You are %f feet away!" % self.distance
      
      















