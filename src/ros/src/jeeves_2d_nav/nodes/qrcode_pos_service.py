#!/usr/bin/env python
'''
# /////////////////////////////////////////////////////////////////////////////////////////////////////
#                ROS QR Code Position Service for MCECS Jeeves Navigation
#
#     Engineer:  Josh Sackos
#                jsackos@pdx.edu
#                503.298.1820
#
#       School:  Portland State University
#       Course:  ECE 579
#      Project:  MCECS Jeeves
#  Description:  This Python module creates a Robot Operating System (ROS) service
#                node, that can be used to determine a robot's location. A Microsoft
#                Kinect is used to take pictures and extract data from detected QR
#                codes found in the image.
#
#                Upon finding a QR code that contains the "project" parameter specified
#                in the ROS launch file "qrcode_pos_service.launch", the distance and
#                angle from the robot to the QR code is calculated.
#
#                A "camera" class was created that contains the functions used for
#                the angle and distance calculations. It also contains functions
#                for computing homogrpahy, RQ-decomposition, etc., using the OpenCV 2.4
#                library.
#
#      Returns:  This ROS node returns a JSON encoded string that contains the follwoing
#                dictionary entries:
#
#                *          valid - Boolean value (True/False) of whether or not
#                                   the service successfully detected a QR code,
#                                   and calculated the angle/distnace.
#
#                *              r - Distance from robot to calculated center of QR code object.
#                *          theta - Angle from center of robot's camera to QR code object.
#                * obj_hemisphere - The image hemisphere that the QR code is located in.
#                * rqdecomp_x_deg - Allegedly the X axis rotation of the camera in the
#                                   pinhole camera model. NOTE: Not reliable yet
#                * rqdecomp_y_deg - Allegedly the Y axis rotation of the camera in the
#                                   pinhole camera model. NOTE: Not reliable yet
#                * rqdecomp_z_deg - Allegedly the Z axis rotation of the camera in the
#                                   pinhole camera model. NOTE: Not reliable yet
#                *             id - The ID code that was extracted from the QR code.
#
#        Notes:  As of now the RQ-Decomposition Euler angles cannot be relied upon as
#                they are not consistent from request to request, and the fact that
#                the data measured so far does not agree with the camera pinhole model.
#
#                There is poor documentation on the OpenCV functions involved, try to
#                avoid using this if possible. The functionallity was left in this
#                service to streamline development for future students working on Jeeves.
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
import sys
import rospy
rospy.init_node('qrcode_pos')       # Initialize ROS node
import freenect
from jeeves_2d_nav.srv import *     # Import qrcode_pos package
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import zbar
import json
# Add "modules" dir to path
abs_node_path = sys.argv[0].split('/')
abs_node_path = '/'.join(abs_node_path[0:len(abs_node_path)-1]) + '/../modules'
sys.path.append(abs_node_path)
from camera import *

'''
# ---------------------------------------------------------------------
#                       Configuration Parameters
# ---------------------------------------------------------------------
'''
# **** These parameters get their value from the ROS "qrcode_pos_service.srv" file. ****
SEC_CNT_TIMEOUT = rospy.get_param('~timeout_value')       # Number of seconds before timeout
PROJECT_VALUE = rospy.get_param('~project')               # Project that this service should look for in QR codes found
DEV_ENV = rospy.get_param('~dev_env')                     # Specifies whether in a development enviornment or not where images can be displayed.
                                                           # If enabled (i.e. set True) the Python code will display several images showing the
                                                           # OpenCV calculations/manipulations. For observing the calibration images used
                                                           # for calibrating the Kinect camera see the "SHOW_CALIB_IMAGES" parameter in
                                                           # "camera.py".
VERBOSITY = rospy.get_param('~verbosity')                 # This specifies whether or not images and/or text will be displayed or not.

# **** Hardcoded values to be used when developing/using rosrun. You do not want to use the launch file or else no text/images will be displayed. ****
#DEV_ENV = bool(0)
#VERBOSITY = bool(1)
#SEC_CNT_TIMEOUT = 3                                        # Number of seconds before timeout
#PROJECT_VALUE = "mcecs_jeeves"                             # Project that this service should look for in QR codes found

if VERBOSITY:
   print "Development Environment:  %s " % str(DEV_ENV)
   print "Modules Path: %s" % abs_node_path

'''
# -----------------------------------------------------------------------------------------------------
#                                              get_position()
#
#   Description:  This is the main task that gets called whenever a new thread is created for a ROS
#                 request. The task initializes a "camera" class instance, waits for a ROS "freenect"
#                 image message, and processes the image received. The actual image analysis is 
#                 performed in a function called "proc_image" that this function calls.
#
#                 If detection fails this main task will timeout and return an error JSON string where
#                 "valid" is set false.
#
#                 Upon a valid detection/calculation this function will return a JSON string to the
#                 client ROS node that contains the data specified below.
#
#     Arguments:  req - Object that provides access to arguments that the requesting ROS client node
#                 passed when it made its request.
#
#       Returns:  A JSON string that contains the following dictionary
#                 entries:
#
#                *          valid - Boolean value (True/False) of whether or not
#                                   the service successfully detected a QR code,
#                                   and calculated the angle/distnace.
#
#                *              r - Distance from robot to calculated center of QR code object.
#                *          theta - Angle from center of robot's camera to QR code object.
#                * obj_hemisphere - The image hemisphere that the QR code is located in.
#                * rqdecomp_x_deg - Allegedly the X axis rotation of the camera in the
#                                   pinhole camera model. NOTE: Not reliable yet
#                * rqdecomp_y_deg - Allegedly the Y axis rotation of the camera in the
#                                   pinhole camera model. NOTE: Not reliable yet
#                * rqdecomp_z_deg - Allegedly the Z axis rotation of the camera in the
#                                   pinhole camera model. NOTE: Not reliable yet
#                *             id - The ID code that was extracted from the QR code.
#
# -----------------------------------------------------------------------------------------------------
'''
def get_position_h(req):

   # First calibrate the camera
   camera_inst = camera(verbosity=VERBOSITY, src_image="../images/qrcode_query_image.png")
   
   # Local variables to thread
   i = 0
   
   valid = False
   bln_timeout = False                                                         # Timeout flag
   sec_cnt = 0                                                                 # Count the number of seconds spent trying to get an image with a QR code, timeout at 30
   json_response = ""
   
   # Pull images until we get one with a QR code, try up to 30 seconds
   while(not bln_timeout):

      # If timeout reached return exception
      sec_cnt += 1
      if sec_cnt == SEC_CNT_TIMEOUT:
         bln_timeout = True
      
      # Get a single image from freenect Kinect node
      image = rospy.wait_for_message("/camera/rgb/image_color", Image, timeout=SEC_CNT_TIMEOUT)

      # Decode QR code if exists
      if image:
         valid, json_resp = proc_image(camera_inst, image)

         if valid:
            if VERBOSITY:
               print "Successfully decoded QR code in image!"
            break
         else:
            if VERBOSITY:
               print "No QR code found in image, trying again..."
            rospy.sleep(1.)                                                     # Sleep for 1 second, note this is a floating point argument
            continue
      # Something went wrong getting an image from the Kinect
      else:
         if VERBOSITY:
            print "There was an error retreiving an image, aborting..."
         break

   if not valid:
      if VERBOSITY:
         print "Returning exception..."
   
   del camera_inst
   return qrcode_pos_serviceResponse(json_resp)
   

'''
# -----------------------------------------------------------------------------------------------------
#                                             proc_image()
#
#   Description:  This function takes the image passed in as an argument, searches for a QR code,
#                 checks whether or not the QR code was meant for the project specified in the launch
#                 file, and calculates the angle/distance from the center of the camera to the detected
#                 QR code upon a successful detection. If the attempt is successful a JSON string with
#                 the dictionary entries listed below is returned. If the attempt was unsuccessful, this
#                 fucntion returns a JSON encoded string that contains a single dictionary entry "valid"
#                 with a value of "False".
#
#     Arguments:  camera_inst - A "camera" class instance. See "camera.py"
#                 
#                   ros_image - The image returned by the "freenect" node.
#
#       Returns:  A JSON string that contains the following dictionary
#                 entries:
#
#                *          valid - Boolean value (True/False) of whether or not
#                                   the service successfully detected a QR code,
#                                   and calculated the angle/distnace.
#
#                *              r - Distance from robot to calculated center of QR code object.
#                *          theta - Angle from center of robot's camera to QR code object.
#                * obj_hemisphere - The image hemisphere that the QR code is located in.
#                * rqdecomp_x_deg - Allegedly the X axis rotation of the camera in the
#                                   pinhole camera model. NOTE: Not reliable yet
#                * rqdecomp_y_deg - Allegedly the Y axis rotation of the camera in the
#                                   pinhole camera model. NOTE: Not reliable yet
#                * rqdecomp_z_deg - Allegedly the Z axis rotation of the camera in the
#                                   pinhole camera model. NOTE: Not reliable yet
#                *             id - The ID code that was extracted from the QR code.
#
# -----------------------------------------------------------------------------------------------------
'''
def proc_image(camera_inst, ros_image):

   # Local variables to thread
   data = False
   img_width      = ros_image.width
   img_height     = ros_image.height
   img_data       = ros_image.data
   img_encoding   = ros_image.encoding
   img_big_endian = ros_image.is_bigendian
   img_step       = ros_image.step
   
   qrcode_data    = None
   qrcode_corners = None

   # Convert ROS RGB image to OpenCV 8-bit grayscale image and 8-bit RGB image
   cv_image       = CvBridge().imgmsg_to_cv2(ros_image, "mono8")
   cv_image_color = CvBridge().imgmsg_to_cv2(ros_image, "bgr8")
   
   # Save image
   cv2.imwrite(abs_node_path + "../images/camera_image.jpg", cv_image_color)

   # Extract only the actual image data and pack as binary data
   string_data = struct.pack('<%dc' % (img_width*img_height), *cv_image.data)

   # Create a Zbar scanner
   scanner = zbar.ImageScanner()
   # Configure the reader
   scanner.parse_config('enable')

   # Bring the image into Zbar
   zbar_img = zbar.Image(img_width, img_height, 'Y800', string_data)
   
   # Scan the image for barcodes
   result = scanner.scan(zbar_img)
   
   # Extract results
   if result == 0:
        pass
   else:
      for symbol in zbar_img:
         print symbol.location
         #Assuming data is encoded in utf8
         data = symbol.data.decode(u'utf-8')
         if data.find('project=%s' % str(PROJECT_VALUE)) != -1:
            print "Good Match = %s" % data
            qrcode_data = data
            qrcode_corners = list(symbol.location)
         else:
            print "Bad Match = %s" % data

      # clean up
      del(ros_image)

   # If a valid string was extracted from the QR code
   if qrcode_data:

      # Make sure the QR code was meant for use with the project specified
      if qrcode_data.find('project=%s' % str(PROJECT_VALUE)) != -1:
         split_data = qrcode_data.split(",")
         qr_id = int(split_data[0].replace('id=', ''))

         # Get homography of QR code
         bln_found_h_mtx, homography_mtx,ltc,lbc,rtc,rbc = camera_inst.find_qr_homography(cv_image_color, qrcode_corners[0],qrcode_corners[1],qrcode_corners[2],qrcode_corners[3])
         
         if not bln_found_h_mtx:
            # Could not successfully calculate homography, return invalid
            return [False,json.dumps({"valid":False})]
         
         # Calculate object center on the x axis
         obj_center = camera_inst.x_obj_center(qrcode_corners[0][0], qrcode_corners[3][0]);
         
         # Calculate distance from object in ft. relative to longest side of bounding rectangle in pixels
         r = camera_inst.calc_distance(qrcode_corners[0],qrcode_corners[1],qrcode_corners[2],qrcode_corners[3]);

         # Calculate angle from robot to object in the XZ plane in the pinhole camera model
         theta = camera_inst.x_obj_angle(obj_center, r);
         
         # Calculate object hemisphere location
         obj_hemisphere = camera_inst.x_obj_hemisphere(obj_center);
         
         # Calculate RQDecomp3x3() of homography matrix
         rqdecomp_rot_mtx,rqdecomp_x_deg,rqdecomp_y_deg,rqdecomp_z_deg,rqdecomp_q_mtx = camera_inst.decompose_homography(homography_mtx);
         
         # Print Jeeves QR code data extracted from image
         if VERBOSITY:
            print "----------- Extracted QR Code Data -----------"
            print qrcode_data
            print "----------------------------------------------"
         
         # Return success
         return [True,json.dumps({"valid":True,"r":r,"theta":theta, "obj_hemisphere":obj_hemisphere, "rqdecomp_x_deg":rqdecomp_x_deg,"rqdecomp_y_deg":rqdecomp_y_deg,"rqdecomp_z_deg":rqdecomp_z_deg,"id":qr_id})]
   
   # Could not successfully find/decode a QR code in image
   return [False,json.dumps({"valid":False})]

'''
# -----------------------------------------------------------------------------------------------------
#                                        qrcode_pos_service()
#
#   Description:  This function creates the ROS service "qrcode_pos_srv". This fucntion gets called
#                 only when the launch file is executed, or when "rosrun" is used to create the
#                 service.
#
#     Arguments:  N/A
#
#       Returns:  N/A
#
# -----------------------------------------------------------------------------------------------------
'''
def qrcode_pos_service_f():
   s = rospy.Service('qrcode_pos_srv', qrcode_pos_service, get_position_h)         # Start the 'qrcode_pos_srv' service, of type 'scripts/qrcode_pos_service.srv', with handler function 'get_position_h'
   if VERBOSITY:
      print "Ready to accept position requests..."
   rospy.spin()                                                                    # Prevents script from terminating until service stops

# ---------------------------------------------------------------------
#                       Application Entry Point
# ---------------------------------------------------------------------
if __name__ == "__main__":

   qrcode_pos_service_f()                                                          # Start the qrcode_pos_srv service
   
   
   
   
   
   
   
   
   
   
   