#!/usr/bin/env python

# ---------------------------------------------------------------------
#                           Python Modules
# ---------------------------------------------------------------------
import sys
import rospy
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
print abs_node_path

from camera import *

# ---------------------------------------------------------------------
#                           get_position()
# ---------------------------------------------------------------------
def get_position_h(req):

   # First calibrate the camera
   camera_inst = camera(verbosity=True)
   
   # Local variables to thread
   x = 0
   y = 0
   z = 0
   theta = 0
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
            print "Successfully decoded QR code in image!"
            break
         else:
            print "No QR code found in image, trying again..."
            rospy.sleep(1.)                                                  # Sleep for 1 second, note this is a floating point argument
            continue
      # Something went wrong getting an image from the Kinect
      else:
         print "There was an error retreiving an image, aborting..."
         break

   if not valid:
      print "Returning exception..."
   
   return qrcode_pos_serviceResponse(json_resp)
   camera_inst = None
   
   
# ---------------------------------------------------------------------
#                           proc_image()
# ---------------------------------------------------------------------
def proc_image(camera_inst, ros_image, print_data=False):

   # Local variables to thread
   data = False
   img_width = ros_image.width
   img_height = ros_image.height
   img_data = ros_image.data
   img_encoding = ros_image.encoding
   img_big_endian = ros_image.is_bigendian
   img_step = ros_image.step

   # Convert ROS RGB image to OpenCV 8-bit grayscale image
   cv_image = CvBridge().imgmsg_to_cv2(ros_image, "mono8")
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
         pass
      # clean up
      del(ros_image)
      #Assuming data is encoded in utf8
      data = symbol.data.decode(u'utf-8')

   # If a valid string was extracted from the QR code
   if data:

      # Make sure the QR code was meant for use with the project specified
      if data.find('project=%s' % str(PROJECT_VALUE)) != -1:
         split_data = data.split(",")
         x = int(split_data[0].replace('x=', ''))
         y = int(split_data[1].replace('y=', ''))
         z = int(split_data[2].replace('z=', ''))
         theta = int(split_data[3].replace('theta=', ''))
      
         # Get homography of QR code
         if not camera_inst.find_qr_homography(cv_image_color):
            # Could not successfully calculate homography, return invalid
            return [False,json.dumps({"valid":False})]
         
         # Print Jeeves QR code data extracted from image
         if print_data:
            print data

         # solvPnP
#         camera_inst.solvePnP()
         
         # Return success
         return [True,json.dumps({"valid":True,"x":x,"y":y,"z":z,"theta":theta})]
   
   # Could not successfully find/decode a QR code in image
   return [False,json.dumps({"valid":False})]

# ---------------------------------------------------------------------
#                        qrcode_pos_service()
# ---------------------------------------------------------------------
def qrcode_pos_service_f():
#   rospy.Subscriber("/camera/rgb/image_color", Image, proc_image)         # Subscribe to the 'chatter' topic, creates a new thread.
   s = rospy.Service('qrcode_pos_srv', qrcode_pos_service, get_position_h)         # Start the 'qrcode_pos_srv' service, of type 'scripts/qrcode_pos.srv', with handler function 'get_position_h'
   print "Ready to accept position requests..."
   rospy.spin()                                                            # Prevents script from terminating until service stops

# ---------------------------------------------------------------------
#                       Application Entry Point
# ---------------------------------------------------------------------
if __name__ == "__main__":

   rospy.init_node('qrcode_pos')                                           # Initialize ROS node
   SEC_CNT_TIMEOUT = rospy.get_param('~timeout_value')                     # Number of seconds before timeout
   PROJECT_VALUE = rospy.get_param('~project')                             # Project that this service should look for in QR codes found
#   SEC_CNT_TIMEOUT = 10                     # Number of seconds before timeout
#   PROJECT_VALUE = "mcecs_jeeves"                             # Project that this service should look for in QR codes found
   qrcode_pos_service_f()                                                    # Start the qrcode_pos_srv service
   
   
   
   
   
   
   
   
   
   
   