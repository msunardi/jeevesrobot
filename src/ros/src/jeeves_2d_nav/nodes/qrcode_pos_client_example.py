#!/usr/bin/env python

# ---------------------------------------------------------------------
#                           Python Modules
# ---------------------------------------------------------------------
import sys
import rospy
from qrcode_pos.srv import *
import json
import string


# ---------------------------------------------------------------------
#                           get_position()
# ---------------------------------------------------------------------
def get_position():
   
   # Wait for service to come up
   rospy.wait_for_service('qrcode_pos_srv')

   qrcode_pos_h = rospy.ServiceProxy('qrcode_pos_srv', qrcode_pos)              # Set handler to service
   resp_data = json.loads(qrcode_pos_h().json_resp)				# Make request to "qrcode_pos_srv", and decode JSON response string
   return resp_data                                                             # Return positional data received from service


# ---------------------------------------------------------------------
#                       Application Entry Point
# ---------------------------------------------------------------------
if __name__ == "__main__":
   print "Requesting position..."
   position = get_position()
   # If the "valid" dictionary entry is True then valid data resides in the dictionary
   if position['valid']:
      print "x = %d, y = %d, z = %d, theta = %d" % (position["x"], position["y"], position["z"], position["theta"])
   # The "valid" dictionary entry was False. This means an attempt to acquire position data from a QR code meant for the project failed.
   else:
      print "Invalid coordinates received..."
