#!/usr/bin/env python

# ---------------------------------------------------------------------
#                           Python Modules
# ---------------------------------------------------------------------
import sys
import rospy
from jeeves_2d_nav.srv import *
import json
import string


# ---------------------------------------------------------------------
#                           get_position()
# ---------------------------------------------------------------------
def get_position():
   
   # Wait for service to come up
   rospy.wait_for_service('qrcode_pos_srv')

   qrcode_pos_h = rospy.ServiceProxy('qrcode_pos_srv', qrcode_pos_service)              # Set handler to service
   resp_data = json.loads(qrcode_pos_h().json_resp)
   return resp_data                                                             # Return positional data received from service


# ---------------------------------------------------------------------
#                       Application Entry Point
# ---------------------------------------------------------------------
if __name__ == "__main__":
   print "Requesting position..."
   position = get_position()
   if position['valid']:
      print "x = %d, y = %d, z = %d, theta = %d" % (position["x"], position["y"], position["z"], position["theta"])
   else:
      print "Invalid coordinates received..."