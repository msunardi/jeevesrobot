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
   return resp_data                                                                     # Return positional data received from service


# ---------------------------------------------------------------------
#                       Application Entry Point
# ---------------------------------------------------------------------
if __name__ == "__main__":
   print "Requesting position..."
   position = get_position()
   if position['valid']:
      print "id = %d, r = %f, theta = %f, obj_hemisphere = %s, rqdecomp_x_deg = %f, rqdecomp_y_deg = %f, rqdecomp_z_deg = %f" % (position["id"], position["r"], position["theta"], position["obj_hemisphere"],position["rqdecomp_x_deg"], position["rqdecomp_y_deg"], position["rqdecomp_z_deg"])
   else:
      print "Invalid coordinates received..."