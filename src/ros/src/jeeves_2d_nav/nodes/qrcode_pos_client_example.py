#!/usr/bin/env python
'''
# /////////////////////////////////////////////////////////////////////////////////////////////////////
#                     ROS QR Code Position Client for MCECS Jeeves Navigation
#
#     Engineer:  Josh Sackos
#                jsackos@pdx.edu
#                503.298.1820
#
#       School:  Portland State University
#       Course:  ECE 579
#      Project:  MCECS Jeeves
#  Description:  This ROS Python node makes a request to the "qrcode_pos_srv" ROS service. The
#                purpose of this node is merely to serve as a template for making requests
#                to the service.
#
#        Notes:  As of now the RQ-Decomposition Euler angles cannot be relied upon as
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
import sys
import rospy
from jeeves_2d_nav.srv import *
import json
import string


'''
# -----------------------------------------------------------------------------------------------------
#                                              get_position()
#
#   Description:  This function is the main example of making a request to the "qrcode_pos_srv"
#                 service.
#
#     Arguments:  N/A
#
#       Returns:  A JSON string that contains the following dictionary entries:
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
def get_position():
   
   # Wait for service to come up
   rospy.wait_for_service('qrcode_pos_srv')
   
   try:
      qrcode_pos_h = rospy.ServiceProxy('qrcode_pos_srv', qrcode_pos_service)              # Set handler to service
      resp_data = json.loads(qrcode_pos_h().json_resp)
      return resp_data                                                                     # Return positional data received from service
   except Exception as e:
      return {'valid':False}



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
      




