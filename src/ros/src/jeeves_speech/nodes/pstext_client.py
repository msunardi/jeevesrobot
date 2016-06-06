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
from jeeves_speech.srv import *
import json
import string


'''
# -----------------------------------------------------------------------------------------------------
#                                               get_text()
#
#   Description: 
#
#     Arguments:  N/A
#
#       Returns: 
#
# -----------------------------------------------------------------------------------------------------
'''
def get_text():
   
   # Wait for service to come up
   rospy.wait_for_service('jeeves_speech_srv')
   
   try:
      jeeves_speech_h = rospy.ServiceProxy('jeeves_speech_srv', jeeves_speech_service)              # Set handler to service
      resp_data = json.loads(jeeves_speech_h().json_resp)
      return resp_data                                                                     # Return positional data received from service
   except Exception as e:
      return {'valid':False}



# ---------------------------------------------------------------------
#                       Application Entry Point
# ---------------------------------------------------------------------
if __name__ == "__main__":
   print "Making request to speech service..."
   speech_to_text = get_text()
   if position['valid']:
      print "text = %s" % (speech_to_text["text"])
   else:
      print "Invalid text response received..."
      




