#!/usr/bin/env python
'''
# /////////////////////////////////////////////////////////////////////////////////////////////////////
#                          ROS Speech Service for MCECS Jeeves Speech Recognition
#
#     Engineer:  Josh Sackos
#                jsackos@pdx.edu
#                503.298.1820
#
#       School:  Portland State University
#       Course:  ECE 510
#      Project:  MCECS Jeeves
#  Description:  
#
#      Returns:  
#
#    Revisions:
#
#                rev 0.0 - File created - 04/15/2016
#
# /////////////////////////////////////////////////////////////////////////////////////////////////////
'''

'''
# ---------------------------------------------------------------------
#                               Imports
# ---------------------------------------------------------------------
'''
import sys, os, time, json, string
import rospy
import subprocess
from std_msgs.msg import String

# Set current working directory
cwd = os.chdir(os.path.dirname(os.path.realpath(__file__)));
cwd = os.getcwd() + '/../';

'''
# ---------------------------------------------------------------------
#                       Configuration Parameters
# ---------------------------------------------------------------------
'''
# **** These parameters get their value from the ROS "qrcode_pos_service.srv" file. ****
#PROJECT_VALUE = rospy.get_param('~project')
#VERBOSITY = rospy.get_param('~verbosity')

# **** Hardcoded values to be used when developing/using rosrun ****
DEV_ENV = bool(1)
VERBOSITY = bool(1)

if VERBOSITY:
   print "Development Environment:  %s " % str(DEV_ENV)

handshake_topic = False;
   
'''
# -----------------------------------------------------------------------------------------------------
#                                            proc_cmd()
#
#   Description:  
#
#     Arguments:  N/A
#
#       Returns:  N/A
#
# -----------------------------------------------------------------------------------------------------
'''
def synthesize(message):
   
   global handshake_topic;
   
   if len(message.data) == 0:
      print "Invalid speech request received..."
   else:
      print message.data
#	subprocess.call(["festival", "--batch", "(voice_rab_diphone)", "(SayText \"" + text.data + "\")"])
   subprocess.call(["festival", "--batch", "(SayText \"" + message.data + "\")"])
   
   # Provide handshake to "jeeves_speech_to_text" node
   handshake_topic.publish("True");
	
   return

'''
# -----------------------------------------------------------------------------------------------------
#                                        jeeves_text_category_f()
#
#   Description:  This is the main() function for the Jeeves text category publisher.
#
#     Arguments:  N/A
#
#       Returns:  N/A
#
# -----------------------------------------------------------------------------------------------------
'''
def jeeves_speech_synthesis_f():
   
   global handshake_topic;
   
   # Create the "jeeves_speech_to_text" ROS node
   rospy.init_node('jeeves_speech_synthesis')

   handshake_topic = rospy.Publisher('jeeves_speech/speech_handshake'   , String, queue_size=10)
   
   # Subscribe to speech_to_text topic
   rospy.Subscriber("jeeves_speech/speech_synthesis", String, synthesize)
   
   if VERBOSITY:
      print "Listening for speech requests..."
   
   # Prevents script from terminating until service stops
   rospy.spin()

# ---------------------------------------------------------------------
#                       Application Entry Point
# ---------------------------------------------------------------------
if __name__ == "__main__":

   jeeves_speech_synthesis_f()                                                         # Start the jeeves_speech_srv service
   
   

