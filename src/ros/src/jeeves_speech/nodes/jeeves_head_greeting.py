#!/usr/bin/env python
'''
# /////////////////////////////////////////////////////////////////////////////////////////////////////
#                          ROS node to make Jeeves head move to greet people.
#
#     Engineer:  Lucas Myers
#                myers4@pdx.edu
#
#       School:  Portland State University
#       Course:  ECE 579
#      Project:  MCECS Jeeves
#  Description:  
#
#      Returns:  
#
#    Revisions:
#
#                rev 0.0 - File created - 02/20/2017
#
# /////////////////////////////////////////////////////////////////////////////////////////////////////
'''

'''
# ---------------------------------------------------------------------
#                               Imports
# ---------------------------------------------------------------------
'''
import sys, os, time, json, string
import numpy
import rospy
from std_msgs.msg import String, Float32, Float64

# Set current working directory
#cwd = os.chdir(os.path.dirname(os.path.realpath(__file__)) + '/../');
#cwd = os.getcwd() + '/';

JEEVES_KEYWORDS = ["jeeves listen", "hi jeeves", "hello jeeves", "bye jeeves", "jeeves"];

'''
# ---------------------------------------------------------------------
#                       Configuration Parameters
# ---------------------------------------------------------------------
'''

# **** Hardcoded values to be used when developing/using rosrun ****
DEV_ENV = bool(1)
VERBOSITY = bool(1)

if VERBOSITY:
   print "Development Environment:  %s " % str(DEV_ENV)

# -----------------------------------------------------------------------------------------------------
#                                           proc_text()
#
#   Description:  This function gets called whenever a message is recieved from the speech_to_text
#                 topic. It is responsible for recievin the text, and publishing
#                 to the neck_control topics to move the head.
#
#     Arguments:  N/A
#
#       Returns:  N/A
#
# -----------------------------------------------------------------------------------------------------

def proc_text(message):
   
   speech_txt = message.data;

   pitch_topic = rospy.Publisher('cmd_yo', Float32, queue_size=10)
   
   pitch_topic.publish(0.8);

   return

'''
# -----------------------------------------------------------------------------------------------------
#                                        jeeves_head_greeting_f()
#
#   Description:  This is the main() function for the Jeeves head greeting publisher.
#
#     Arguments:  N/A
#
#       Returns:  N/A
#
# -----------------------------------------------------------------------------------------------------
'''
def jeeves_head_greeting_f():
   
   # Create the "jeeves_head_greeting" ROS node
   rospy.init_node('jeeves_head_greeting')  

   # ---------------------------------------
   # Subscribe to speech_to_text topic
   # ---------------------------------------
   rospy.Subscriber("jeeves_speech/speech_to_text", String, proc_text)

   if VERBOSITY:
      print "Listening for text..."
   
   # Prevents script from terminating until service stops
   rospy.spin()

# ---------------------------------------------------------------------
#                       Application Entry Point
# ---------------------------------------------------------------------
if __name__ == "__main__":

   jeeves_head_greeting_f()
   
   
   
   
   
   
   
   
   
   
   
