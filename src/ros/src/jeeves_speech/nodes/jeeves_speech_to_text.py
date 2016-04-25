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
import sys, os, time, json
import rospy
import aiml                               # Artificial Intelligence Markup Language
import subprocess                         # import for calling festival from command line
from std_msgs.msg import String

# Set current working directory
cwd = os.chdir(os.path.dirname(os.path.realpath(__file__)) + '/../');
cwd = os.getcwd() + '/';

#from speech_to_text.msg import speech_to_text_str   # Import speech message

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

'''
# -----------------------------------------------------------------------------------------------------
#                                        jeeves_speech_to_text_f()
#
#   Description:  This is the main() function for the Jeeves speech to text publisher.
#
#     Arguments:  N/A
#
#       Returns:  N/A
#
# -----------------------------------------------------------------------------------------------------
'''
def jeeves_speech_to_text_f():

   global jeeves
   aiml_string = ''
   
   # Create the "jeeves_speech_to_text" ROS node
   rospy.init_node('jeeves_speech_to_text')
   
   # Create topic to publish to
   s2t_topic = rospy.Publisher('jeeves_speech/speech_to_text', String, queue_size=10)   

   # Start PocketSphinx
#   proc = subprocess.Popen(["pocketsphinx_continuous -dict " + cwd + "jeeves.dic"], shell=True, stdout=subprocess.PIPE)
   proc = subprocess.Popen(["pocketsphinx_continuous -lm " + cwd + "jeeves.lm -dict " + cwd + "jeeves.dic"], shell=True, stdout=subprocess.PIPE)

   if VERBOSITY:
      print "Ready to process speech requestions and/or commands..."
   
   # Always listen for speech
   while(1):
      # Speech to text 
      speech_txt = proc.stdout.readline().rstrip()
      
      try:
         # If a string actually exists
         if len(speech_txt) > 0:
            # If a speech to text conversion actually occurred 

            if speech_txt[0].isdigit():
               speech_txt = speech_txt.split(': ')                                         # Break apart the CMU Sphinx response. It will always start with a 32-bit number and colon-space.

               if len(speech_txt) > 1:
                  # Print the speech to text string
                  if VERBOSITY:
                     print "CMU Sphinx Text:  %s" % speech_txt[1]
                     s2t_topic.publish(speech_txt[1])
                     
      except Exception as e:
         print "========================================"
         print "THERE WAS AN EXCEPTION"
         print speech_txt
         print type(speech_txt)
         print e

      # Sleep for 100ms before checking for another sentence
      time.sleep(0.1)
   
   # Prevents script from terminating until service stops
   rospy.spin()

# ---------------------------------------------------------------------
#                       Application Entry Point
# ---------------------------------------------------------------------
if __name__ == "__main__":

   jeeves_speech_to_text_f()                                                         # Start the jeeves_speech_srv service
   
   
   
   
   
   
   
   
   
   
   