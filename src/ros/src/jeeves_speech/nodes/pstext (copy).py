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
#       Course:  ECE 579
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
from std_msgs.msg import String
import rospy
import aiml                               # Artificial Intelligence Markup Language
import subprocess                         # import for calling festival from command line
from jeeves_speech.srv import *           # Import qrcode_pos package

# Add "modules" dir to path
abs_node_path = sys.argv[0].split('/')
abs_node_path = '/'.join(abs_node_path[0:len(abs_node_path)-1]) + '/../'
sys.path.append(abs_node_path)

'''
# ---------------------------------------------------------------------
#                       Configuration Parameters
# ---------------------------------------------------------------------
'''
# **** These parameters get their value from the ROS "qrcode_pos_service.srv" file. ****
#PROJECT_VALUE = rospy.get_param('~project')
#VERBOSITY = rospy.get_param('~verbosity')

# **** Hardcoded values to be used when developing/using rosrun ****
DEV_ENV = bool(0)
VERBOSITY = bool(0)

if VERBOSITY:
   print "Development Environment:  %s " % str(DEV_ENV)
#   print "Modules Path: %s" % abs_node_path

cwd = os.path.dirname( os.path.realpath(__file__) )

# initialize aiml speech system, and load aiml xml files
jeeves = aiml.Kernel()

jeeves.learn(cwd + "/../jeeves.xml")
jeeves.respond('load aiml b') #finished initializing

print cwd


'''
# -----------------------------------------------------------------------------------------------------
#                                              get_text_h()
#
#   Description:  
#
#     Arguments:  
#
#       Returns:  A JSON string that contains the following dictionary
#                 entries:
#
#                *          valid - Boolean value (True/False) of whether or not
#
# -----------------------------------------------------------------------------------------------------
'''
'''
def get_text_h(req):
   
   proc = subprocess.Popen(["pocketsphinx_continuous -lm " + cwd + "/../jeeves.lm -dict " + cwd + "/../jeeves.dic"], shell=True, stdout=subprocess.PIPE)
#   proc = subprocess.Popen(["pocketsphinx_continuous -lm " + cwd + "/../hub4.5000.DMP -dict " + cwd + "/../cmu07a.dic"], shell=True, stdout=subprocess.PIPE)
#   proc = subprocess.Popen(["pocketsphinx_continuous -dict " + cwd + "/../cmu07a.dic"], shell=True, stdout=subprocess.PIPE)
   
   while(1):
      output = proc.stdout.readline().rstrip()
      if output[0].isdigit():
         output = output.split(': ')
         print "CMU Sphinx Text:  %s" % output[1]
         aiml_string = jeeves.respond( output[1].lower() )
         if len(aiml_string) == 0:
            print "I'm sorry, I did not understand what you said."
         else:
            print "Jeeves's Response: %s" % aiml_string
      time.sleep(1)
   json_resp = {"valid":True};
   return jeeves_speech_serviceResponse(json_resp)
'''

'''
# -----------------------------------------------------------------------------------------------------
#                                        jeeves_speech_to_text()
#
#   Description:  This is the main() function for the Jeeves speech to text publisher.
#
#     Arguments:  N/A
#
#       Returns:  N/A
#
# -----------------------------------------------------------------------------------------------------
'''
def jeeves_speech_to_text():

   # Create the "jeeves_speech_to_text" ROS node
   rospy.init_node('jeeves_speech_to_text')
   
   # Create topic to publish to
	s2t_topic = rospy.Publisher('speech_recognition/output', String, queue_size=10)   

   # Start PocketSphinx
   proc = subprocess.Popen(["pocketsphinx_continuous -lm " + cwd + "/../jeeves.lm -dict " + cwd + "/../jeeves.dic"], shell=True, stdout=subprocess.PIPE)

   if VERBOSITY:
      print "Ready to process speech requestions and/or commands..."
   
   # Always listen for speech
   while(1):
      # Speech to text 
      speech_txt = proc.stdout.readline().rstrip()
      
      # If a speech to text conversion actually occurred 
      if speech_txt[0].isdigit():
         speech_txt = speech_txt.split(': ')                                         # Break apart the CMU Sphinx response. It will always start with a 32-bit number and colon-space.
         
         # Print the speech to text string
         if VERBOSITY:
            print "CMU Sphinx Text:  %s" % speech_txt[1]
            
         aiml_string = jeeves.respond( speech_txt[1].lower() )
         
         # Print AIML match response, if any
         if len(aiml_string) == 0:
            if VERBOSITY:
               print "I'm sorry, I did not understand what you said."
         else:
            if VERBOSITY:
               s2t_topic.publish()
               print "Jeeves's Response: %s" % aiml_string
            
      # Sleep for 100ms before checking for another sentence
      time.sleep(0.1)
   
   
   
   # Prevents script from terminating until service stops
   rospy.spin()

# ---------------------------------------------------------------------
#                       Application Entry Point
# ---------------------------------------------------------------------
if __name__ == "__main__":

   jeeves_speech_to_text()                                                         # Start the jeeves_speech_srv service
   
   
   
   
   
   
   
   
   
   
   