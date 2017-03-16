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
import aiml                               # Artificial Intelligence Markup Language
from std_msgs.msg import String

# Set current working directory
cwd = os.chdir(os.path.dirname(os.path.realpath(__file__)) + '/../');
cwd = os.getcwd() + '/';

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

# initialize aiml speech system, and load aiml xml file
jeeves = aiml.Kernel()
jeeves.learn(cwd + "jeeves.xml")
jeeves.respond('load aiml b') #finished initializing


# Create empty ROS topic variables for global variable scope in Python file.
proc_cmd_topic    = '';
wolfram_cmd_topic = '';
wiki_cmd_topic    = '';
t2s_topic         = '';
speech_aiml_resp  = '';

'''
# -----------------------------------------------------------------------------------------------------
#                                          proc_aiml_req()
#
#   Description:  This function gets called whenever a message is published to the 
#                 "jeeves_speech/speech_aiml_req" topic.  This function essentially provides a backdoor
#                 to the AIML contents from other nodes.
#
#     Arguments:  N/A
#
#       Returns:  N/A
#
# -----------------------------------------------------------------------------------------------------
'''
def proc_aiml_req(message):

   global jeeves, speech_aiml_resp;
   
   text = message.data;

   # Check the Jeeves AIML for a pattern match
   aiml_string      = jeeves.respond( text )
   
   # Print the speech to text string
   if VERBOSITY:
      print " AIML Req. Text:  %s" % text
      print "AIML Resp. Text: = %s" % aiml_string
      
   # Publish the AIML data obtained
   speech_aiml_resp.publish(aiml_string);

'''
# -----------------------------------------------------------------------------------------------------
#                                           proc_text()
#
#   Description:  This function gets called whenever a message is recieved from the speech_to_text
#                 topic. It is responsible for classifying the received text string, and publishing
#                 the string to a topic for processing the command, etc.
#
#     Arguments:  N/A
#
#       Returns:  N/A
#
# -----------------------------------------------------------------------------------------------------
'''
def proc_text(message):

   global jeeves, proc_cmd_topic, wolfram_cmd_topic, wiki_cmd_topic
   
   speech_txt = message.data;

   # Check the Jeeves AIML for a pattern match
   aiml_string      = jeeves.respond( speech_txt )
   
   # Print the speech to text string
   if VERBOSITY:
      print "Received Text:  %s" % speech_txt
      print "aiml_string = %s" % aiml_string

   # -----------------------------------------------------
   #                  AIML Command
   # -----------------------------------------------------
   if( aiml_string.lower().find('command') != -1 ):
      if VERBOSITY:
         print "Jeeves: %s" % speech_txt
      json_aiml_string = json.dumps({'command':aiml_string.lower().replace(' command', ''), 'text':speech_txt});
      proc_cmd_topic.publish(json_aiml_string);
      
   # -----------------------------------------------------
   #                Calculate Command
   # -----------------------------------------------------
   elif( aiml_string.lower().find('wolfram') != -1 ):
      if VERBOSITY:
         print "Wolfram: %s" % speech_txt
      proc_cmd_topic.publish(speech_txt)
      
   # -----------------------------------------------------
   #                 Wikipedia Command
   # -----------------------------------------------------
   elif( aiml_string.lower().find('wikipedia') != -1 ):
      if VERBOSITY:
         print "Wikipedia: %s" % speech_txt
      proc_cmd_topic.publish(speech_text)
      
   # -----------------------------------------------------
   #                    Acknowledge
   # -----------------------------------------------------
   elif( aiml_string.lower().find('jeeves acknowledge') != -1 ):
      t2s_topic.publish('How can I help you');
      if VERBOSITY:
         print "How can I help you?"
         
   # -----------------------------------------------------
   #                    Jeeves Listen
   # -----------------------------------------------------
   elif( aiml_string.lower().find('i am') != -1 ):
      t2s_topic.publish(aiml_string);
      if VERBOSITY:
         print "I am"
      
   # -----------------------------------------------------
   #                      No Match
   # -----------------------------------------------------
   else:
      t2s_topic.publish('I\'m sorry, I did not understand what you said.');
      if VERBOSITY:
         print "I'm sorry, I did not understand what you said."
   
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
def jeeves_text_category_f():
   
   global proc_cmd_topic, wolfram_cmd_topic, wiki_cmd_topic, t2s_topic, speech_aiml_resp;
   
   # Create the "jeeves_speech_to_text" ROS node
   rospy.init_node('jeeves_text_category')

   # ---------------------------------------
   # Create topics to publish to
   # ---------------------------------------
   proc_cmd_topic    = rospy.Publisher('jeeves_speech/speech_proc_cmd'   , String, queue_size=10)   
   wolfram_cmd_topic = rospy.Publisher('jeeves_speech/speech_wolfram_cmd', String, queue_size=10)   
   wiki_cmd_topic    = rospy.Publisher('jeeves_speech/speech_wiki_cmd'   , String, queue_size=10)
   t2s_topic         = rospy.Publisher('jeeves_speech/speech_synthesis'   , String, queue_size=10)
   speech_aiml_resp  = rospy.Publisher('jeeves_speech/speech_aiml_resp'   , String, queue_size=10)

   # ---------------------------------------
   # Subscribe to speech_to_text topic
   # ---------------------------------------
   rospy.Subscriber("jeeves_speech/speech_to_text", String, proc_text)
   
   # ---------------------------------------
   #   Subscribe to speech_aiml_req topic
   # ---------------------------------------
   rospy.Subscriber("jeeves_speech/speech_aiml_req", String, proc_aiml_req)
   
   if VERBOSITY:
      print "Listening for text..."
   
   # Prevents script from terminating until service stops
   rospy.spin()

# ---------------------------------------------------------------------
#                       Application Entry Point
# ---------------------------------------------------------------------
if __name__ == "__main__":

   jeeves_text_category_f()                                                         # Start the jeeves_speech_srv service
   
   
   
   
   
   
   
   
   
   
   
