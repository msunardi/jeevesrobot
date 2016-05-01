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

# initialize aiml speech system, and load aiml xml file
jeeves = aiml.Kernel()
jeeves.learn(cwd + "jeeves.xml")
jeeves.respond('load aiml b') #finished initializing


t2s    = '';
t2s_topic = '';


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
def proc_cmd(message):
   
   global t2s_topic, t2s;
   
   if len(message.data) == 0:
      print "Invalid command received..."
   
   msg_dict = json.loads(message.data);
   command  = msg_dict['command'];
   text     = msg_dict['text'];
   
   if VERBOSITY:
      print "Received Command:  %s" % command;
      print "Received Text:  %s" % text;
   
   # Escort
   if command == 'escort':
      print "ESCORT COMMAND";
      t2s_topic.publish('I will escort you');
   # Query
   elif command == 'query':
      print "QUERY COMMAND";
      t2s_topic.publish('One moment while I search my database');
   # Dance
   elif command == 'dance':
      print "DANCE COMMAND";
      t2s_topic.publish('DANCE COMMAND');
   # Tour
   elif command == 'tour':
      print "TOUR COMMAND";
      t2s_topic.publish('TOUR COMMAND');
   # Tour
   elif command == 'location':
      print "LOCATION COMMAND";
      t2s_topic.publish('LOCATION COMMAND');
   # Unknown
   else:
      t2s_topic.publish('I am sorry, I did not understand your request');
      
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
def jeeves_proc_cmd_f():
   
   global t2s_topic;
   
   # Create the "jeeves_speech_to_text" ROS node
   rospy.init_node('jeeves_proc_cmd')
   
   # Publish to "jeeves_speech_synthesis.py" node
   t2s_topic         = rospy.Publisher('jeeves_speech/speech_synthesis'   , String, queue_size=10)
   
   # Subscribe to speech_to_text topic
   rospy.Subscriber("jeeves_speech/speech_proc_cmd", String, proc_cmd)
   
   if VERBOSITY:
      print "Listening for command..."
   
   # Prevents script from terminating until service stops
   rospy.spin()

# ---------------------------------------------------------------------
#                       Application Entry Point
# ---------------------------------------------------------------------
if __name__ == "__main__":

   jeeves_proc_cmd_f()                                                         # Start the jeeves_speech_srv service
   
   
   
   
   
   
   
   
   
   
   