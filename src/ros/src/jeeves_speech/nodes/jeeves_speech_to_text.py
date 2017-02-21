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


# -----------------------------------------------------------------------------------------------------
#                                           Global Variables
# -----------------------------------------------------------------------------------------------------

jeeves_handshake         = False;                           # Jeeves handshake. Indicates when a speech to text publication has been processed.
JEEVES_KEYWORDS          = ["jeeves listen", "hi jeeves", "hello jeeves", "bye jeeves", "jeeves"];  # Keywords that cause Jeeves to begin paying attention to words spoken. Should be ALL lowercase.

# Jeeves Speech State Machine
JEEVES_IDLE_STATE        = 0;                # IDLE state
JEEVES_ACKNOWLEDGE_STATE = 1;                # ACKNOWLEDGE state
JEEVES_LISTENING_STATE   = 2;                # LISTENING state
JEEVES_HANDSHAKE_STATE   = 3;                # HANDSHAKE state

# Initial state
state                    = JEEVES_IDLE_STATE;



'''
# -----------------------------------------------------------------------------------------------------
#                                          proc_handshake()
#
#   Description:  This function gets called whenever a message is published to the 
#                 "jeeves_speech/jeeves_handshake" topic.
#
#     Arguments:  N/A
#
#       Returns:  N/A
#
# -----------------------------------------------------------------------------------------------------
'''
def proc_handshake(message):

   global jeeves_handshake
   
   # Set jeeves_handshake to True. This returns the state
   # machine back to the IDLE state.
   jeeves_handshake = True;


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

   global jeeves, jeeves_handshake, state;
   aiml_string = '';
   speech_txt = '';
   ack_state_pub = False;
   
   # ----------------------------------------------
   #  Create the "jeeves_speech_to_text" ROS node
   # ----------------------------------------------
   rospy.init_node('jeeves_speech_to_text')
   
   # ---------------------------------------
   #      Create speech_to_text Topic
   # ---------------------------------------
   s2t_topic = rospy.Publisher('jeeves_speech/speech_to_text', String, queue_size=10)
   
   # ---------------------------------------
   #   Subscribe to speech_handshake Topic
   # ---------------------------------------
   rospy.Subscriber("jeeves_speech/speech_handshake", String, proc_handshake)

   # ---------------------------------------
   #          Start PocketSphinx
   # ---------------------------------------
#   proc = subprocess.Popen(["pocketsphinx_continuous -dict " + cwd + "jeeves_small.dic"], shell=True, stdout=subprocess.PIPE)
#   proc = subprocess.Popen(["pocketsphinx_continuous -lm " + cwd + "jeeves_small.lm -dict " + cwd + "jeeves_small.dic"], shell=True, stdout=subprocess.PIPE)

#   proc = subprocess.Popen(["pocketsphinx_continuous -dict " + cwd + "jeeves.dic"], shell=True, stdout=subprocess.PIPE)
   proc = subprocess.Popen(["pocketsphinx_continuous -lm " + cwd + "jeeves.lm -dict " + cwd + "jeeves.dic"], shell=True, stdout=subprocess.PIPE)

   if VERBOSITY:
      print "Ready to process speech requests..."

   # ---------------------------------------
   #       Always listen for speech
   # ---------------------------------------
   while(1):

      # Try and read text from PocketSphinx
      speech_txt = proc.stdout.readline().rstrip()
      
      # Sometimes responses from PocketSphinx can be weird, just in case something goes awry use "try"
      try:
         # If a string actually exists
         if len(speech_txt) > 0:
            # If a speech to text conversion actually occurred
            if speech_txt[0].isdigit():
               speech_txt = speech_txt.split(': ')                                         # Break apart the CMU Sphinx response. It will always start with a 32-bit number and colon-space.

               if len(speech_txt) > 1:

                  speech_txt = speech_txt[1]
                  if VERBOSITY:
                     print "CMU Sphinx Text:  %s" % speech_txt
                     
               else:
                  speech_txt = '';
            else:
               speech_txt = '';
            
      except Exception as e:
         print "========================================"
         print "THERE WAS AN EXCEPTION"
         print speech_txt
         print type(speech_txt)
         print e
         speech_txt = '';

         
      # *************************************************************************
      #                           Update State Machine
      # *************************************************************************
      
      # -----------------------------------------------------
      #                  JEEVES IDLE STATE
      # -----------------------------------------------------
      if(state == JEEVES_IDLE_STATE):
         # If the string contains "jeeves listen", advance to next state
         if((speech_txt.lower() == JEEVES_KEYWORDS[0].lower()) or (speech_txt.lower() == JEEVES_KEYWORDS[4].lower())):
            state = JEEVES_ACKNOWLEDGE_STATE;
            jeeves_handshake = False;
            ack_state_pub = False;

            if VERBOSITY:
               print "Idle State Text:  %s" % speech_txt;

         # If the user greets Jeeves
         if(speech_txt.lower() == JEEVES_KEYWORDS[1].lower() or speech_txt.lower() == JEEVES_KEYWORDS[2].lower() or speech_txt.lower() == JEEVES_KEYWORDS[3].lower()):
            state = JEEVES_HANDSHAKE_STATE;
            jeeves_handshake = False;
            ack_state_pub = False;

            # Publish the text string so that the jeeves_text_category.py node can pick it up
            s2t_topic.publish(speech_txt);

            if VERBOSITY:
               print "Greetings Text:  %s" % speech_txt;
         
         if VERBOSITY:
            print "JEEVES_IDLE_STATE"

      # -----------------------------------------------------
      #               JEEVES ACKNOWLEDGE STATE
      # -----------------------------------------------------
      elif(state == JEEVES_ACKNOWLEDGE_STATE):
         
         if(ack_state_pub == False):
            s2t_topic.publish('JEEVES ACKNOWLEDGE');
            ack_state_pub = True;
         
         if VERBOSITY:
            print "JEEVES_ACKNOWLEDGE_STATE"
            
         if(jeeves_handshake == True):
            time.sleep(1);
            state = JEEVES_LISTENING_STATE;
            jeeves_handshake = False;

      # -----------------------------------------------------
      #                JEEVES LISTENING STATE
      # -----------------------------------------------------
      elif(state == JEEVES_LISTENING_STATE):

         ack_state_pub = False;

         if(len(speech_txt) > 0):
            # Publish the text string so that the jeeves_text_category.py node can pick it up
            s2t_topic.publish(speech_txt);
            
            if VERBOSITY:
               print "Listening State Text:  %s" % speech_txt
            
            state = JEEVES_HANDSHAKE_STATE;
            jeeves_handshake = False;
         
         if VERBOSITY:
            print "JEEVES_LISTENING_STATE"

      # -----------------------------------------------------
      #                JEEVES HANDSHAKE STATE
      # -----------------------------------------------------
      elif(state == JEEVES_HANDSHAKE_STATE):
         
         ack_state_pub = False;
         if VERBOSITY:
            print "JEEVES_HANDSHAKE_STATE"
         # Wait for a response from the speech synthesis node
         if(jeeves_handshake == True):
            state = JEEVES_IDLE_STATE;
            jeeves_handshake = False;
            



      # -----------------------------------------------------
      #                   UNDEFINED STATE
      # -----------------------------------------------------
      else:
         # Just return to the idle state
         state = JEEVES_IDLE_STATE;
         jeeves_handshake = False;
         ack_state_pub = False;

         if VERBOSITY:
            print "JEEVES_UNDEFINED_STATE"
         
         
      # Sleep for 100ms before checking for another sentence
      time.sleep(0.1)
   
   # Prevents script from terminating until service stops
   rospy.spin()

# ---------------------------------------------------------------------
#                       Application Entry Point
# ---------------------------------------------------------------------
if __name__ == "__main__":

   jeeves_speech_to_text_f()                                                         # Start the jeeves_speech_srv service
   
   
   
   
   
   
   
   
   
   
   
