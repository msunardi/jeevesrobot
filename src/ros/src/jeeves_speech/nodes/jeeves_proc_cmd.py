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
import sys, os, time, json, string, time
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

# **** Hardcoded values to be used when developing/using rosrun ****
DEV_ENV = bool(1)
VERBOSITY = bool(0)

if VERBOSITY:
   print "Development Environment:  %s " % str(DEV_ENV)

# Create empty ROS topic variables for global variable scope in Python file.
t2s             = '';
t2s_topic       = '';
scmd_topic      = '';
speech_aiml_req = '';


# Global variables
aiml_resp = '';
aiml_resp_flag = False;

'''
# -----------------------------------------------------------------------------------------------------
#                                          proc_aiml_resp()
#
#   Description:  This function gets called whenever a message is published to the 
#                 "jeeves_speech/speech_aiml_resp" topic.
#
#     Arguments:  N/A
#
#       Returns:  N/A
#
# -----------------------------------------------------------------------------------------------------
'''
def proc_aiml_resp(message):

   global jeeves, aiml_resp, aiml_resp_flag;
   aiml_resp_flag = True;
   aiml_resp = message.data;
   

'''
# -----------------------------------------------------------------------------------------------------
#                                            extract_noun()
#
#   Description:  
#
#     Arguments:  cmd  : The command that was detected.
#                 sent : The sentence to extract a noun from.
#
#       Returns:  A string containing the assumed noun.
#
# -----------------------------------------------------------------------------------------------------
'''
def extract_noun(cmd, sent):
   
   # Take the last two words from the sentence, this is the noun. "Intel lab", "Doctor Perkowski", etc.
   sent = sent.split(' ');
   if VERBOSITY:
      print sent;
   
   # If Portland State University
   if(sent[len(sent)-1].lower() == 'university'):
      return "%s %s %s" % (sent[len(sent)-3], sent[len(sent)-2], sent[len(sent)-1]);
   # If regular noun
   else:
      return "%s %s" % (sent[len(sent)-2], sent[len(sent)-1]);

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
   
   global t2s_topic, t2s, aiml_resp, aiml_resp_flag;
   
   if len(message.data) == 0:
      if VERBOSITY:
         print "Invalid command received..."
   
   msg_dict = json.loads(message.data);
   command  = msg_dict['command'];
   text     = msg_dict['text'];
   
   if VERBOSITY:
      print "Received Command:  %s" % command;
      print "Received Text:  %s" % text;

   # --------------------------------------------
   #                  Greeting
   # --------------------------------------------
   if command.find('greeting') != -1:
      command = command.replace('greeting ', '');
      
      if VERBOSITY:
         print "GREETING COMMAND";
      t2s_topic.publish(command);

      if(command.find('bye') != -1):
         json_str = json.dumps({'command':'greeting', 'args':[0, text]});
      else:
         json_str = json.dumps({'command':'greeting', 'args':[1, text]});

      scmd_topic.publish(json_str);
      print json_str


   # --------------------------------------------
   #                   Escort
   # --------------------------------------------
   elif command == 'escort':
      # EXTRACT NOUN
      noun = extract_noun(command, text);
      
      if VERBOSITY:
         print "ESCORT COMMAND, noun = %s" % noun;
         
      if(string.find(text.lower(), 'leader') != -1):
         t2s_topic.publish('I am not sure where Doctor Perkowski is at the moment')
      else:
         if(string.find(text.lower(), 'lab') != -1):
            t2s_topic.publish('I will escort you to the %s' % noun);
         else:
            t2s_topic.publish('I will escort you to %s' % noun);

         json_str = json.dumps({'command':'escort', 'args':[noun, text]});
         scmd_topic.publish(json_str);
      
   # --------------------------------------------
   #                    Query
   # --------------------------------------------
   elif command == 'query':
      # EXTRACT NOUN
      noun = extract_noun(command, text);
      
      if VERBOSITY:
         print "QUERY COMMAND, noun = %s" % noun;
      
      aiml_resp = '';
      speech_aiml_req.publish(noun.upper());
      
      # Wait for a response from the AIML node
      while(aiml_resp_flag == False):
         None
      aiml_resp_flag = False;    # Clear the flag
         
      if VERBOSITY:
         print "AIML response = %s" % aiml_resp;
      
      # If a match was found, send it to the text to speech node.
      if(len(aiml_resp) > 0):
         t2s_topic.publish(aiml_resp);
      # If no match was found
      else:
         t2s_topic.publish('I am sorry, I do not have that person, place, or thing in my database');
         
   # --------------------------------------------
   #                    Dance
   # --------------------------------------------
   elif command == 'dance':
      if VERBOSITY:
         print "DANCE COMMAND";
#      t2s_topic.publish('DANCE COMMAND');
      t2s_topic.publish('stayin alive, stayin alive, ah, ah, ah, ah stayin alive!');
      json_str = json.dumps({'command':'dance', 'args':[text]});
      scmd_topic.publish(json_str);
      
   # --------------------------------------------
   #                    Tour
   # --------------------------------------------
   elif command == 'tour':
      if VERBOSITY:
         print "TOUR COMMAND";
      t2s_topic.publish('TOUR COMMAND');
      json_str = json.dumps({'command':'tour', 'args':[text]});
      scmd_topic.publish(json_str);
      
   # --------------------------------------------
   #                    Story
   # --------------------------------------------
   elif command == 'story':
      if VERBOSITY:
         print "STORY COMMAND";
      
      aiml_resp = '';
      speech_aiml_req.publish('STORIES');
      
      # Wait for a response from the AIML node
      while(aiml_resp_flag == False):
         None
      aiml_resp_flag = False;    # Clear the flag

      if VERBOSITY:
         print "AIML response = %s" % aiml_resp;
      
      # If a match was found, send it to the text to speech node.
      if(len(aiml_resp) > 0):
         t2s_topic.publish(aiml_resp);
      # If no match was found
      else:
         t2s_topic.publish('I am sorry, I do not have any stories currently');

   # --------------------------------------------
   #                 Date/Time
   # --------------------------------------------
   elif command == 'time':
      
      if VERBOSITY:
         print "TIME COMMAND"
      
      # Default to am
      ampm = 'am';
      tm_hour = 0;
      
      # Get the date/time structure
      the_time = time.localtime(time.time());
      
      # Calculate non-military time
      if(the_time.tm_hour >= 12):
         ampm = 'pm';
         
         if(the_time.tm_hour >= 13):
            tm_hour = the_time.tm_hour - 12;
         else:
            tm_hour = the_time.tm_hour;
            
      else:
         tm_hour = the_time.tm_hour;
      
      tm_min = the_time.tm_min;

      # Determine month
      tm_mon = '';
      if(the_time.tm_mon == 1):
         tm_mon = 'January';
      elif(the_time.tm_mon == 2):
         tm_mon = 'February';
      elif(the_time.tm_mon == 3):
         tm_mon = 'March';
      elif(the_time.tm_mon == 4):
         tm_mon = 'April';
      elif(the_time.tm_mon == 5):
         tm_mon = 'May';
      elif(the_time.tm_mon == 6):
         tm_mon = 'June';
      elif(the_time.tm_mon == 7):
         tm_mon = 'July';
      elif(the_time.tm_mon == 8):
         tm_mon = 'August';
      elif(the_time.tm_mon == 9):
         tm_mon = 'September';
      elif(the_time.tm_mon == 10):
         tm_mon = 'October';
      elif(the_time.tm_mon == 11):
         tm_mon = 'November';
      elif(the_time.tm_mon == 12):
         tm_mon = 'December';
         
      # Determine Day suffix
      tm_mday_suffix = '';
      if(the_time.tm_mday == 1):
         tm_mday_suffix = 'st';
      elif(the_time.tm_mday == 2):
         tm_mday_suffix = 'nd';
      elif(the_time.tm_mday == 3):
         tm_mday_suffix = 'rd';
      elif(the_time.tm_mday >= 4 and the_time.tm_mday <= 20):
         tm_mday_suffix = 'th';
      elif(the_time.tm_mday == 21):
         tm_mday_suffix = 'st';
      elif(the_time.tm_mday == 22):
         tm_mday_suffix = 'nd';
      elif(the_time.tm_mday == 23):
         tm_mday_suffix = 'rd';
      elif(the_time.tm_mday >= 24 and the_time.tm_mday <= 30):
         tm_mday_suffix = 'th';
      elif(the_time.tm_mday == 31):
         tm_mday_suffix = 'st';
         
      # Determine Day
      tm_wday = '';
      if(the_time.tm_wday == 0):
         tm_wday = 'Monday';
      elif(the_time.tm_wday == 1):
         tm_wday = 'Tuesday';
      elif(the_time.tm_wday == 2):
         tm_wday = 'Wednesday';
      elif(the_time.tm_wday == 3):
         tm_wday = 'Thursday';
      elif(the_time.tm_wday == 4):
         tm_wday = 'Friday';
      elif(the_time.tm_wday == 5):
         tm_wday = 'Saturday';
      elif(the_time.tm_wday == 6):
         tm_wday = 'Sunday';
         
      
      if(tm_min < 10):
         t2s_topic.publish('It is currently %d 0 %d %s on %s %s %d %s %s' % (tm_hour, the_time.tm_min, ampm, tm_wday, tm_mon, the_time.tm_mday, tm_mday_suffix, the_time.tm_year));
      else:
         t2s_topic.publish('It is currently %d %d %s on %s %s %d %s %s' % (tm_hour, the_time.tm_min, ampm, tm_wday, tm_mon, the_time.tm_mday, tm_mday_suffix, the_time.tm_year));
      
   # --------------------------------------------
   #                    FACT
   # --------------------------------------------
   elif command == 'fact':
      if VERBOSITY:
         print "FACT COMMAND";
      
      aiml_resp = '';
      speech_aiml_req.publish('FACTS');
      
      # Wait for a response from the AIML node
      while(aiml_resp_flag == False):
         None
      aiml_resp_flag = False;    # Clear the flag

      if VERBOSITY:
         print "AIML response = %s" % aiml_resp;
      
      # If a match was found, send it to the text to speech node.
      if(len(aiml_resp) > 0):
         t2s_topic.publish(aiml_resp);
      # If no match was found
      else:
         t2s_topic.publish('I am sorry, I do not have any fun facts currently');

         
   # --------------------------------------------
   #                    JOKE
   # --------------------------------------------
   elif command == 'joke':
      if VERBOSITY:
         print "JOKE COMMAND";
      
      aiml_resp = '';
      speech_aiml_req.publish('JOKES');
      
      # Wait for a response from the AIML node
      while(aiml_resp_flag == False):
         None
      aiml_resp_flag = False;    # Clear the flag

      if VERBOSITY:
         print "AIML response = %s" % aiml_resp;
      
      # If a match was found, send it to the text to speech node.
      if(len(aiml_resp) > 0):
         t2s_topic.publish(aiml_resp);
      # If no match was found
      else:
         t2s_topic.publish('I am sorry, I do not have any jokes currently');
         
   # --------------------------------------------
   #                  Unknown
   # --------------------------------------------
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
   
   global t2s_topic, scmd_topic, speech_aiml_req;
   
   # Create the "jeeves_speech_to_text" ROS node
   rospy.init_node('jeeves_proc_cmd')

   # ---------------------------------------
   # Create speech synthesis topic
   # ---------------------------------------
   t2s_topic         = rospy.Publisher('jeeves_speech/speech_synthesis'   , String, queue_size=10)

   # ---------------------------------------
   #      Create speech_command Topic
   # ---------------------------------------
   scmd_topic = rospy.Publisher('jeeves_speech/speech_command', String, queue_size=10)
   
   # ---------------------------------------
   #      Create speech_aiml_req Topic
   # ---------------------------------------
   speech_aiml_req = rospy.Publisher('jeeves_speech/speech_aiml_req', String, queue_size=10)
   
   # ---------------------------------------
   # Subscribe to speech_to_text topic
   # ---------------------------------------
   rospy.Subscriber("jeeves_speech/speech_proc_cmd", String, proc_cmd)
   
   # ---------------------------------------
   # Subscribe to speech_aiml_resps topic
   # ---------------------------------------
   rospy.Subscriber("jeeves_speech/speech_aiml_resp", String, proc_aiml_resp)
   
   if VERBOSITY:
      print "Listening for command..."
   
   # Prevents script from terminating until service stops
   rospy.spin()

# ---------------------------------------------------------------------
#                       Application Entry Point
# ---------------------------------------------------------------------
if __name__ == "__main__":

   jeeves_proc_cmd_f()                                                         # Start the jeeves_speech_srv service
   
   
   
   
   
   
   
   
   
   
   
