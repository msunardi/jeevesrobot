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
scmd_topic = '';


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

   # --------------------------------------------
   #                  Greeting
   # --------------------------------------------
   if command.find('greeting') != -1:
      command = command.replace('greeting ', '');
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
      print "ESCORT COMMAND";
      if(string.find(text.lower(), 'leader') != -1):
         t2s_topic.publish('I am not sure where Doctor Perkowski is at the moment')
      else:
         t2s_topic.publish('I will escort you');
      
      
   # --------------------------------------------
   #                    Query
   # --------------------------------------------
   elif command == 'query':
      print "QUERY COMMAND";
      
      # Tell me about Doctor Perkowski
      if(string.find(text.lower(), 'doctor perkowski') != -1):
         t2s_topic.publish('Marek Perkowski is a Professor of Computer Engineering at the ECE Dept of PSU. His PhD is in automatic control from the Department of Electronics, Warsaw University of Technology, Warsaw, Poland. He has been on faculties of Warsaw University of Technology, University of Minnesota, Minneapolis, Technical University of Eindhoven, Eindhoven, The Netherlands, University of Montpellier, Montpellier, France, and Korea Advanced Institute of Science and Technology, Daejeon, Korea. He has been involved in research on Computer Aided Design of VLSI, intelligent robotics and Machine Learning since the 1980s and recently works on automatic synthesis and optimization of quantum circuits. He has obtained several grants from NSF, Intel, Sharp, Air Force Office of Scientific Research, KAIST and others, and worked as a programmer and consultant. M. Perkowski is currently a chair of IEEE Technical Committee on Multiple-Valued Logic and he was the chair of several conferences on this and other topics. He is an author of more than 300 papers and several books. Several of his programs are used in CAD industry and his research is widely cited. He is on the NEC list of most cited computer science authors. His dream is to build a complete interactive theater of humanoid robots controlled by quantum immunological software.');

      # Tell me about Doctor Hall
      elif(string.find(text.lower(), 'doctor hall') != -1):
         t2s_topic.publish('Doctor Hall is an associate professor in the Maseeh College of Engineering at Portland State University. He specializes in computer architecture, and he obtained his PhD from Portland State University.');

      # Tell me about Doctor Song
      elif(string.find(text.lower(), 'doctor song') != -1):
         t2s_topic.publish('Doctor Song is a professor of electrical and computer engineering in the Maseeh College of Engineering at Portland State University. He specializes in design automation and formal methods. Doctor Song obtained his PhD from the University of Pisa in Italy.');

      # Tell me about Rick Armstrong
      elif(string.find(text.lower(), 'rick armstrong') != -1):
         t2s_topic.publish('Rick Armstrong obtained his Bachelor of Science in Mathematics at Portland State University. He is interested in the intersection between Computer Vision, Mathematics, and Robotics. He has contributed greatly to the development of me.');

      # Tell me about Mathias Sunardi
      elif(string.find(text.lower(), 'mathias sunardi') != -1):
         t2s_topic.publish('Mathias Sunardi is a PhD student in the Maseeh College of Engineering at Portland State University.');

      # Tell me about Josh Sackos
      elif(string.find(text.lower(), 'josh sackos') != -1):
         t2s_topic.publish('Josh Sackos is an electrical and computer engieering embedded systems student at Portland State University. He obtained a Bachelor of Science in Computer Engineering from Washington State University. He is interested in automation, the internet of things, high performance systems, data collection, and developing embedded systems for the space environment. Josh worked on some of my computer vision and speech recognition capabilities.');

      # Tell me about the Robotics Lab
      elif(string.find(text.lower(), 'robotics lab') != -1):
         t2s_topic.publish('In the Intelligent Robotics Laboratory we design and program mobile, stationary and humanoid robots on levels of mechanical, electrical and software design. Many of our robots have new types of controllers. Theoretical research is dedicated to applying machine learning and data analysis algorithms to solve practical problems in electrical and computer engineering, especially in Data Mining, robot vision, robot motion, robot theatre  and human-robot interface (such as emotion recognition). The laboratory is also involved in the research on quantum and reversible computing as well as nano-technologies such as quantum dots and memristors. In a related research we develop new quantum algorithms, for instance those used in robotics, thus defining a new research area of Quantum Robotics. Of laboratory interests are also highly parallel robotics algorithms on GPU platform and emulation of problem-solving architectures with FPGAs and VELOCE emulator from Mentor.');

      # Tell me about the Intel Lab
      elif(string.find(text.lower(), 'intel lab') != -1):
         t2s_topic.publish('The FAB Intel Lab is a general computer lab that is open to all engineering students. Within the facility are Windows-based computers, Linux-based workstations, and several B/W and color laser printers and scanners. Wired and wireless access is also available for laptop computers. A variety of Windows software packages are offered, including Microsoft Office, Cadence design tools, QuestaSim, SynaptiCAD, Agilent ADS, LT Spice, LabView, MATLAB, Maple, Mathcad, Microsoft Visual Studio, Python, AutoCAD, SolidWorks, Arduino, LabJack, and Cygwin.');

      # Tell me about the Portland State University
      elif(string.find(text.lower(), 'psu') != -1 or string.find(text.lower(), 'portland state university') != -1):
         t2s_topic.publish('Portland States 49-acre downtown campus is located in the heart of one of Americas most vibrant centers of culture, business and technology. We are recognized throughout the world for programs like Urban Planning, Social Work, and Environmental Studies that directly engage the community, and aim our students towards the creation of a better, more sustainable world.');
         
      else:
         t2s_topic.publish('I am sorry, I do not have that person or place in my database');
         
   # --------------------------------------------
   #                    Dance
   # --------------------------------------------
   elif command == 'dance':
      print "DANCE COMMAND";
      t2s_topic.publish('DANCE COMMAND');
      
   # --------------------------------------------
   #                    Tour
   # --------------------------------------------
   elif command == 'tour':
      print "TOUR COMMAND";
      t2s_topic.publish('TOUR COMMAND');
      
   # --------------------------------------------
   #                  Location
   # --------------------------------------------
   elif command == 'location':
      print "LOCATION COMMAND";
      t2s_topic.publish('LOCATION COMMAND');
      
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
   
   global t2s_topic, scmd_topic;
   
   # Create the "jeeves_speech_to_text" ROS node
   rospy.init_node('jeeves_proc_cmd')
   
   # Publish to "jeeves_speech_synthesis.py" node
   t2s_topic         = rospy.Publisher('jeeves_speech/speech_synthesis'   , String, queue_size=10)

   # ---------------------------------------
   #      Create speech_command Topic
   # ---------------------------------------
   scmd_topic = rospy.Publisher('jeeves_speech/speech_command', String, queue_size=10)
   
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
   
   
   
   
   
   
   
   
   
   
   
