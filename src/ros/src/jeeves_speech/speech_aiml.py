#!/usr/bin/env python
# license removed for brevity

# import for ros
import rospy
from std_msgs.msg import String
# import for aiml (ALICE AI chat system)
import aiml
# import to use current working directory files
import os

pub = rospy.Publisher('speech_synthesis/input', String, queue_size=10)
#initialize aiml speech system, and load aiml xml files
k = aiml.Kernel()
cwd = os.path.dirname( os.path.realpath(__file__) )
os.chdir(cwd)
k.learn(cwd + "/std-startup.xml")
k.respond('load aiml b') #finished initializing

def aiml_query(text):
	
	print "input: " + text.data
	response = k.respond( text.data )
	print "response: " + response + "\n"
	pub.publish(response)

def speech_aiml():
	rospy.init_node('speech_aiml')
	rospy.Subscriber("speech_recognition/output", String, aiml_query)
	rospy.spin()

if __name__ == '__main__':
	speech_aiml()
