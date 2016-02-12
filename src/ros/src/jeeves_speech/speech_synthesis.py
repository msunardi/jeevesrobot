#!/usr/bin/env python
# license removed for brevity

# import for ros
import rospy
from std_msgs.msg import String
# import for calling festival from command line
import subprocess

def synthesize(text):
	subprocess.call(["festival", "--batch", "(voice_rab_diphone)", "(SayText \"" + text.data + "\")"])


def speech_synthesis():
	rospy.init_node('speech_synthesis')
	# subscribes to node "speech_synthesis/input", and upon message calls function callback()
	rospy.Subscriber("speech_synthesis/input", String, synthesize)
	rospy.spin()

if __name__ == '__main__':
	speech_synthesis()
