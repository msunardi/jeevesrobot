#!/usr/bin/env python
# license removed for brevity

# import for ros
import rospy
from std_msgs.msg import String
# import for calling festival from command line
import subprocess
# import to use current working directory for loading lm and dict files
import os

def speech_recognition():
	rospy.init_node('speech_recognition_nb')
	pub = rospy.Publisher('speech_recognition/nb/output', String, queue_size=10)
	cwd = os.path.dirname( os.path.realpath(__file__) )
	proc = subprocess.Popen(["pocketsphinx_continuous -lm " + cwd + "/jeeves.lm -dict " + cwd + "/jeeves.dic"], shell=True, stdout=subprocess.PIPE)
	while not rospy.is_shutdown():
		output = proc.stdout.readline().rstrip()
		if output[0].isdigit(): # if the first character of the line read from stdout of pocketsphinx_continuous is a digit
			pub.publish(output[11:]) # then slice the string and publish only the meaningful text (all characters after the 11th character)
	rospy.spin()
	remainder = proc.communicate()[0]

if __name__ == '__main__':
	try:
		speech_recognition()
	except rospy.ROSInterruptException:
		pass
