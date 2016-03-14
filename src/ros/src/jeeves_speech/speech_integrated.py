#!/usr/bin/env python
# license removed for brevity

# import for ros
import rospy
from std_msgs.msg import String
# import to use current working directory files
import os
# import for aiml (ALICE AI chat system)
import aiml
# import for classifier and keyword extraction
import nb_keywords

# start publisher and publish to speech_synthesis/input
pub = rospy.Publisher('speech_synthesis/input', String, queue_size=10)

# initialize aiml speech system, and load aiml xml files
k = aiml.Kernel()
cwd = os.path.dirname( os.path.realpath(__file__) )
os.chdir(cwd)
k.learn(cwd + "/jeevespersonality-startup.xml")
k.respond('load aiml b') #finished initializing

# initialize nb_keywords and load training data
nb_keywords.make_probabilities()
nb_keywords.make_keywords()

# initialize strings for latch
aiml_string = ""
nb_string = ""


def aiml_query(text):
	global aiml_string
	print "aiml_input: " + text.data
	aiml_string = k.respond( text.data )
	if len(aiml_string) == 0:
		print "aiml_response string empty - no match"
		aiml_string = "0"
	else:
		print "aiml_response: " + aiml_string
	
def nb_query(text):
	global nb_string
	text_no_keywords, keyword_dict = nb_keywords.get_keywords(text.data)
	#textclass and classprob are assigned the two elements from the first tuple in the list returned by nb.get_category()
	maxlikelylist = nb_keywords.get_category(text_no_keywords, bestofn=1)
	textclass, classprob = maxlikelylist[0]
	nb_string = "Phrase %r belongs to statement class %r according to my training data, and includes keywords %r" % (text.data, textclass, str(keyword_dict))
	print "nb_response: " + nb_string

def speech_integrated():
	global aiml_string
	global nb_string
	rospy.init_node('speech_aiml')
	rospy.Subscriber("speech_recognition/output", String, aiml_query)
	rospy.Subscriber("speech_recognition/nb/output", String, nb_query)
	while not rospy.is_shutdown():
		if (len(aiml_string) > 0) and (len(nb_string) > 0):
			if aiml_string == "0":
				pub.publish(nb_keywords.sanitize(nb_string))
				print "published nb_string\n"
			else:
				pub.publish(aiml_string)
				print "published aiml_string\n"
			aiml_string = ""
			nb_string = ""
	rospy.spin()

if __name__ == '__main__':
	try:
		speech_integrated()
	except rospy.ROSInterruptException:
		pass



