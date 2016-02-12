#!/usr/bin/env python
# license removed for brevity

# import for ros
import rospy
from std_msgs.msg import String as Stringmsg

# import nb.py - script with naive bayes classifing functions and keyword extraction functions
import nb_keywords

from itertools import islice
import string

# make probabilities and keywords from training data
nb_keywords.make_probabilities()
nb_keywords.make_keywords()

def classify(text):
	
	#textclass and classprob are the two elements from the first tuple in the list returned by nb.get_category()
	categorylist = nb_keywords.get_category(text.data, bestofn=1)
	textclass, classprob = categorylist[0]
	soundhandle.say("I heard you say the phrase %r, and this belongs to statement class %r according to my training data." % (text.data, textclass), "voice_rab_diphone")

rospy.init_node('speech_classify/nb', anonymous=True)
# subscribes to node pocketsphinx recognizer.py, and upon message calls function callback()
rospy.Subscriber("speech_recognition/nb/output", Stringmsg, callback)
#? needs to publish msg type sound_play/SoundRequest(sound: -3, command: 1, arg: this is to say, arg2: voice_rab_diphone)
# but only if SoundClient.say() is not functioning properly
#? pub = rospy.Publisher('robotsound', SoundRequest, queue_size=10)

# instantiate SoundClient for say(text, voicename)
soundhandle = SoundClient()
	
rospy.spin()
