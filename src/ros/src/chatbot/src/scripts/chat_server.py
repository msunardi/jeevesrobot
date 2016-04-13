#!/usr/bin/env python

import aiml
import wikipedia
import pyttsx
import random
from chatbot.srv import *
import rospy
import rospkg

rospack = rospkg.RosPack()
engine = pyttsx.init('espeak')
parser = aiml.Kernel()
parser.learn("%s/aiml1.6/wiki.aiml" % rospack.get_path('chatbot'))

def handle_chat_request(req):
    print "Requested entry: %s" % (req.pattern)
    response = isWiki(parser.respond(req.pattern))
    if response:
        if 'ambiguous: ' in response:
            r = response.replace('ambiguous: ', '')
            #print r
            engine.say('...')
            engine.say(r)
            engine.say(r)
            engine.runAndWait()
        else:
            print response
            engine.say('...')
            engine.say(response)
            engine.say(response)
            engine.runAndWait()
        engine.stop()
    return ChatResponse(response)

def isWiki(answer):
    if 'wiki:' in answer:
        subject = answer.replace('wiki: ', '')
        try:
            result = wikipedia.summary(subject, sentences=2)
        except wikipedia.exceptions.DisambiguationError as e:
            options = e.options
            print options
            foo = ''
            for i in range(4):
                foo += "%s, " % random.choice(options)
                options.pop()
            foo += "and %s." % random.choice(options)
            result = "ambiguous: I found ambiguous articles on %s ... Here are some options ... %s" % (subject, foo)
        return result
    else: return None

def chat_server():
    rospy.init_node('chat_server')
    s = rospy.Service('chat', Chat, handle_chat_request)
    print "I'm listening ..."
    rospy.spin()

if __name__ == "__main__":
    engine.say('hello!')
    engine.runAndWait()
    chat_server()
