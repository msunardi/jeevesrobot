#!/usr/bin/env python

import aiml
import wikipedia
import wolframalpha
import pyttsx
import random
from chatbot.srv import *
import rospy
import rospkg

rospack = rospkg.RosPack()
engine = pyttsx.init('espeak')
app_id = 'RYRQ27-9LL7V4EXAU'
wa_client = wolframalpha.Client(app_id)
parser = aiml.Kernel()
parser.learn("%s/aiml1.6/wiki.aiml" % rospack.get_path('chatbot'))

def handle_chat_request(req):
    print "Requested entry: %s" % (req.pattern)
    response = isWiki(parser.respond(req.pattern))
    if response and 'Not found' not in response:
        if 'ambiguous: ' in response:
            response = response.replace('ambiguous: ', '')
        
        print response
        # engine.say('...')
        # engine.say('...')
        # engine.say(response)
        # engine.runAndWait()
        say(response)
        engine.stop()
    else:
        engine.say('...')
        if not findNumbers(req.pattern):
            engine.say('Cannot find %s in wikipedia. Trying Wolframalpha...' % req.pattern)
        response = isWolframalpha(req.pattern)
        if response:
            say(response)
        else:
            say("Hmm ... could not find it in Wolframalpha either. \
                Are you %s" % (response, random.choice(\
                    ['sure?', 'pulling my hypothetical leg?', 'sure about this?'])))

    return ChatResponse(response)

def say(words):
    engine.say('...')
    engine.say('...')
    engine.say(words)
    engine.runAndWait()

def findNumbers(pattern):
    l = []
    for t in pattern.split():
        try:
            l.append(float(t))
        except ValueError:
            try:
                l.append(int(t))
            except ValueError:
                pass
    return True if l else False 

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
        except wikipedia.exceptions.PageError as e:
            return "Not found in Wikipedia"
        return result
    else: return None

def isWolframalpha(pattern):
    try:
        result = wa_client.query(pattern)
        return next(result.results).text

    except ValueError:
        return None

def chat_server():
    rospy.init_node('chat_server')
    s = rospy.Service('chat', Chat, handle_chat_request)
    print "I'm listening ..."
    rospy.spin()

if __name__ == "__main__":
    engine.say('hello!')
    engine.runAndWait()
    chat_server()
