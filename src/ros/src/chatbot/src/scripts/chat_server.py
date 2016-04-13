#!/usr/bin/env python

import aiml
import wikipedia
import wolframalpha
import pyttsx
import random
import re

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
            # engine.say('Cannot find %s in wikipedia. Trying Wolframalpha...' % req.pattern)
            think_choices = ['Cannot find %s in wikipedia. Trying Wolframalpha...' % req.pattern,\
                             'Let me think ...', 'Hang on ...', 'I\'m not sure, let me check Wolframalpha']
            engine.say(random.choice(think_choices))
        try:
            response = isWolframalpha(req.pattern)

            if response:
                if 'sqrt' in response:
                    response = response.replace('sqrt', 'square root of ')
                say(response)
            else:
                say("Hmm ... could not find it in Wolframalpha either. \
                    Are you %s" % (random.choice(\
                        ['sure?', 'pulling my hypothetical leg?', 'sure about this?'])))
        except ValueError:
            say("Sorry - I don't understand that question.")

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
    new_pattern, skip = findQuestionPattern(pattern)
    # if not new_pattern:
    #     return None
    
    print "Trying %s" % new_pattern
    try:
        result = wa_client.query(new_pattern)
        try:
            answer = []
            # if next(result.results):
            for p in result.pods:
                if p.title in ['Input', 'Result']:
                    answer.append(p.text)
                elif p.title == 'Decimal approximation':
                    answer.append(p.text[:7])
            return "The %s is %s" % (answer[0], answer[1])
            # return "%s is %s" % (new_pattern[skip:], next(result.results).text)
        except Exception:
            try:
                answer = ''
                for p in result.pods:
                    if p.title == 'Input interpretation':
                        answer += "You mean %s? ... " % p.text
                    if p.title in ['Result', 'Definition', 'Description', 'Alternate description']:
                        answer += p.text
                    if p.title in ['Unit conversions']:
                        answer += " or %s" % p.text
                return answer
            except Exception:
                return "Not found"

    except ValueError:
        return None

def findQuestionPattern(pattern):
    p = re.compile('(what|how (much|many|long|far|heavy|big|short|small|hot|cold|tall|deep|shallow|wide|narrow)|who|) (is|are)')
    i = 0
    skip = 0
    iterator = p.finditer(pattern)
    for it in iterator:
        i = it.start()
        skip = len(it.group())

    return pattern[i:], skip
    
def chat_server():
    rospy.init_node('chat_server')
    s = rospy.Service('chat', Chat, handle_chat_request)
    print "I'm listening ..."
    rospy.spin()

if __name__ == "__main__":
    engine.say('hello!')
    engine.runAndWait()
    chat_server()
