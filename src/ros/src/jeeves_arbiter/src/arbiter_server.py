#!/usr/bin/env python
'''
    Arbiter
    Handles interaction from Jeeves Web UI
    Currently handles tour info and actions
    Invoke by calling 'arbiter_server' Service with arguments:
    - itype: interaction type - [nav, tour, info, play, or other]
    - value: keyword for information
    - description: additional information/description
'''
from arbiter.srv import *
import rospy
from std_msgs.msg import *
from basics.srv import *
import random as r
import subprocess

spiel = rospy.Publisher('/spiel', String, queue_size=10)

prompt = ['Excellent choice .', 'Very good .', 'Certainly .', 'Of course .', 'Right . ', 'Naturally .', 'Indeed .', 'Obviously .', 'Indubitably .', 'Why, of course .', 'Marvelous .', 'Magnificent .']

def festival(say):
    subprocess.call(["festival", "--batch", "(SayText \"" + say + "\")"])

def handler(request):
    print "Getting request: %s - %s - %s" % (request.itype, request.value, request.description)
    itype = request.itype
    val = request.value
    description = request.description
    response = "I don't know what you're asking"

    # Make decisions based on type
    if itype == 'nav':
        response = "Navigation request to: %s" % val
        p = r.choice(prompt)
        msg = r.choice(['Ah, ... ', '']) +  description + ', ...  ' + p
        festival(msg)
        spiel.publish('%s:%s' % (itype, val))

    elif itype == 'tour':
        response = "Tour request"
    elif itype == 'info':
        response = "Requesting info for: %s" % val
        p = r.choice(prompt)
        msg = r.choice(['Ah, ... ', 'Ah, yes,', '']) +  description + ', ...  ' + p
        festival(msg)
        spiel.publish('%s:%s' % (itype, val))
    elif itype == 'play':
        response = "Requesting play behavior"

    return InteractResponse(response)
    # return InteractResponse("Got foo -- itype: %s - value: %s" % (request.itype, request.value))

def arbiter_server():
    rospy.init_node('arbiter_server')
    s = rospy.Service('arbiter_server', Interact, handler)
    print "Ready to receive interaction messages..."
    rospy.spin()

if __name__ == "__main__":
    arbiter_server()
