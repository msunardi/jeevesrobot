#!/usr/bin/env python

import sys
import rospy
from chatbot.srv import *

def chat_client(pattern):
    rospy.wait_for_service('chat')
    try:
        chat_service = rospy.ServiceProxy('chat', Chat)
        response = chat_service(pattern)
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def usage():
    return "Foo!"

if __name__ == "__main__":
    if str(sys.argv[1]):
        pattern = str(sys.argv[1])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s" % pattern
    print "Summary for %s: %s" % (pattern, chat_client(pattern))