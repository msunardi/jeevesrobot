#!/usr/bin/env python

import sys
import rospy
from arbiter.srv import *

def arbiter_client(req_itype, req_value, req_description):
    rospy.wait_for_service('arbiter_server')
    try:
        foo = rospy.ServiceProxy('arbiter_server', Interact)
        resp = foo(req_itype, req_value, req_description)
        return resp.response
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        rospy.logerr("Service call failed: %s" % e)

def usage():
    return "%s [type] [value] [description]" % sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        itype = str(sys.argv[1])
        value = str(sys.argv[2])
        description = str(sys.argv[3])
    else:
        print usage()
        sys.exit(1)
    print "Requesting {Type: %s, Value: %s, Description: %s}" % (itype, value, description)
    print "Response: %s" % arbiter_client(itype, value, description)
