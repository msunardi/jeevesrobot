#!/usr/bin/env python
import pdb
import sys
import rospy

import jeeves_2d_nav
from jeeves_2d_nav.srv import *


def get_waypoints():
    try:
        rospy.wait_for_service('waypoint_manager/get_waypoints', timeout=3)
        get_waypts = rospy.ServiceProxy('waypoint_manager/get_waypoints',
                                       jeeves_2d_nav.srv.GetWaypoints)
        response = get_waypts()
        return response
    except rospy.ROSException, e:
        return "Service waypoint_manager/get_waypoints is not available."
    except rospy.ServiceException, e:
        return "ServiceException: %s"%e

if __name__ == "__main__":
    wp = get_waypoints()
    pdb.set_trace()