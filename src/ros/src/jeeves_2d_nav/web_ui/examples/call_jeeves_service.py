#!/usr/bin/env python
"""Some examples of calling services on Jeeves."""
import pdb
import rospy
import sys
import yaml

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
    
    # returns a jeeves_2d_nav.srv._GetWaypoints.GetWaypointsResponse 
    wp = get_waypoints()
       
    # wp.waypoints is a yaml string representing a list 
    # of the current set of waypoints. Parsing the yaml
    # string gets a list of dicts, with each dict
    # representing one waypoint (name, x, y, theta).
    # See jeeves_2d_nav/srv/GetWaypoints.srv and
    # jeeves_2d_nav/nodes/waypoint_manager.py.
    print "YAML representation:\n", wp.waypoints
    print "Parsed:\n", yaml.load(wp.waypoints)