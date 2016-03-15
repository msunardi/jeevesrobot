import rospy
import yaml
import pdb
import sys

import jeeves_2d_nav
from jeeves_2d_nav.srv import *

class WaypointManager:

    def __init__(self):
        self.prxy_get_waypoints = rospy.ServiceProxy('waypoint_manager/get_waypoints', jeeves_2d_nav.srv.GetWaypoints)
    
    def get_waypoints(self):
        try:
            rospy.wait_for_service('waypoint_manager/get_waypoints', timeout=3)
            return yaml.load(self.prxy_get_waypoints().waypoints)
        except rospy.ROSException, e:
            return e
