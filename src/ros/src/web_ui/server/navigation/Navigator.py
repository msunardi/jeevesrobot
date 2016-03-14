import rospy
import yaml
import pdb
import sys

import jeeves_2d_nav
from jeeves_2d_nav.srv import *

class WaypointManager:

    def __init__(self):
        self.prxy_get_waypoints = rospy.ServiceProxy('waypoint_manager/get_waypoints', jeeves_2d_nav.srv.GetWaypoints)
         
