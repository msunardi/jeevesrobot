import pdb
import rospy
import sys
import yaml

import platform
import cherrypy
import datetime
from configserver.tools.common import  get_version, render_template, info, error, success, warning

import jeeves_2d_nav
from jeeves_2d_nav.srv import *

class WaypointServer:
    def __init__(self):
        self.get_waypts = rospy.ServiceProxy('waypoint_manager/get_waypoints',
                                       jeeves_2d_nav.srv.GetWaypoints)
    @cherrypy.expose
    def index(self, **kwargs):
        return render_template("waypoints.html", waypoints=self.get_waypoints())
    
    def get_waypoints(self):
        try:
            rospy.wait_for_service('waypoint_manager/get_waypoints', timeout=3)
        except rospy.ROSException, e:
            return [{'name': 'Service get_waypoints is currently unavailable'}]
        return yaml.load(self.get_waypts().waypoints)