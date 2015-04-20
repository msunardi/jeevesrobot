#!/usr/bin/env python
import json
import pdb
import threading
import yaml

import actionlib
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import tf

from jeeves_2d_nav.srv import *

DEFAULT_WAYPOINT_FILENAME = 'waypoints.yaml'
# EDIT THIS: default initial pose
HOME_X = 26.033
HOME_Y = 10.657
HOME_THETA = 1.528


class WaypointManager(threading.Thread):
    def __init__(self, waypoint_file):
        self.sleeper = rospy.Rate(1)
        self.waypoint_file = waypoint_file
        self.waypoints = []
        self.load_waypoints_from_file(waypoint_file)
        rospy.Service('/waypoint_manager/get_waypoints',
                      GetWaypoints,
                      self.handle_get_waypoints)
        rospy.Service('/waypoint_manager/add_waypoint',
                      AddWaypoint,
                      self.handle_add_waypoint)
        threading.Thread.__init__(self)

    def load_waypoints_from_file(self, f):
        rospy.loginfo("Loading waypoints from " + f)
        try:
            wp = yaml.load(file(f))
            for p in wp:
                self.add_waypoint(p)
        except IOError:
            msg = "IOError: could not open waypoint file " + f + "...skipping."
            rospy.logerr(msg)
            return
        except Exception as e:
            msg = "Caught exception parsing waypoint file " + f + "...skipping."
            msg += "Exception: " + str(e.args)
            rospy.logerr(msg)
            return

    def run(self):
        self.sleeper.sleep()
        while not rospy.is_shutdown():
            msg = "waypoints: "
            for wp in self.waypoints:
                msg += wp['name'] + ','
            rospy.loginfo(msg)
            self.sleeper.sleep()
        file(self.waypoint_file, 'w').write(yaml.dump(self.waypoints,
                                                    default_flow_style=False))

    def add_waypoint(self, wp):
        names = [p['name'] for p in self.waypoints]
        if wp['name'] not in names:
            rospy.loginfo("Adding new waypoing name: " + wp['name'])
            self.waypoints.append(wp)
            return 0
        else:
            rospy.logwarn("Ignoring duplicate waypoint name: " + wp['name'])
            return 1

    def handle_get_waypoints(self, req):
        rospy.logdebug("waypoint_manager.handle_get_waypoints()")
        return yaml.dump(self.waypoints)

    def handle_add_waypoint(self, req):
        wp = {'name': req.name,
             'x': req.x,
             'y': req.y,
             'theta': req.theta}
        return self.add_waypoint(wp)


if __name__ == '__main__':
    rospy.init_node('waypoint_manager')
    mgr = WaypointManager(rospy.get_param('waypoint_file', 'waypoints.yaml'))
    mgr.start()
    rospy.spin()
