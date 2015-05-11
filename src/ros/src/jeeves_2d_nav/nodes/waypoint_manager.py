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
from tf import TransformListener
from jeeves_2d_nav.srv import *

DEFAULT_WAYPOINT_FILENAME = 'waypoints.yaml'
RESULT_OK = 0
RESULT_DUPLICATE_WAYPOINT = -1
RESULT_DNE = -2
RESULT_POSE_NOT_AVAILABLE = -3

# EDIT THIS: default initial pose
HOME_X = 26.033
HOME_Y = 10.657
HOME_THETA = 1.528


class WaypointManager(threading.Thread):
    def __init__(self, waypoint_file, base_frame):
        self.sleeper = rospy.Rate(1)
        self.sleeper.sleep()
        self.tl = TransformListener()
        self.base_frame = base_frame
        self.waypoint_file = waypoint_file
        self.waypoints = []
        self.load_waypoints_from_file(waypoint_file)
        rospy.Service('/waypoint_manager/get_waypoints',
                      GetWaypoints,
                      self.handle_get_waypoints)
        rospy.Service('/waypoint_manager/add_waypoint',
                      AddWaypoint,
                      self.handle_add_waypoint)
        rospy.Service('/waypoint_manager/delete_waypoint',
                      DeleteWaypoint,
                      self.handle_delete_waypoint)
        rospy.Service('/waypoint_manager/save_current_pose',
                      SaveCurrentPose,
                      self.handle_save_current_pose)
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

        msg = "waypoints: "
        for wp in self.waypoints:
            msg += wp['name'] + ', '
        rospy.loginfo(msg)

    def run(self):
        self.sleeper.sleep()
        while not rospy.is_shutdown():
            try:
                self.sleeper.sleep()
            except Exception:
                pass
        file(self.waypoint_file, 'w').write(yaml.dump(self.waypoints,
                                                    default_flow_style=False))

    def add_waypoint(self, wp):
        names = [p['name'] for p in self.waypoints]
        if wp['name'] not in names:
            rospy.logdebug("Adding new waypoint name: " + wp['name'])
            self.waypoints.append(wp)
            return RESULT_OK
        else:
            rospy.logwarn("Ignoring duplicate waypoint name: " + wp['name'])
            return RESULT_DUPLICATE_WAYPOINT

    def handle_get_waypoints(self, req):
        rospy.logdebug("waypoint_manager.handle_get_waypoints()")
        return yaml.dump(self.waypoints, default_flow_style=False)

    def handle_add_waypoint(self, req):
        wp = {'name': req.name,
             'x': req.x,
             'y': req.y,
             'theta': req.theta}
        return self.add_waypoint(wp)

    def handle_delete_waypoint(self, req):
        try:
            found = next(x for x in self.waypoints if x['name'] == req.name)
            self.waypoints.remove(found)
            return RESULT_OK
        except StopIteration:
            return RESULT_DNE

    def handle_save_current_pose(self, req):
        if self.tl.frameExists(self.base_frame) and self.tl.frameExists("/map"):
            t = self.tl.getLatestCommonTime("/map", self.base_frame)
            p, q = self.tl.lookupTransform("/map", self.base_frame, t)
            theta = tf.transformations.euler_from_quaternion(q)[2]
            wp = {'name': req.name, 'x': p[0], 'y': p[1], 'theta': theta}
            return self.add_waypoint(wp)
        else:
            return RESULT_POSE_NOT_AVAILABLE

if __name__ == '__main__':
    rospy.init_node('waypoint_manager_node')
    mgr = WaypointManager(
        rospy.get_param('/waypoint_manager/waypoint_file', 'waypoints.yaml'),
        rospy.get_param('base_frame', '/base_footprint'))
    mgr.start()
    rospy.spin()
