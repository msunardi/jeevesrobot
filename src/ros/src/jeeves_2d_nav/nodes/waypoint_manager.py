#!/usr/bin/env python
import json
import pdb
import threading
import yaml

import actionlib
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
import rospy
import tf
from tf import transformations
from tf import TransformListener

from jeeves_2d_nav.srv import *

DEFAULT_WAYPOINT_FILENAME = 'waypoints.yaml'
RESULT_OK = 0
RESULT_DUPLICATE_WAYPOINT = -1
RESULT_DNE = -2
RESULT_POSE_NOT_AVAILABLE = -3


class Waypoint(object):
    def __init__(self, wp_dict):
        """Construct from a dictionary."""
        # first, sane defaults
        self.name = 'default'
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.enabled = True
        
        # load in anything from the incoming dict that
        # matches one of our attributes
        for attr in self.__dict__.keys():
            if attr in wp_dict.keys():
                self.__dict__[attr] = wp_dict[attr]

    def __getitem__(self, key):
        return self.__dict__[key]
    
    def as_dict(self):
        d = {}
        for attr in self.__dict__.keys():
            d[attr] = self.__dict__[attr]
        return d

    
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
        rospy.Service('/waypoint_manager/set_current_pose_to_waypoint',
                      SetCurrentPoseToWaypoint,
                      self.handle_set_current_pose_to_waypoint)
        self.pose_pub = rospy.Publisher('initialpose',
                                       PoseWithCovarianceStamped,
                                       latch=True,
                                       queue_size=10)        
        threading.Thread.__init__(self)

    def load_waypoints_from_file(self, f):
        """ See waypoints.yaml for examples of the waypoint format."""
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
        self.save_waypoints()

    def add_waypoint(self, wp):
        names = [p['name'] for p in self.waypoints]
        if wp['name'] not in names:
            rospy.logdebug("Adding new waypoint name: " + wp['name'])
            self.waypoints.append(Waypoint(wp))
            self.save_waypoints()
            return RESULT_OK
        else:
            rospy.logwarn("Ignoring duplicate waypoint name: " + wp['name'])
            return RESULT_DUPLICATE_WAYPOINT

    def save_waypoints(self):
        s = yaml.dump([wp.as_dict() for wp in self.waypoints], default_flow_style=False)    
        with open(self.waypoint_file, 'w') as f:
            f.write(s)

    def handle_get_waypoints(self, req):
        rospy.logdebug("waypoint_manager.handle_get_waypoints()")
        return yaml.dump(
            [wp.as_dict() for wp in self.waypoints],
            default_flow_style=False)

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
            self.save_waypoints()
            return RESULT_OK
        except StopIteration:
            return RESULT_DNE

    def handle_save_current_pose(self, req):
        if self.tl.frameExists(self.base_frame) and self.tl.frameExists("map"):
            t = self.tl.getLatestCommonTime("map", self.base_frame)
            p, q = self.tl.lookupTransform("map", self.base_frame, t)
            theta = tf.transformations.euler_from_quaternion(q)[2]
            wp = {'name': req.name, 'x': p[0], 'y': p[1], 'theta': theta}
            return self.add_waypoint(wp)
        else:
            return RESULT_POSE_NOT_AVAILABLE

    def handle_set_current_pose_to_waypoint(self, req):
        """Set initialpose to pose described by req.name"""
        wp = None
        try:
            wp = next(wp for wp in self.waypoints if wp['name'] == req.name)
        except StopIteration:
            msg = "KeyError: waypoint '" + req.name + "' not found."
            rospy.logerr(msg)
            return RESULT_DNE

        # Instantiate a pose from the waypoint.
        # Covariance numbers copy/pasted from navigation/amcl/test/set_pose.py
        p = PoseWithCovarianceStamped()
        p.header.frame_id = "map"
        p.header.stamp = rospy.Time.now()
        p.pose.pose.position.x = wp['x']
        p.pose.pose.position.y = wp['y']
        (p.pose.pose.orientation.x,
         p.pose.pose.orientation.y,
         p.pose.pose.orientation.z,
         p.pose.pose.orientation.w) = transformations.quaternion_from_euler(0, 0, wp['theta'])
        p.pose.covariance[6*0+0] = 0.5 * 0.5
        p.pose.covariance[6*1+1] = 0.5 * 0.5
        p.pose.covariance[6*3+3] = np.pi/12.0 * np.pi/12.0
        msg = "Setting initialpose to waypoint " + wp['name']
        rospy.loginfo(msg)
        self.pose_pub.publish(p)
        return RESULT_OK

if __name__ == '__main__':
    rospy.init_node('waypoint_manager_node')
    mgr = WaypointManager(
        rospy.get_param('/waypoint_manager/waypoint_file', 'waypoints.yaml'),
        rospy.get_param('base_frame', 'base_footprint'))
    mgr.start()
    rospy.spin()
