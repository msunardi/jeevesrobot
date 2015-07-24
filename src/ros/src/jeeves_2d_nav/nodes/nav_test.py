#!/usr/bin/env python
"""
.. module:: nav_test
   :synopsis: Node that waits for the service /waypoint_manager/get_waypoints
   service to become available, loads all waypoints then visits each waypoint
   in turn before exiting.
"""
import numpy as np
import threading
import yaml

import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
import rospy
import tf
from tf import TransformListener

import jeeves_2d_nav
from jeeves_2d_nav.srv import *

class NavTest(threading.Thread):
    def __init__(self):
        self.halt = True
        self.sleeper = rospy.Rate(1)
        self.waypoints = {}
        self.breadcrumbs = []
        self.total_distance_traveled_m = 0.0
        self.mbc = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.cmd_subscriber = rospy.Subscriber("/nav_test/cmd", String,
                                               self.cmd_callback)
        self.tf = TransformListener()
        threading.Thread.__init__(self)
        self.get_waypoints = rospy.ServiceProxy(
            '/waypoint_manager/get_waypoints', jeeves_2d_nav.srv.GetWaypoints)

    def run(self):
        rospy.loginfo("Waiting for move_base action server.")
        self.mbc.wait_for_server(rospy.Duration(600))
        rospy.loginfo("Connected to move_base action server.")

        rospy.loginfo("Waiting for /waypoint_manager/get_waypoints service.")
        rospy.wait_for_service('/waypoint_manager/get_waypoints')
        rospy.loginfo("Connected to move_base action service.")
        try:
            self.waypoints = yaml.load(self.get_waypoints().waypoints)
        except Exception as e:
            rospy.logerr("Exception getting waypoints: " + str(e.args))
        self.breadcrumbs.append(self.get_current_pose())

        while not rospy.is_shutdown():
            if not self.halt:
                idx = np.random.randint(0, len(self.waypoints))
                wp = self.waypoints[idx]
                name = wp['name']
                x = wp['x']
                y = wp['y']
                q = tf.transformations.quaternion_from_euler(0.0, 0.0, wp['theta'])
                goal = MoveBaseGoal()
                goal.target_pose.pose = Pose(Point(x, y, 0.0),
                                             Quaternion(q[0], q[1], q[2], q[3]))
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                rospy.loginfo("Proceeding to waypoint: " + name)
                trip_start_idx = len(self.breadcrumbs)
                self.mbc.send_goal(goal,
                                   feedback_cb=self.move_base_feedback_callback)
                result = self.mbc.wait_for_result(rospy.Duration(300))
                if result:
                    rospy.loginfo("Arrived at waypoint: " + name)
                    tl = trip_length(self.breadcrumbs[trip_start_idx:])
                    self.total_distance_traveled_m += tl
                    rospy.loginfo("trip length: " + str(tl))
                    rospy.loginfo("total_distance_traveled_m: " +
                                  str(self.total_distance_traveled_m))
                    rospy.logdebug("breadcrumbs: " + str(len(self.breadcrumbs)))
                self.sleeper.sleep()
            else:
                self.sleeper.sleep()

    def get_current_pose(self):
        if self.tf.frameExists("base_footprint") and self.tf.frameExists("map"):
            t = self.tf.getLatestCommonTime("base_footprint", "map")
            pos, q = self.tf.lookupTransform("base_footprint", "map", t)
            return Pose(pos, q)

    def cmd_callback(self, cmd_msg):
        if cmd_msg.data == "HALT":
            rospy.loginfo("Received cmd: HALT. Canceling goal.")
            self.halt = True
            self.mbc.cancel_goal()
        else:
            rospy.loginfo("Received cmd: " + cmd_msg.data)
            self.halt = False

    def move_base_feedback_callback(self, feedback_msg):
        # type(feedback_msg) =
        # <class 'move_base_msgs.msg._MoveBaseFeedback.MoveBaseFeedback'>
        self.breadcrumbs.append(feedback_msg.base_position.pose)


def trip_length(poses):
    trip_len = 0.0
    for pose in enumerate(poses):
        idx = pose[0]
        if idx == 0:
            continue
        dx = pose[1].position.x - poses[idx-1].position.x
        dy = pose[1].position.y - poses[idx-1].position.y
        d = np.sqrt((dx * dx) + (dy * dy))
        trip_len += d
    return trip_len

def main(args):
    rospy.init_node('nav_test_node', anonymous=True)
    tester = NavTest()
    tester.start()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
