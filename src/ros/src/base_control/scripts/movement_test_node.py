#!/usr/bin/env python
from collections import deque
import copy
import numpy as np
import pdb
import sys
import threading
import time

import roslib
import rospy
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf
from roboteq_msgs.msg import Command as MotorCommand
from roboteq_msgs.msg import Feedback as MotorFeedback
import PyKDL as kdl
import tf


class MovementTester(threading.Thread):

    def __init__(self):
        self.arrived = True
        self.waypoints = deque()
        self.current_waypoint = Point()
        self.current_cmd = Twist()
        self.sleeper = rospy.Rate(5.0)
        self.subscriber = rospy.Subscriber("/move_base_simple/goal",
                                           PoseStamped, self.nav_goal_callback)
        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist)
        self.tf_listener = tf.TransformListener()
        threading.Thread.__init__(self)

    def get_position_odom_frame(self):
        now = rospy.Time.now()
        self.tf_listener.waitForTransform("/odom", "/base_footprint",
                                          now, rospy.Duration(4.0))
        (trans, rot) = self.tf_listener.lookupTransform(
            "/odom", "/base_footprint", now)
        return (trans, rot)

    def distance_to_current_waypoint(self, position):
        xdiff = position[0] - self.current_waypoint.pose.position.x
        ydiff = position[1] - self.current_waypoint.pose.position.y
        d = np.sqrt((xdiff * xdiff) + (ydiff * ydiff))
        return d

    def heading_to_current_waypoint(self):
        """Return a PointStamped, in the base_footprint frame, of unit length,
        pointing toward the current waypoint."""
        wp = PointStamped()
        wp.header = self.current_waypoint.header
        wp.point = self.current_waypoint.pose.position
        self.tf_listener.waitForTransform("/odom", "/base_footprint",
                                          rospy.Time.now(), rospy.Duration(4.0))
        v = self.tf_listener.transformPoint('/base_footprint', wp)
        (pos, rot) = self.get_position_odom_frame()
        d = self.distance_to_current_waypoint(pos)
        v.point.x = v.point.x / d
        v.point.y = v.point.y / d
        return v

    def run(self):
        self.tf_listener.waitForTransform("/odom", "/base_footprint",
                                          rospy.Time(), rospy.Duration(4.0))
        loop_count = 0
        while not rospy.is_shutdown():
            self.sleeper.sleep()
            if (len(self.waypoints) == 0):
                continue
            else:
                self.current_waypoint = self.waypoints.popleft()
                v = self.heading_to_current_waypoint()
                rospy.loginfo("Heading to current waypoint: " + str(v))
            speed = 0.2
            msg = Twist()
            msg.linear.x = v.point.x * speed
            msg.linear.y = v.point.y * speed
            self.current_cmd = msg
            self.arrived = False

                # (trans, rot) = self.get_position_odom_frame()
                # d = self.distance_to_current_waypoint(trans)
                # if (loop_count % 5) == 0:
                #     rospy.loginfo("Distance to nav goal: " + str(d))
                # if self.arrived == False:
                #     self.cmd_publisher.publish(self.current_cmd)
                # if d < 0.1:
                #     self.arrived = True
                #     rospy.loginfo("Arrived.")
                #     if len(self.waypoints) > 0:
                #         self.current_waypoint = self.waypoints.popleft()
                #         rospy.loginfo("Proceeding to waypoint: " +
                #                   str(self.current_waypoint))
                #         d_new = self.distance_to_current_waypoint(trans)
                #         v = PointStamped(
                #             Header(),
                #             Point(
                #                 (self.current_waypoint.x - trans[0]) / d_new,
                #                 (self.current_waypoint.y - trans[1]) / d_new,
                #                 0.0))
                #         v.header.frame_id = '/odom'
                #         pdb.set_trace()
                #         v = self.tf_listener.transformPoint('/base_footprint', v)
                #         speed = 0.2
                #         msg = Twist()
                #         msg.linear.x = v.point.x * speed
                #         msg.linear.y = v.point.y * speed
                #         self.current_cmd = msg
                #         self.arrived = False
                #else:
                #    self.arrived = False
                loop_count += 1

    def nav_goal_callback(self, goal_msg):
        nav_goal = goal_msg # PoseStamped
        rospy.loginfo("nav goal received: " + str(nav_goal))
        self.waypoints.append(nav_goal)


def main(args):
    rospy.init_node('movement_test_node', anonymous=True, log_level=rospy.INFO)
    tester = MovementTester()
    tester.start()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)