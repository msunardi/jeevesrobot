#!/usr/bin/env python
import threading

import actionlib
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import tf

from jeeves_2d_nav.srv import *

# EDIT THIS: default initial pose
HOME_X = 0.0
HOME_Y = 0.0
HOME_THETA = 0.0


class WaypointManager(threading.Thread):
    def __init__(self):
        self.sleeper = rospy.Rate(1)
        rospy.Service('/waypoint_manager/cancel_current_nav_goal',
                      CancelCurrentNavGoal,
                      self.handle_cancel_current_nav_goal)
        rospy.Service('/waypoint_manager/set_default_initial_pose',
                      SetDefaultInitialPose,
                      self.handle_set_default_initial_pose)
        self.initialpose_publisher = rospy.Publisher("initialpose",
                                                     PoseWithCovarianceStamped,
                                                     queue_size=5)

        # Move Base Client
        self.mbc = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        threading.Thread.__init__(self)

    def run(self):
        while not rospy.is_shutdown():
            self.sleeper.sleep()

    def handle_cancel_current_nav_goal(self, req):
        rospy.loginfo("waypoint_manager: "
                      "received request to cancel current nav goal.")
        self.mbc.cancel_goal()
        return 0

    def handle_set_default_initial_pose(self, req):
        rospy.loginfo("waypoint_manager: "
                      "received request to set default initial pose: " +
                      str(HOME_X) + ", " + str(HOME_Y) +
                      ", " + str(HOME_THETA))
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, HOME_THETA)
        initialpose = PoseWithCovarianceStamped()
        initialpose.pose.pose = Pose(Point(HOME_X, HOME_Y, 0.000),
                                     Quaternion(*q))
        self.initialpose_publisher.publish(initialpose)
        return 0

if __name__ == '__main__':
    rospy.init_node('waypoint_manager', log_level=rospy.INFO)
    mgr = WaypointManager()
    mgr.start()
    rospy.spin()