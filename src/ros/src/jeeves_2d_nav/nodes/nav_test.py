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

import jeeves_2d_nav
from jeeves_2d_nav.srv import *

class NavTest(threading.Thread):
    def __init__(self):
        self.halt = True
        self.sleeper = rospy.Rate(1)
        self.waypoints = {}
        self.mbc = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.cmd_subscriber = rospy.Subscriber("/nav_test/cmd", String,
                                               self.cmd_callback)
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
                self.mbc.send_goal(goal)
                result = self.mbc.wait_for_result(rospy.Duration(300))
                if result == True:
                    rospy.loginfo("Arrived at waypoint: " + name)
                self.sleeper.sleep()
            else:
                self.sleeper.sleep()

    def cmd_callback(self, cmd_msg):
        if cmd_msg.data == "HALT":
            rospy.loginfo("Received cmd: HALT. Canceling goal.")
            self.halt = True
            self.mbc.cancel_goal()
        else:
            rospy.loginfo("Received cmd: " + cmd_msg.data)
            self.halt = False

def main(args):
    rospy.init_node('nav_test_node', anonymous=True)
    tester = NavTest()
    tester.start()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)