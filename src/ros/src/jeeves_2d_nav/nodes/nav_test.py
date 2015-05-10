#!/usr/bin/env python
"""
.. module:: nav_test
   :synopsis: Node that waits for the service /waypoint_manager/get_waypoints
   service to become available, loads all waypoints then visits each waypoint
   in turn before exiting.
"""
import threading
import yaml

import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import tf

import jeeves_2d_nav
from jeeves_2d_nav.srv import *

class NavTest(threading.Thread):
    def __init__(self):
        self.halt = False
        self.sleeper = rospy.Rate(1)
        self.waypoints = {}
        self.mbc = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        threading.Thread.__init__(self)
        self.get_waypoints = rospy.ServiceProxy(
            '/waypoint_manager/get_waypoints', jeeves_2d_nav.srv.GetWaypoints)

    def run(self):
        # load waypoints
        rospy.loginfo("Waiting for /waypoint_manager/get_waypoints service.")
        rospy.wait_for_service('/waypoint_manager/get_waypoints')
        try:
            self.waypoints = yaml.load(self.get_waypoints().waypoints)
        except Exception as e:
            rospy.logerr("Exception getting waypoints: " + str(e.args))

        self.mbc.wait_for_server(rospy.Duration(60))
        wp = self.waypoints[0]
        name = wp['name']
        x = wp['x']
        y = wp['y']
        theta = wp['theta']
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
        goal = MoveBaseGoal()
        goal.target_pose.pose = Pose(Point(x, y, 0.0),
                                     Quaternion(q[0], q[1], q[2], q[3]))
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        self.mbc.send_goal(goal)
        result = self.mbc.wait_for_result(rospy.Duration(300))
        rospy.loginfo("wait_for_result returned: " + str(result))

def main(args):
    rospy.init_node('nav_test_node', anonymous=True)
    tester = NavTest()
    tester.start()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)