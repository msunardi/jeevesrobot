#!/usr/bin/env python
"""
.. module:: nav_test
   :synopsis: Node that waits for the service /waypoint_manager/get_waypoints
   service to become available, loads all waypoints then visits each waypoint
   in turn before exiting.
"""
import sys
import threading

import rospy

import jeeves_2d_nav
from jeeves_2d_nav.srv import *

class NavTest(threading.Thread):
    def __init__(self):
        self.sleeper = rospy.Rate(1)
        threading.Thread.__init__(self)
        self.get_waypoints = rospy.ServiceProxy(
            '/waypoint_manager/get_waypoints', jeeves_2d_nav.srv.GetWaypoints)

    def run(self):
        rospy.loginfo("Waiting for /waypoint_manager/get_waypoints service.")
        rospy.wait_for_service('/waypoint_manager/get_waypoints')
        while not rospy.is_shutdown():
            rospy.loginfo(self.get_waypoints())
            self.sleeper.sleep()


def main(args):
    rospy.init_node('nav_test_node', anonymous=True)
    tester = NavTest()
    tester.start()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)