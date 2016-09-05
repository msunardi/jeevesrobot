#!/usr/bin/env python
import itertools
import time
import yaml

import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import smach
import smach_ros
from std_msgs.msg import String
import tf
from tf import TransformListener

import jeeves_2d_nav
from jeeves_2d_nav.srv import *

# Points of Interest. Last item should be the 'home' starting point of the tour.
# Tour ends when we reach home.
TOUR_AGENDA = [
    'ee',
    'NEAR_lab',
    'robot_lab',
    'Biomedical_lab',
    'stairs'
]


class InTransitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived_at_poi', 'arrived_at_home'])
        self.mbc = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.tfl = TransformListener()
        self.get_waypoints = rospy.ServiceProxy(
            '/waypoint_manager/get_waypoints',
            jeeves_2d_nav.srv.GetWaypoints)

        # can't do much until we have a waypoint server
        rospy.loginfo("Waiting for /waypoint_manager/get_waypoints service.")
        rospy.wait_for_service('/waypoint_manager/get_waypoints')
        rospy.loginfo("Connected to waypoint_manager service.")

        # can't go anywhere until the Nav stack is up
        rospy.loginfo("Waiting for move_base action server.")
        self.mbc.wait_for_server(rospy.Duration(600))
        rospy.loginfo("Connected to move_base action server.")

        # init a circular iterator, assuming that we start at the first
        self.next_poi = itertools.cycle(TOUR_AGENDA)

    def execute(self, userdata):
        waypoints = yaml.load(self.get_waypoints().waypoints)
        next_poi = self.next_poi.next()
        # assume waypoints with unique names
        wp = [wp for wp in waypoints if wp['name'] == next_poi][0]

        # proceed
        x = wp['x']
        y = wp['y']
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, wp['theta'])
        goal = MoveBaseGoal()
        goal.target_pose.pose = Pose(Point(x, y, 0.0),
                                     Quaternion(q[0], q[1], q[2], q[3]))
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        msg = "Proceeding to waypoint: {0}".format(wp['name'])
        rospy.loginfo(msg)
        self.mbc.send_goal(goal)

        # wait for arrival at poi
        self.mbc.wait_for_result(rospy.Duration(300))
        rospy.loginfo('arrived at waypoint {0}'.format(wp['name']))

        # home is the last stop
        if wp['name'] == TOUR_AGENDA[-1]:
            rospy.loginfo('arrived_at_home')
            return 'arrived_at_home'
        else:
            return 'arrived_at_poi'


# define state Bar
class GivingSpielState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['spiel_complete'])

    def execute(self, userdata):
        time.sleep(5)
        return 'spiel_complete'

