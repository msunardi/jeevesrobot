#!/usr/bin/env python

import rospy
import tf
import pdb
import sys
import yaml
import actionlib

from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from basics.srv import Waypoint
# from std_msgs import float32

rospy.init_node('waypoint_client')
rospy.wait_for_service('way_point')
word_counter = rospy.ServiceProxy('way_point', Waypoint)

name = sys.argv[1]
x = float(sys.argv[2])
y = float(sys.argv[3])
theta = float(sys.argv[4])

print(name, x, y, theta)
word_count = word_counter(name, x, y, theta)

print(name, '->', word_count.result)
# print(word_count.result)
