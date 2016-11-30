#!/usr/bin/env python

import rospy
import tf
import pdb
import sys
import yaml
import actionlib
import threading

from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from basics.srv import Waypoint, WaypointResponse


mbc = actionlib.SimpleActionClient('move_base', MoveBaseAction)

class WaypointBasics(threading.Thread):
    def __init__(self, waypoint_file):
        self.sleeper = rospy.Rate(1)
        self.sleeper.sleep()
        self.waypoint_file = waypoint_file
        self.waypoints = []
        rospy.Service('way_point', Waypoint, self.process_waypoint)
        threading.Thread.__init__(self)

    def process_waypoint(self, request):
        # return WaypointResponse(len(request.name.split()))
        #name = name.translate(None, "{")
        #name = name.translate(None, "}")
        #name = name.translate(None, ":")
        #name = name.replace("name", '')
        #name = name.translate(None, '"')
        #print name
        name = request.name
        #print request
        #stream = open("waypoints.yaml", 'r')
        stream = open(self.waypoint_file, 'r')
        doc = yaml.load(stream)
        #print doc
        wp = [wp for wp in doc if wp['name'] == name][0]
        x = wp['x']
        y = wp['y']
        theta = wp['theta']
        print (name, x, y, theta)
        rospy.loginfo("%s, %s, %s, %s" % (name, x, y, theta))
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, wp['theta'])
        #print q
        goal = MoveBaseGoal()
        goal.target_pose.pose = Pose(Point(x, y, 0.0),
                                 Quaternion(q[0], q[1], q[2], q[3]))
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        #print goal.target_pose.header.stamp
        #print goal
        mbc.send_goal(goal)
        return WaypointResponse("%s: %s: %s: %s" % (name, x, y, theta))

if __name__ == '__main__':
    rospy.init_node('service_waypoint_server')
    #service = rospy.Service('way_point', Waypoint, process_waypoint)
    service = WaypointBasics(rospy.get_param('/basics/waypoint_file', 'waypoints.yaml'))
    service.start()
    
    rospy.spin()
