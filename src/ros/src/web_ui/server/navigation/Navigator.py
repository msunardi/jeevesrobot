import rospy
import yaml
import pdb
import sys
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import tf

import jeeves_2d_nav
from jeeves_2d_nav.srv import *
from nav_msgs.srv import GetMap

class WaypointManager:

    def __init__(self):
        self.prxy_get_waypoints = rospy.ServiceProxy(
            'waypoint_manager/get_waypoints',
            jeeves_2d_nav.srv.GetWaypoints)
        self.prxy_save_current_pose = rospy.ServiceProxy(
            'waypoint_manager/save_current_pose',
            jeeves_2d_nav.srv.SaveCurrentPose)
        self.prxy_delete_waypoint = rospy.ServiceProxy(
            'waypoint_manager/delete_waypoint',
            jeeves_2d_nav.srv.DeleteWaypoint)    
        self.prxy_set_current_pose_to_waypoint = rospy.ServiceProxy(
            'waypoint_manager/set_current_pose_to_waypoint',
            jeeves_2d_nav.srv.SetCurrentPoseToWaypoint)
        self.mbc = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def get_waypoints(self):
    # Returns a dictionary of waypoints
    
        try:
            rospy.wait_for_service('waypoint_manager/get_waypoints', timeout=3)
            return self._get_wp()
        except rospy.ROSException, e:
            return e
   
    def _get_wp(self):
        return yaml.load(self.prxy_get_waypoints().waypoints)

    def save_current_pose(self, waypoint_name):
    # Save the current pose as new waypoint
        try:
            rospy.wait_for_service('waypoint_manager/save_current_pose', timeout=3)
        except rospy.ROSException, e:
            pass

        rc = self.prxy_save_current_pose(waypoint_name).result
        if rc != 0:
            msg = "waypoint_manager/save_current_pose() returned error code: " + str(rc) + ' '
            #return msg
            return rc
        else:
            pass

    def delete_waypoint(self, waypoint_name):
        try:
            rospy.wait_for_service('waypoint_manager/delete_waypoint', timeout=3)
        except rospy.ROSException, e:
            pass
        self.prxy_delete_waypoint(waypoint_name)
        return waypoint_name

    def goto_waypoint(self, waypoint_name):
        wp = [wp for wp in self._get_wp() if wp['name'] == waypoint_name][0]
        x = wp['x']
        y = wp['y']
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, wp['theta'])
        goal = MoveBaseGoal()
        goal.target_pose.pose = Pose(Point(x, y, 0.0),
                                     Quaternion(q[0], q[1], q[2], q[3]))
        goal.target_pose.header.frame_id = 'map'
        return {'x': x, 'y': y, 'q': q, 'goal': goal}
        goal.target_pose_header.stamp = rospy.Time.now()
        self.mbc.send_goal(goal)
        return "Proceeding to waypoint <b>" + waypoint_name + "</b>"

    def cancel_current_goal(self):
        self.mbc.cancel_goal()
        return "Navigation canceled."

    def set_current_pose_to_waypoint(self, waypoint_name):
        try:
            rospy.wait_for_service('waypoint_manager/set_current_pose_to_waypoint', timeout=3)
        except rospy.ROSException, e:
            return -1
        self.prxy_set_current_pose_to_waypoint(waypoint_name)
        return 0

class Mapper:
    
    def __init__(self):
        self.map_sub = rospy.Subscriber(
            '/map', nav_msgs/OccupancyGrid, self.map_cb)
        self.prxy_static_map = rospy.ServiceProxy(
            '/static_map',
            map_server.srv.static_map)

class NavTest:
    def __init__(self):
        self.cmd_sub = rospy.Subscriber('/nav_test/cmd', 
                                        String, self.cmd_cb)
        self.msg_sub = rospy.Subscriber('/nav_test/last_message', 
                                        String, self.last_msg_cb)
        self.prog_sub = rospy.Subscriber('/nav_test/progress',
                                        String, self.progress_cb)
        self.cmd_pub = rospy.Publisher('/nav_test/cmd',
                                        String, latch=True, queue_size=10)
        self.cmd = ""
        self.last_message = ""
        self.progress = ""

    def cmd_nav_test(self, cmd):
        self.cmd_pub.publish(cmd)
        rospy.wait_for_messge('/nav_test/cmd', String, timeout=10)
        return cmd

    def cmd_cb(self, msg):
        self.cmd = msg.data

    def last_msg_cb(self, msg):
        self.last_message = msg.data

    def progress_cb(self, msg):
        self.progress = msg.data

    def get_cmd(self):
        return self.cmd

    def get_last_msg(self):
        return self.last_message

    def get_progress(self):
        return self.progress 
        
