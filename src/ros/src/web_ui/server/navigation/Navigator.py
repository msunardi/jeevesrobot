import rospy
import yaml
import pdb
import sys

import jeeves_2d_nav
from jeeves_2d_nav.srv import *

class WaypointManager:

    def __init__(self):
        self.prxy_get_waypoints = rospy.ServiceProxy('waypoint_manager/get_waypoints', jeeves_2d_nav.srv.GetWaypoints)
        self.prxy_save_current_pose = rospy.ServiceProxy('waypoint_manager/save_current_pose', jeeves_2d_nav.srv.SaveCurrentPose)
        self.prxy_delete_waypoint = rospy.ServiceProxy('waypoint_manager/delete_waypoint', jeeves_2d_nav.srv.DeleteWaypoint)    

    def get_waypoints(self):
    # Returns a dictionary of waypoints
    
        try:
            rospy.wait_for_service('waypoint_manager/get_waypoints', timeout=3)
            return yaml.load(self.prxy_get_waypoints().waypoints)
        except rospy.ROSException, e:
            return e

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
