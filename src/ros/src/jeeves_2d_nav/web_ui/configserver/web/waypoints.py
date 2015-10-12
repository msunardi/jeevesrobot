import pdb
import sys
import yaml

import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import tf

import platform
import cherrypy
import datetime
from configserver.tools.common import  get_version, render_template, info, error, success, warning

import jeeves_2d_nav
from jeeves_2d_nav.srv import *

class WaypointServer:
    def __init__(self):
        self.mgr_get_waypoints = rospy.ServiceProxy(
            'waypoint_manager/get_waypoints',
            jeeves_2d_nav.srv.GetWaypoints)
        self.mgr_save_current_pose = rospy.ServiceProxy(
            'waypoint_manager/save_current_pose',
            jeeves_2d_nav.srv.SaveCurrentPose)
        self.mgr_delete_waypoint = rospy.ServiceProxy(
            'waypoint_manager/delete_waypoint',
            jeeves_2d_nav.srv.DeleteWaypoint)
        self.mbc = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
    @cherrypy.expose
    def index(self, **kwargs):
        try:
            rospy.wait_for_service('waypoint_manager/get_waypoints', timeout=3)
            return render_template("waypoints.html",
                                   waypoints=self.get_waypoints())
        except rospy.ROSException, e:
            return render_template("waypoints.html")
        
    @cherrypy.expose    
    def save_current_pose(self, **kwargs):
        if cherrypy.request.method == 'POST':
            try:
                rospy.wait_for_service('waypoint_manager/save_current_pose', timeout=3)
            except rospy.ROSException, e:
                pass
            rc = self.mgr_save_current_pose(kwargs['waypoint_name']).result
            if rc !=0:
                msg = "waypoint_manager/save_current_pose() \
                      returned error code: " + str(rc) + ' '
                return msg + "<a href='/waypoints'>Back to waypoints</a>"
            raise cherrypy.HTTPRedirect("/waypoints")
        else:			
            return render_template("save_current_waypoint.html")
    
    @cherrypy.expose
    def delete_waypoint(self, waypoint_name):
        try:
            rospy.wait_for_service('waypoint_manager/delete_waypoint', timeout=3)
        except rospy.ROSException, e:
            pass
        self.mgr_delete_waypoint(waypoint_name)
        raise cherrypy.HTTPRedirect("/waypoints")
    
    @cherrypy.expose
    def goto_waypoint(self, name):
        wp = [wp for wp in self.get_waypoints() if wp['name'] == name][0]
        x = wp['x']
        y = wp['y']
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, wp['theta'])
        goal = MoveBaseGoal()
        goal.target_pose.pose = Pose(Point(x, y, 0.0),
                                     Quaternion(q[0], q[1], q[2], q[3]))
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        self.mbc.send_goal(goal)
        return "Proceeding to waypoint <b>" + name + "</b>" + \
            "<br><a href='/waypoints'>Back to waypoints</a>"

    @cherrypy.expose
    def cancel_current_goal(self):
        self.mbc.cancel_goal()
        return "Navigation canceled.<br><a href='/waypoints'>Back to waypoints</a>"
        
    def get_waypoints(self):
        return yaml.load(self.mgr_get_waypoints().waypoints)
    

        