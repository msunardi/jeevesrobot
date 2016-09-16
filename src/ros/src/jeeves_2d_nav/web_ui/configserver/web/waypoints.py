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

from std_msgs.msg import String

class WaypointServer:
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
        
        # Create waypoint_manager/start_nav_wait topic
        self.start_nav_wait_topic = rospy.Publisher('waypoint_manager/start_nav_wait', String, queue_size=10)
        
        # Subscribe to the start_nav_wait topic
        rospy.Subscriber("waypoint_manager/start_nav_wait", String, self.wait_nav_finish)
        
        # Create speech synthesis topic if it doesn't exist yet
        self.t2s_topic = rospy.Publisher('jeeves_speech/speech_synthesis'   , String, queue_size=10)
        
        # Create AIML request topic, and listen on speech_aiml_resp topic for response
        self.aiml_resp = ''
        self.aiml_resp_flag = False
        self.speech_aiml_req = rospy.Publisher('jeeves_speech/speech_aiml_req', String, queue_size=10)
        rospy.Subscriber("jeeves_speech/speech_aiml_resp", String, self.proc_aiml_resp)
        

        
        
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
            rc = self.prxy_save_current_pose(kwargs['waypoint_name']).result
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
        self.prxy_delete_waypoint(waypoint_name)
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
        self.start_nav_wait_topic.publish(name)
        return "Proceeding to waypoint <b>" + name + "</b>" + \
            "<br><a href='/waypoints'>Back to waypoints</a>"

    @cherrypy.expose
    def cancel_current_goal(self):
        self.mbc.cancel_goal()
        return "Navigation canceled.<br><a href='/waypoints'>Back to waypoints</a>"

    @cherrypy.expose
    def set_current_pose_to_waypoint(self, waypoint_name):
        try:
            rospy.wait_for_service('waypoint_manager/set_current_pose_to_waypoint', timeout=3)
        except rospy.ROSException, e:
            pass
        self.prxy_set_current_pose_to_waypoint(waypoint_name)
        raise cherrypy.HTTPRedirect("/waypoints")

    def get_waypoints(self):
        return yaml.load(self.prxy_get_waypoints().waypoints)

    '''
    # -----------------------------------------------------------------------------------------------------
    #                                          wait_nav_finish()
    #
    #   Description:  
    #
    #     Arguments:  N/A
    #
    #       Returns:  N/A
    #
    # -----------------------------------------------------------------------------------------------------
    '''
    def wait_nav_finish(self, message):
        
        name = message.data.replace('_', ' ').upper()
        self.aiml_resp_flag = False
        
        self.t2s_topic.publish('Escorting you to the %s' % name);
        
        # Wait for the navigation goal to finish, timeout after 600 seconds
        self.mbc.wait_for_result(rospy.Duration(600))

        # Clear AIML response
        self.aiml_resp = '';
        
        self.speech_aiml_req.publish(name);                          # Publish the name of the location to the AIML node

        # Wait for a response from the AIML node
        while(self.aiml_resp_flag == False):
            None
        self.aiml_resp_flag = False;    # Clear the flag

        # If a match was found, send it to the text to speech node.
        if(len(self.aiml_resp) > 0):
            self.t2s_topic.publish(self.aiml_resp);
            
    '''
    # -----------------------------------------------------------------------------------------------------
    #                                          proc_aiml_resp()
    #
    #   Description:  This function gets called whenever a message is published to the 
    #                 "jeeves_speech/speech_aiml_resp" topic.
    #
    #     Arguments:  N/A
    #
    #       Returns:  N/A
    #
    # -----------------------------------------------------------------------------------------------------
    '''
    def proc_aiml_resp(self, message):

        self.aiml_resp = message.data;      # Store the response string form the AIML node
        self.aiml_resp_flag = True;         # Set flag to indicate that the AIML node has responded with a string

        