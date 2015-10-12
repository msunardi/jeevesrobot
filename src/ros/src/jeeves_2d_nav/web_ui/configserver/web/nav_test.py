import pdb
import sys
import yaml

import rospy
from std_msgs.msg import String

import platform
import cherrypy
import datetime
from configserver.tools.common import  get_version, render_template, info, error, success, warning

import jeeves_2d_nav
from jeeves_2d_nav.srv import *

class NavTestServer:
    def __init__(self):
        self.cmd_sub = rospy.Subscriber('/nav_test/cmd', String,
                                           self.cmd_cb)
        self.msg_sub = rospy.Subscriber('/nav_test/last_message', String,
                                           self.last_msg_cb)
        self.msg_sub = rospy.Subscriber('/nav_test/progress', String,
                                           self.progress_cb)        
        self.cmd_pub = rospy.Publisher('/nav_test/cmd', String,
                                       latch=True, queue_size=10)
        self.cmd = ""
        self.last_message = ""
        self.progress = ""
        
    @cherrypy.expose
    def index(self, **kwargs):
        return render_template("nav_test.html", cmd=self.cmd,
                               last_message=self.last_message,
                               progress=self.progress)
        
    @cherrypy.expose
    def cmd_nav_test(self, cmd):
        self.cmd_pub.publish(cmd)
        rospy.wait_for_message('/nav_test/cmd', String, timeout=10)
        raise cherrypy.HTTPRedirect("/nav_test")
        
    def cmd_cb(self, msg):
        self.cmd = msg.data

    def last_msg_cb(self, msg):
        self.last_message = msg.data        
    
    def progress_cb(self, msg):
        self.progress = msg.data
