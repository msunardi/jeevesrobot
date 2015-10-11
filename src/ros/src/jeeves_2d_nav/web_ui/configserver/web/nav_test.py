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
        self.status_sub = rospy.Subscriber("/nav_test/cmd", String, self.nav_test_cmd_status_cb)
        self.cmd_pub = rospy.Publisher('/nav_test/cmd', String, queue_size=10)
        self.cmd_status = ""
        
    @cherrypy.expose
    def index(self, **kwargs):
        return render_template("nav_test.html", status=self.get_nav_test_status())

    def nav_test_cmd_status_cb(self, status):
        self.cmd_status = status.data
        
    def get_nav_test_status(self):
        return self.cmd_status