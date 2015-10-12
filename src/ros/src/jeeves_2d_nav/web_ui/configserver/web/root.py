"""
Root web server
"""
import cherrypy
import datetime
import platform
import rospy
from configserver.tools.common import  get_version, render_template, info, error, success, warning

from battery_monitor.msg import BatteryStatus

class RootServer:
    def __init__(self):
        self.batt_sub = rospy.Subscriber('/battery_status', BatteryStatus,
                                         self.battery_status_cb)
        self.amps = 0.0
        self.amp_hours = 0.0
        
    @cherrypy.expose
    def index(self, **kwargs):
        sw_version = get_version()
        return render_template("index.html",
                               sw_version=sw_version,
                               current_time=datetime.datetime.now(),
                               os_info=', '.join(platform.uname()[:4]),
                               amps=self.amps,
                               amp_hours=self.amp_hours)
    
    def battery_status_cb(self, msg):
        self.amps = msg.amps
        self.amp_hours = msg.amp_hours