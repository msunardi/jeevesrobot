#!/usr/bin/env python
from collections import deque
import numpy
import sys
import threading
import time

import roslib
import rospy
from geometry_msgs.msg import Twist

import roboclaw as rc

SIMULATE_ROBOCLAWS = True

class BaseController(threading.Thread):
    
    def __init__(self):
        #start a RoboClawManager
        ports = ('/dev/ttyUSB0', '/dev/ttyACM0')
        baudrate = 2400
        accel = 250
        max_ticks_per_second = 3336
        poll_interval_s = 0.1
        self.motor_mgr_cmd_queue = deque()
        self.motor_mgr_output_queue = deque()
        self.motor_mgr = rc.RoboClawManager(ports, baudrate, accel,
                                max_ticks_per_second,
                                rc.TICKS_PER_REV,
                                poll_interval_s,
                                self.motor_mgr_cmd_queue,
                                self.motor_mgr_output_queue,
                                SIMULATE_ROBOCLAWS)
        self.subscriber = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.cmd_vel_incoming = deque()
        self.lock = threading.Lock()
        threading.Thread.__init__(self)    

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                if len(self.cmd_vel_incoming) != 0:
                    twist = self.cmd_vel_incoming.popleft()
                    rospy.logdebug("Twist message received: " + str(twist))
            time.sleep(0.1)
        rospy.loginfo("BaseController.run(): exiting.")
        
    def cmd_vel_callback(self, twist_msg):
        with self.lock:
            self.cmd_vel_incoming.append(twist_msg)

def main(args):
    controller = BaseController()
    controller.start()
    rospy.init_node('base_controller_node', anonymous=True)
    rospy.spin()
    

if __name__ == '__main__':
    main(sys.argv)
