#!/usr/bin/env python
from collections import deque
import copy
import numpy as np
import sys
import threading
import time

import roslib
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from roboteq_msgs.msg import Command as MotorCommand
from roboteq_msgs.msg import Feedback as MotorFeedback
import PyKDL as kdl
import tf

import roboclaw as rc

WHEEL_RADIUS_m = 0.1016 # 4" radius wheels, in meters
HALF_WHEELBASE_X_m = 0.2032 # 16" / 2, in meters
HALF_WHEELBASE_Y_m = 0.2667 # 21" / 2, in meters
MOTOR_CONTROLLER_CMD_RATE_Hz = 50


class BaseController(threading.Thread):
    
    def __init__(self):
        self.sleeper = rospy.Rate(MOTOR_CONTROLLER_CMD_RATE_Hz)
        self.motor_1_cmd_publisher = rospy.Publisher("/motor_1/cmd", MotorCommand, queue_size=5)
        self.motor_2_cmd_publisher = rospy.Publisher("/motor_2/cmd", MotorCommand, queue_size=5)
        self.motor_3_cmd_publisher = rospy.Publisher("/motor_3/cmd", MotorCommand, queue_size=5)
        self.motor_4_cmd_publisher = rospy.Publisher("/motor_4/cmd", MotorCommand, queue_size=5)

        self.bth = BaseTransformHandler(WHEEL_RADIUS_m, HALF_WHEELBASE_X_m,
                                  HALF_WHEELBASE_Y_m)

        self.subscriber = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.cmd_vel_incoming = Twist()
        self.lock = threading.Lock()
        threading.Thread.__init__(self)

    def run(self):
        """Pull a cmd_vel (Twist) message from the incoming queue, covert it
          to a tuple of motor speeds, and put them into the motor manager's
          work queue, ad infinitum"""
        twist = Twist()
        while not rospy.is_shutdown():
            
            # we send command to the controllers at a steady rate
            # that doesn't depend on the rate of incoming cmd_vel messaes
            self.sleeper.sleep()
            
            # get the current command
            with self.lock:
                twist = self.cmd_vel_incoming
                
            # convert the incoming velocity vector into wheel speeds,
            # (rad/s) and publish them 
            rospy.logdebug("current cmd_vel: " + str(twist))
            w = self.bth.twist_to_wheel_velocities(twist)
            c = MotorCommand()
            self.motor_1_cmd_publisher.publish(MotorCommand(w[0]))
            self.motor_2_cmd_publisher.publish(MotorCommand(w[1]))
            self.motor_3_cmd_publisher.publish(MotorCommand(w[2]))
            self.motor_4_cmd_publisher.publish(MotorCommand(w[3]))

        rospy.loginfo("BaseController.run(): exiting.")
        
    def cmd_vel_callback(self, twist_msg):
        with self.lock:
            self.cmd_vel_incoming = twist_msg


class BaseTransformHandler(object):
    def __init__(self, R, l1, l2):
        L = l1 + l2
        assert l1 > 0.0
        assert l2 > 0.0
        assert R > 0.0
        self.R = R
        self.w4_to_v3 =  np.array([
            [1.0, 1.0, 1.0, 1.0],
            [-1.0, 1.0, -1.0, 1.0],
            [-1.0/L, -1.0/L, 1.0/L, 1.0/L]])
        self.v3_to_w4 = np.array([
            [1.0, -1.0, -L],
            [1.0, 1.0, -L],
            [1.0, -1.0, L],
            [1.0, 1.0, L]])

    def wheel_velocities_to_twist(self, w):
        w = np.array([w[0], w[1], w[2], w[3]])
        v = np.dot((self.R / 4.0) * self.w4_to_v3, w)
        twist = Twist()
        twist.linear.x = v[0]
        twist.linear.y = v[1]
        twist.angular.z = v[2]
        return twist

    def twist_to_wheel_velocities(self, twist):
        t = np.array([twist.linear.x, twist.linear.y, twist.angular.z])
        w = np.dot((1.0 / self.R) * self.v3_to_w4, t)
        return tuple(w)
    

def main(args):
    rospy.init_node('base_controller_node', anonymous=True, log_level=rospy.INFO)
    controller = BaseController()
    controller.start()
    rospy.spin()
    

if __name__ == '__main__':
    main(sys.argv)
