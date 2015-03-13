#!/usr/bin/env python

## 2015.02.28, Peter D: replaced 8-spaced TABS with standard Python's 
## 4-space indentation, using automatic feature of Eric IDE.
## Result: 8-spaced TAB was replaced with two "4-spaced" standard Python's indent.


import copy
import numpy as np
import sys
import threading
import time
from collections import deque
import roslib
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from roboteq_msgs.msg import Command as MotorCommand
from roboteq_msgs.msg import Feedback as MotorFeedback
import PyKDL as kdl
import tf
from waist_control.msg import CommandPosition
from sensor_msgs.msg import Joy

x_bend_min = -18
x_bend_max = 18
y_bend_min = -28
y_bend_max = 28
tuning_factor_x = 2.9
tuning_factor_y = 3.1
tuning_factor_z = 4

class WaistMotionPlanner(threading.Thread):
    
    def __init__(self):
        self.cmd_x_publisher = rospy.Publisher("pose_x", CommandPosition)       
        self.cmd_y_publisher = rospy.Publisher("pose_y", CommandPosition)
        self.cmd_z_publisher = rospy.Publisher("pose_z", CommandPosition)
        self.subscriber = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.cmd_vel_incoming = deque()
        self.joy_subscriber = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.axes = []
        self.buttons = []
        # cmd_x is the angular command according to the x-axis  
        self.cmd_x = 0          # potentially reachable values are between -18 and 18 degree
        # cmd_y is the angular command according to the y-axis  
        self.cmd_y = 0          # potentially reachable values are between -28 and 28 degree
        # cmd_z is the height command   
        self.cmd_z = 45         # boundaries are set between 0mm and 90mm
        self.lock = threading.Lock()
        threading.Thread.__init__(self)

    def run(self):
        
        while not rospy.is_shutdown():
            
            # get a new command from the game pad or the base
            with self.lock:
                # reading the topic "/vmd_vel"
                if len(self.cmd_vel_incoming) != 0:
                    twist = self.cmd_vel_incoming.popleft()
                else:
                    continue

                # reading the commands from the game pad                
                axes = copy.copy(self.axes)
                buttons = copy.copy(self.buttons)
            if (len(axes) == 0) or (len(buttons) == 0):
                continue

            #print twist                # displays the /cmd_vel topic
            
            # convert the incoming velocity vector into bending,
            #rospy.logdebug("cmd_vel message received: " + str(twist))
            
            # create an array w with the incoming velocities in x and y direction
            # and convert it into commands for the waist
            w = [twist.linear.x, twist.linear.y]
            self.base_cmd_x_float = w[1] * -20  # 20 is just a tuning factor for the game pad
            self.base_cmd_y_float = w[0] * 20
            self.base_cmd_x = int(self.base_cmd_x_float)
            self.base_cmd_y = int(self.base_cmd_y_float)
            #print "bending x: %d (incoming)" %w[1]
            #print "bending x: %d (cmd)" %self.base_cmd_x
            #print "bending y: %d (incoming)" %w[0]
            #print "bending y: %d (cmd)" %self.base_cmd_y
            
            # set commands directly with the game pad if no commands are coming from the base
            if self.base_cmd_x == 0 and self.base_cmd_y == 0:
                
                # commands for manually control the waist with the direction pad
                # First, there is a control if the actuators are within their range (bend_max/min)
                # Then, cmd_x/y are either increased or decreased
                # The tuning_factors are needed for a smooth control
                # If the speed for the actuators are changed, the tuning factors needs to be changed, too.              
                if -1 == axes[-1]: # direction pad upwards
                    if self.cmd_y < y_bend_max:
                        self.cmd_y += tuning_factor_y
                    else:
                        continue
                if 1 == axes[-1]: # direction pad downwards
                    if self.cmd_y > y_bend_min:
                        self.cmd_y -= tuning_factor_y
                    else:
                        continue
                if -1 == axes[-2]: # direction pad left
                    if self.cmd_x > x_bend_min:
                        self.cmd_x -= tuning_factor_x
                    else:
                        continue             
                if 1 == axes[-2]: # direction pad right
                    if self.cmd_x < x_bend_max:
                        self.cmd_x += tuning_factor_x
                    else:
                        continue 
                if 1 == axes [3]: # right analog stick upwards
                    if self.cmd_z < 90:
                        self.cmd_z += tuning_factor_z
                    else:
                        continue
                if -1 == axes [3]: # right analog stick downwards
                    if self.cmd_z > 0:
                        self.cmd_z -= tuning_factor_z
                    else:
                        continue
                
                # commands for general bending poses
                # more poses can be added by assigning buttons and commanded positions in x, y, and z
                
                # this is a forward bending pose 
                if 1 == buttons [0]: # button marked with a pink square
                    self.cmd_x = 0
                    self.cmd_y = -25
                    self.cmd_z = 45

                # publishing the cmd to the topis "pose_x", "pose_y" and "pose_z"
                self.cmd_x_publisher.publish(CommandPosition(self.cmd_x))
                self.cmd_y_publisher.publish(CommandPosition(self.cmd_y))
                self.cmd_z_publisher.publish(CommandPosition(self.cmd_z)) 
            
            # if commands are coming from the base via "/cmd_vel" topic
            # ignore direct commands from the game pad and create commands for the waist according to the base velocity
            else:
                # set waist commands according the the base velocity
                self.cmd_x = self.base_cmd_x
                self.cmd_y = self.base_cmd_y
                if self.cmd_x < x_bend_min: 
                    self.cmd_x_publisher.publish(CommandPosition(x_bend_min))
                else:
                    self.cmd_x_publisher.publish(CommandPosition(self.cmd_x))
                if self.cmd_x > x_bend_max: 
                    self.cmd_x_publisher.publish(CommandPosition(x_bend_max))
                else:
                    self.cmd_x_publisher.publish(CommandPosition(self.cmd_x))
                if self.cmd_y < y_bend_min: 
                    self.cmd_y_publisher.publish(CommandPosition(y_bend_min))
                else:
                    self.cmd_y_publisher.publish(CommandPosition(self.cmd_y))
                if self.cmd_y > y_bend_max: 
                    self.cmd_y_publisher.publish(CommandPosition(y_bend_max))
                else:
                    self.cmd_y_publisher.publish(CommandPosition(self.cmd_y))
                self.cmd_z_publisher.publish(CommandPosition(self.cmd_z))
                # reset the bending commands so that the upper body returns to 
                # a straight position when the base is no longer moving         
                self.cmd_x, self.cmd_y = 0, 0 
            
                print "x: %d; y: %d; z: %d" %(self.cmd_x, self.cmd_y, self.cmd_z)
                               
    def cmd_vel_callback(self, twist_msg):
        with self.lock:
            self.cmd_vel_incoming.append(twist_msg)

    def joy_callback(self, data):
        with self.lock:
            self.axes = list(data.axes)
            self.buttons = list(data.buttons)


def main(args):
    rospy.init_node('waist_motion_planner_joy_node', anonymous=True, log_level=rospy.INFO)
    controller = WaistMotionPlanner()
    controller.start()
    rospy.spin()
    

if __name__ == '__main__':
    main(sys.argv)
