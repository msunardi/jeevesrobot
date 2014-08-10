#!/usr/bin/env python
import copy
import numpy as np
import sys
import threading
import time
import roslib
import rospy
import PyKDL as kdl
import tf
from waist_control.msg import CommandPosition
from waist_control.msg import FeedbackPosition
import serial
import roboclaw_waist as rc_waist

range_position = 30

class WaistController(threading.Thread):
    
    def __init__(self):
	#initialize subscribers to get the actual positions from the node "waist_position_monitor"
	self.actuator_1_pos_subscriber = rospy.Subscriber("actuator_1/position", FeedbackPosition, self.actuator_1_position_callback)
	self.actuator_2_pos_subscriber = rospy.Subscriber("actuator_2/position", FeedbackPosition, self.actuator_2_position_callback)
	self.actuator_3_pos_subscriber = rospy.Subscriber("actuator_3/position", FeedbackPosition, self.actuator_3_position_callback)
	self.actuator_4_pos_subscriber = rospy.Subscriber("actuator_4/position", FeedbackPosition, self.actuator_4_position_callback)
	self.pos1, self.pos2, self.pos3, self.pos4 = 0,0,0,0	
	#initialize subscribers to get the commanded positions from the node "waist_cmd_generator"	
	self.actuator_1_cmd_subscriber = rospy.Subscriber("actuator_1/cmd", CommandPosition, self.actuator_1_cmd_callback)
	self.actuator_2_cmd_subscriber = rospy.Subscriber("actuator_2/cmd", CommandPosition, self.actuator_2_cmd_callback)
	self.actuator_3_cmd_subscriber = rospy.Subscriber("actuator_3/cmd", CommandPosition, self.actuator_3_cmd_callback)
	self.actuator_4_cmd_subscriber = rospy.Subscriber("actuator_4/cmd", CommandPosition, self.actuator_4_cmd_callback)
	self.cmd1, self.cmd2, self.cmd3, self.cmd4 = 0,0,0,0	
	#initialize subscribers to get the velocity from the node "waist_cmd_generator"	
	self.actuator_1_vel_subscriber = rospy.Subscriber("actuator_1/vel", CommandPosition, self.actuator_1_vel_callback)
	self.actuator_2_vel_subscriber = rospy.Subscriber("actuator_2/vel", CommandPosition, self.actuator_2_vel_callback)
	self.actuator_3_vel_subscriber = rospy.Subscriber("actuator_3/vel", CommandPosition, self.actuator_3_vel_callback)
	self.actuator_4_vel_subscriber = rospy.Subscriber("actuator_4/vel", CommandPosition, self.actuator_4_vel_callback)
	self.vel1, self.vel2, self.vel3, self.vel4 = 20,20,20,20		
	#initialize port to the motor controllers	
	self.r1 = rc_waist.RoboClaw("/dev/waist_actuator_control_right")
	self.r2 = rc_waist.RoboClaw("/dev/waist_actuator_control_left")
	self.counter = 0
	threading.Thread.__init__(self)

    def run(self):
	while not rospy.is_shutdown():
	    print "pos1: %d, pos2: %d, pos3: %d, pos4: %d" %(self.pos1, self.pos2, self.pos3, self.pos4)
	    print "cmd1: %d, cmd2: %d, cmd3: %d, cmd4: %d" %(self.cmd1, self.cmd2, self.cmd3, self.cmd4)
	    if self.pos1 != 0 and self.pos2 != 0 and self.pos3 != 0 and self.pos4 != 0 and self.cmd1 != 0 and self.cmd2 != 0 and self.cmd3 != 0 and self.cmd4 != 0:		# this if loop is checking if cmd and positions are coming in
		
		# The control of the actuators works like this:
		# First, the target corridor is defined by the cmd and the "range_position"	        
		# Then, the feedback value "pos" is compared with the target corridor 		
		
		# control for actuator right back
	        self.rangeup_1 = self.cmd1 + range_position
	        self.rangedown_1 = self.cmd1 - range_position
	        if self.pos1 < self.rangedown_1:
	            #print "I should move up"
	            #self.r1.M1Backward(self.vel1)
		    self.r1.DriveM1(64 - self.vel1)
	        elif self.pos1 > self.rangeup_1:
	            #print "I should move down"
	            #self.r1.M1Forward(self.vel1)
		    self.r1.DriveM1(64 + self.vel1)
	        else:
	            #print "reached position"
	            #self.r1.M1Forward(0)
		    self.r1.DriveM1(64)
	        #control for actuator right front
	        self.rangeup_2 = self.cmd2 + range_position
	        self.rangedown_2 = self.cmd2 - range_position
	        if self.pos2 < self.rangedown_2:
	            #print "I should move up"
	            #self.r1.M2Backward(self.vel2)
		    self.r1.DriveM2(64 - self.vel2)
	        elif self.pos2 > self.rangeup_2:
	            #print "I should move down"
	            #self.r1.M2Forward(self.vel2)
		    self.r1.DriveM2(64 + self.vel2)
	        else:
	            #print "reached position"
	            #self.r1.M2Forward(0)
		    self.r1.DriveM2(64)
	        #control for actuator left front
	        self.rangeup_3 = self.cmd3 + range_position
	        self.rangedown_3 = self.cmd3 - range_position
	        if self.pos3 < self.rangedown_3:
	            #print "I should move up"
	            #self.r2.M1Backward(self.vel3)
		    self.r2.DriveM1(64 - self.vel3)
	        elif self.pos3 > self.rangeup_3:
	            #print "I should move down"
	            #self.r2.M1Forward(self.vel3)
		    self.r2.DriveM1(64 + self.vel3)
	        else:
	            #print "reached position"
	            #self.r2.M1Forward(0)
		    self.r2.DriveM1(64)
	        #control for actuator left back
	        self.rangeup_4 = self.cmd4 + range_position
	        self.rangedown_4 = self.cmd4 - range_position
	        if self.pos4 < self.rangedown_4:
	            print "I should move up"
	            #self.r2.M2Backward(self.vel4)
	            self.r2.DriveM2(64 - self.vel4)
		elif self.pos4 > self.rangeup_4:
	            print "I should move down"
	            #self.r2.M2Forward(self.vel4)
		    self.r2.DriveM2(64 + self.vel4)	        
		else:
	            print "reached position"
	            #self.r2.M2Forward(0)
		    self.r2.DriveM2(64)
	    else:
		print "don't have feedback or command values"
	    
	    # will constantly reset the port to roboclaw "waist_actuator_control_left" every 1000th threading
	    # this was done because this roboclaw randomly said "Goodbye..." without any obvious reason
	    if self.counter < 1000:
		self.counter = self.counter + 1
	    else:
		self.r2 = rc_waist.RoboClaw("/dev/waist_actuator_control_left")
		self.counter = 0

    def actuator_1_position_callback(self, feedback_position):
        self.pos1 = feedback_position.feedback_position
    
    def actuator_2_position_callback(self, feedback_position):
        self.pos2 = feedback_position.feedback_position
	    
    def actuator_3_position_callback(self, feedback_position):
        self.pos3 = feedback_position.feedback_position
	
    def actuator_4_position_callback(self, feedback_position):
        self.pos4 = feedback_position.feedback_position
	
    def actuator_1_cmd_callback(self, commanded_position):
        self.cmd1 = commanded_position.commanded_position
	
    def actuator_2_cmd_callback(self, commanded_position):
        self.cmd2 = commanded_position.commanded_position
	
    def actuator_3_cmd_callback(self, commanded_position):
        self.cmd3 = commanded_position.commanded_position
	
    def actuator_4_cmd_callback(self, commanded_position):
        self.cmd4 = commanded_position.commanded_position

    def actuator_1_vel_callback(self, commanded_position):
        self.vel1 = commanded_position.commanded_position
	
    def actuator_2_vel_callback(self, commanded_position):
        self.vel2 = commanded_position.commanded_position
	
    def actuator_3_vel_callback(self, commanded_position):
        self.vel3 = commanded_position.commanded_position
	
    def actuator_4_vel_callback(self, commanded_position):
        self.vel4 = commanded_position.commanded_position
		
            
def main(args):
    rospy.init_node('waist_controller_node', anonymous=True, log_level=rospy.INFO)
    controller = WaistController()
    controller.start()
    rospy.spin()
    

if __name__ == '__main__':
    main(sys.argv)
