#!/usr/bin/env python
import rospy
from waist_control.msg import CommandPosition
import sys
import threading
import math

conv_factor_mm_to_pot = 8.8888888889	# 800 / 90
# defines the area between which the actuators should move
min_pos = 100
max_pos = 900
# measures of the actuators location
width = 265		# in mm
depth = 165		# in mm

class WaistController(threading.Thread):
    def __init__(self):
	self.actuator_1_cmd_publisher = rospy.Publisher("actuator_1/cmd", CommandPosition)
	self.actuator_2_cmd_publisher = rospy.Publisher("actuator_2/cmd", CommandPosition)
	self.actuator_3_cmd_publisher = rospy.Publisher("actuator_3/cmd", CommandPosition)
	self.actuator_4_cmd_publisher = rospy.Publisher("actuator_4/cmd", CommandPosition)
	self.cmd_1 = 200
	self.cmd_2 = 200
	self.cmd_3 = 200
	self.cmd_4 = 200
	self.actuator_1_vel_publisher = rospy.Publisher("actuator_1/vel", CommandPosition)
	self.actuator_2_vel_publisher = rospy.Publisher("actuator_2/vel", CommandPosition)
	self.actuator_3_vel_publisher = rospy.Publisher("actuator_3/vel", CommandPosition)
	self.actuator_4_vel_publisher = rospy.Publisher("actuator_4/vel", CommandPosition)
	self.vel_1 = 30
	self.vel_2 = 30
	self.vel_3 = 30
	self.vel_4 = 30
	self.cmd_x_subscriber = rospy.Subscriber("pose_x", CommandPosition, self.pose_x_callback)
	self.cmd_y_subscriber = rospy.Subscriber("pose_y", CommandPosition, self.pose_y_callback)
	self.cmd_z_subscriber = rospy.Subscriber("pose_z", CommandPosition, self.pose_z_callback)
	self.cmd_x_degree = 0			# in degree
	self.cmd_y_degree = 0			# in degree
	self.cmd_z_mm = 10			# in mm
        threading.Thread.__init__(self)

    def run(self):
        while not rospy.is_shutdown():
	    # convert mm values (0 - 90) to potentiometer range (100 - 900)
	    # create a radiant value of the incoming angular command
	    # this radiant value is a float variable
	    self.cmd_rad_x = math.radians(self.cmd_x_degree)
	    self.cmd_rad_y = math.radians(self.cmd_y_degree)
	    # calculate the individual actuator command positions in mm
	    self.cmd_1_float = self.cmd_z_mm - (width/2) * math.tan(self.cmd_rad_x) - (depth/2) * math.tan(self.cmd_rad_y)
	    self.cmd_2_float = self.cmd_z_mm - (width/2) * math.tan(self.cmd_rad_x) + (depth/2) * math.tan(self.cmd_rad_y)
	    self.cmd_3_float = self.cmd_z_mm + (width/2) * math.tan(self.cmd_rad_x) + (depth/2) * math.tan(self.cmd_rad_y)
	    self.cmd_4_float = self.cmd_z_mm + (width/2) * math.tan(self.cmd_rad_x) - (depth/2) * math.tan(self.cmd_rad_y)
	    # convert mm values (0 - 90) to potentiometer range (100 - 900)
	    self.cmd_1_conv_float = self.cmd_1_float * conv_factor_mm_to_pot
	    self.cmd_1_conv = int(self.cmd_1_conv_float)
	    self.cmd_1 = self.cmd_1_conv + min_pos		# final cmd height converted to the potentiometer values
	    self.cmd_2_conv_float = self.cmd_2_float * conv_factor_mm_to_pot
	    self.cmd_2_conv = int(self.cmd_2_conv_float)
	    self.cmd_2 = self.cmd_2_conv + min_pos		# final cmd height converted to the potentiometer values
	    self.cmd_3_conv_float = self.cmd_3_float * conv_factor_mm_to_pot
	    self.cmd_3_conv = int(self.cmd_3_conv_float)
	    self.cmd_3 = self.cmd_3_conv + min_pos		# final cmd height converted to the potentiometer values
	    self.cmd_4_conv_float = self.cmd_4_float * conv_factor_mm_to_pot
	    self.cmd_4_conv = int(self.cmd_4_conv_float)
	    self.cmd_4 = self.cmd_4_conv + min_pos		# final cmd height converted to the potentiometer values
	    # transform individual actuator command positions to int so that waist_controller can use them
	    #print "cmd_1: %d" %self.cmd_1
	    #print "cmd_2: %d" %self.cmd_2
	    #print "cmd_3: %d" %self.cmd_3
	    #print "cmd_4: %d" %self.cmd_4

	    # check if cmds are in a reachable position for the actuators
	    if self.cmd_1 >= min_pos and self.cmd_1 <= max_pos and self.cmd_2 >= min_pos and self.cmd_2 <= max_pos and self.cmd_3 >= min_pos and self.cmd_3 <= max_pos and self.cmd_4 >= min_pos and self.cmd_4 <= max_pos:
		# publish cmds to topic "actuator_.../cmd"
		self.actuator_1_cmd_publisher.publish(CommandPosition(self.cmd_1))
                self.actuator_2_cmd_publisher.publish(CommandPosition(self.cmd_2))
                self.actuator_3_cmd_publisher.publish(CommandPosition(self.cmd_3))
                self.actuator_4_cmd_publisher.publish(CommandPosition(self.cmd_4))
	        
		# publish vels to topic "actuator_.../vel"
	        self.actuator_1_vel_publisher.publish(CommandPosition(self.vel_1))
                self.actuator_2_vel_publisher.publish(CommandPosition(self.vel_2))
                self.actuator_3_vel_publisher.publish(CommandPosition(self.vel_3))
                self.actuator_4_vel_publisher.publish(CommandPosition(self.vel_4))
	        
	    else:
		print "cannot reach this position"

    def pose_x_callback(self, commanded_position):
	self.cmd_x_degree = commanded_position.commanded_position

    def pose_y_callback(self, commanded_position):
	self.cmd_y_degree = commanded_position.commanded_position

    def pose_z_callback(self, commanded_position):
	self.cmd_z_mm = commanded_position.commanded_position
            
def main(args):
    rospy.init_node('waist_cmd_generator_node', anonymous=True, log_level=rospy.INFO)
    pub = WaistController()
    pub.start()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
