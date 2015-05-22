#!/usr/bin/env python

import rospy
from smach_jeeves.msg import Feedback


def motor_pub():
	
	rospy.init_node('motor', anonymous = True )
	pub = rospy.Publisher('/feedback',Feedback,queue_size = 5)
	
	fb = Feedback()
	fb.motor_current = 1.2
	fb.motor_power = 63.5
		
	fb.commanded_velocity = 5.1
	fb.measured_velocity = 3.4	
	fb.measured_position = 3.3
	
	fb.supply_voltage = 4.3
	fb.supply_current = 1.1
	
	fb.motor_temperature = 345.7

	fb.channel_temperature = 300.2
	
	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		
		pub.publish((fb))
		rate.sleep()	
	
if __name__ == '__main__':
	motor_pub()

	
	
