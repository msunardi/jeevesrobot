#!/usr/bin/env python

import rospy
from diagnostic.msg import Feedback


def motor_pub():
	
	rospy.init_node('motor', anonymous = True )
	pub1 = rospy.Publisher('motor_1/feedback',Feedback,queue_size = 5)
		
	pub2 = rospy.Publisher('motor_2/feedback',Feedback,queue_size = 5)

	pub3 = rospy.Publisher('motor_3/feedback',Feedback,queue_size = 5)

	pub4 = rospy.Publisher('motor_4/feedback',Feedback,queue_size = 5)

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
		
		pub1.publish((fb))
		pub2.publish((fb))
		pub3.publish((fb))
		pub4.publish((fb))

		rate.sleep()	
	
if __name__ == '__main__':
	motor_pub()

	
	
