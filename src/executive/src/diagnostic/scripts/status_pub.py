#!/usr/bin/env python

import rospy
from smach_jeeves.msg import Status


def status_pub():
	
	rospy.init_node('status', anonymous = True )
	pub = rospy.Publisher('/status',Status,queue_size = 5)
	
	stat = Status()
	stat.status = 129
	
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		
		pub.publish((stat))
		rate.sleep()	
	
if __name__ == '__main__':
	status_pub()

	
	
