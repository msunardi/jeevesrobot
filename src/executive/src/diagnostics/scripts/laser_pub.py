#!/usr/bin/env python

import rospy
from diagnostics.msg import LaserScan

def laser_pub():
	
	rospy.init_node('laser', anonymous = True )
	pub = rospy.Publisher('scan',LaserScan,queue_size = 5)
	ls = LaserScan()
	ls.angle_min = 45.9
	ls.angle_max = 34.4
	ls.angle_increment = 23.4
	ls.time_increment = 22.5
	ls.scan_time = 54
	ls.range_min = 50.3
	ls.range_max = 44.3
	ls.ranges = [34, 45, 50.3]
	ls.intensities = [ 21.2, 33.3, 32]
	

	
	rate = rospy.Rate(75)
	while not rospy.is_shutdown():
		
		pub.publish((ls))
		rate.sleep()	

if __name__ == '__main__':
	laser_pub()

