#!/usr/bin/env python

import rospy
import smach
import smach_ros
from diagnostic.msg import Status

import diag




#define state for motor controller front status check
class status_front_check(smach.State):
        def __init__(self):
                smach.State.__init__(self,outcomes=['success','fail'])
	
		self.stat = Status()
		self.hz = 0
		self.done = 0
		self.lastn = 0
		self.retry = 0
		self.s = 0
		self.sleep_time = diag.sleep_time
		self.status_hz = diag.status_hz
		self.retry_times = diag.retry_times

				
        def execute(self,userdata):
                rospy.loginfo(' Executing sub-state status_front_check')
                # code for output
		sub = rospy.Subscriber('motor_controller_front/status', Status, self.status_callback)	
		while( not self.done):
			self.n = self.hz
			self.count = (self.n - self.lastn)/self.sleep_time
			self.lastn = self.n
			#print ' n =%d  count=%d' %(self.n,self.count)
			if self.count == self.status_hz:
				self.done = 1
			else:
				self.retry = self.retry + 1
				if self.retry < self.retry_times:
					rospy.sleep(self.sleep_time)
				else:
					return 'fail'
		
		if (self.s == 129):
			return 'success'
		else:		
			return 'fail'

	def status_callback(self,stat):
		self.hz = self.hz + 1 
		self.s = stat.status
		#rospy.loginfo(self.s)


#define state for motor controller rear status check
class status_rear_check(smach.State):
        def __init__(self):
                smach.State.__init__(self,outcomes=['success','fail'])
	
		self.stat = Status()
		self.hz = 0
		self.done = 0
		self.lastn = 0
		self.retry = 0
		self.s = 0
		self.sleep_time = diag.sleep_time
		self.status_hz = diag.status_hz
		self.retry_times = diag.retry_times

        def execute(self,userdata):
                rospy.loginfo(' Executing sub-state status_front_check')
                # code for output
		sub = rospy.Subscriber('motor_controller_rear/status', Status, self.status_callback)	
		while( not self.done):
			self.n = self.hz
			self.count = (self.n - self.lastn)/self.sleep_time
			self.lastn = self.n
			#print ' n =%d  count=%d' %(self.n,self.count)
			if self.count == self.status_hz:
				self.done = 1
			else:
				self.retry = self.retry + 1
				if self.retry < self.retry_times:
					rospy.sleep(self.sleep_time)
				else:
					return 'fail'
		
		if (self.s == 129):
			return 'success'
		else:		
			return 'fail'

	def status_callback(self,stat):
		self.hz = self.hz + 1 
		self.s = stat.status
		#rospy.loginfo(self.s)




