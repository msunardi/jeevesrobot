#!/usr/bin/env python

import rospy
import smach
import smach_ros
from diagnostic.msg import Feedback

from diagnostics_node import sleep_time
from diagnostics_node import motor_hz
from diagnostics_node import retry_times


#define state for motor check
class motor_one(smach.State):
        def __init__(self):
                smach.State.__init__(self,outcomes=['success','fail'])
		#self.rate = rospy.Rate(50)
		#self.fb = Feedback()
		self.hz = 0
		self.done = 0
		self.lastn = 0
		self.retry = 0

        def execute(self,userdata):
                rospy.loginfo(' Executing sub-state motor_one')
                # code for output
		sub = rospy.Subscriber('motor_1/feedback', Feedback, self.feedback_callback)	
		while( not self.done):
			self.n = self.hz
			self.count = (self.n - self.lastn)/sleep_time
			self.lastn = self.n
			#print ' n =%d  count=%d' %(self.n,self.count)
			if self.count == motor_hz:
				self.done = 1
			else:
				self.retry = self.retry + 1
				if self.retry < retry_times:
					rospy.sleep(sleep_time)
				else:
					return 'fail'

		return 'success'
			

	def feedback_callback(self,fb):
		self.hz = self.hz + 1 
		

#define state for motor check
class motor_two(smach.State):
        def __init__(self):
                smach.State.__init__(self,outcomes=['success','fail'])
		#self.rate = rospy.Rate(50)
		#self.fb = Feedback()
		self.hz = 0
		self.done = 0
		self.lastn = 0
		self.retry = 0

        def execute(self,userdata):
                rospy.loginfo(' Executing sub-state motor_two')
                # code for output
		sub = rospy.Subscriber('motor_2/feedback', Feedback, self.feedback_callback)	
		while( not self.done):
			self.n = self.hz
			self.count = (self.n - self.lastn)/sleep_time
			self.lastn = self.n
			#print ' n =%d  count=%d' %(self.n,self.count)
			if self.count == motor_hz:
				self.done = 1
			else:
				self.retry = self.retry + 1
				if self.retry < retry_times:
					rospy.sleep(sleep_time)
				else:
					return 'fail'

		return 'success'
			

	def feedback_callback(self,fb):
		self.hz = self.hz + 1 


#define state for motor check
class motor_three(smach.State):
        def __init__(self):
                smach.State.__init__(self,outcomes=['success','fail'])
		#self.rate = rospy.Rate(50)
		#self.fb = Feedback()
		self.hz = 0
		self.done = 0
		self.lastn = 0
		self.retry = 0

        def execute(self,userdata):
                rospy.loginfo(' Executing sub-state motor_three')
                # code for output
		sub = rospy.Subscriber('motor_3/feedback', Feedback, self.feedback_callback)	
		while( not self.done):
			self.n = self.hz
			self.count = (self.n - self.lastn)/sleep_time
			self.lastn = self.n
			#print ' n =%d  count=%d' %(self.n,self.count)
			if self.count == motor_hz:
				self.done = 1
			else:
				self.retry = self.retry + 1
				if self.retry < retry_times:
					rospy.sleep(sleep_time)
				else:
					return 'fail'

		return 'success'
			

	def feedback_callback(self,fb):
		self.hz = self.hz + 1 


#define state for motor check
class motor_four(smach.State):
        def __init__(self):
                smach.State.__init__(self,outcomes=['success','fail'])
		#self.rate = rospy.Rate(50)
		#self.fb = Feedback()
		self.hz = 0
		self.done = 0
		self.lastn = 0
		self.retry = 0

        def execute(self,userdata):
                rospy.loginfo(' Executing sub-state motor_four')
                # code for output
		sub = rospy.Subscriber('motor_4/feedback', Feedback, self.feedback_callback)	
		while( not self.done):
			self.n = self.hz
			self.count = (self.n - self.lastn)/sleep_time
			self.lastn = self.n
			#print ' n =%d  count=%d' %(self.n,self.count)
			if self.count == motor_hz:
				self.done = 1
			else:
				self.retry = self.retry + 1
				if self.retry < retry_times:
					rospy.sleep(sleep_time)
				else:
					return 'fail'

		return 'success'
			

	def feedback_callback(self,fb):
		self.hz = self.hz + 1 


