#!/usr/bin/env python

import rospy
import smach
import smach_ros
from smach_jeeves.msg import Feedback
from smach_jeeves.msg import LaserScan
from smach_jeeves.msg import Status


sleep_time = 5		  # number of seconds to sleep
retry_times = 5		  # number of retries
motor_hz = 50		  # frequency of motor feedback	 
laser_hz = 75		  # frequency of laserscan
status_hz = 10		  # frequency of status 

#define state for motor check
class motor_check(smach.State):
        def __init__(self):
                smach.State.__init__(self,outcomes=['success','fail'])
		#self.rate = rospy.Rate(50)
		self.fb = Feedback()
		self.hz = 0
		self.done = 0
		self.lastn = 0
		self.retry = 0

        def execute(self,userdata):
                rospy.loginfo(' Executing state motor_check')
                # code for output
		sub = rospy.Subscriber('/feedback', Feedback, self.feedback_callback)	
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
		
#define state for laser check
class laser_check(smach.State):
        def __init__(self):
                smach.State.__init__(self,outcomes=['success','fail'])
		#self.rate = rospy.Rate(50)
		self.ls = LaserScan()
		self.hz = 0
		self.done = 0
		self.lastn = 0
		self.retry = 0

        def execute(self,userdata):
                rospy.loginfo(' Executing state laser_check')
                # code for output
		sub = rospy.Subscriber('/laserscan', LaserScan, self.laserscan_callback)	
		while( not self.done):
			self.n = self.hz
			self.count = (self.n - self.lastn)/sleep_time
			self.lastn = self.n
			#print ' n =%d  count=%d' %(self.n,self.count)
			if self.count == laser_hz:
				self.done = 1
			else:
				self.retry = self.retry + 1
				if self.retry < retry_times:
					rospy.sleep(sleep_time)
				else:
					return 'fail'

		return 'success'
			

	def laserscan_callback(self,ls):
		self.hz = self.hz + 1 
		
#define state for motor controller status check
class status_check(smach.State):
        def __init__(self):
                smach.State.__init__(self,outcomes=['success','fail'])
	
		self.stat = Status()
		self.hz = 0
		self.done = 0
		self.lastn = 0
		self.retry = 0
		self.s = 0

        def execute(self,userdata):
                rospy.loginfo(' Executing state status_check')
                # code for output
		sub = rospy.Subscriber('motor_controller_front/status', Status, self.status_callback)	
		while( not self.done):
			self.n = self.hz
			self.count = (self.n - self.lastn)/sleep_time
			self.lastn = self.n
			#print ' n =%d  count=%d' %(self.n,self.count)
			if self.count == status_hz:
				self.done = 1
			else:
				self.retry = self.retry + 1
				if self.retry < retry_times:
					rospy.sleep(sleep_time)
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





def main():
        rospy.init_node('smach_diagnostics_test')

        # create a SMACH state machine
        sm = smach.StateMachine(outcomes=['SUCCESS','FAIL'])

        # open the container
        with sm:
                # Add states to the container
                smach.StateMachine.add('MOTOR_CHECK', motor_check(), transitions={'success':'LASER_CHECK','fail':'FAIL'})
		smach.StateMachine.add('LASER_CHECK', laser_check(), transitions={'success':'STATUS_CHECK','fail':'FAIL'})
		smach.StateMachine.add('STATUS_CHECK', status_check(), transitions={'success':'SUCCESS','fail':'FAIL'})

        # Execute SMACH plan
        outcome = sm.execute()

	
	
if __name__ == '__main__':
        main()
