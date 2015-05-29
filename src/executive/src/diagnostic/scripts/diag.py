#!/usr/bin/env python

import rospy
import smach
import smach_ros
from motor_states import *
from status_states import *

from diagnostic.msg import LaserScan


sleep_time = 3		  # number of seconds to sleep
retry_times = 5		  # number of retries
motor_hz = 50		  # frequency of motor feedback	 
laser_hz = 75		  # frequency of laserscan
status_hz = 10		  # frequency of status 

#define state for laser check
class laser_check(smach.State):
        def __init__(self):
                smach.State.__init__(self,outcomes=['success','fail'])
		#self.rate = rospy.Rate(50)
		#self.ls = LaserScan()
		self.hz = 0
		self.done = 0
		self.lastn = 0
		self.retry = 0

        def execute(self,userdata):
                rospy.loginfo(' Executing state laser_check')
                # code for output
		sub = rospy.Subscriber('scan', LaserScan, self.laserscan_callback)	
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
		


def main():
        rospy.init_node('smach_diagnostics_test')

        # create a SMACH state machine
        sm = smach.StateMachine(outcomes=['SUCCESS','FAIL'])

        # open the container
        with sm:
                
		sm_motor = smach.StateMachine(outcomes=['success','fail'])
		
		with sm_motor:
			smach.StateMachine.add('MOTOR_ONE', motor_one(), transitions={'success':'MOTOR_TWO','fail':'fail'})				
			smach.StateMachine.add('MOTOR_TWO', motor_two(), transitions={'success':'MOTOR_THREE','fail':'fail'})				
			smach.StateMachine.add('MOTOR_THREE', motor_three(), transitions={'success':'MOTOR_FOUR','fail':'fail'})				
			smach.StateMachine.add('MOTOR_FOUR', motor_four(), transitions={'success':'success','fail':'fail'})				
		

		# Add states to the container
                smach.StateMachine.add('MOTOR_CHECK', sm_motor, transitions={'success':'LASER_CHECK','fail':'FAIL'})
		smach.StateMachine.add('LASER_CHECK', laser_check(), transitions={'success':'STATUS_CHECK','fail':'FAIL'})

		sm_status = smach.StateMachine(outcomes=['success','fail'])
		
		with sm_status:
			smach.StateMachine.add('STATUS_FRONT_CHECK', status_front_check(), transitions={'success':'STATUS_REAR_CHECK','fail':'fail'})
			smach.StateMachine.add('STATUS_REAR_CHECK', status_rear_check(), transitions={'success':'success','fail':'fail'})
		
		smach.StateMachine.add('STATUS_CHECK', sm_status, transitions={'success':'SUCCESS','fail':'FAIL'})

		# Create and start the introspection server
		sis = smach_ros.IntrospectionServer('diagnostics_server', sm, '/SM_ROOT')
		sis.start()


        # Execute SMACH plan
        outcome = sm.execute()

	# Wait for ctrl-c to stop the application
	#rospy.spin()
	sis.stop()

	
if __name__ == '__main__':
        main()
