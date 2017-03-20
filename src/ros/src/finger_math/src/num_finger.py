#!/usr/bin/env python
import rospy
from time import sleep
import string
import threading
from std_msgs.msg import String
from ros_pololu.msg import MotorCommand
from random import random

class Num_Finger(threading.Thread):

	def __init__(self):
		self.sub = rospy.Subscriber("finger_solved", String, self.callback)
		self.finger_pub = rospy.Publisher('/pololu/command', MotorCommand, queue_size=10)

		self.r_pinkie = MotorCommand()
		self.r_pinkie.joint_name = 'r_hand_to_pinkie'
		self.r_pinkie.speed = 1.0
		self.r_pinkie.acceleration = 1.0

		self.r_thumb = MotorCommand()
		self.r_thumb.joint_name = 'r_hand_to_thumb'
		self.r_thumb.speed = 1.0
		self.r_thumb.acceleration = 1.0
		
		self.r_index = MotorCommand()
		self.r_index.joint_name = 'r_hand_to_index'
		self.r_index.speed = 1.0
		self.r_index.acceleration = 1.0
	
		self.r_middle = MotorCommand()
		self.r_middle.joint_name = 'r_hand_to_middle'
		self.r_middle.speed = 1.0
		self.r_middle.acceleration = 1.0

		self.r_ring = MotorCommand()
		self.r_ring.joint_name = 'r_hand_to_ring'
		self.r_ring.speed = 1.0
		self.r_ring.acceleration = 1.0

		self.l_pinkie = MotorCommand()
		self.l_pinkie.joint_name = 'l_hand_to_pinkie'
		self.l_pinkie.speed = 0.8
		self.l_pinkie.acceleration = 0.2

		self.l_ring = MotorCommand()
		self.l_ring.joint_name = 'l_hand_to_ring'
		self.l_ring.speed = 0.8
		self.l_ring.acceleration = 0.2

		self.l_middle = MotorCommand()
		self.l_middle.joint_name = 'l_hand_to_middle'
		self.l_middle.speed = 0.8
		self.l_middle.acceleration = 0.2

		self.l_index = MotorCommand()
		self.l_index.joint_name = 'l_hand_to_index'
		self.l_index.speed = 0.8
		self.l_index.acceleration = 0.3

		self.l_thumb = MotorCommand()
		self.l_thumb.joint_name = 'l_hand_to_thumb'
		self.l_thumb.speed = 0.8
		self.l_thumb.acceleration = 0.3

		self.fingers = [self.l_thumb, self.l_index, self.l_middle, self.l_ring, self.l_pinkie, \
                                self.r_index, self.r_thumb, self.r_middle, self.r_ring, self.r_pinkie]
		self.ALL = range(5)
		self.ALL += [7, 6, 5, 8, 9]

		threading.Thread.__init__(self)
		self.sleeper = rospy.Rate(10)

	def callback(self, data):
		numbers = data.data
		rospy.loginfo("Got data: %s" % numbers)
		# if(number == '1'):
		# 	self.r_hand_to_thumb.position = 0.68 #0
		# 	self.r_hand_to_index.position = 0.75 #45
		# 	self.r_hand_to_middle.position = 0.77 #45
		# 	self.r_hand_to_ring.position = 0.4 #45
		# 	self.r_hand_to_pinkie.position = 0.75 #45
			
		# elif(number == '2'):
		# 	self.r_hand_to_thumb.position = 0.0
		# 	self.r_hand_to_index.position = 0.0
		# 	self.r_hand_to_middle.position = 0.0 #45
		# 	self.r_hand_to_ring.position = 0.0 #45
		# 	self.r_hand_to_pinkie.position = 0.0 #45
		
		# elif(number == '3'):
		# 	self.r_hand_to_thumb.position = 0.75
		# 	self.r_hand_to_index.position = 0.0
		# 	self.r_hand_to_middle.position = 0.0
		# 	self.r_hand_to_pinkie.position = 0.75
		# 	self.r_hand_to_ring.position = 0.0
		
		# elif(number == '4'):
		# 	self.r_hand_to_thumb.position = 0.75
		# 	self.r_hand_to_index.position = 0.0
		# 	self.r_hand_to_middle.position = 0.0
		# 	self.r_hand_to_ring.position = 0.0
		# 	self.r_hand_to_pinkie.position = 0.0
		
		# elif(number == '5'):
		# 	self.r_hand_to_thumb.position = 45
		# 	self.(number == '1'):
		# 	self.r_hand_to_thumb.position = 0.68 #0
		# 	self.r_hand_to_index.position = 0.75 #45
		# 	self.r_hand_to_middle.position = 0.77 #45
		# 	self.r_hand_to_ring.position = 0.4 #45
		# 	self.r_hand_to_pinkie.position = 0.75 #45
			
		# elif(number == '2'):
		# 	self.r_hand_to_thumb.position = 0.0
		# 	self.r_hand_to_index.position = 0.0
		# 	self.r_hand_to_middle.position = 0.0 #45
		# 	self.r_hand_to_ring.position = 0.0 #45
		# 	self.r_hand_to_pinkie.position = 0.0 #45
		
		# elif(number == '3'):
		# 	self.r_hand_to_thumb.position = 0.75
		# 	self.r_hand_to_index.position = 0.0
		# 	self.r_hand_to_middle.position = 0.0
		# 	self.r_hand_to_pinkie.position = 0.75
		# 	self.r_hand_to_ring.position = 0.0
		
		# elif(number == '4'):
		# 	self.r_hand_to_thumb.position = 0.75
		# 	self.r_hand_to_index.position = 0.0
		# 	self.r_hand_to_middle.position = 0.0
		# 	self.r_hand_to_ring.position = 0.0
		# 	self.r_hand_to_pinkie.position = 0.0
		
		# elif(number == '5'):
		# 	self.r_hand_to_thumb.position = 45
		# 	self.r_hand_to_index.position = 45
		# 	self.r_hand_to_middle.position = 45
		# 	self.r_hand_to_ring.position = 45
		# 	self.r_hand_to_pinkie.position = 45
			
		# 	self.r_hand_to_thumb.position = 0
		# 	self.r_hand_to_index.position = 0
		# 	self.r_hand_to_middle.position = 0
		# 	self.r_hand_to_ring.position = 0
		# 	self.r_hand_to_pinkie.position = 0
		
		# elif(number == 0):
		# 	self.r_hand_to_thumb.position = 45
		# 	self.r_hand_to_index.position = 45
		# 	self.r_hand_to_middle.position = 0
		# 	self.r_hand_to_ring.position = 0
		# 	self.r_hand_to_pinkie.position = 0

		# else:
		# 	self.r_hand_to_thumb.position = 45
		# 	self.r_hand_to_index.position = 0
		# 	self.r_hand_to_middle.position = 45
		# 	self.r_hand_to_ring.position   = 45
		# 	self.r_hand_to_pinkie.position = 45r_hand_to_index.position = 45
		# 	self.r_hand_to_middle.position = 45
		# 	self.r_hand_to_ring.position = 45
		# 	self.r_hand_to_pinkie.position = 45
			
		# 	self.r_hand_to_thumb.position = 0
		# 	self.r_hand_to_index.position = 0
		# 	self.r_hand_to_middle.position = 0
		# 	self.r_hand_to_ring.position = 0
		# 	self.r_hand_to_pinkie.position = 0
		
		# elif(number == 0):
		# 	self.l_hand_to_thumb.position = 45
		# 	self.l_hand_to_index.position = 45
		# 	self.l_hand_to_middle.position = 0
		# 	self.l_hand_to_ring.position = 0
		# 	self.l_hand_to_pinkie.position = 0

		# else:
		# 	self.l_hand_to_thumb.position = 45
		# 	self.l_hand_to_index.position = 0
		# 	self.l_hand_to_middle.position = 45
		# 	self.l_hand_to_ring.position   = 45
		# 	self.l_hand_to_pinkie.position = 45

		
		positions = [0.75, 0.75, 0.75, 0.75, 0.75]
		closed = self.ALL
		opens = []
		self.close_fingers(closed)
		sleep(0.5)
		self.open_fingers(self.ALL)
		sleep(0.5)
		self.close_fingers(self.ALL)
		closed = []

		if numbers == '10':
			opens = self.ALL
			self.open_fingers(opens)
			sleep(2)
			self.close_fingers(self.ALL)
			return

		for number in numbers:
			if(number == '1'):
				# closed = [4, 3, 2, 0]
				# opens = [0]
				opens = [1]

			elif(number == '2'):
				# closed = [4, 3, 0]
				# opens = [0, 1]
				opens = [1, 2]

			elif(number == '3'):
				# closed = [2, 3]
				opens = [1, 2, 3]

			elif(number == '4'):
				# closed = [0]
				opens = [1,2,3,4]

			elif(number == '5'):
				opens = range(5)
			
			#elif(number == '6'):
				opens = range(5)
				#opens += [8]

			#elif(number == '7'):
			#	opens = range(5)
			#	opens += [9, 8, 7]
			#	closed = [9]

			#elif(number == '8'):
			#	opens = range(5)
			#	opens += [9,8,7,6]
			#	closed = [9]

			#elif(number == '9'):
			#	opens = range(5)
			#	opens += [9,8,7,6,5]
			#	closed = [9]
			elif number in '6789':
				opens = range(int(number))
			elif(number == '0'):
				opens = [2, 3, 4]

			else:
				pass

			# self.close_fingers(closed)
			self.open_fingers(opens)
			self.close_fingers(closed)
			# for pos in zip(fingers, positions):
			# 	pos[0].position = pos[1]
			# 	self.finger_pub.publish(pos[0])
			# 	sleep(0.05)

			sleep(1.75)
			# for p in fingers:
			# 	p.position = 0.75
			# 	self.finger_pub.publish(p)
			# 	sleep(0.05)
			self.close_fingers(self.ALL)
		sleep(2*random())	
		self.open_fingers(self.ALL)
		self.close_fingers(self.ALL)

	def close_fingers(self, close):
		for c in close:
			self.fingers[c].position = 0.0
			self.finger_pub.publish(self.fingers[c])
			sleep(0.1)
	
	def open_fingers(self, opens):
		for o in opens:
			self.fingers[o].position = 0.70
			rospy.loginfo(self.fingers[o])
			self.finger_pub.publish(self.fingers[o])
			sleep(0.1)	

	def run(self):

		while not rospy.is_shutdown():
			self.sleeper.sleep()

def main():
	rospy.init_node('num_finger', anonymous=True)
	finger_math = Num_Finger()
	finger_math.start()
	rospy.spin()

if __name__ == '__main__':
	main()
