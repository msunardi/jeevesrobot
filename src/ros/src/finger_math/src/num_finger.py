#!/usr/bin/env python
import rospy
from time import sleep
import string
import threading
from std_msgs.msg import String
from ros_pololu.msg import MotorCommand
from random import random

import redis

class Num_Finger(threading.Thread):

	def __init__(self):
		self.sub = rospy.Subscriber("finger_solved", String, self.callback)
		self.pub = rospy.Publisher('/pololu/command', MotorCommand, queue_size=10)

		self.r_thumb = MotorCommand()
		self.r_thumb.joint_name = 'r_hand_to_thumb'
		self.r_thumb.speed = 0.8
		self.r_thumb.acceleration = 0.3
		
		self.r_index = MotorCommand()
		self.r_index.joint_name = 'r_hand_to_index'
		self.r_index.speed = 1.0
		self.r_index.acceleration = 0.3
	
		self.r_middle = MotorCommand()
		self.r_middle.joint_name = 'r_hand_to_middle'
		self.r_middle.speed = 1.0
		self.r_middle.acceleration = 0.3

		self.r_ring = MotorCommand()
		self.r_ring.joint_name = 'r_hand_to_ring'
		self.r_ring.speed = 1.0
		self.r_ring.acceleration = 0.3

		self.r_pinkie = MotorCommand()
		self.r_pinkie.joint_name = 'r_hand_to_pinkie'
		self.r_pinkie.speed = 0.8
		self.r_pinkie.acceleration = 1.0

		self.l_pinkie = MotorCommand()
		self.l_pinkie.joint_name = 'l_hand_to_pinkie'
		self.l_pinkie.speed = 0.8
		self.l_pinkie.acceleration = 0.2

		self.l_ring = MotorCommand()
		self.l_ring.joint_name = 'l_hand_to_ring'
		self.l_ring.speed = 0.8
		self.l_ring.acceleration = 1.0

		self.l_middle = MotorCommand()
		self.l_middle.joint_name = 'l_hand_to_middle'
		self.l_middle.speed = 0.8
		self.l_middle.acceleration = 1.0

		self.l_index = MotorCommand()
		self.l_index.joint_name = 'l_hand_to_index'
		self.l_index.speed = 0.8
		self.l_index.acceleration = 1.0

		self.l_thumb = MotorCommand()
		self.l_thumb.joint_name = 'l_hand_to_thumb'
		self.l_thumb.speed = 0.6
		self.l_thumb.acceleration = 0.3

		self.r_base_one = MotorCommand()
		self.r_base_one.joint_name = 'base_to_r_arm_one'
		self.r_base_one.speed = 0.8
		self.r_base_one.acceleration = 0.3

		self.r_one_two = MotorCommand()
		self.r_one_two.joint_name = 'r_arm_one_to_arm_two'
		self.r_one_two.speed = 1.0
		self.r_one_two.acceleration = 0.3

		self.r_two_three = MotorCommand()
		self.r_two_three.joint_name = 'r_arm_two_to_arm_three'
		self.r_two_three.speed = 0.7
		self.r_two_three.acceleration = 0.3

		self.l_base_one = MotorCommand()
		self.l_base_one.joint_name = 'base_to_l_arm_one'
		self.l_base_one.speed = 0.8
		self.l_base_one.acceleration = 0.3

		self.l_one_two = MotorCommand()
		self.l_one_two.joint_name = 'l_arm_one_to_arm_two'
		self.l_one_two.speed = 1.0
		self.l_one_two.acceleration = 0.3

		self.l_two_three = MotorCommand()
		self.l_two_three.joint_name = 'l_arm_two_to_arm_three'
		self.l_two_three.speed = 0.7
		self.l_two_three.acceleration = 0.3

		# Set finger order here
		self.fingers = [self.l_thumb, self.l_index, self.l_middle, self.l_ring, self.l_pinkie, \
						self.r_index, self.r_middle, self.r_ring, self.r_pinkie, self.r_thumb]

		self.shoulders = [self.r_base_one, self.r_one_two, self.r_two_three, \
 				  self.l_base_one, self.l_one_two, self.l_two_three]

		self.shoulder_open = [1.0, 0.7, 0.0, 1.0, 0.8, 1.3]
		self.shoulder_rest = [0.0, 0.0, -1.0, 0.0, 0.0, -1.0]

		# self.ALL = range(5)
		# self.ALL += [7, 6, 5, 8, 9]
		self.ALL = range(10)

		#r = redis.StrictRedis(host='jeeves', port=6379, db=0)
		#self.redis_sub = r.pubsub()
		#self.old_message = ""

		threading.Thread.__init__(self)
		self.sleeper = rospy.Rate(10)

	def callback(self, data):
		numbers = data.data
                try:
                    num = int(numbers)
                    self.goFingers(numbers)
                except TypeError as e:
                    rospy.logwarn('Not a number')

		#self.goFingers(numbers)


	def goFingers(self, numbers):
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

		
		positions = [0.75, 0.75, 0.70, 0.70, 0.70]
		closed = self.ALL
		opens = []
		self.close_fingers(closed)
		sleep(0.5)
		self.open_fingers(self.ALL)
		sleep(0.5)
		self.move_shoulders(self.shoulder_open)
		self.close_fingers(self.ALL)
		closed = []
		thumbout = False

		if numbers == '10':
			opens = self.ALL
			self.open_fingers(opens)
			sleep(2)
			self.close_fingers(self.ALL)
			return

		if len(numbers) >= 2:
			self.move_shoulders([0.0, 0.0, -1.0, 1.0, 1.0, 1.3])

		for number in numbers:
			if number in '012345':
				self.move_shoulders([0.0, 0.0, -1.0, 1.0, 1.0, 1.3])
			
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
				self.move_shoulders([1.0, 1.0, 0.8, 1.0, 1.0, 1.2])
				opens = range(int(number))
				thumbout = True
			elif(number == '0'):
				opens = [2, 3, 4]

			else:
				pass

			# self.close_fingers(closed)
			if thumbout:
				self.open_fingers([9])
			self.open_fingers(opens)
			if thumbout:
				self.close_fingers([9])
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
		self.move_shoulders(self.shoulder_rest)

	def close_fingers(self, close):
		for c in close:
			self.fingers[c].position = 0.0
			self.pub.publish(self.fingers[c])
			sleep(0.1)
	
	def open_fingers(self, opens):
		for o in opens:
			self.fingers[o].position = 0.65
			rospy.loginfo(self.fingers[o])
			self.pub.publish(self.fingers[o])
			sleep(0.1)	

	def move_shoulders(self, openclose):
		for s in zip(self.shoulders, openclose):
			rospy.loginfo(s[0].joint_name)
			s[0].position = s[1]
			rospy.loginfo(s[0])
			self.pub.publish(s[0])

	#def redis_handler(self, message):
	#	rospy.loginfo("REDIS MESSAGE: %s" % message['data'])
	#	print("REDIS MESSAGE: %s" % message['data'])
	#	if message['data'] == self.old_message:
	#		return
 	#	self.old_message = message['data']
	#	answer = message['data'].split('is ')[-1]
	#	self.goFingers(answer)

	def run(self):
		#self.redis_sub.subscribe(**{'response': self.redis_handler})
		while not rospy.is_shutdown():
			#self.redis_sub.get_message()
			self.sleeper.sleep()

def main():
	rospy.init_node('num_finger', anonymous=True)
	finger_math = Num_Finger()
	finger_math.start()
	rospy.spin()

if __name__ == '__main__':
	main()
