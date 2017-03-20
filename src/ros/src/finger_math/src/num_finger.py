#!/usr/bin/env python
import rospy
from time import sleep
import string
import threading
from std_msgs.msg import String
from ros_pololu.msg import MotorCommand

class Num_Finger(threading.Thread):

	def __init__(self):
		self.sub = rospy.Subscriber("finger_solved", String, self.callback)
		self.finger_pub = rospy.Publisher('/pololu/command', MotorCommand, queue_size=10)

		# self.r_hand_to_pinkie = MotorCommand()
		# self.r_hand_to_pinkie.joint_name = 'r_hand_to_pinkie'
		# self.r_hand_to_pinkie.speed = 1.0
		# self.r_hand_to_pinkie.acceleration = 1.0

		# self.r_hand_to_thumb = MotorCommand()
		# self.r_hand_to_thumb.joint_name = 'r_hand_to_thumb'
		# self.r_hand_to_thumb.speed = 1.0
		# self.r_hand_to_thumb.acceleration = 1.0
		
		# self.r_hand_to_index = MotorCommand()
		# self.r_hand_to_index.joint_name = 'r_hand_to_index'
		# self.r_hand_to_index.speed = 1.0
		# self.r_hand_to_index.acceleration = 1.0

		# self.r_hand_to_middle = MotorCommand()
		# self.r_hand_to_middle.joint_name = 'r_hand_to_middle'
		# self.r_hand_to_middle.speed = 1.0
		# self.r_hand_to_middle.acceleration = 1.0

		# self.r_hand_to_ring = MotorCommand()
		# self.r_hand_to_ring.joint_name = 'r_hand_to_ring'
		# self.r_hand_to_ring.speed = 1.0
		# self.r_hand_to_ring.acceleration = 1.0

		self.l_pinkie = MotorCommand()
		self.l_pinkie.joint_name = 'l_pinkie'
		self.l_pinkie.speed = 1.0
		self.l_pinkie.acceleration = 1.0

		self.l_ring = MotorCommand()
		self.l_ring.joint_name = 'l_ring'
		self.l_ring.speed = 1.0
		self.l_ring.acceleration = 1.0

		self.l_middle = MotorCommand()
		self.l_middle.joint_name = 'l_middle'
		self.l_middle.speed = 1.0
		self.l_middle.acceleration = 1.0

		self.l_index = MotorCommand()
		self.l_index.joint_name = 'l_index'
		self.l_index.speed = 1.0
		self.l_index.acceleration = 1.0

		self.l_thumb = MotorCommand()
		self.l_thumb.joint_name = 'l_thumb'
		self.l_thumb.speed = 1.0
		self.l_thumb.acceleration = 1.0

		self.fingers = [self.l_thumb, self.l_index, self.l_middle, self.l_ring, self.l_pinkie]
		self.ALL = [0,1,2,3,4]

		threading.Thread.__init__(self)
		self.sleeper = rospy.Rate(10)

	def callback(self, data):
		number = data.data
		rospy.loginfo("Got data: %s" % number)
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
		sleep(1)
		self.open_fingers(self.ALL)
		sleep(1)
		self.close_fingers(self.ALL)

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
			opens = self.ALL

		elif(number == '0'):
			opens = [2, 3, 4]

		else:
			pass

		# self.close_fingers(closed)
		self.open_fingers(opens)

		# for pos in zip(fingers, positions):
		# 	pos[0].position = pos[1]
		# 	self.finger_pub.publish(pos[0])
		# 	sleep(0.05)

		sleep(2)
		# for p in fingers:
		# 	p.position = 0.75
		# 	self.finger_pub.publish(p)
		# 	sleep(0.05)
		# self.open_fingers(self.ALL)
		self.close_fingers(self.ALL)

	def close_fingers(self, close):
		for c in close:
			self.fingers[c].position = 0.0
			self.finger_pub.publish(self.fingers[c])
			sleep(0.1)
	
	def open_fingers(self, opens):
		for o in opens:
			self.fingers[o].position = 0.75
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
