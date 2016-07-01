#!/usr/bin/env python
import sys, threading
import rospy
from sensor_msgs.msg import JointState
from collections import deque
from std_msgs.msg import Float32, Float64
from ros_pololu_servo.srv import MotorRange
from ros_pololu_servo.msg import MotorCommand

from arbotix_msgs.srv import *

from poses import Poses

class ArmGesture(threading.Thread):

    def __init__(self):
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)

        self.right_sho_pitch = rospy.Subscriber('/right_sho_pitch/command', Float64, self.right_shoulder_pitch)
        self.right_sho_roll = rospy.Subscriber('/right_sho_roll/command', Float64, self.right_shoulder_roll)
        self.left_sho_pitch = rospy.Subscriber('/left_sho_pitch/command', Float64, self.left_shoulder_pitch)
        self.left_sho_roll = rospy.Subscriber('/left_sho_roll/command', Float64, self.left_shoulder_roll)
        #self.head_pan_relax = rospy.ServiceProxy(
        #    '/head_pan_joint/relax', Relax)
        #self.head_tilt_relax = rospy.ServiceProxy(
        #    '/head_tilt_joint/relax', Relax)
        #self.left_elbow_relax = rospy.ServiceProxy(
        #    '/left_elbow/relax', Relax)
        #self.left_sho_pitch_relax = rospy.ServiceProxy(
        #    '/left_sho_pitch/relax', Relax)
        #self.left_sho_roll_relax = rospy.ServiceProxy(
        #    '/left_sho_roll/relax', Relax)
        #self.right_elbow_relax = rospy.ServiceProxy(
        #    '/right_elbow/relax', Relax)        
        #self.right_sho_pitch_relax = rospy.ServiceProxy(
        #    '/right_sho_pitch/relax', Relax)        
        #self.right_sho_roll_relax = rospy.ServiceProxy(
        #    '/right_sho_roll/relax', Relax)

        # Ref: https://github.com/geni-lab/hri_common/blob/master/scripts/mini_head_lip_sync_node.py
        self.motor_pub = rospy.Publisher('pololu/command', MotorCommand, queue_size=1)
        self.motor_range_srv = rospy.ServiceProxy('pololu/motor_range', MotorRange)

        # Setup motor command
        self.left_sho_roll = MotorCommand()
        self.left_sho_roll.joint_name = 'left_shoulder_roll_joint'
        self.left_sho_roll.speed = 1.0
        self.left_sho_roll.acceleration = 1.0

        self.left_sho_pitch = MotorCommand()
        self.left_sho_pitch.joint_name = 'left_shoulder_pitch_joint'
        self.left_sho_pitch.speed = 1.0
        self.left_sho_pitch.acceleration = 1.0

        self.right_sho_roll = MotorCommand()
        self.right_sho_roll.joint_name = 'right_shoulder_roll_joint'
        self.right_sho_roll.speed = 1.0
        self.right_sho_roll.acceleration = 1.0

        self.right_sho_pitch = MotorCommand()
        self.right_sho_pitch.joint_name = 'right_shoulder_pitch_joint'
        self.right_sho_pitch.speed = 1.0
        self.right_sho_pitch.acceleration = 1.0

        threading.Thread.__init__(self)
        self.sleeper = rospy.Rate(10)

    def joint_callback(self, data):
    	names = data.name
    	position = data.position
    	fubar = zip(names, position) # create pairs
    	rospy.loginfo("data: %s" % fubar)
    	# rospy.loginfo("Positions: %s" % position)
        for joint in fubar:
            # rospy.loginfo(joint)
            joint_name = joint[0]
            joint_pos = joint[1]
            if joint_name == 'left_sho_roll':
                self.left_shoulder_roll(joint_pos)
            if joint_name == 'left_sho_pitch':
                self.left_shoulder_pitch(joint_pos)
            if joint_name == 'right_sho_roll':
                self.right_shoulder_roll(joint_pos)
            if joint_name == 'right_sho_pitch':
                self.right_shoulder_pitch(joint_pos)

    def left_shoulder_roll(self, data):
        # jimmy range: -1.0 - 0.6 (arm: lifted - lowered)
        # jeeves range: 0.0 - 1.0 (arm: lifted - lowered)
        datax = data.data
        # Cap data
        if datax < -1.0: datax = -1.0
        if datax > 0.6: datax = 0.6
        pos = self.translate(datax, 0.6, -1.0, 0.0, 1.0)
        self.left_sho_roll.position = pos
        rospy.loginfo("Translated Left Sho Roll: %s -> %s" % (datax, pos))
        self.motor_pub.publish(self.left_sho_roll)

    def left_shoulder_pitch(self, data):
        # jimmy range: -2.0 - 0.0 (arm: lifted - lowered)
        # jeeves range: 1.0 - 0.0 (arm: lifted - lowered)
        datax = data.data
        # Cap data
        if datax < -2.0: datax = -2.0
        if datax > 0.0: datax = 0.0
        pos = self.translate(datax, -2.0, 0.0, 1.0, 0.0)
        self.left_sho_pitch.position = pos
        rospy.loginfo("Translated Left Sho Pitch: %s -> %s" % (datax, pos))
        self.motor_pub.publish(self.left_sho_pitch)

    def right_shoulder_roll(self, data):
        # jimmy range: -1.0 - 0.6 (arm: lifted - lowered)
        # jeeves range: 0.0 - 1.0 (arm: lifted - lowered)
        datax = data.data
        # Cap data
        if datax < -0.5: datax = -0.5
        if datax > 1.0: datax = 1.0
        pos = self.translate(datax, -0.5, 1.0, 0.0, 1.0)
        self.right_sho_roll.position = pos
        rospy.loginfo("Translated Right Sho Roll: %s -> %s" % (datax, pos))
        self.motor_pub.publish(self.right_sho_roll)

    def right_shoulder_pitch(self, data):
        # jimmy range: -2.0 - 0.0 (arm: lifted - lowered)
        # jeeves range: 1.0 - 0.0 (arm: lifted - lowered)
        datax = data.data
        # Cap data
        if datax < 0.0: datax = 0.0
        if datax > 2.0: datax = 2.0
        pos = self.translate(datax, 0.0, 2.0, 0.0, 1.0)
        self.right_sho_pitch.position = pos
        rospy.loginfo("Translated Right Sho Pitch: %s -> %s" % (datax, pos))
        self.motor_pub.publish(self.right_sho_pitch)

    # Ref: http://stackoverflow.com/a/1969274/1019170
    def translate(self, value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan)


    def run(self):
    	#rospy.wait_for_service('/head_pan_joint/relax')
        #rospy.wait_for_service('/head_tilt_joint/relax')
        #rospy.wait_for_service('/left_elbow/relax')
        #rospy.wait_for_service('/left_sho_pitch/relax')
        #rospy.wait_for_service('/left_sho_roll/relax')
        #rospy.wait_for_service('/right_elbow/relax')
    	#rospy.wait_for_service('/right_sho_pitch/relax')
    	#rospy.wait_for_service('/right_sho_roll/relax')

    	#self.relax_servos()

    	while not rospy.is_shutdown():
    		self.sleeper.sleep()

    #def relax_servos(self):
    #	self.head_pan_relax()
    #	self.head_tilt_relax()
    # 	self.left_elbow_relax()
    #	self.left_sho_pitch_relax()
    #	self.left_sho_roll_relax()
    #	self.right_elbow_relax()
    #	self.right_sho_pitch_relax()
    #	self.right_sho_roll_relax()

def main(args):
    rospy.init_node('arm_gesture_node', anonymous=True)
    ag = ArmGesture()
    ag.start()
    rospy.spin()

if __name__=="__main__":
    main(sys.argv)   
