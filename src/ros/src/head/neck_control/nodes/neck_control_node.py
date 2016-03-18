#!/usr/bin/env python
import sys, threading
import random as r
import rospy
from std_msgs.msg import Float32, Float64
from collections import deque

class NeckController(threading.Thread):
    roll = 0.0
    pan = 0.0
    then = 0

    def __init__(self):
        self.yaw_sub = rospy.Subscriber('/head/cmd_pose_yaw', Float32, self.yaw_callback)
        self.pitch_sub = rospy.Subscriber('/head/cmd_pose_pitch', Float32, self.pitch_callback)
        self.pan_pub = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=10)
        self.tilt_pub = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=10)
        self.roll_pub = rospy.Publisher('/head_roll_joint/command', Float64, queue_size=10)
        threading.Thread.__init__(self)
        self.sleeper = rospy.Rate(10)
        
    def run(self):
        rospy.loginfo("Setting up neck controller...")
        while not rospy.is_shutdown():
            dtime = rospy.get_time() - self.then
            if dtime > 2 + r.randint(1, 5):
                self.adjust_roll()
                self.then = rospy.get_time()
            self.sleeper.sleep() 

    def yaw_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Got pan data: %s" % data.data)
        self.pan = data.data * 0.8
        self.roll = data.data * 0.3
        self.pan_pub.publish(Float64(self.pan))
        #self.roll_pub.publish(Float64(self.roll))

    def pitch_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Got pitch data: %s" % data.data)
        tilt = data.data * -0.8
        self.tilt_pub.publish(Float64(tilt))

    def adjust_roll(self):
        x = r.random()
        if x < 0.2:
            self.roll += r.random() * self.pan
            self.roll = min(self.roll, 0.8)
        elif x >= 0.2 and x < 0.4:
            self.roll -= r.random() * self.pan
            self.roll = max(self.roll, -0.8)
        self.roll_pub.publish(Float64(self.roll))
            

def main(args):
    rospy.init_node('neck_controller_node', anonymous=True)
    necker = NeckController()
    necker.start()
    rospy.spin()

if __name__=="__main__":
    main(sys.argv)

