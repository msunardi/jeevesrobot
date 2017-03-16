#!/usr/bin/env python
import sys, threading
import random as r
import rospy
from std_msgs.msg import Float32, Float64
from collections import deque

import numpy as np


class NeckWaver(threading.Thread):
    roll = 0.0
    pan = 0.0
    then = 0
    Fs = 100
    f = 5
    sample = 100
    running = False

    def __init__(self):
        self.yo_sub = rospy.Subscriber('/cmd_yo', Float32, self.yo_callback)
        # self.pitch_sub = rospy.Subscriber('/head/cmd_pose_pitch', Float32, self.pitch_callback)
        self.pan_pub = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=10)
        # self.pan_pub = rospy.Publisher('/head/cmd_pose_yaw', Float32, queue_size=10)
        self.tilt_pub = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=10)
        # self.roll_pub = rospy.Publisher('/head_roll_joint/command', Float64, queue_size=10)
        threading.Thread.__init__(self)
        self.sleeper = rospy.Rate(30)

    def run(self):
        rospy.loginfo("Setting up neck waver...")

        while not rospy.is_shutdown():
            # i = i % self.Fs
            # y = np.sin(2 * np.pi * self.f * i / self.Fs)
            # print y
            # i += 1
            # self.pan_pub.publish(Float32(y))
            # dtime = rospy.get_time() - self.then
            # if dtime > 2 + r.randint(1, 5):
            #     self.adjust_roll()
            #     self.then = rospy.get_time()

            self.sleeper.sleep()

    def yo_callback(self, data):
        if self.running:
            return
        r = data.data
        rospy.loginfo(rospy.get_caller_id() + "Got pan data: %s" % data.data)
        x = np.arange(self.Fs)
        self.running = True
        for i in x:
            # Making things up here ...
            # Make movement based on sine waves
            y = np.sin(r * np.pi * self.f * i / self.Fs) # This one for yaw
            p = -0.5*np.sin(0.7*r * np.pi * self.f * i / self.Fs) # This one for pitch
            # print y

            self.pan_pub.publish(Float64(y))
            self.tilt_pub.publish(Float64(p))
            self.sleeper.sleep() # Give a little delay before sending the next one
        self.running = False

def main(args):
    rospy.init_node('neck_wave_node', anonymous=True)
    necker = NeckWaver()
    necker.start()
    rospy.spin()


if __name__ == "__main__":
    main(sys.argv)

