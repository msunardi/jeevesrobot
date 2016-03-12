#!/usr/bin/env python
import sys, threading
from arbotix import Arbotix
import rospy
from std_msgs.msg import Float32, Float64
from collections import deque

class NeckController(threading.Thread):
    def __init__(self):
        self.yaw_sub = rospy.Subscriber('/head/cmd_pose_yaw', Float32, self.yaw_callback)
        self.pitch_sub = rospy.Subscriber('/head/cmd_pose_pitch', Float32, self.pitch_callback)
        self.pan_pub = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=10)
        self.tilt_pub = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=10)
        threading.Thread.__init__(self)
        self.sleeper = rospy.Rate(10)
        
    def run(self):
        rospy.loginfo("Setting up neck controller...")
        while not rospy.is_shutdown(): 
            self.sleeper.sleep() 

    def yaw_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Got pan data: %s" % data.data)
        pan = data.data * -1
        self.pan_pub.publish(Float64(pan))

    def pitch_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Got pitch data: %s" % data.data)
        tilt = data.data * -1
        self.tilt_pub.publish(Float64(tilt))

def main(args):
    rospy.init_node('neck_controller_node', anonymous=True)
    necker = NeckController()
    necker.start()
    rospy.spin()

if __name__=="__main__":
    main(sys.argv)

