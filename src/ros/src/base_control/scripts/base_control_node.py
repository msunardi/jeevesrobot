#!/usr/bin/env python
import sys
import time
import roslib
import rospy
from geometry_msgs.msg import Twist

import roboclaw

class BaseController(object):
    
    def __init__(self):
        roboclaw.init_port("")
        self.subscriber = rospy.Subscriber("/cmd_vel", Twist, self.callback)
        self.x_prev = 0.0
        
    def callback(self, data):
        x = data.linear.x
        theta = data.angular.z
        rospy.loginfo("linear.x: " + str(x) + " angular.z: " + str(theta))
        
        if 0.0 == x:
            rospy.loginfo("Sending roboclaw.MxForward(0)")
            roboclaw.M1Forward(0)
            time.sleep(0.1)
            roboclaw.M2Forward(0)
        else:
            cmd = x * 10.0
            if 0.0 < x:
                rospy.loginfo("Sending roboclaw.MxForward(" + str(x * 10.0) + ")")
                roboclaw.M1Forward(int(x * 10.0))
                time.sleep(0.1)
                roboclaw.M2Forward(int(x * 10.0))
            else:
                rospy.loginfo("Sending roboclaw.MxBackward(" + str(-x * 10.0) + ")")
                roboclaw.M1Backward(int(-x * 10.0))
                time.sleep(0.1)
                roboclaw.M2Backward(int(-x * 10.0))

def main(args):
    controller = BaseController()
    rospy.init_node('base_controller_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down base_controller."
        roboclaw.port.close()

if __name__ == '__main__':
    main(sys.argv)