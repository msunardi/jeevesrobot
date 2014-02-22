#!/usr/bin/env python
import sys
import time
import roslib
import rospy
from geometry_msgs.msg import Twist

import roboclaw

class BaseController(object):
    
    def __init__(self):
        self.front_drive = roboclaw.RoboClawSim('/dev/ttyUSB0', 2400, 250, 3336)
        self.rear_drive = roboclaw.RoboClawSim('/dev/ttyACM0', 2400, 250, 3336)
        self.subscriber = rospy.Subscriber("/cmd_vel", Twist, self.callback)
        self.x_prev = 0.0
        
    def callback(self, data):
        x = data.linear.x
        theta = data.angular.z
        rospy.loginfo("linear.x: " + str(x) + " angular.z: " + str(theta))

        if 0.0 != theta:
            cmd = theta * 10.0
            if 0.0 < theta:
                rospy.loginfo("Sending roboclaw.M2Forward(" + str(cmd) + ")")
                rospy.loginfo("Sending roboclaw.M1Backward(" + str(cmd) + ")")
                roboclaw.M1Forward(int(cmd))
                time.sleep(0.1)
                roboclaw.M2Backward(int(cmd))
            else:
                rospy.loginfo("Sending roboclaw.M1Forward(" + str(cmd) + ")")
                rospy.loginfo("Sending roboclaw.M2Backward(" + str(cmd) + ")")
                roboclaw.M2Forward(int(-cmd))
                time.sleep(0.1)
                roboclaw.M1Backward(int(-cmd))

        else:
            if 0.0 == x:
                rospy.loginfo("Sending roboclaw.MxForward(0)")
                roboclaw.M1Forward(0)
                time.sleep(0.1)
                roboclaw.M2Forward(0)
            else:
                cmd = x * 10.0
                if 0.0 < x:
                    rospy.loginfo("Sending roboclaw.MxForward(" + str(cmd) + ")")
                    roboclaw.M1Forward(int(cmd))
                    time.sleep(0.1)
                    roboclaw.M2Forward(int(cmd))
                else:
                    rospy.loginfo("Sending roboclaw.MxBackward(" + str(cmd) + ")")
                    roboclaw.M1Backward(int(-cmd))
                    time.sleep(0.1)
                    roboclaw.M2Backward(int(-cmd))

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
