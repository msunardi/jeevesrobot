#!/usr/bin/env python
import copy
import sys
import roslib
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyController(object):
    
    def __init__(self):
        self.subscriber = rospy.Subscriber("/joy", Joy, self.callback)
        self.publisher = rospy.Publisher("/cmd_vel", Twist)
        self.prev_axes = []
        self.prev_buttons = []
        self.prev_msg = Twist()
        self.linear_rate = 0.0
        self.angular_rate = 0.0
        
    def callback(self, data):
        axes = list(data.axes)
        buttons = list(data.buttons)
        
        # sometimes the right thumbstick sponaneously spews spurious
        # updates on the non-existent axis 2, so only react if there's
        # a change on any of the other axes or buttons
        axes[2] = 0.0
        if (self.prev_axes == axes) and (self.prev_buttons == buttons):
            return
        self.prev_axes = axes
        self.prev_buttons = buttons
        
        # populate a Twist message from the joystick state
        msg = Twist()
        #rospy.loginfo("got a callback from Joy: \n" + "axes: " + str(axes) + '\n' + "buttons: " + str(buttons))            
        msg.linear.x = axes[1] * self.linear_rate

        if 1 == buttons[1]:
            msg.angular.z = -1.0 * self.angular_rate
                        
        if 1 == buttons[3]:
            msg.angular.z = 1.0 * self.angular_rate
        
        # only publish if we have new information
        if ((msg.linear.x != self.prev_msg.linear.x) or
            (msg.angular.z != self.prev_msg.angular.z)):
            rospy.loginfo("Publishing to topic /cmd_vel: " + str(msg))
            self.publisher.publish(msg)
        self.prev_msg = copy.deepcopy(msg)
        
        # set command rates
        if 1 == buttons[8]: # button marked "9" on the top of the gamepad
            self.linear_rate = 0.0
            
        if 1 == buttons[9]: # button marked "10" on the top of the gamepad
            self.angular_rate = 0.0

        if 1 == buttons[4]: # left upper trigger
            self.linear_rate += 0.25
            
        if 1 == buttons[6]: # left lower trigger
            if self.linear_rate != 0.0:
                self.linear_rate -= 0.25
            
        if 1 == buttons[5]: # right upper trigger
            self.angular_rate += 0.25
            
        if 1 == buttons[7]: # right lower trigger
            if self.angular_rate != 0.0:
                self.angular_rate -= 0.25

def main(args):
    controller = JoyController()
    rospy.init_node('joy_controller_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down joy_controller."

if __name__ == '__main__':
    main(sys.argv)