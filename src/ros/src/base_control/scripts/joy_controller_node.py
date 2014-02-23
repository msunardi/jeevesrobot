#!/usr/bin/env python
import copy
import sys
import threading
import time

import roslib
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyController(threading.Thread):
    
    def __init__(self):
        self.quit = False
        self.subscriber = rospy.Subscriber("/joy", Joy, self.callback)
        self.publisher = rospy.Publisher("/cmd_vel", Twist)
        self.linear_rate = 0.0
        self.angular_rate = 0.0
        self.axes = []
        self.buttons = []
        self.lock = threading.Lock()
        threading.Thread.__init__(self)    
    
    def run(self):
        while not rospy.is_shutdown():
            time.sleep(0.1)
            with self.lock:
                axes = copy.copy(self.axes)
                buttons = copy.copy(self.buttons)
            if (len(axes) == 0) or (len(buttons) == 0):
                continue     
            
            # populate a Twist message from the joystick state
            msg = Twist()
            rospy.logdebug("got a callback from Joy: \n" + "axes: " + str(axes) + '\n' + "buttons: " + str(buttons))            
            msg.linear.x = axes[1] * self.linear_rate
    
            if 1 == buttons[1]:
                msg.angular.z = -1.0 * self.angular_rate
                            
            if 1 == buttons[3]:
                msg.angular.z = 1.0 * self.angular_rate
            
            # publish            
            rospy.logdebug("Publishing to topic /cmd_vel: " + str(msg))
            self.publisher.publish(msg)
            
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
        
        rospy.loginfo("JoyController.run(): exiting.")
        
    def callback(self, data):
        with self.lock:
            self.axes = list(data.axes)
            self.buttons = list(data.buttons)


def main(args):
    controller = JoyController()
    controller.start()
    rospy.init_node('joy_controller_node', anonymous=True)
    rospy.spin()
        
if __name__ == '__main__':
    main(sys.argv)