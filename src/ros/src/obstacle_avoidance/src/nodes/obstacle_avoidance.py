#!/usr/bin/env python

import copy
import sys
import threading
import time
import numpy as np

import rospy
from std_msgs.msg import String
from imu_thing.msg import RazorImu
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from fuzzy import *
from math import *

UPDATE_RATE_Hz = 5

class ObstacleAvoider(threading.Thread):

    def __init__(self):
        self.imu_subscriber = rospy.Subscriber("/imuRaw", RazorImu, self.imu_callback)
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.publisher = rospy.Publisher("cmd_vel", Twist)
        self.lock = threading.Lock()
        threading.Thread.__init__(self)
        self.imu_data = None
        self.scan_data = None

    def run(self):
        sleeper = rospy.Rate(UPDATE_RATE_Hz)

        left_near = MembershipFunction( ((0.,1.), (1.,1.), (1.5,0.)) )
        left_mid = MembershipFunction( ((1.,0.), (1.5,1.), (2.,1.), (2.5,0.)) )
        left_far = MembershipFunction( ((2.,0.), (2.5,1.), (5.,1.)) )

        front_near = MembershipFunction( ((0.,1.), (1.,1.), (2.,0.)) )
        front_mid = MembershipFunction( ((1.,0.), (2.,1.), (3.,1.), (4.5,0.)) )
        front_far = MembershipFunction( ((3.,0.), (4.5,1.),(6.,1.)) )

        right_near = MembershipFunction( ((0.,1.), (1.,1.), (1.5,0.)) )
        right_mid = MembershipFunction( ((1.,0.), (1.5,1.), (2.,1.), (2.5,0.)) )
        right_far = MembershipFunction( ((2.,0.), (2.5,1.), (5.,1.)) )

        output_turn_left = MembershipFunction( ((-pi/3., 1.), (-pi/4.5,1.), (-pi/5.,0.)) )
        output_no_turn = MembershipFunction( ((-pi/4.5,0.), (-pi/5.,1.), (pi/5.,1.), (pi/4.5,0.)) )
        output_turn_right = MembershipFunction( ((pi/5.,0.),(pi/4.5,1.),(pi/3.,1.)) )

        heading_delta_right = MembershipFunction( ((-1., 1.), (-0.07,1.),(-0.03,0.)) )
        heading_delta_front = MembershipFunction( ((-0.07,0.), (-0.03,1.), (0.03,1.), (0.07,0.)) )
        heading_delta_left = MembershipFunction( ((0.03,0.),(0.07,1.),(1.,1.)) )

        fuzz = Fuzzy()
        fuzz.add_input_membership('left')
        fuzz.add_input_membership_func('left','near', left_near)
        fuzz.add_input_membership_func('left','mid', left_mid)
        fuzz.add_input_membership_func('left','far', left_far)

        fuzz.add_input_membership('front')
        fuzz.add_input_membership_func('front','near', front_near)
        fuzz.add_input_membership_func('front','mid', front_mid)
        fuzz.add_input_membership_func('front','far', front_far)

        fuzz.add_input_membership('right')
        fuzz.add_input_membership_func('right','near', right_near)
        fuzz.add_input_membership_func('right','mid', right_mid)
        fuzz.add_input_membership_func('right','far', right_far)

        fuzz.add_input_membership('heading_delta')
        fuzz.add_input_membership_func('heading_delta','right',heading_delta_right)
        fuzz.add_input_membership_func('heading_delta','front',heading_delta_front)
        fuzz.add_input_membership_func('heading_delta','left',heading_delta_left)

        old_angular = 0.0
        delta_angular = 0.0
        angular_scaling = 0.7
        linear_rate = 0.15

        # Get heading
        while not self.imu_data:
            pass
        rospy.logdebug("Got IMU data: %s" % self.imu_data)
        initial_heading = self.imu_data.yaw
        delta_heading = 0.0

        while not rospy.is_shutdown():
            sleeper.sleep()
            #msg = Twist()
            rospy.logdebug("Ding!")
            if self.imu_data:
                rospy.logdebug("Got IMU data: %s" % self.imu_data)

                delta_heading = initial_heading - self.imu_data.yaw
                print "Initial: %s vs. current %s -- delta: %s" % (initial_heading, self.imu_data.yaw, delta_heading)
                heading = fuzz.get_membership_for('heading_delta', delta_heading)
            else:
                heading = fuzz.get_membership_for('heading_delta',0.0)

            if self.scan_data:
                rospy.logdebug("Got Scan data:")
                rospy.logdebug("Scan.ranges size: %s" % len(self.scan_data.ranges))

                scan = self.array_cleanup(self.scan_data.ranges)

                # Dividing the laser scan data to left, front and right perception
                # Scan data [right front left] (yes, it's flipped)
                # Subject to parameter changes
                left_distance = np.mean(scan[380:])
                right_distance = np.mean(scan[:260])
                front_distance = np.mean(scan[260:380])

                left = fuzz.get_membership_for('left', left_distance)
                right = fuzz.get_membership_for('right', right_distance)
                front = fuzz.get_membership_for('front', front_distance)
                

                print "Left: %s (Distance: %s)" % (left, left_distance)
                print "Right: %s (Distance: %s)" % (right, right_distance)
                print "Front: %s (Distance: %s)" % (front, front_distance)
                print "Heading offset: %s" % heading
                
                turn_right = fuzzyAND(fuzzyNOT(right['near']), fuzzyAND(front['near'], left['near']))**2\
                             + fuzzyAND(fuzzyNOT(right['near']), fuzzyAND(front['near'], left['mid']))**2\
                             + fuzzyAND(fuzzyNOT(right['near']), fuzzyAND(front['mid'], left['near']))**2\
                             + fuzzyAND(fuzzyNOT(right['near']), fuzzyAND(front['mid'], left['mid']))**2\
                             + fuzzyAND(fuzzyNOT(right['near']), fuzzyNOT(left['far']))**2\
                             + fuzzyAND(fuzzyNOT(right['near']), heading['left'])**2
         
                go_forward = fuzzyNOT(front['near'])**2\
                             + fuzzyAND(fuzzyNOT(front['near']), fuzzyAND(left['near'],right['near']))**2\
                             + fuzzyAND(fuzzyNOT(front['near']), fuzzyAND(left['near'], fuzzyNOT(right['near'])))**2\
                             + fuzzyAND(fuzzyNOT(front['near']), heading['front'])**2
                              
                turn_left = fuzzyAND(fuzzyNOT(left['near']), fuzzyAND(right['near'],front['near']))**2\
                            + fuzzyAND(fuzzyNOT(left['near']), fuzzyAND(right['near'], front['mid']))**2\
                            + fuzzyAND(fuzzyNOT(left['near']), fuzzyAND(right['mid'], front['near']))**2\
                            + fuzzyAND(fuzzyNOT(left['near']), fuzzyAND(right['mid'], front['mid']))**2\
                            + fuzzyAND(fuzzyNOT(left['near']), fuzzyNOT(right['far']))**2\
                            + fuzzyAND(fuzzyNOT(left['near']), heading['right'])**2
                                      
                print "Turn_right: %s" % turn_right
                print "Go_forward: %s" % go_forward
                print "Turn_left: %s" % turn_left

                if turn_right > 0.0 or turn_left > 0.0:
                    output = (sqrt(turn_right)*(-pi/4.) + sqrt(go_forward)*0.0 + sqrt(turn_left)*(pi/4.))/(sqrt(turn_right) + sqrt(go_forward) + sqrt(turn_left))
                else:
                    output = 0.0

                print "Output (turn degree): %s" % output

                delta_angular = output - old_angular

                msg = Twist()

                #angular_z = delta_angular * angular_rate
                angular_z = output * angular_scaling

                if angular_z > 0.38:
                    angular_z = 0.38

                print "Angular_z: %s" % angular_z

                msg.angular.z = angular_z

                if go_forward > 1.0:
                    go_forward = 1.0
                msg.linear.x = go_forward * linear_rate

                self.publisher.publish(msg)
                old_angular = output
            

        rospy.loginfo("ObstacleAvoider ... kicking the bucket ... Agh! *CLANG!!!* *bucket kicked*")

    def imu_callback(self, data):
        with self.lock:
            self.imu_data = data
        """ Imu data:
            float32 roll
            float32 pitch
            float32 yaw
        """

    def scan_callback(self, data):
        with self.lock:
            self.scan_data = data
        """ Scan data:
            uint32 seq
            time stamp
            string frame_id
            float32 angle_min
            float32 angle_max
            float32 angle_increment
            float32 time_increment
            float32 scan_time
            float32 range_min
            float32 range_max
            float32[] ranges
            float32[] intensities
        """

    def array_cleanup(self, data):
        return [x if str(x) != "nan" else 0.0 for x in data]
    
def main(args):
    rospy.init_node("obstacle_avoidance_node", anonymous=True, log_level=rospy.DEBUG)
    controller = ObstacleAvoider()
    controller.start()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
