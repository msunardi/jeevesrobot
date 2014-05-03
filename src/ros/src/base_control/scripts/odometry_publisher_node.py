#!/usr/bin/env python
from collections import deque
import copy
import numpy as np
import sys
import threading
import time

import roslib
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from roboteq_msgs.msg import Command as MotorCommand
from roboteq_msgs.msg import Feedback as MotorFeedback
from sensor_msgs.msg import Imu
import PyKDL as kdl
import tf

from base_control_node import BaseTransformHandler, WHEEL_RADIUS_m, \
    HALF_WHEELBASE_X_m, HALF_WHEELBASE_Y_m

ODOMETRY_UPDATE_RATE_Hz = 10


class OdometryPublisher(threading.Thread):
    def __init__(self, transformer, odometry_update_rate_hz):

        # front motors are #1, #4
        self.motor_1_listener = rospy.Subscriber(
            "/motor_1/feedback",
            MotorFeedback,
            self.motor_1_feedback_callback)

        self.motor_4_listener = rospy.Subscriber(
            "/motor_4/feedback",
            MotorFeedback,
            self.motor_4_feedback_callback)

        # rear motors are #2, #3
        self.motor_2_listener = rospy.Subscriber(
            "/motor_2/feedback",
            MotorFeedback,
            self.motor_2_feedback_callback)

        self.motor_3_listener = rospy.Subscriber(
            "/motor_3/feedback",
            MotorFeedback,
            self.motor_3_feedback_callback)

        # wheel angular velocities, rads/s
        self.w_1 = 0.0
        self.w_2 = 0.0
        self.w_3 = 0.0
        self.w_4 = 0.0

        self.transformer = transformer
        self.sleeper = rospy.Rate(odometry_update_rate_hz)
        self.delta_t = 1.0 / odometry_update_rate_hz
        self.odom_publisher = rospy.Publisher("/odom", Odometry)
        self.imu_subscriber = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.frame_id = '/odom'
        self.child_frame_id = '/base_footprint'
        self.P = np.mat(np.diag([0.0]*3)) # covariance matrix
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.imu_orientation = Quaternion()
        threading.Thread.__init__(self)

    def run(self):
        loop_count = 0
        while not rospy.is_shutdown():
            self.sleeper.sleep()

            # get the incoming update
            w = [self.w_1, self.w_2, self.w_3, self.w_4]
            twist = self.transformer.wheel_velocities_to_twist(w)
            rospy.logdebug("OdometryPublisher.run(): wheel velocities: " + str(w))
            rospy.logdebug("OdometryPublisher.run(): twist: " + str(twist))

            # calculate our new position using a
            # simple deterministic model
            q = self.imu_orientation
            self.theta = (kdl.Rotation.Quaternion(q.x, q.y, q.z, q.w).GetRPY())[2]
            delta_x = ((twist.linear.x * np.cos(self.theta) - twist.linear.y
                        * np.sin(self.theta)) * self.delta_t)
            delta_y = ((twist.linear.x * np.sin(self.theta) + twist.linear.y
                        * np.cos(self.theta)) * self.delta_t)
#            delta_theta = twist.angular.z * self.delta_t
            self.x += delta_x
            self.y += delta_y
#            self.theta += delta_theta

            # create an Odometry message
            msg = Odometry()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id # i.e. '/odom'
            msg.child_frame_id = self.child_frame_id # i.e. '/base_footprint'

            msg.twist.twist = twist
            msg.pose.pose.position = Point(self.x, self.y, 0.0)
            msg.pose.pose.orientation = self.imu_orientation

            pos = (msg.pose.pose.position.x,
                   msg.pose.pose.position.y,
                   msg.pose.pose.position.z)

            ori = (msg.pose.pose.orientation.x,
                   msg.pose.pose.orientation.y,
                   msg.pose.pose.orientation.z,
                   msg.pose.pose.orientation.w)

            if 0 == (loop_count % 5):
                # Publish odometry message and transform
                rospy.logdebug("OdometryPublisher.run(): publishing to /odom")
                self.odom_publisher.publish(msg)
                self.tf_broadcaster.sendTransform(pos, ori, msg.header.stamp,
                                                  msg.child_frame_id,
                                                  msg.header.frame_id)

    def motor_1_feedback_callback(self, feedback_msg):
        self.w_1 = feedback_msg.measured_velocity

    def motor_2_feedback_callback(self, feedback_msg):
        self.w_2 = feedback_msg.measured_velocity

    def motor_3_feedback_callback(self, feedback_msg):
        self.w_3 = feedback_msg.measured_velocity

    def motor_4_feedback_callback(self, feedback_msg):
        self.w_4 = feedback_msg.measured_velocity

    def imu_callback(self, imu_msg):
        self.imu_orientation = imu_msg.orientation
        self.imu_orientation.z = -self.imu_orientation.z
        self.imu_orientation.x = 0.0
        self.imu_orientation.y = 0.0
        self.imu_orientation.w = 1.0

def main(args):
    rospy.init_node('base_odometry', anonymous=True, log_level=rospy.INFO)
    bth = BaseTransformHandler(WHEEL_RADIUS_m,
                               HALF_WHEELBASE_X_m,
                               HALF_WHEELBASE_X_m)
    pub = OdometryPublisher(bth, ODOMETRY_UPDATE_RATE_Hz)
    pub.start()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
