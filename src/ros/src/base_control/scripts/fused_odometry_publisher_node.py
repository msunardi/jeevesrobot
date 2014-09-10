#!/usr/bin/env python
from collections import deque
import copy
import numpy as np
import sys
import threading
from threading import Lock
import time

import roslib
import rospy
from geometry_msgs.msg import Twist, Point, Pose2D, Quaternion
from nav_msgs.msg import Odometry
from roboteq_msgs.msg import Command as MotorCommand
from roboteq_msgs.msg import Feedback as MotorFeedback
from sensor_msgs.msg import Imu
import PyKDL as kdl
import tf

from base_control_node import BaseTransformHandler, WHEEL_RADIUS_m, \
    HALF_WHEELBASE_X_m, HALF_WHEELBASE_Y_m

# we get feeback messages from the motor controllers at 50 Hz,
# so let's publish at the same rate
ODOMETRY_UPDATE_RATE_Hz = 50

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

        self.lsm_listener = rospy.Subscriber(
            "/pose2D",
            Pose2D,
            self.lsm_pose2D_callback)

        # angular velocities, rads/s
        self.w_1 = 0.0
        self.w_2 = 0.0
        self.w_3 = 0.0
        self.w_4 = 0.0
        self.w_1_lock = threading.Lock()
        self.w_2_lock = threading.Lock()
        self.w_3_lock = threading.Lock()
        self.w_4_lock = threading.Lock()
        
        self.transformer = transformer
        self.sleeper = rospy.Rate(odometry_update_rate_hz)
        self.delta_t = 1.0 / odometry_update_rate_hz
        self.odom_publisher = rospy.Publisher("/odom", Odometry, queue_size=5)

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.frame_id = '/odom'
        self.child_frame_id = '/base_footprint'
        self.x = 0.0
        self.y = 0.0
        self.theta = (0.0, 0.0)         # theta, timestamp (seconds)
        self.theta_prev = (0.0, 0.1)    # theta, timestamp (seconds)
        self.lsm_theta = 0.0
        self.pose_lock = Lock()
        threading.Thread.__init__(self)

    def run(self):
        loop_count = 0
        w = [0.0, 0.0, 0.0, 0.0]
        while not rospy.is_shutdown():
            self.sleeper.sleep()

            # get the latest incoming update
            with self.w_1_lock:
                w[0] = self.w_1
            with self.w_2_lock:
                w[1] = self.w_1
            with self.w_3_lock:
                w[2] = self.w_1
            with self.w_4_lock:
                w[3] = self.w_1

            # we don't trust the angular part of the twist message, so use
            # the info from laser_scan_matcher
            twist = self.transformer.wheel_velocities_to_twist(w)
            with self.pose_lock:
                twist.angular.z =  (self.theta[0] - self.theta_prev[0]) / (self.theta[1] - self.theta_prev[1])
            rospy.logdebug("OdometryPublisher.run(): wheel velocities: " + str(w))
            rospy.logdebug("OdometryPublisher.run(): twist: " + str(twist))

            # calculate our new position using a
            # simple deterministic model
            delta_x = ((twist.linear.x * np.cos(self.theta[0]) - twist.linear.y
                        * np.sin(self.theta[0])) * self.delta_t)
            delta_y = ((twist.linear.x * np.sin(self.theta[0]) + twist.linear.y
                        * np.cos(self.theta[0])) * self.delta_t)
            self.x += delta_x
            self.y += delta_y

            # create an Odometry message
            msg = Odometry()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id # i.e. '/odom'
            msg.child_frame_id = self.child_frame_id # i.e. '/base_footprint'

            msg.twist.twist = twist
            msg.pose.pose.position = Point(self.x, self.y, 0.0)
            msg.pose.pose.orientation = Quaternion(*(kdl.Rotation.RPY(0.0, 0.0, self.theta[0]).GetQuaternion()))

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
        with self.w_1_lock:
            self.w_1 = feedback_msg.measured_velocity

    def motor_2_feedback_callback(self, feedback_msg):
        with self.w_2_lock:
            self.w_2 = feedback_msg.measured_velocity

    def motor_3_feedback_callback(self, feedback_msg):
        with self.w_3_lock:
            self.w_3 = feedback_msg.measured_velocity

    def motor_4_feedback_callback(self, feedback_msg):
        with self.w_4_lock:
            self.w_4 = feedback_msg.measured_velocity

    def lsm_pose2D_callback(self, pose_msg):
        with self.pose_lock:
            self.theta_prev = self.theta
            self.theta = (pose_msg.theta, rospy.get_time())

def main(args):
    rospy.init_node('base_odometry', anonymous=True, log_level=rospy.INFO)
    bth = BaseTransformHandler(WHEEL_RADIUS_m,
                               HALF_WHEELBASE_X_m,
                               HALF_WHEELBASE_Y_m)
    pub = OdometryPublisher(bth, ODOMETRY_UPDATE_RATE_Hz)
    pub.start()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
