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
import PyKDL as kdl
import tf


# SIMULATE_ROBOCLAWS = True
# WHEEL_RADIUS_m = 0.1016 # 4" radius wheels, in meters
# HALF_WHEELBASE_X_m = 0.2413 # 9.5" in meters
# HALF_WHEELBASE_Y_m = 0.2032 # 8" in meters
# MOTOR_CONTROLLER_POLL_RATE_Hz = 10
#
#         self.odom_publisher = OdometryPublisher(self.motor_mgr_output_queue,
#                                            BaseTransformHandler(WHEEL_RADIUS_m,
#                                                             HALF_WHEELBASE_X_m,
#                                                             HALF_WHEELBASE_Y_m),
#                                            MOTOR_CONTROLLER_POLL_RATE_Hz)
#
#                 self.odom_publisher.start()

class OdometryPublisher(threading.Thread):
    def __init__(self, transformer, motor_update_rate_hz):

        # front motors are #1, #4
        self.motor_1_listener = rospy.Subscriber(
            "/motor_controller_front/motor_1/feedback",
            MotorFeedback,
            self.motor_1_feedback_callback)

        self.motor_4_listener = rospy.Subscriber(
            "/motor_controller_front/motor_4/feedback",
            MotorFeedback,
            self.motor_4_feedback_callback)

        # rear motors are #2, #3
        self.motor_2_listener = rospy.Subscriber(
            "/motor_controller_rear/motor_2/feedback",
            MotorFeedback,
            self.motor_2_feedback_callback)

        self.motor_3_listener = rospy.Subscriber(
            "/motor_controller_rear/motor_3/feedback",
            MotorFeedback,
            self.motor_3_feedback_callback)

        # wheel angular velocities, rads/s
        self.w_1 = 0.0
        self.w_2 = 0.0
        self.w_3 = 0.0
        self.w_4 = 0.0

        self.transformer = transformer
        self.sleeper = rospy.Rate(motor_update_rate_hz)
        self.delta_t = 1.0 / motor_update_rate_hz
        self.motor_update_rate_hz = motor_update_rate_hz
        self.odom_publisher = rospy.Publisher("/odom", Odometry)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.frame_id = '/odom'
        self.child_frame_id = '/base_footprint'
        self.P = np.mat(np.diag([0.0]*3)) # covariance matrix
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        threading.Thread.__init__(self)

    def run(self):
        loop_count = 0
        while not rospy.is_shutdown():
            self.sleeper.sleep()
            if len(self.work_queue) == 0:
                continue
            else:
                # get the incoming update
                w = [self.w_1, self.w_2, self.w_3, self.w_4]
                twist = self.transformer.wheel_velocities_to_twist(w)
                rospy.logdebug("OdometryPublisher.run(): wheel velocities: " + str(w))
                rospy.logdebug("OdometryPublisher.run(): twist: " + str(twist))

                # calculate our new position using a
                # simple deterministic model
                delta_x = ((twist.linear.x * np.cos(self.theta) - twist.linear.y
                            * np.sin(self.theta)) * self.delta_t)
                delta_y = ((twist.linear.x * np.sin(self.theta) + twist.linear.y
                            * np.cos(self.theta)) * self.delta_t)
                delta_theta = twist.angular.z * self.delta_t
                self.x += delta_x
                self.y += delta_y
                self.theta += delta_theta

                # create an Odometry message
                msg = Odometry()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.frame_id # i.e. '/odom'
                msg.child_frame_id = self.child_frame_id # i.e. '/base_footprint'

                msg.twist.twist = twist
                msg.pose.pose.position = Point(self.x, self.y, 0.0)
                msg.pose.pose.orientation = Quaternion(*(kdl.Rotation.RPY(0.0, 0.0, self.theta).GetQuaternion()))

                p_cov = np.array([0.0]*36).reshape(6,6)

                # position covariance
                p_cov[0:2,0:2] = self.P[0:2,0:2]
                # orientation covariance for Yaw
                # x and Yaw
                p_cov[5,0] = p_cov[0,5] = self.P[2,0]
                # y and Yaw
                p_cov[5,1] = p_cov[1,5] = self.P[2,1]
                # Yaw and Yaw
                p_cov[5,5] = self.P[2,2]

                msg.pose.covariance = tuple(p_cov.ravel().tolist())

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


def main(args):
    rospy.init_node('base_controller_node', anonymous=True, log_level=rospy.INFO)
    pub = OdometryPublisher()
    pub.start()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)