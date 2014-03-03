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
import PyKDL as kdl
import tf

import roboclaw as rc

SIMULATE_ROBOCLAWS = False
WHEEL_RADIUS_m = 0.1016 # 4" radius wheels, in meters
HALF_WHEELBASE_X_m = 0.2413 # 9.5" in meters
HALF_WHEELBASE_Y_m = 0.2032 # 8" in meters
MOTOR_CONTROLLER_POLL_RATE_Hz = 10


class BaseController(threading.Thread):
    
    def __init__(self):
        #start a RoboClawManager
        ports = ('/dev/ttyUSB0', '/dev/ttyACM0')
        baudrate = 2400
        accel = 1000
        max_ticks_per_second = 3336
        self.sleeper = rospy.Rate(MOTOR_CONTROLLER_POLL_RATE_Hz)
        self.motor_mgr_cmd_queue = deque()
        self.motor_mgr_output_queue = deque()
        self.motor_mgr = rc.RoboClawManager(ports, baudrate, accel,
                                max_ticks_per_second,
                                rc.TICKS_PER_REV,
                                MOTOR_CONTROLLER_POLL_RATE_Hz,
                                self.motor_mgr_cmd_queue,
                                self.motor_mgr_output_queue,
                                SIMULATE_ROBOCLAWS)
        self.publisher = OdometryPublisher(self.motor_mgr_output_queue,
                                           BaseTransformHandler(WHEEL_RADIUS_m,
                                                            HALF_WHEELBASE_X_m,
                                                            HALF_WHEELBASE_Y_m),
                                           MOTOR_CONTROLLER_POLL_RATE_Hz)
        # start up worker threads
        self.motor_mgr.start()
        self.publisher.start()
        self.bth = BaseTransformHandler(WHEEL_RADIUS_m, HALF_WHEELBASE_X_m,
                                  HALF_WHEELBASE_Y_m)

        self.subscriber = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.cmd_vel_incoming = deque()
        self.cmd_vel_last = Twist()
        self.lock = threading.Lock()
        threading.Thread.__init__(self)    

    def run(self):
        """Pull a cmd_vel (Twist) message from the incoming queue, covert it
          to a tuple of motor speeds, and put them into the motor manager's
          work queue, ad infinitum"""
        twist = Twist()
        while not rospy.is_shutdown():
            # get a new command
            with self.lock:
                if len(self.cmd_vel_incoming) != 0:
                    twist = self.cmd_vel_incoming.popleft()

            # if it's a new command, convert it into motor speeds, else discard.
            if not (self.cmd_vel_last == twist):
                rospy.logdebug("cmd_vel message received: " + str(twist))
                w = self.bth.twist_to_wheel_velocities(twist)
                self.motor_mgr_cmd_queue.append(w)
            self.cmd_vel_last = twist                    
            self.sleeper.sleep()

        rospy.loginfo("BaseController.run(): waiting for motor_mgr to stop...")
        self.motor_mgr.quit = True
        self.motor_mgr.join()
        rospy.loginfo("BaseController.run(): exiting.")
        
    def cmd_vel_callback(self, twist_msg):
        with self.lock:
            self.cmd_vel_incoming.append(twist_msg)


class BaseTransformHandler(object):
    def __init__(self, R, l1, l2):
        L = l1 + l2
        assert l1 > 0.0
        assert l2 > 0.0
        assert R > 0.0
        self.R = R
        self.w4_to_v3 =  np.array([
            [1.0, 1.0, 1.0, 1.0],
            [-1.0, 1.0, -1.0, 1.0],
            [-1.0/L, -1.0/L, 1.0/L, 1.0/L]])
        self.v3_to_w4 = np.array([
            [1.0, -1.0, -L],
            [1.0, 1.0, -L],
            [1.0, -1.0, L],
            [1.0, 1.0, L]])

    def wheel_velocities_to_twist(self, w):
        w = np.array([w[0], w[1], w[2], w[3]])
        v = np.dot((self.R / 4.0) * self.w4_to_v3, w)
        twist = Twist()
        twist.linear.x = v[0]
        twist.linear.y = v[1]
        twist.angular.z = v[2]
        return twist

    def twist_to_wheel_velocities(self, twist):
        t = np.array([twist.linear.x, twist.linear.y, twist.angular.z])
        w = np.dot((1.0 / self.R) * self.v3_to_w4, t)
        return tuple(w)
    
class OdometryPublisher(threading.Thread):
    def __init__(self, work_queue, transformer, motor_update_rate_hz):
        self.work_queue = work_queue
        self.transformer = transformer
        self.sleeper = rospy.Rate(motor_update_rate_hz)
        self.delta_t = 1.0 / motor_update_rate_hz
        self.motor_update_rate_hz = motor_update_rate_hz
        self.publisher = rospy.Publisher("/odom", Odometry)
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
                w = self.work_queue.popleft()
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
                    self.publisher.publish(msg)          
                    self.tf_broadcaster.sendTransform(pos, ori, msg.header.stamp,
                                                        msg.child_frame_id,
                                                        msg.header.frame_id)
            

def main(args):
    rospy.init_node('base_controller_node', anonymous=True, log_level=rospy.DEBUG)
    controller = BaseController()
    controller.start()
    rospy.spin()
    

if __name__ == '__main__':
    main(sys.argv)
