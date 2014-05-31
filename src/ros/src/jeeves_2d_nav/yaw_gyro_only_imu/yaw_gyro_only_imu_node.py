#!/usr/bin/env python

import roslib
import rospy

import serial
import string
import math
import pdb

from time import time
from sensor_msgs.msg import Imu
import tf

grad2rad = 3.141592/180.0

rospy.init_node("yaw_gyro_only_node")
pub = rospy.Publisher('imu', Imu)

imuMsg = Imu()
imuMsg.orientation_covariance = [999999 , 0 , 0,
0, 9999999, 0,
0, 0, 999999]
imuMsg.angular_velocity_covariance = [9999, 0 , 0,
0 , 99999, 0,
0 , 0 , 0.02]
imuMsg.linear_acceleration_covariance = [0.2 , 0 , 0,
0 , 0.2, 0,
0 , 0 , 0.2]

default_port='/dev/ttyUSB0'
port = rospy.get_param('device', default_port)
ser = serial.Serial(port=port,baudrate=57600, timeout=1)

GYRO_GAIN_RADS = 0.06957 * 0.01745329252 # GAIN * pi / 180

roll=0
pitch=0
yaw=0
dt = 0.020 # default IMU output rate is 50 Hz 
rospy.sleep(5) # Sleep for 8 seconds to wait for the board to boot then only write command.
ser.write('#osct' + chr(13)) # Calibrated sensor data, Text format
line = ser.readline()
line = ser.readline()

while not rospy.is_shutdown():
    
    # Typical gyro line looks like this: #G-C=-325.89,68.62,-37.15
    line = ser.readline()
    if line.find('#G') == 0:
        gyro_z = float(string.split(line, ',')[-1])
        gyro_z = gyro_z * GYRO_GAIN_RADS
        if abs(gyro_z) > 0.01:
            yaw = yaw + gyro_z * dt
            print yaw
    
    q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]
    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = 'imu'
    pub.publish(imuMsg)
            
ser.close
