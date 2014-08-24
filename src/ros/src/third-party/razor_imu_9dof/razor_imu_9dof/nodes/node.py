#!/usr/bin/env python

# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('razor_imu_9dof')
import rospy

import serial
import string
import math

from time import time
from sensor_msgs.msg import Imu
from razor_imu_9dof.msg import RazorImu
import tf

grad2rad = 3.141592/180.0

imu_pub = rospy.Publisher('imu', Imu, queue_size=5)
euler_pub = rospy.Publisher('imu_euler', RazorImu, queue_size=5)
imuMsg = Imu()
imuMsg.orientation_covariance = [999999, 0, 0, 0, 9999999, 0, 0, 0, 999999]
imuMsg.angular_velocity_covariance = [999999, 0, 0, 0, 99999, 0, 0, 0, 999999]
imuMsg.linear_acceleration_covariance = [0.2 , 0 , 0, 0, 0.2, 0,0 , 0 , 0.2]
eulerMsg = RazorImu()

def extract_triplet(s):
     vals = (s[s.find('=') + 1 : s.find('\r')]).split(',')
     return [float(x) for x in vals]

def main():
    
    # Check your COM port and baud rate
    default_port='/dev/imu'
    port = rospy.get_param('device', default_port)
    ser = serial.Serial(port=port,baudrate=57600, timeout=1.0)
    
    rospy.sleep(5) # Sleep for 8 seconds to wait for the board to boot then only write command.
    ser.write('#osct' + chr(13)) # 'Calibrated sensor readings'
    line = ser.readline()
    line = ser.readline()
    got_omegas = False
    got_accels = False
    got_eulers = False
    while True:
        line = ser.readline()
        try:
            if(line.find('#G') == 0):
                omegas = extract_triplet(line)
                imuMsg.angular_velocity.x = omegas[0]
                imuMsg.angular_velocity.y = omegas[1]
                imuMsg.angular_velocity.z = omegas[2]
                got_omegas = True
    
            if(line.find('#A') == 0):
                accels = extract_triplet(line)
                imuMsg.linear_acceleration.x = accels[0]
                imuMsg.linear_acceleration.y = accels[1]
                imuMsg.linear_acceleration.z = accels[2]
                got_accels = True

            if(line.find('#Y') == 0):
                ypr = extract_triplet(line)
                eulerMsg.yaw = ypr[0]
                eulerMsg.pitch = ypr[1]
                eulerMsg.roll = ypr[2]
                got_eulers = True
    
        except ValueError:
            pass
            
        #q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        #imuMsg.orientation.x = q[0]
        #imuMsg.orientation.y = q[1]
        #imuMsg.orientation.z = q[2]
        #imuMsg.orientation.w = q[3]

        if got_accels and got_omegas and got_eulers:
            imuMsg.header.stamp = rospy.Time.now()
            imuMsg.header.frame_id = 'base_footprint'
            imu_pub.publish(imuMsg)
            euler_pub.publish(eulerMsg)
            got_accels = False
            got_omegas = False
            got_eulers = False
            
    ser.close

if __name__ == '__main__':
    rospy.init_node("imu_node")
    main()